// Teensy 4.0 Robot Code with WiFi Integration (V2.9.4)
#include <math.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveUART.h> // ODrive v0.5.x ASCII UART interface
#include <Ramp.h>
#include <SerialTransfer.h>
#include "structdef.h"

// ---------------------------------------------------------
// WiFi / Communication Configuration START
// ---------------------------------------------------------
constexpr uint32_t WiFi_BAUD = 115200;
HardwareSerial& WiFi_serial = Serial7;
enum State { FIND_AA, FIND_55, READ_PAYLOAD, READ_SUM };
State comm_state = FIND_AA;
uint8_t payload[6];
int idx = 0;
uint8_t seq = 0;
// Control inputs from WiFi
int16_t wifi_a = 0;
int16_t wifi_b = 0;
int16_t wifi_c = 0;
// ---------------------------------------------------------
// WiFi / Communication Configuration END
// ---------------------------------------------------------

// ---------------------------------------------------------
// Robot Hardware and Tuning Configuration START
// ---------------------------------------------------------
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

HardwareSerial& odrive_serial1 = Serial5; 
HardwareSerial& odrive_serial2 = Serial6;
HardwareSerial& odrive_serial3 = Serial4;
HardwareSerial& odrive_serial4 = Serial1;
HardwareSerial& odrive_serial5 = Serial2; 
HardwareSerial& odrive_serial6 = Serial3;
// ODrive object initialization
// Leg index convention (matches structdef.h and gait tables): 0=FR, 1=BL, 2=FL, 3=BR
// Each leg ODrive controls two axes: axis0 and axis1 (hip and knee motors).
// Two shoulder ODrives each serve two legs (one axis per leg).
ODriveUART odrive1(odrive_serial1);  // ODrive 1 — Leg 0 FR (Serial5)
ODriveUART odrive2(odrive_serial2);  // ODrive 2 — Shoulder  BL(ax0) + BR(ax1) (Serial6)
ODriveUART odrive3(odrive_serial3);  // ODrive 3 — Leg 2 FL (Serial4)
ODriveUART odrive4(odrive_serial4);  // ODrive 4 — Leg 3 BR  (Serial1)
ODriveUART odrive5(odrive_serial5);  // ODrive 5 — Shoulder  FL(ax0) + FR(ax1) (Serial2)
ODriveUART odrive6(odrive_serial6);  // ODrive 6 — Leg 1 BL  (Serial3)


// UNUSED (testing purposes)
//matrix goes [FR, FL,BR,BL] - this matrix used to map the measured absolute encoder pos, can be put into code later
// double limitfrontintialPos[legn][coor] = {{ //axis 0 ext and axis 1comp
//   0.4987383484840393 ,  0.1413793563842 , 1. },  //leg x M0, legx M1, is shoulder postion(unused) 
//   {-1.25954151153564458, -0.3403044044971466 , 1. }, 
//   { -1.0587282180786133 ,   -0.8294696807861328 , 1. }, 
//   { 1.1962547302246094 ,  0.26903823018074036, 1.
// }};
// double limitbackintialPos[legn][coor] = {{//axis 0 comp axis 1 ext
//      -0.843109130859375 ,  -1.1795721054077148 , 1. },  //leg x M0, legx M1, is shoulder postion(unused) 
//   {  0.07016944885253906 , 0.9914951324462891 , 1. },
//   {0.2774391174316406 ,   0.5063495635986328 , 1. },
//   { -0.1145868375 ,  -1.0471487045288086 , 1.
// }};

// Motor Limits (Were initially 100)
float vellim = 100.0;
float accellim = 100.0;
float decellim = 100.0;
// Odrive Tuning Parameters
float posg = 100; //100
float velg = 0.3; //0.3
float velintg = 0.4; //0.4
float posgsh = 40.0; //shoulder
float velgsh = 0.08; //shoulder
float velintgsh = 0.2; //shoulder
// ---------------------------------------------------------
// Robot Hardware and Tuning Configuration END
// ---------------------------------------------------------

// Simulated packet items for compatibility with existing logic
// (Since the new WiFi packet only has 3 values, we need to decide how to map buttons)
int closedloopRequest = 0; // 1 = request IDLE, 2 = request closed-loop control
int stopRequest = 0; 
int moveEnableRequest = 0; 

// Gait Control Ratios
// set from WiFi commands, range typically -1.0 to +1.0
float LRrat;    // left-right strafe ratio. maps to 'b' in create_mat()
float FBrat;    // forward-backward ratio. maps to 'a' in create_mat()
float RTrat;    // yaw rotation ratio. maps to 'c' in create_mat()
float oldLRrat; // previous frame's value. used to detect command changes
float oldFBrat; // and trigger a gait matrix recompute when any ratio changes
float oldRTrat;
float ws_gain = 0.0; // weight shift gain: 0 = disabled, 1 = full centroid shift (~27mm lat, ~33mm fwd)
// Robot Physical Parameters
float leglen = 140.0;
float feetlen = 85.0;  
float hleg[4]      = { 280.0f, 280.0f, 280.0f, 280.0f };  
float hleg_base[4] = { 270.0f, 275.0f, 280.0f, 280.0f };  //  real measured physical standing height (mm) 
// Global Variables
int loop_timer = 13;  // 80ms base → ~200ms/phase at FBrat=1.0 (demand=1.0 → ×2.5), 3.2s full cycle
int citr; //current gait phase index (0-15)
int itrflag; //set to 1 when all 12 interpolations finish one phase
int closedloopflag;
int mode; //used first to set trap_traj (0 = standing, 1 = walking)
int offsetflag; //used in setup for finding motor postions in interpolation
int homeflag; //either 1 or 0 depending on leg at home pos (moves back to inipos when set to 1)
int itrchange; // when it has finished its (VARIABLE) point leg cycle in case it wants to change to a new gait
int modechange;//used to know when movement is called so it can assess wether a new gait is needed
int finishcount; //counts for all 4 legs and 3 coords (to 12) that myramp function is finished

long comm_millis; 
unsigned long currentmillis; 

double interpout [legn][coor];
double inipos[legn][coor];
double mat[legn][coor]; //inital gait matrix
double invmat[legn][coor];

// Gait Initialisation
gait_struct crawl;

// Function Prototypes
void state(int x);
void setInputMode(int mode); //added to set input mode to trap traj
void pos_est(double pos[legn][coor]);
void create_mat(double q[4][3],gait_struct *gait,float a,float b,float c,float ws,int iter);
void kinematics_mat(double inp[legn][coor], double out[legn][coor],double pos[legn][coor]);
void actuate(double inp[legn][coor]);
void actuate2();
void checkchange();
void modify_lim_gain();
void compute_offsets(float h, float &off1, float &off2);
void sendResponse(float, float, float);
void body_level_update(float terrain_pitch_deg, float terrain_roll_deg);

// ---------------------------------------------------------
// Interpolation Class START
// ---------------------------------------------------------
//takes an input and makes a linear contst movement to the input. interp flag used to track if it is interpolating still. and saved value used to tell if a new interp is required
//checkfinish makes sure the ramp function has finished its calcs
class Interpolation{
  public:
  rampDouble myRamp;
  int interpolationFlag = 0;
  double savedValue;

  double go_dbl(double input, int duration) { //duration always set to 20 microseconds
   if (input != savedValue){
     interpolationFlag = 0;
   }
     savedValue = input;
   if (interpolationFlag == 0){
     myRamp.go(input, duration, LINEAR, ONCEFORWARD);
     interpolationFlag = 1;
    }
   double output = myRamp.update();
   return output;
}
  bool checkfinish(){
    return myRamp.isFinished();
  }
};
Interpolation interp[legn][coor];

// ---------------------------------------------------------
// Interpolation Class END
// ---------------------------------------------------------

// ---------------------------------------------------------
// Setup
// ---------------------------------------------------------
void setup() {
  // Initialize Debug Serial
  Serial.begin(9600);

  for (int i = 0; i < 4; i++) hleg_base[i] = hleg[i]; // initialise hleg_base (see body_level.ino)
  
  // Initialize ODrive Serials
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial6.begin(115200);
  Serial5.begin(115200);
  WiFi_serial.begin(WiFi_BAUD);
  WiFi_serial.setTimeout(50);
  Serial.println("Teensy 4.0 Robot with WiFi Started");
  //Set IDLE state first
  state(1);
  closedloopflag = 0;
  Serial.println("IDLE");
  delay(500);
  modify_lim_gain();//sets trapt traj and limits and PID
  checkchange();
  delay(500);
  state(8); //CLC
  setInputMode(5); // Set Input Mode to TRAP_TRAJ
  closedloopflag = 1;
  delay(1000);
  actuate2(); // move all joints to the hardcoded encoder home position (280 mm standing height)
  Serial.println("gone to start pos");
  delay(3000);
  Serial.println("starting interp");



  // Obtain initial pos estimate at actuate2() calibrated home (280 mm)
  pos_est(inipos);
  // hleg_base[] stays at 280 mm (the encoder reference, never change this).
  // hleg[] is the target standing height; set it here for the desired walking height.
  for(int ii=0;ii<legn;ii++) {
    hleg[ii] = 280.0f; 
  }
  Serial.println("Changed Hleg");
  delay(3000);
  for(int i=0;i<legn;i++){
    mat[i][0] = 0;
    mat[i][1] = 0;
    mat[i][2] = (hleg[i]-hleg_base[i])/-2.2;
  }
  kinematics_mat(mat,invmat,inipos);
  actuate(invmat);
  Serial.println("Actuate Invmat");
  delay (5000);
  imu_setup();  // run setup function from imu_func.ino (might neeed to change serial baud) 

  // Initialize Interpolation objects
  for(int ii=0;ii<legn;ii++){
        for(int jj=0;jj<coor;jj++){
          interpout[ii][jj] = interp[ii][jj].go_dbl(inipos[ii][jj],20);
          Serial<<"inipos["<<ii<<"]["<<jj<<"] = "<<inipos[ii][jj]<<'\t';

          if(inipos[ii][jj] == 0){
            Serial.println("Offset error");
            offsetflag = 1;
          }
        }
        Serial.print('\n');
    }
  delay(20);
  // Second update ensures the ramp library's internal state is consistent
  // with the target before the main loop starts polling it.
  for(int ii=0;ii<legn;ii++){
      for(int jj=0;jj<coor;jj++){
          interpout[ii][jj] = interp[ii][jj].go_dbl(inipos[ii][jj],20);
        }
    }

  // Print initial positions
  for(int ii=0;ii<legn;ii++){
      for(int jj=0;jj<coor;jj++){
          Serial.print(interpout[ii][jj]);
          Serial.print('\t');
          }
      Serial.print('\n');
    }

  if(offsetflag!= 1){
    Serial.println("MQRPII is ready");
    Serial.println("IDLE");}
    
  Serial.print("Finished Startup");
}

// ---------------------------------------------------------
// Main Loop
// ---------------------------------------------------------
void loop() {
  
  imu_loop();  // run loop function from imu_func.ino
  
  currentmillis = millis();
  // --- WiFi Packet Handling ---
  while (WiFi_serial.available()) {
    Serial.println("In Wifi Loop");
    uint8_t b = (uint8_t)WiFi_serial.read();
    switch (comm_state) {
      case FIND_AA:
        if (b == 0xAA) comm_state = FIND_55;
        break;
      case FIND_55:
        comm_state = (b == 0x55) ? READ_PAYLOAD : FIND_AA;
        idx = 0;
        break;
      case READ_PAYLOAD:
        payload[idx++] = b;
        if (idx >= 6) comm_state = READ_SUM;
        break;
      case READ_SUM: {
        uint8_t sum = 0;
        for (int i = 0; i < 6; ++i) sum += payload[i];
        if (sum == b) {
          // Parse payload
          wifi_a = (int16_t)((payload[0] << 8) | payload[1]);
          wifi_b = (int16_t)((payload[2] << 8) | payload[3]);
          wifi_c = (int16_t)((payload[4] << 8) | payload[5]);

          // Serial.printf("RX: a=%d b=%d c=%d\n", wifi_a, wifi_b, wifi_c);
          // Map WiFi inputs to Control Ratios and State Machine Items
          // wifi_a: Command (1 = Start/Forward, 0 = Stop)
          // wifi_b: Left
          // wifi_c: Right
          // if all wifi inputs are 0 then stop (LiDAR triggered or explicit stop)
          if (wifi_a == 0 && wifi_b == 0 && wifi_c == 0) {
            Serial.println("WiFi Stop");
            // STOP COMMAND
            // zero packet from ESP (LiDAR obstacle or controller stop)
            // 1. Request Stop/Home
            stopRequest = 1;
            // 2. Keep in closed loop (do not go IDLE, motors stay energised to hold position)
            closedloopRequest = 2;
            // 3. Reset movement flag
            moveEnableRequest = 0;

            // Zero all movement ratios immediately
            FBrat = 0;
            LRrat = 0;
            RTrat = 0;
          }
          //If wifi_a=wifi_b=wifi_c=1 then go backwards
          if (wifi_a == 1 && wifi_b == 1 && wifi_c == 1) {
            Serial.println("WiFi Backwards");
            // BACKWARDS COMMAND
            // 1. Request Closed Loop Control
            closedloopRequest = 2;
            // 2. Request Movement Mode
            moveEnableRequest = 1;
            // 3. Reset Stop flags
            stopRequest = 0;
            // Set Backwards Speed (adjust value as needed, e.g. 0.5)
            FBrat = -0.5;
            LRrat = 0;
            RTrat = 0;
          }
          //If wifi_a=1 then go forwards
          if (wifi_a == 1 && wifi_b == 0 && wifi_c == 0) {
            Serial.println("WiFi Start");
            // START COMMAND
            // 1. Request Closed Loop Control
            closedloopRequest = 2;
            // 2. Request Movement Mode
            moveEnableRequest = 1;
            // 3. Reset Stop flags
            stopRequest = 0;

            // Set Forward Speed (adjust value as needed, e.g. 0.5)
            FBrat = 0.5;
            LRrat = 0;
            RTrat = 0;
          }
          //if wifi_b=1 then go left
          if (wifi_a == 0 && wifi_b == 1 && wifi_c == 0) {
            Serial.println("WiFi Left");
            // LEFT COMMAND
            // 1. Request Closed Loop Control
            closedloopRequest = 2;
            // 2. Request Movement Mode
            moveEnableRequest = 1;
            // 3. Reset Stop flags
            stopRequest = 0;

            // Set Left Speed (adjust value as needed, e.g. 0.5)
            FBrat = 0;
            LRrat = -0.5;
            RTrat = 0;
          }
          //if wifi_c=1 then go right
          if (wifi_a == 0 && wifi_b == 0 && wifi_c == 1) {
            Serial.println("WiFi Right");
            // RIGHT COMMAND
            // 1. Request Closed Loop Control
            closedloopRequest = 2;
            // 2. Request Movement Mode
            moveEnableRequest = 1;
            // 3. Reset Stop flags
            stopRequest = 0;

            // Set Right Speed (adjust value as needed, e.g. 0.5)
            FBrat = 0;
            LRrat = 0.5;
            RTrat = 0;
          }
           comm_millis = currentmillis;
          // Send Response
          // Echo a, negate b, add seq to c (as per example)
          int16_t r1 = wifi_a;
          int16_t r2 = (int16_t)(-wifi_b);
          int16_t r3 = (int16_t)(wifi_c + (int16_t)seq);
          sendResponse(r1, r2, r3);
          seq++;
        } else {
          Serial.println("Checksum mismatch, packet dropped");
        }
        comm_state = FIND_AA;
        break;
      }
    }
  }

  // --- Transitions ---
  // Transition to Closed Loop Control (CLC)
  if(closedloopflag==0 && closedloopRequest == 2){
    state(8);
    setInputMode(5); // Set Input Mode to TRAP_TRAJ
    closedloopflag = 1;
    delay(50);
 
    Serial.println("CLC ");
  }
  // Transition to IDLE
  if(closedloopflag==1 && closedloopRequest == 1){
    state(1);
    closedloopflag = 0;
    closedloopRequest = 2;
    Serial.println("IDLE");
    delay(50);
  }
  // Enable Movement Mode
  if(closedloopflag == 1 && moveEnableRequest == 1){
    mode = 1;
    homeflag = 0;
    modechange = 1;
    //Serial.println("Enable Movement");
  }

  // Disable Movement Mode (Stop/Home)
  // When stopRequest is set, immediately stop gait and command all feet
  //to home position (inipos) via actuate(inipos).
  if(mode == 1 && stopRequest == 1){
    mode = 0;
    homeflag = 1;
    Serial.println("Mode: STOP/HOME");
    citr = 0;
  }

  //Homing-move all feet to initial (home) position
  if(homeflag == 1){
    actuate(inipos);
    //actuate2(); if actuate does not work
    homeflag = 0;
    Serial.println("Feet homed");
  }

 //--- Main Control Loop (Gait Execution) ---
  if(mode == 1 && closedloopflag == 1){
    //creates new walking step
    // Recompute the gait matrix whenever the walking mode starts, a phase
    // completes, or the speed command changes; avoids stale step targets.
    if((modechange == 1 || itrchange == 1) || (LRrat != oldLRrat || FBrat != oldFBrat || RTrat != oldRTrat)){ 
      //need an old legn[4] check here
      create_mat(mat,&crawl,FBrat,LRrat,RTrat,ws_gain,citr);
      kinematics_mat(mat,invmat,inipos);
      modechange = 0;
      itrchange = 0;
    }
     float demand = max(max(abs(FBrat), abs(LRrat)), abs(RTrat));
  int dynamic_timer = (int)(loop_timer * (1.0f + demand * 1.5f));

  for (int i = 0; i < legn; i++)
    for (int j = 0; j < coor; j++)
      interpout[i][j] = interp[i][j].go_dbl(invmat[i][j], dynamic_timer);

  actuate(interpout);

    for(int ii=0;ii<legn;ii++){
        for(int jj=0;jj<coor;jj++){
          if(interp[ii][jj].checkfinish() == true){
            finishcount += 1;
          }
        }
    }
    if(finishcount == 12){
      itrflag = 1;
      itrchange = 1;
      finishcount =0;
    }
    if(itrflag==1 && citr<15){
      citr += 1;
      itrflag = 0;
    }
    else if(itrflag == 1 && citr == 15){
      citr = 0;
      itrflag = 0;
    }
    oldLRrat = LRrat;
    oldFBrat = FBrat;
    oldRTrat = RTrat;
    finishcount = 0;
  }
}
