#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <SPI.h>
#include "imu_calibration.h"

// ---------------------------------------------------------
// IMU Configuration
// ---------------------------------------------------------
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t imu_timer; // Renamed from 'timer' to avoid conflict with structdef.h or other globals
// uint8_t i2cData[14]; // Buffer for I2C data - Not used in this implementation


// --- Slope detection constants --- 
// --- Gait wobble compensation (must be tuned per robot) ---
// These values cancel the periodic pitch/roll oscillation the walking gait
// itself induces on the IMU, so body_level_update() only sees real terrain tilt.
//
// How to tune: run the robot on flat ground, log kalAngleY_corr over one full
// gait cycle, then fit a 2-term Fourier series to the result.
// gaitPeriod: measure with a stopwatch (time for 16 phases at your normal FBrat).
// phaseOffset_p/r: shift the fitted curve to align with gait phase 0.
// a1/b1/a2/b2: Fourier coefficients from the fit.
// binWobble_p/r: alternative lookup-table approach (50 bins over one gait cycle_)
//
// Currently set to placeholder values (1.23)
// NOTE: binWobble_p and binWobble_r are currently identical placeholders.
// Replace each with the real fitted values for pitch and roll separately.
float gaitPeriod = 1.464;   // seconds
float phaseOffset_p = -0.12;
float phaseOffset_r = -0.12;

// Fourier constants //   to be changed
float a1_p = 1.23;
float b1_p = 1.23;
float a2_p = 1.23;
float b2_p = 1.23;

float a1_r = 1.23;
float b1_r = 1.23;
float a2_r = 1.23;
float b2_r = 1.23;


// Mean pitch per bin (corrected) //  to be changed
float binWobble_p[50] = {
  0.12, 0.25, 0.40, 0.61, 0.75, 0.82, 0.90, 1.02, 1.10, 1.18,
  1.20, 1.15, 1.05, 0.90, 0.70, 0.50, 0.30, 0.10, -0.10, -0.30,
  -0.50, -0.70, -0.90, -1.05, -1.15, -1.20, -1.18, -1.10, -1.02, -0.90,
  -0.82, -0.75, -0.61, -0.40, -0.25, -0.12, 0.05, 0.18, 0.32, 0.45,
  0.60, 0.75, 0.85, 0.95, 1.05, 1.10, 1.05, 0.90, 0.70, 0.40
};

// Mean roll per bin (corrected) //  to be changed
float binWobble_r[50] = {
  0.12, 0.25, 0.40, 0.61, 0.75, 0.82, 0.90, 1.02, 1.10, 1.18,
  1.20, 1.15, 1.05, 0.90, 0.70, 0.50, 0.30, 0.10, -0.10, -0.30,
  -0.50, -0.70, -0.90, -1.05, -1.15, -1.20, -1.18, -1.10, -1.02, -0.90,
  -0.82, -0.75, -0.61, -0.40, -0.25, -0.12, 0.05, 0.18, 0.32, 0.45,
  0.60, 0.75, 0.85, 0.95, 1.05, 1.10, 1.05, 0.90, 0.70, 0.40
};



void imu_setup() {
  // Initialize Debug Serial
  Serial.begin(115200); // Updated to 115200 for IMU compatibility
  
  // --- IMU Setup --- //
  Wire.begin();
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  // ---------------------------------------------------------
  // IMU Calibration
  // ---------------------------------------------------------  

  //  runCalibration(bno); // auto calibration (using the other files) runs first
  
  // Manually load calibration data to BNO
  bno.setMode(OPERATION_MODE_CONFIG);  // ENTER CONFIG MODE

  adafruit_bno055_offsets_t manualoffsets;
  manualoffsets.accel_offset_x = 73;
  manualoffsets.accel_offset_y = -75;
  manualoffsets.accel_offset_z = -19;

  manualoffsets.gyro_offset_x = -2;
  manualoffsets.gyro_offset_y = -3;
  manualoffsets.gyro_offset_z = -1;

  manualoffsets.mag_offset_x = -2;
  manualoffsets.mag_offset_y = -2;
  manualoffsets.mag_offset_z = -34;

  manualoffsets.accel_radius = 1000;
  manualoffsets.mag_radius = 679;

  bno.setSensorOffsets(manualoffsets);

  bno.setMode(OPERATION_MODE_NDOF);  // EXIT CONFIG MODE


  // Read calib data from EEPROM and load them into BNO
  // // Load the offsets from EEPROM (starting at address 0)
  // adafruit_bno055_offsets_t offsets;
  // EEPROM.get(0, offsets);

  // // Check if valid offsets were retrieved (basic validation)
  // if (offsets.accel_offset_x != 0 && offsets.gyro_offset_x != 0) { // Simple check
  //     bno.setSensorOffsets(offsets);
  //     Serial.println("Calibration data restored from EEPROM.");
  //
  // } else {
  //     Serial.println("No saved calibration data found in EEPROM, starting fresh.");
  // }

  bno.setExtCrystalUse(true);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  // Remap BNO055 axes to match the IMU's physical mounting orientation on this robot.
  accX = acc.z();
  accY = acc.y();
  accZ = -acc.x();

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  imu_timer = micros();

  // Seed the notch filters with the initial Kalman angles so they don't ring at startup
  imu_filter_init((float)kalAngleY, (float)kalAngleX);
}


void imu_loop() {
  currentmillis = millis();
  float seconds = currentmillis / 1000.0f;  // time in seconds

  // --- IMU Update ---
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  
  /* Update all the values */
  accX = acc.z();
  accY = acc.y();
  accZ = -acc.x();
  gyroX = gyr.z();
  gyroY = gyr.y();
  gyroZ = -gyr.x(); 

  double dt = (double)(micros() - imu_timer) / 1000000; // Calculate delta time
  imu_timer = micros();

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX;
  double gyroYrate = gyroY;

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.85 * (compAngleX + gyroXrate * dt) + 0.15 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.85 * (compAngleY + gyroYrate * dt) + 0.15 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


  // ---------------------------------------------------------
  // Rotate values to match IMU orientation + correction
  // ---------------------------------------------------------
  // Trim offsets to correct for IMU mounting angle on the chassis (measured manually).
  // Adjust these if the robot thinks it's tilted on a known-flat surface. 
  float offsetY = -3.0;
  float kalAngleY_corr = -((kalAngleY - 0.0 + offsetY));

  float offsetX = -3.3;
  float kalAngleX_corr = -((kalAngleX + 0.0 + offsetX));


  // ---------------------------------------------------------
  // Slope detection calcs - pitch
  // ---------------------------------------------------------

  bool robotMoving = (mode == 1 && closedloopflag == 1); // is the robot moving?

  float phase_p = fmod((seconds-phaseOffset_p) / gaitPeriod, 1.0);  // Calculate the normalised phase 0 -> 1

  //Fourier Method
  float thetaWobble_F_p =
  a1_p * sin(2 * PI * phase_p) +
  b1_p * cos(2 * PI * phase_p) +
  a2_p * sin(4 * PI * phase_p) +
  b2_p * cos(4 * PI * phase_p);

  float effectiveWobble_F_p = robotMoving ? thetaWobble_F_p : 0.0f; // if not moving wobble = 0
  
  float thetaTerrain_F_p = kalAngleY_corr - effectiveWobble_F_p;  //Terrain estimation

  //Bins Method
  // int bin_p = (int)(phase_p * 50);
  // if (bin_p >= 50)
  // {
  //   bin_p = 49;
  // }

  // float thetaWobble_B_p = binWobble_p[bin_p];
  // float effectiveWobble_B_p = robotMoving ? thetaWobble_B_p : 0.0f;
  
  // float thetaTerrain_B_p = kalAngleY_corr - effectiveWobble_B_p;


  // ---------------------------------------------------------
  // Slope detection calcs - roll
  // ---------------------------------------------------------

  float phase_r = fmod((seconds-phaseOffset_r) / gaitPeriod, 1.0);  // Calculate the normalised phase 0 -> 1

  //Fourier Method
  float thetaWobble_F_r =
  a1_r * sin(2 * PI * phase_r) +
  b1_r * cos(2 * PI * phase_r) +
  a2_r * sin(4 * PI * phase_r) +
  b2_r * cos(4 * PI * phase_r);

  float effectiveWobble_F_r = robotMoving ? thetaWobble_F_r : 0.0f;
  
  float thetaTerrain_F_r = kalAngleX_corr - effectiveWobble_F_r;  //Terrain estimation

  //Bins Method
  int bin_r = (int)(phase_r * 50);
  if (bin_r >= 50)
  {
    bin_r = 49;
  }

  float thetaWobble_B_r = binWobble_r[bin_r];
  float effectiveWobble_B_r = robotMoving ? thetaWobble_B_r : 0.0f;
  
  float thetaTerrain_B_r = kalAngleX_corr - effectiveWobble_B_r;

  // ---------------------------------------------------------
  // Notch filter
  // remove gait oscillations from Kalman output
  // ---------------------------------------------------------
  float terrain_pitch_notch, terrain_roll_notch;
  imu_filter_update(kalAngleY_corr, kalAngleX_corr,
                    &terrain_pitch_notch, &terrain_roll_notch);

  // ---------------------------------------------------------
  // Body level inputs
  // ---------------------------------------------------------
  body_level_update(terrain_pitch_notch, terrain_roll_notch);  // notch-filtered terrain estimate


  // ---------------------------------------------------------
  // IMU Output (Serial Monitor)
  // ---------------------------------------------------------
  // Format: kalRoll, kalPitch, notchPitch, notchRoll, hleg[0..3]
  Serial.print(kalAngleX_corr);
  Serial.print(",");
  Serial.print(kalAngleY_corr);
  Serial.print(",");
  Serial.print(terrain_pitch_notch);
  Serial.print(",");
  Serial.print(terrain_roll_notch);

  Serial.print(",");
  Serial.print(hleg[0], 1); Serial.print(",");
  Serial.print(hleg[1], 1); Serial.print(",");
  Serial.print(hleg[2], 1); Serial.print(",");
  Serial.println(hleg[3], 1);

}
