//---Set odrive state i.e. 8 for CLC---
void state(int x){
  for(int i=0;i<2;i++){
    setAxisState(odrive_serial1, i, x);
    setAxisState(odrive_serial2, i, x);
    setAxisState(odrive_serial3, i, x);
    setAxisState(odrive_serial5, i, x);
    setAxisState(odrive_serial4, i, x);
    setAxisState(odrive_serial6, i, x);
  }
}

//---Set odrive input mode---
void setInputMode(int mode){
  for(int i=0;i<2;i++){
  // Send raw ASCII command to set input_mode
 // Format: w axisX.controller.config.input_mode <mode>
   Serial1.print("w axis"); Serial1.print(i); Serial1.print(".controller.config.input_mode "); Serial1.println(mode);
   Serial2.print("w axis"); Serial2.print(i); Serial2.print(".controller.config.input_mode "); Serial2.println(mode);
   Serial3.print("w axis"); Serial3.print(i); Serial3.print(".controller.config.input_mode "); Serial3.println(mode);
   Serial4.print("w axis"); Serial4.print(i); Serial4.print(".controller.config.input_mode "); Serial4.println(mode);
   Serial6.print("w axis"); Serial6.print(i); Serial6.print(".controller.config.input_mode "); Serial6.println(mode);
   Serial5.print("w axis"); Serial5.print(i); Serial5.print(".controller.config.input_mode "); Serial5.println(mode);
 }
}

//---gets axis pos absolute---
void pos_est(double pos[legn][coor]){
  pos[0][1] = (double) getAxisPosition(odrive_serial1, 0); //Leg 1
  pos[0][0] = (double) getAxisPosition(odrive_serial1, 1);
  pos[0][2] = (double) getAxisPosition(odrive_serial5, 1);
  pos[1][1] = (double) getAxisPosition(odrive_serial6, 0); //Leg 2
  pos[1][0] = (double) getAxisPosition(odrive_serial6, 1);
  pos[1][2] = (double) getAxisPosition(odrive_serial2, 0);  
  pos[2][1] = (double) getAxisPosition(odrive_serial3, 1); //Leg 3
  pos[2][0] = (double) getAxisPosition(odrive_serial3, 0);
  pos[2][2] = (double) getAxisPosition(odrive_serial5, 0);
  pos[3][1] = (double) getAxisPosition(odrive_serial4, 0); //Leg 4
  pos[3][0] = (double) getAxisPosition(odrive_serial4, 1);
  pos[3][2] = (double) getAxisPosition(odrive_serial2, 1);  
}

//---Moves odrive to pos absolute---
void actuate(double inp[legn][coor]){
   setAxisPosition(odrive_serial1, 0, inp[0][1]); // Leg 1
   setAxisPosition(odrive_serial1, 1, inp[0][0]);
   setAxisPosition(odrive_serial5, 1, inp[0][2]);
   setAxisPosition(odrive_serial6, 0, inp[1][1]); // Leg 2
   setAxisPosition(odrive_serial6, 1, inp[1][0]);
   setAxisPosition(odrive_serial2, 0, inp[1][2]);  
   setAxisPosition(odrive_serial3, 1, inp[2][1]); // Leg 3
   setAxisPosition(odrive_serial3, 0, inp[2][0]);
   setAxisPosition(odrive_serial5, 0, inp[2][2]);
   setAxisPosition(odrive_serial4, 0, inp[3][1]); // Leg 4
   setAxisPosition(odrive_serial4, 1, inp[3][0]);
   setAxisPosition(odrive_serial2, 1, inp[3][2]);  
}

//---moves odrive to knwoing starting position---
void actuate2(){
   setAxisPosition(odrive_serial1, 0,  -0.593109130859375); // Leg 1
   setAxisPosition(odrive_serial1, 1, -0.108620643);
   //setAxisPosition(odrive_serial2, 1, -0.0994833106994);   
   setAxisPosition(odrive_serial6, 0,  0.2015); // Leg 2
   setAxisPosition(odrive_serial6, 1, -0.40164);
   //setAxisPosition(odrive_serial5, 0, -0.08377933502);     
   setAxisPosition(odrive_serial3, 0, 0.0274391174); // Leg 3
   setAxisPosition(odrive_serial3, 1,  -0.5794696807861328);
   //setAxisPosition(odrive_serial2, 0, -0.16192886);         
   setAxisPosition(odrive_serial4, 0,   0.135413162); // Leg 4
   setAxisPosition(odrive_serial4, 1, 0.019038230180);
   //setAxisPosition(odrive_serial5, 1,  -0.4450721740);     
}

void modify_lim_gain(){
  for (int axis = 0; axis < 2; ++axis) {
    // Set Input Mode to TRAP_TRAJ (3)
    Serial1 << "w axis"<<axis<<".controller.config.input_mode "<<5<<'\n';
    Serial2 << "w axis"<<axis<<".controller.config.input_mode "<<5<<'\n';
    Serial3 << "w axis"<<axis<<".controller.config.input_mode "<<5<<'\n';
    Serial4 << "w axis"<<axis<<".controller.config.input_mode "<<5<<'\n';
    Serial6 << "w axis"<<axis<<".controller.config.input_mode "<<5<<'\n';
    Serial5 << "w axis"<<axis<<".controller.config.input_mode "<<5<<'\n';
    delay(50);
    Serial1 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial2 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial3 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial4 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial6 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial5 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    delay(50);
    Serial1 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial2 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial3 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial4 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial6 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial5 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    delay(50);
    Serial1 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial2 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial3 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial4 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial6 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial5 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    delay(50);
    Serial1 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial3 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial4 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial5 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    delay(50);
    Serial1 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial3 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial4 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial5 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    delay(50);
    Serial1 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial3 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial4 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial5 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    delay(50);
  }
}

void checkchange(){
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_limit "<< Serial1.parseFloat() << '\n';
    Serial2 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "vel_limit "<< Serial2.parseFloat() << '\n';
    Serial3 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_limit "<< Serial3.parseFloat() << '\n';
    Serial4 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial4 ="<< "vel_limit "<< Serial4.parseFloat() << '\n';
    Serial6 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "vel_limit "<< Serial6.parseFloat() << '\n';
    Serial5 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_limit "<< Serial5.parseFloat() << '\n';


    Serial1 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "accel_limit "<< Serial1.parseFloat() << '\n';
    Serial2 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "accel_limit "<< Serial2.parseFloat() << '\n';
    Serial3 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"accel_limit "<< Serial3.parseFloat() << '\n';
    Serial4 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial4 ="<< "accel_limit "<< Serial4.parseFloat() << '\n';
    Serial6 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "accel_limit "<< Serial6.parseFloat() << '\n';
    Serial5 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "accel_limit "<< Serial5.parseFloat() << '\n';

    Serial1 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "decel_limit "<< Serial1.parseFloat() << '\n';
    Serial2 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "decel_limit "<< Serial2.parseFloat() << '\n';
    Serial3 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"decel_limit "<< Serial3.parseFloat() << '\n';
    Serial4 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial4 ="<< "decel_limit "<< Serial4.parseFloat() << '\n';
    Serial6 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "decel_limit "<< Serial6.parseFloat() << '\n';
    Serial5 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "decel_limit "<< Serial5.parseFloat() << '\n';

    Serial1 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "pos_gain"<< Serial1.parseFloat() << '\n';
    Serial3 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"pos_gain"<< Serial3.parseFloat() << '\n';
    Serial4 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial4 ="<< "pos_gain"<< Serial4.parseFloat() << '\n';
    Serial5 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "pos_gain"<< Serial5.parseFloat() << '\n';

    Serial1 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_gain"<< Serial1.parseFloat() << '\n';
    Serial3 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_gain"<< Serial3.parseFloat() << '\n';
    Serial4 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial4 ="<< "vel_gain"<< Serial4.parseFloat() << '\n';
    Serial5 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_gain"<< Serial5.parseFloat() << '\n';

    Serial1 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_integrator_gain"<< Serial1.parseFloat() << '\n';
    Serial3 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_integrator_gain"<< Serial3.parseFloat() << '\n';
    Serial4 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial4 ="<< "vel_integrator_gain"<< Serial4.parseFloat() << '\n';
    Serial5 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_integrator_gain"<< Serial5.parseFloat() << '\n';
  }
}
