# Odrive Calibration 

___

Using the python odrive library(<a href="https://docs.odriverobotics.com/v/latest/interfaces/odrivetool.html">odrivetool</a>)

It is recommended to individually calibrate each driver on a PSU



### Useful Commands:

**// Factory reset Odrive**

odrv0.erase\_configuration()



**//test voltage**

odrv0.vbus\_voltage



**//check errors**

dump\_errors(odrv0)

odrv0.clear\_errors()



**//check axis state: no state 0, IDLE state 1, closed loop 8,** 

odrv0.axis0.current\_state 



**//check input mode: trap traj 5**

odrv0.axis0.controller.config.input\_mode



**//To save the configuration to a file on the PC**

odrivetool backup-config my\_config.json



**//To restore the configuration form such a file**

odrivetool restore-config my\_config.json



### General Calibration Commands:

Dependent on the type of motor and use the following set of commands outline the list of instructions to calibrate the motor.

To achieve a reliable and accurate setup both a configuration and anti-cogging set of commands should be saved onto the odrive.

#### Turnigy Configuration Commands:

odrv0.axis0.motor.config.current\_lim = 40

odrv0.axis0.controller.config.vel\_limit = 5



odrv0.axis1.motor.config.current\_lim = 40

odrv0.axis1.controller.config.vel\_limit = 5



**\#only for firmware 0.5.5 and 0.5.6 else remove**

odrv0.config.enable\_brake\_resistor = True  



odrv0.config.brake\_resistance = 2

odrv0.config.dc\_bus\_overvoltage\_trip\_level = 26





**\#Motor Configuration for Turnigy 100 kV**

odrv0.axis0.motor.config.motor\_type = 0

odrv0.axis0.motor.config.pole\_pairs = 20

odrv0.axis0.motor.config.torque\_constant = 8.27/100

odrv0.axis0.motor.config.calibration\_current = 20



odrv0.axis1.motor.config.motor\_type = 0

odrv0.axis1.motor.config.pole\_pairs = 20

odrv0.axis1.motor.config.torque\_constant = 8.27/100

odrv0.axis1.motor.config.calibration\_current = 20





**\#Encoder configuration for AS5047D - SPI Mode (enter the correct pin)**

odrv0.axis0.encoder.config.abs\_spi\_cs\_gpio\_pin = 8

odrv0.axis0.encoder.config.mode = ENCODER\_MODE\_SPI\_ABS\_AMS

odrv0.axis0.encoder.config.cpr = 2\*\*14



odrv0.axis1.encoder.config.abs\_spi\_cs\_gpio\_pin = 7

odrv0.axis1.encoder.config.mode = ENCODER\_MODE\_SPI\_ABS\_AMS

odrv0.axis1.encoder.config.cpr = 2\*\*14



**//save \& reboot**

odrv0.save\_configuration()

odrv0.reboot()

**//Note: Power cycling may be required (as there can be encoder positional errors)**



**//Encoder calibration SPI mode (nothing should happend)**

odrv0.axis0.requested\_state = AXIS\_STATE\_ENCODER\_OFFSET\_CALIBRATION

odrv0.axis1.requested\_state = AXIS\_STATE\_ENCODER\_OFFSET\_CALIBRATION





**//full calibration (beep sound + motor rotate in both direction)**

odrv0.axis0.requested\_state = AXIS\_STATE\_FULL\_CALIBRATION\_SEQUENCE

odrv0.axis1.requested\_state = AXIS\_STATE\_FULL\_CALIBRATION\_SEQUENCE





**//flag calibration as true**

odrv0.axis0.encoder.config.pre\_calibrated = True

odrv0.axis1.encoder.config.pre\_calibrated = True



odrv0.axis0.motor.config.pre\_calibrated = True

odrv0.axis1.motor.config.pre\_calibrated = True





**//PID Control**

odrv0.axis0.controller.config.pos\_gain = 100

odrv0.axis0.controller.config.vel\_gain = 0.3

odrv0.axis0.controller.config.vel\_integrator\_gain = 0.4



odrv0.axis1.controller.config.pos\_gain = 100

odrv0.axis1.controller.config.vel\_gain = 0.3

odrv0.axis1.controller.config.vel\_integrator\_gain = 0.4





**\#trajectory control**

odrv0.axis0.trap\_traj.config.vel\_limit = 5.0

odrv0.axis0.trap\_traj.config.accel\_limit = 5.0

odrv0.axis0.trap\_traj.config.decel\_limit = 5.0



odrv0.axis1.trap\_traj.config.vel\_limit = 5.0

odrv0.axis1.trap\_traj.config.accel\_limit = 5.0

odrv0.axis1.trap\_traj.config.decel\_limit = 5.0





**//Save and reboot**

odrv0.save\_configuration()

odrv0.reboot()

**//Now the motors are capable of being used without further calibration**

#### Surpass Configuration Commands:

odrv0.axis0.motor.config.current\_lim = 40

odrv0.axis0.controller.config.vel\_limit = 5



odrv0.axis1.motor.config.current\_lim = 40

odrv0.axis1.controller.config.vel\_limit = 5



**\#only for firmware 0.5.5 and 0.5.6 else remove**

odrv0.config.enable\_brake\_resistor = True 



odrv0.config.brake\_resistance = 2

odrv0.config.dc\_bus\_overvoltage\_trip\_level = 25





**\#Motor Configuration for C5065 435 kV**

odrv0.axis0.motor.config.motor\_type = 0

odrv0.axis0.motor.config.pole\_pairs = 7

odrv0.axis0.motor.config.torque\_constant = 8.27/435

odrv0.axis0.motor.config.calibration\_current = 20



odrv0.axis1.motor.config.motor\_type = 0

odrv0.axis1.motor.config.pole\_pairs = 7

odrv0.axis1.motor.config.torque\_constant = 8.27/435

odrv0.axis1.motor.config.calibration\_current = 20





**\#Encoder configuration for AS5047D - SPI Mode (enter the correct pin)**

odrv0.axis0.encoder.config.abs\_spi\_cs\_gpio\_pin = 8

odrv0.axis0.encoder.config.mode = ENCODER\_MODE\_SPI\_ABS\_AMS

odrv0.axis0.encoder.config.cpr = 2\*\*14



odrv0.axis1.encoder.config.abs\_spi\_cs\_gpio\_pin = 7

odrv0.axis1.encoder.config.mode = ENCODER\_MODE\_SPI\_ABS\_AMS

odrv0.axis1.encoder.config.cpr = 2\*\*14



**//save \& reboot**

odrv0.save\_configuration()

odrv0.reboot()

**//Note: Power cycling may be required (as there can be encoder positional errors)**



**//Encoder calibration SPI mode (nothing should happend)**

odrv0.axis0.requested\_state = AXIS\_STATE\_ENCODER\_OFFSET\_CALIBRATION

odrv0.axis1.requested\_state = AXIS\_STATE\_ENCODER\_OFFSET\_CALIBRATION





**//full calibration (beep sound + motor rotate in both direction)**

odrv0.axis0.requested\_state = AXIS\_STATE\_FULL\_CALIBRATION\_SEQUENCE

odrv0.axis1.requested\_state = AXIS\_STATE\_FULL\_CALIBRATION\_SEQUENCE





**//flag calibration as true: so on the next boot you can skip the full calibration and go straight to closed loop**

odrv0.axis0.encoder.config.pre\_calibrated = True

odrv0.axis1.encoder.config.pre\_calibrated = True



odrv0.axis0.motor.config.pre\_calibrated = True

odrv0.axis1.motor.config.pre\_calibrated = True





**//PID Control**

odrv0.axis0.controller.config.pos\_gain = 40

odrv0.axis0.controller.config.vel\_gain = 0.125

odrv0.axis0.controller.config.vel\_integrator\_gain = 0.2



odrv0.axis1.controller.config.pos\_gain = 40

odrv0.axis1.controller.config.vel\_gain = 0.125

odrv0.axis1.controller.config.vel\_integrator\_gain = 0.2





**\#trajectory control**

odrv0.axis0.trap\_traj.config.vel\_limit = 5.0

odrv0.axis0.trap\_traj.config.accel\_limit = 5.0

odrv0.axis0.trap\_traj.config.decel\_limit = 5.0



odrv0.axis1.trap\_traj.config.vel\_limit = 5.0

odrv0.axis1.trap\_traj.config.accel\_limit = 5.0

odrv0.axis1.trap\_traj.config.decel\_limit = 5.0



**//Save and reboot**

odrv0.save\_configuration()

odrv0.reboot()

**//Now the motors are capable of being used without further calibration**

### Anti-cogging Commands:

**\#starting axis0 anti-cogging**

odrv0.axis0.controller.config.control\_mode = CONTROL\_MODE\_POSITION\_CONTROL

odrv0.axis0.requested\_state = AXIS\_STATE\_CLOSED\_LOOP\_CONTROL

odrv0.axis0.controller.config.anticogging.pre\_calibrated = False

odrv0.axis0.controller.start\_anticogging\_calibration()



**\\#starting axis0 anti-cogging**

odrv0.axis1.controller.config.control\_mode = CONTROL\_MODE\_POSITION\_CONTROL

odrv0.axis1.requested\_state = AXIS\_STATE\_CLOSED\_LOOP\_CONTROL

odrv0.axis1.controller.config.anticogging.pre\_calibrated = False

odrv0.axis1.controller.start\_anticogging\_calibration()



**//run until anticogging.calib\_anticogging == false or anticogging\_valid == True**  

**//odrv0.axis0.controller.config.anticogging.calib\_anticogging**

**//	True -> calibration is running**

**//	False -> calibration routine finished (or never started)**

**//odrv0.axis0.controller.anticogging\_valid**

**//	False -> no valid map yet**

**//	True -> map is complete and considered valid**

**//Flags**

odrv0.axis0.controller.config.anticogging.calib\_anticogging



odrv0.axis0.controller.anticogging\_valid



odrv0.axis1.controller.config.anticogging.calib\_anticogging



odrv0.axis1.controller.anticogging\_valid





**// note run this command if the number increase: anti-cogging calibration routine is actively stepping through positions**

**// if the command return 0 usually anti-cogging finish run test or error occurred then dump errors and restart**

**// It can be slow (take \~20 mins) as default steps is 3600 (odrv0.axis0.config.anticogging.calib\_anticogging\_num\_steps)**

**//Check how many steps through anti-cogging the odrive is**

odrv0.axis0.controller.config.anticogging.index



odrv0.axis1.controller.config.anticogging.index



**//after anticogging\_valid == True or anticogging.calib\_anticogging == false** 

**//flag calibration as true**

odrv0.axis0.controller.config.anticogging.pre\_calibrated = True

odrv0.axis0.controller.config.anticogging.anticogging\_enabled = True

odrv0.axis0.requested\_state = AXIS\_STATE\_IDLE



odrv0.axis1.controller.config.anticogging.anticogging\_enabled = True

odrv0.axis1.controller.config.anticogging.pre\_calibrated = True

odrv0.axis1.requested\_state = AXIS\_STATE\_IDLE



**//This should not return false, if it does it is likely not in IDLE**

odrv0.save\_configuration() 



**//save**

odrv0.save\_configuration()

odrv0.reboot()



### Running Commands:

**//optional check**

odrv0.axis0.motor.is\_calibrated    \#should print True



odrv0.axis0.encoder.is\_ready      \#should print True



odrv0.axis1.motor.is\_calibrated    \#should print True



odrv0.axis1.encoder.is\_ready      \#should print True



**//Used to find absolute position to place legs on start up**

odrv0.axis0.encoder.pos\_estimate 

odrv0.axis1.encoder.pos\_estimate 



**//Enter close loop and trap traj control**

odrv0.axis0.requested\_state = AXIS\_STATE\_CLOSED\_LOOP\_CONTROL

odrv0.axis1.requested\_state = AXIS\_STATE\_CLOSED\_LOOP\_CONTROL

odrv0.axis0.controller.config.input\_mode = INPUT\_MODE\_TRAP\_TRAJ

odrv0.axis1.controller.config.input\_mode = INPUT\_MODE\_TRAP\_TRAJ



**//this is an absolute position not a movement** 

odrv0.axis0.controller.input\_pos = 1

odrv0.axis1.controller.input\_pos = 1

**//**

odrv0.axis0.controller.input\_pos = 0

odrv0.axis1.controller.input\_pos = 0





