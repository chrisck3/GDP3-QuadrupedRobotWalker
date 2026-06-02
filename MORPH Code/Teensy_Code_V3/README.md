# MQRPII Quadruped Firmware (Teensy 4.0)

Firmware for a four-legged walking robot, running on a Teensy 4.0. It drives 12 motors through six ODrive controllers, walks under WiFi command, and uses a BNO055 IMU to keep the body level on sloped ground.

This was part of a Group Design Project at the University of Southampton. I'm putting it up open-source so the next person who picks up a quadruped like this doesn't have to start from scratch. The code works and the robot walks, but it is research firmware, not a polished product. There are bits that are tuned and proven on the rig, and bits that are still placeholders waiting for someone to finish them. We've tried to be honest about which is which throughout, both here and in the comments.

## What it does

- Walks forward, backward, left, right, and yaws on the spot, commanded over WiFi.
- Converts foot position targets into ODrive encoder commands through a closed-form 2-link inverse kinematic model.
- Reads body pitch and roll from a BNO055 (Kalman-filtered) and adjusts individual leg heights so the body stays level on a slope.
- Stops on command, including a zero-packet stop intended for a LiDAR obstacle trigger.

The default gait is a 16-phase crawl (one leg at a time). A trot gait is included in the source but commented out, with instructions for switching.

## Hardware

| Teensy 4.0 : Main controller. Uses six hardware serial ports (Serial1-Serial6) for the ODrives and Serial7 for WiFi. |
| 6x ODrive (v0.5.x):| Driven over ASCII UART at 115200 baud. Four control a leg each (hip + knee); two are shoulder drives that each serve two legs. |
| 12x BLDC motors : Two per leg (hip, knee) plus shoulder abduction. |
| BNO055 IMU : On I2C (address 0x28). Used for slope detection. |
| ESP (WiFi link) : Sends the 3-value control packet to the Teensy. |

### Leg and ODrive mapping

The leg index convention is used everywhere in the code, so it's worth stating up front:

```
0 = FR (front right)
1 = BL (back left)
2 = FL (front left)
3 = BR (back right)
```

The mapping lives in `pos_est()` and `actuate()` in `odrv_rw_functions.ino`. If you rewire anything, that is the single place you have to get right, or a leg will move on the wrong axis.

## Code structure

This is an Arduino multi-tab sketch. All the `.ino` files compile together as one program alongside the main `Teensy_Code_V2_9_4.ino`. 

| File | What's in it |
|------|--------------|
| `Teensy_Code_V2_9_4.ino` | Main sketch. Setup, the WiFi packet state machine, the command mapping, and the main gait loop. |
| `structdef.h` | The gait tables. Pre-computed foot trajectories for the crawl gait (and a commented-out trot gait), phase by phase. |
| `matrix_kinematics.ino` | Inverse kinematics. Turns x/y/z foot targets into ODrive encoder turns. Also builds the per-phase gait matrix in `create_mat()`. |
| `odrv_rw_functions.ino` | Higher-level ODrive routines: set state, set input mode, read positions, actuate all 12 axes, push tuning gains. |
| `odrv_dual_axis_ctrl_Func.ino` | Low-level ODrive ASCII command wrappers (read/write a single axis). |
| `wifi_func.ino` | Builds and sends the framed response packet back to the ESP. |
| `imu_func.ino` | BNO055 setup and the per-loop IMU read, Kalman filtering, axis remapping, and gait-wobble removal. |
| `imu_notch_filter.ino` | Cascaded IIR notch filters that strip the gait oscillation out of the IMU signal so only real terrain tilt is left. |
| `imu_calibration.h` / `.cpp` | Save and restore BNO055 calibration offsets to EEPROM. |
| `body_level.ino` | Takes the cleaned terrain pitch/roll and works out how much to raise or lower each leg to keep the body level. |

## How it works

### Walking

The robot walks by stepping through a fixed table of foot positions (`structdef.h`). Each of the 16 phases gives every foot a target offset, made up of a vertical lift arc plus forward/back, lateral, and yaw components. The WiFi command scales those components: forward speed scales the FB table, strafe scales LR, turning scales RT. `create_mat()` sums them, `kinematics_mat()` converts the result to encoder positions, and an interpolation step ramps the motors smoothly between phases.

A phase only advances once all 12 interpolations have finished, so the gait timing follows the slowest joint rather than a fixed clock. The loop timer also scales with commanded speed, so a faster command means a faster cycle.

### Staying level on a slope

The IMU gives body pitch and roll, but walking itself rocks the body, so the raw signal is a mix of real slope and self-induced wobble. The firmware deals with this in two stages. `imu_notch_filter.ino` notches out the gait frequency and its second harmonic, leaving a cleaner terrain estimate. `body_level.ino` then converts that pitch and roll into a height correction for each foot and feeds it into `hleg[]`, the per-leg standing height the kinematics uses.

The slope correction is an integrator with memory, not a simple proportional adjustment. The reasoning behind that (and why a naive version caused the body to snap back and oscillate) is written out in the header comment of `body_level.ino`. If you're going to touch the levelling, read that first.

### Communication

The ESP sends an 8-byte frame: `0xAA 0x55` header, six payload bytes (three big-endian int16 values), and a one-byte checksum. The Teensy parses it with a small state machine in the main loop, maps the three values to movement commands, and sends a framed acknowledgement back. An all-zero packet is treated as a stop, which is how the LiDAR obstacle trigger is meant to halt the robot.

## Getting it running

1. Install the Arduino IDE, and the libraries below.
2. Wire the six ODrives to Serial1-Serial6 and the WiFi module to Serial7. Check the leg mapping in `odrv_rw_functions.ino` matches your wiring.
3. Calibrate and tune your ODrives first using ODrive's own tools. This firmware assumes the motors are already configured and only pushes gains and limits on top.
4. Open `Teensy_Code_V2_9_4.ino` (it pulls in the other tabs automatically) and flash it.
5. On the first run, watch the serial monitor at startup. It prints the initial leg positions and flags an offset error if any axis reads zero, which usually means an ODrive isn't responding.

### Dependencies

- `ODriveUART` (ODrive v0.5.x ASCII interface)
- `Adafruit_BNO055` and `Adafruit_Sensor`
- `Kalman` by TKJ Electronics (https://github.com/TKJElectronics/KalmanFilter)
- `Ramp`
- `SerialTransfer`

## Things to know before you trust it

I'd rather you hit these in the README than discover them mid-test:

- **The IMU mounting offsets are hand-measured.** The trim values in `imu_func.ino` correct for how the BNO055 sits on this chassis. On a different build they will be wrong. If the robot thinks it's tilted while sitting on a flat floor, that's the first thing to adjust.
- **The gait-wobble compensation is not finished.** The notch filter is the working path for removing wobble; the Fourier and bin methods are earlier experiments left in for reference. Don't expect the Fourier terrain estimate to mean anything until those coefficients are fitted to real logged data. The how-to-tune notes are in the comments.
- **`actuate2()` uses hardcoded encoder positions.** These are the measured home positions for this specific robot, found by hand. They are not general. If your assembly differs, these numbers will send the legs to the wrong place.
- **ODrive input mode numbers.** The code sets input mode 5 for trajectory control. Note that the surrounding comments occasionally call this TRAP_TRAJ; check the mode against your ODrive firmware version, as the numbering matters and the comment may not match.
