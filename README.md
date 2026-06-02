# MORPH — Modular Quadrupedal Robot Platform

**FEEG6013 Group Design Project, University of Southampton, 2025/26**

MORPH is an open-source quadrupedal robot built as a research and inspection platform. This repository hosts the full design package for the project: CAD, mechanical and electrical documentation, firmware for both the on-board controller and the wireless controller, the gait-generation GUI, and the supporting test and calibration material.

This year's project did not start from scratch. The mechanical chassis, the leg topology, and the original Teensy gait code were inherited from a previous group's GDP. When we picked it up, the platform was not running. The aim for this iteration was to reactivate it, identify where it was falling short for real research and inspection use, and design, manufacture, and test the upgrades that would make it a credible open-source alternative to the commercial quadrupeds.

The robot now walks reliably on flat ground and slopes up to 12.5 degrees, stays statically stable up to 30 degrees, detects obstacles with LiDAR and stops before hitting them, and is driven from a phone over WiFi. What we did not finish, and what would be sensible next steps, is documented.

## What's new in this iteration

The mechanical core is iterated rather than rebuilt. The electronics, sensing, control, and software stack are mostly new. A short summary of the changes:

**Mechanical**
- Topology-optimised PLA leg structures (40% infill), validated by FEA, replacing the original plywood legs that were responsible for over 5% positional error.
- Custom AS40 silicone feet, cast in-house, with a measured static friction coefficient of around 1.15 against the design target surfaces.
- Adjustable leg-housing clasp with continuous variable tension, removing the slack and fatigue cracking that the original press-fit clasp suffered from.

**Electronics**
- Bespoke soldered protoboard with JST-XH connectors and header pins. Replaces the inherited breadboard and hot-glued wiring that was failing under vibration.
- BNO055 IMU mounted at the centre of mass for pitch and roll sensing.
- RPLidar A1 M8 mounted on top for 360-degree obstacle detection.
- Arduino Nano ESP32 added alongside the Teensy 4.0 for WiFi communication and to run the LiDAR safety stop loop.

**Software and sensing**
- Browser-based wireless controller served by the ESP32. Replaces the broken Bluetooth module on the inherited platform. MAC whitelist for access control.
- IMU sensor-fusion pipeline using a Kalman filter, plus a Periodic Disturbance Cancellation algorithm to remove the gait-induced body oscillation from the orientation signal. Lets the robot tell true terrain slope apart from its own walking wobble.
- Slope-responsive body levelling. The cleaned IMU pitch and roll are used to adjust individual leg heights so the body stays level on an incline.
- LiDAR cluster-detection algorithm that triggers an automatic stop 0.25 m before an obstacle, with debouncing across full sweeps to avoid twitching.
- Refactored, modular Arduino codebase with calibration tooling, structured to be readable by the next group rather than be a single 1500-line `.ino`.


## Hardware at a glance

| Subsystem | Part | Role |
|-----------|------|------|
| Main controller | Teensy 4.0 | Runs the gait, kinematics, IMU fusion, and ODrive comms. |
| Wireless and safety | Arduino Nano ESP32 | WiFi web server, LiDAR polling and obstacle stop. |
| Motor drivers | 6x ODrive (v0.5.x) | Drive 12 BLDC motors over UART. |
| Actuation | 12x BLDC motors with herringbone and worm-wheel gear reduction | Hip, knee, and shoulder for each of four legs. |
| Inertial sensing | Adafruit BNO055 | Body pitch and roll for slope detection. |
| Obstacle sensing | Slamtec RPLidar A1 M8 | 360-degree 2D scan, drives the auto-stop. |
| Operator interface | Any phone, tablet, or laptop with a browser | Connects to the ESP32's hosted web page over local WiFi. |

Targets the platform was designed to and (within limits) achieved: 19 kg total mass, 5 kg additional payload, 0.25 m/s locomotion speed, 60 mm step length, 10% positional uncertainty benchmarked against the ANYmal platform.

## Getting started

The full assembly, calibration, and bring-up procedure is in repository. The short version:

1. Open `CAD/full_assembly.f3z` in Solidworks if you want to review or modify the mechanical design. 
2. Wire the electronics per the wiring diagram. The protoboard is the central distribution point.
3. Calibrate the six ODrives one at a time using their own tools. The firmware tunes gains and limits on top but assumes the motor configuration is already valid.
4. Flash the Teensy firmware and the ESP32 firmware located on the repo. Each has its own README with the dependencies and bring-up steps.
5. Run the IMU calibration routine once on first power-up. Offsets are saved to EEPROM and reloaded automatically afterwards.
6. On the ESP32 serial monitor, note the printed IP address. Open it in a browser on a whitelisted device and the controller page loads.


## Group members (2025/26)

| Name | Primary contribution areas |
|------|----------------------------|
| James Over | Project lead, mechanical design, legs design, ODrive Calibration |
| Antoine Khoury | Mechanical design, Market Analysis, kinematics, ODrive Calibration |
| Nikolas Roussos | Wireless Communication, Software Lead, IMU filtering, Stakeholder Analysis |
| Christos Karayiannis | Mechanical design, foot design and manufacturing, testing |
| Antonin Develay | CAD Lead, Clasp and Leg Design |
| Andreas Savvidis | IMU Lead, lab testing |


## Acknowledgements

- **Dr. Thomas Bull** — primary supervisor, for guidance throughout the project.
- **Prof. Suleiman Sharkh** — co-supervisor, for technical advice on the motor drive and electronics side.
- **Paul Leggett** — for technical support and access to the mechatronics lab.

## Limitations and recommended future work

Some things we did not finish, and a few that we'd genuinely change if doing this again. Worth knowing before you build on the project:

- **Trot gait is not validated.** The trot gait table exists in the firmware and the robot can be made to trot in a controlled setting, but the dynamic stability has not been characterised. The default and tested gait is the crawl.
- **Weight-shift is implemented but disabled.** A separate weight-shift gait table is in `structdef.h` with the gain set to zero. It was never finished or tuned.
- **The Fourier and binned-angle terrain estimators in the IMU code are experimental.** The notch filter is the working production path. The Fourier path is left in for future work and currently uses placeholder coefficients.
- **LiDAR is 2D and forward-mounted.** Anything above or below the LiDAR's scan plane is invisible. A second sensor (or a 3D LiDAR) would close that gap.
- **The mechanical clasp design is improved but not perfect.** Long-duration fatigue testing was not in scope. Inspect periodically.
- **Outdoor LiDAR exposure.** The RPLidar A1 is vulnerable to direct sunlight. A simple optical shield is sketched in CAD but not manufactured.
- **Autonomy is limited to obstacle stopping.** There is no path planning or SLAM. The LiDAR data and ESP32 are capable of running this, and the framework is in place to extend in that direction.

If you take this further, the firmware READMEs are the right place to start.

## License

Open-source for academic and non-commercial use. 
