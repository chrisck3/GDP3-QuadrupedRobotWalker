# ODrive Calibration

Using the Python ODrive library ([odrivetool](https://docs.odriverobotics.com/v/0.5.6/getting-started.html))

> **Recommended:** Calibrate each driver individually on a PSU.

---

## Useful Commands

### Factory Reset

```python
odrv0.erase_configuration()
```

### Check Bus Voltage

```python
odrv0.vbus_voltage
```

### Error Handling

```python
dump_errors(odrv0)
```

```python
odrv0.clear_errors()
```

### Check Axis State

> `0` = No state, `1` = Idle, `8` = Closed loop

```python
odrv0.axis0.current_state
```

### Check Input Mode

> `5` = Trapezoidal trajectory

```python
odrv0.axis0.controller.config.input_mode
```

### Backup & Restore Configuration

```python
# Save config to file
odrivetool backup-config my_config.json
```

```python
# Restore config from file
odrivetool restore-config my_config.json
```

---

## Motor Calibration

Both a **configuration** and **anti-cogging** calibration should be saved to the ODrive for a reliable and accurate setup.

---

### Turnigy 100 kV — Configuration

#### 1. Current & Velocity Limits

```python
odrv0.axis0.motor.config.current_lim = 40
odrv0.axis0.controller.config.vel_limit = 5

odrv0.axis1.motor.config.current_lim = 40
odrv0.axis1.controller.config.vel_limit = 5
```

#### 2. Brake Resistor

> Only required for firmware **0.5.5** and **0.5.6** — remove this line for other versions.

```python
odrv0.config.enable_brake_resistor = True
```

```python
odrv0.config.brake_resistance = 2
odrv0.config.dc_bus_overvoltage_trip_level = 26
```

#### 3. Motor Configuration

```python
odrv0.axis0.motor.config.motor_type = 0
odrv0.axis0.motor.config.pole_pairs = 20
odrv0.axis0.motor.config.torque_constant = 8.27 / 100
odrv0.axis0.motor.config.calibration_current = 20

odrv0.axis1.motor.config.motor_type = 0
odrv0.axis1.motor.config.pole_pairs = 20
odrv0.axis1.motor.config.torque_constant = 8.27 / 100
odrv0.axis1.motor.config.calibration_current = 20
```

#### 4. Encoder Configuration (AS5047D — SPI Mode)

> Set the correct CS GPIO pin for your wiring.

```python
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 8
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis0.encoder.config.cpr = 2**14

odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 7
odrv0.axis1.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis1.encoder.config.cpr = 2**14
```

#### 5. Save & Reboot

```python
odrv0.save_configuration()
odrv0.reboot()
```

> **Note:** A full power cycle may be required if encoder positional errors appear.

#### 6. Encoder Offset Calibration (SPI Mode)

> Nothing should physically happen during this step.

```python
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
```

#### 7. Full Calibration Sequence

> The motor should beep and rotate in both directions.

```python
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```

#### 8. Flag as Pre-Calibrated

> This allows the ODrive to skip full calibration on the next boot and go straight to closed loop.

```python
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True

odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
```

#### 9. PNI Control Gains

```python
odrv0.axis0.controller.config.pos_gain = 100
odrv0.axis0.controller.config.vel_gain = 0.3
odrv0.axis0.controller.config.vel_integrator_gain = 0.4

odrv0.axis1.controller.config.pos_gain = 100
odrv0.axis1.controller.config.vel_gain = 0.3
odrv0.axis1.controller.config.vel_integrator_gain = 0.4
```

#### 10. Trajectory Control Limits

```python
odrv0.axis0.trap_traj.config.vel_limit = 5.0
odrv0.axis0.trap_traj.config.accel_limit = 5.0
odrv0.axis0.trap_traj.config.decel_limit = 5.0

odrv0.axis1.trap_traj.config.vel_limit = 5.0
odrv0.axis1.trap_traj.config.accel_limit = 5.0
odrv0.axis1.trap_traj.config.decel_limit = 5.0
```

#### 11. Save & Reboot

```python
odrv0.save_configuration()
odrv0.reboot()
```

> Motors are now ready to use without further calibration.

---

### Surpass C5065 435 kV — Configuration

#### 1. Current & Velocity Limits

```python
odrv0.axis0.motor.config.current_lim = 40
odrv0.axis0.controller.config.vel_limit = 5

odrv0.axis1.motor.config.current_lim = 40
odrv0.axis1.controller.config.vel_limit = 5
```

#### 2. Brake Resistor

> Only required for firmware **0.5.5** and **0.5.6** — remove this line for other versions.

```python
odrv0.config.enable_brake_resistor = True
```

```python
odrv0.config.brake_resistance = 2
odrv0.config.dc_bus_overvoltage_trip_level = 25
```

#### 3. Motor Configuration

```python
odrv0.axis0.motor.config.motor_type = 0
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.torque_constant = 8.27 / 435
odrv0.axis0.motor.config.calibration_current = 20

odrv0.axis1.motor.config.motor_type = 0
odrv0.axis1.motor.config.pole_pairs = 7
odrv0.axis1.motor.config.torque_constant = 8.27 / 435
odrv0.axis1.motor.config.calibration_current = 20
```

#### 4. Encoder Configuration (AS5047D — SPI Mode)

> Set the correct CS GPIO pin for your wiring.

```python
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 8
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis0.encoder.config.cpr = 2**14

odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 7
odrv0.axis1.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis1.encoder.config.cpr = 2**14
```

#### 5. Save & Reboot

```python
odrv0.save_configuration()
odrv0.reboot()
```

> **Note:** A full power cycle may be required if encoder positional errors appear.

#### 6. Encoder Offset Calibration (SPI Mode)

> Nothing should physically happen during this step.

```python
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
```

#### 7. Full Calibration Sequence

> The motor should beep and rotate in both directions.

```python
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```

#### 8. Flag as Pre-Calibrated

```python
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True

odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
```

#### 9. PNI Control Gains

```python
odrv0.axis0.controller.config.pos_gain = 40
odrv0.axis0.controller.config.vel_gain = 0.125
odrv0.axis0.controller.config.vel_integrator_gain = 0.2

odrv0.axis1.controller.config.pos_gain = 40
odrv0.axis1.controller.config.vel_gain = 0.125
odrv0.axis1.controller.config.vel_integrator_gain = 0.2
```

#### 10. Trajectory Control Limits

```python
odrv0.axis0.trap_traj.config.vel_limit = 5.0
odrv0.axis0.trap_traj.config.accel_limit = 5.0
odrv0.axis0.trap_traj.config.decel_limit = 5.0

odrv0.axis1.trap_traj.config.vel_limit = 5.0
odrv0.axis1.trap_traj.config.accel_limit = 5.0
odrv0.axis1.trap_traj.config.decel_limit = 5.0
```

#### 11. Save & Reboot

```python
odrv0.save_configuration()
odrv0.reboot()
```

> Motors are now ready to use without further calibration.

---

## Anti-Cogging Calibration

> Anti-cogging takes approximately **20 minutes** by default (3600 steps). Run for both axes.

#### 1. Start Anti-Cogging Calibration

```python
# Axis 0
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.anticogging.pre_calibrated = False
odrv0.axis0.controller.start_anticogging_calibration()

# Axis 1
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.controller.config.anticogging.pre_calibrated = False
odrv0.axis1.controller.start_anticogging_calibration()
```

#### 2. Monitor Progress

> Poll these flags to track calibration status.
>
> **`calib_anticogging`**: `True` = calibration running, `False` = finished (or not started)  
> **`anticogging_valid`**: `False` = no valid map yet, `True` = map complete and valid

```python
odrv0.axis0.controller.config.anticogging.calib_anticogging
odrv0.axis0.controller.anticogging_valid
```

```python
odrv0.axis1.controller.config.anticogging.calib_anticogging
odrv0.axis1.controller.anticogging_valid
```

> Check the current step index — if it's increasing, calibration is actively running.
> If it returns `0`, calibration may have finished or an error occurred — run `dump_errors` to check.

```python
odrv0.axis0.controller.config.anticogging.index
```

```python
odrv0.axis1.controller.config.anticogging.index
```

#### 3. Finalise Anti-Cogging

> Run this once `anticogging_valid == True` or `calib_anticogging == False`.

```python
# Axis 0
odrv0.axis0.controller.config.anticogging.pre_calibrated = True
odrv0.axis0.controller.config.anticogging.anticogging_enabled = True
odrv0.axis0.requested_state = AXIS_STATE_IDLE

# Axis 1
odrv0.axis1.controller.config.anticogging.anticogging_enabled = True
odrv0.axis1.controller.config.anticogging.pre_calibrated = True
odrv0.axis1.requested_state = AXIS_STATE_IDLE
```

> `save_configuration()` may return `False` if the axis is not in IDLE — ensure both axes are idle before saving.

```python
odrv0.save_configuration()
odrv0.reboot()
```

---

## Running the Motors

#### 1. Verify Calibration (Optional)

```python
odrv0.axis0.motor.is_calibrated    # Expected: True
odrv0.axis0.encoder.is_ready       # Expected: True
```

```python
odrv0.axis1.motor.is_calibrated    # Expected: True
odrv0.axis1.encoder.is_ready       # Expected: True
```

#### 2. Check Absolute Position

> Used to find the absolute position and place legs correctly on start-up.

```python
odrv0.axis0.encoder.pos_estimate
```

```python
odrv0.axis1.encoder.pos_estimate
```

#### 3. Enter Closed Loop & Trapezoidal Trajectory Mode

```python
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
```

#### 4. Set Target Position

> These are **absolute positions**, not relative movements.

```python
odrv0.axis0.controller.input_pos = 1
odrv0.axis1.controller.input_pos = 1
```

```python
# Return to zero
odrv0.axis0.controller.input_pos = 0
odrv0.axis1.controller.input_pos = 0
```
