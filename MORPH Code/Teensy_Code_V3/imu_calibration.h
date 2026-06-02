// BNO055 calibration persistence
// save/load offset data to/from EEPROM.
// loadCalibrationFromEEPROM(): call on startup to restore a previous calibration.
// runCalibration():            call once manually to generate and save new offsets.
//                              NOTE: not called automatically
#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <Adafruit_BNO055.h>

// Read sensor ID from EEPROM address 0; if it matches the current sensor, load
// the stored offsets. Returns false if no valid data is found (first run or wrong sensor).
bool loadCalibrationFromEEPROM(Adafruit_BNO055 &bno);

// Blocks until the BNO055 reports full calibration (move/tilt the sensor as prompted).
// Saves sensor ID + offsets to EEPROM so loadCalibrationFromEEPROM() can restore them.
void runCalibration(Adafruit_BNO055 &bno);

#endif
