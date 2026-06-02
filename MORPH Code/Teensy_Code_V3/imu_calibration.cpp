#include "imu_calibration.h"
#include <EEPROM.h>

bool loadCalibrationFromEEPROM(Adafruit_BNO055 &bno) {
    int eeAddress = 0;
    long storedID;
    EEPROM.get(eeAddress, storedID);

    sensor_t sensor;
    bno.getSensor(&sensor);

    if (storedID != sensor.sensor_id) {
        return false;   // No valid calibration stored
    }

    eeAddress += sizeof(long);
    adafruit_bno055_offsets_t calib;
    EEPROM.get(eeAddress, calib);
    bno.setSensorOffsets(calib);
    return true;
}

void runCalibration(Adafruit_BNO055 &bno) {
    bool found = loadCalibrationFromEEPROM(bno);

    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);

    while (!bno.isFullyCalibrated()) {
        bno.getEvent(&event);
        delay(100);
    }

    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);

    int eeAddress = 0;
    sensor_t sensor;
    bno.getSensor(&sensor);
    long id = sensor.sensor_id;

    EEPROM.put(eeAddress, id);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
}
