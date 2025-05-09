#ifndef BNO055_SENSOR_PRINTER_HPP
#define BNO055_SENSOR_PRINTER_HPP

#include <Arduino.h>
#include <Adafruit_BNO055.h>

/**
 * @class BNO055Printer
 * @brief Utility class to print formatted sensor data from an Adafruit BNO055 IMU.
 *
 * This class provides methods for printing various sensor readings and metadata
 * from a BNO055 inertial measurement unit using the Adafruit_BNO055 library.
 *
 * The class does not own the sensor instance; it keeps a reference to an
 * existing Adafruit_BNO055 object, allowing the caller to manage sensor
 * initialization and configuration.
 * 
 * @author eric.aquilotti@aurorarocketry.eu
 */
class BNO055Printer {
    public:
        BNO055Printer(Adafruit_BNO055& sensor);
        void displayCalibrationStatus();
        void displaySensorDetails();
        void displayOrientation();
        void displayAccelleration();
        void displayGyroscope();
        void displayMagnetometer();

    private:
        Adafruit_BNO055& bno055;
};

#endif
