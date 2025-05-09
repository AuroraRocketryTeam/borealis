#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "BNO055/BNO055Printer.hpp"

// Pin I2C for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
BNO055Printer bnoPrinter(bno);

void setup(void) {
    Serial.begin(115200);
    while (!Serial) {
      ; // Wait for the serial port to connect.
    }
    
    Serial.println(F("Test BNO055 sensor con ESP32"));
    
    // I2C initialization with defined pins.
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Sensor initialization.
    if (!bno.begin()) {
      Serial.println(F("Cannot detect the BNO055 sensor. Check the conditions of the I2C address."));
      while (1);
    }
    
    delay(1000);
    bno.setExtCrystalUse(true);

    bnoPrinter.displaySensorDetails();
}
  
void loop(void) {
    bnoPrinter.displayCalibrationStatus();

    bnoPrinter.displayOrientation();

    bnoPrinter.displayAccelleration();

    bnoPrinter.displayGyroscope();

    bnoPrinter.displayMagnetometer();

    delay(1000);
}
