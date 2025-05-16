#include <Arduino.h>
#include <Wire.h>
#include "BNO055/BNO055Printer.hpp"
#include "sensors/ISensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"

// Pin I2C for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

ISensor* bno;
BNO055Printer* bnoPrinter;

void setup(void) {
    Serial.begin(115200);
    while (!Serial) {
      ; // Wait for the serial port to connect.
    }
    
    Serial.println(F("Test BNO055 sensor con ESP32"));
    // I2C initialization with defined pins.
    Wire.begin(I2C_SDA, I2C_SCL);
    
    bno = new BNO055Sensor();
    bno->init();
    bnoPrinter = new BNO055Printer(bno);
    delay(1000);
}
  
void loop(void) {
    bnoPrinter->displayCalibrationStatus();

    bnoPrinter->displayOrientation();

    bnoPrinter->displayAccelleration();

    bnoPrinter->displayGyroscope();

    bnoPrinter->displayMagnetometer();

    delay(1000);
}
