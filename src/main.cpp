#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <optional>
#include <vector>
#include <string>
#include "global/config.h"
#include "utils/utilities/functions.h"
#include "utils/logger/ILogger.hpp"
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include "sensors/ISensor.hpp"
#include "sensors/BME680/BME680Sensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"
#include "sensors/MPRLS/MPRLSSensor.hpp"
#include "telemetry/LoRa/E220LoRaTransmitter.hpp"

#define I2C_MULTIPLEXER_ADDRESS 0x70
#define I2C_MULTIPLEXER_BUS_NUMBER 0 // 0 = SD0 and SC0, 1 = SD1 and SC1, 2 = SD2 and SC2 ...

ILogger *rocketLogger;
// ISensor *bme680;
ISensor *bno055;
// ISensor *mprls_1;
// ISensor *mprls_2;
// ITransmitter *loraTransmitter;
// HardwareSerial loraSerial(LORA_SERIAL);

// Struct to store sensor information for initialization and logging
struct SensorInfo
{
    ISensor *sensor;            // Pointer to the sensor object
    std::string name;           // Name of the sensor
    std::optional<int> address; // I2C address of the sensor (if applicable)
};

// Vector of sensors to initialize (add the used sensors here)
std::vector<SensorInfo> sensors = {
    // {mprls_1, "MPRLS_1", MPRLS_I2C_ADDR},
    // {mprls_2, "MPRLS_2", MPRLS_I2C_ADDR},
    {bno055, "BNO055", BNO055_I2C_ADDR}};

void logInitializationResult(const std::string &sensorName, const std::optional<int> &address, bool success);
bool initSensor(ISensor *sensor, const std::string &name, const std::optional<int> &address);
void initAllSensorsAndLogStatus();
void selectDevice(uint8_t bus);

void setup()
{
    rocketLogger = new RocketLogger();
    rocketLogger->logInfo("Setup started.");
    Serial.begin(SERIAL_BAUD_RATE);
    //! TODO: Delete after testing phase is over.
    delay(500);
    // mprls_1 = new MPRLSSensor();
    // mprls_2 = new MPRLSSensor();
    bno055 = new BNO055Sensor();

    initAllSensorsAndLogStatus();

    rocketLogger->logInfo("Setup complete.");

    //! TODO: Delete after testing phase is over.
    delay(2000);
    Serial.write(rocketLogger->getJSONAll().dump(4).c_str());
}

void loop()
{
    // Read data from all sensors inside the sensors vector and log it.
    for (const auto &[sensor, name, address] : sensors)
    {
        // selectDevice(n) seleziona il device collegato a SDn e SCn nel multiplexer
        if (name == "MPRLS_2")
        {
            selectDevice(0);
        }
        auto data = sensor->getData();
        if (data.has_value())
        {
            rocketLogger->logSensorData(data.value());
        }
    }

    Serial.println("######################################");
    Serial.write((rocketLogger->getJSONAll().dump(4) + "\n").c_str());
    Serial.println("######################################");
    //! TODO: Delete after testing phase is over.
    delay(500);
    rocketLogger->clearData();
}

// Log transmitter initialization status
// void logTransmitterStatus(ResponseStatusContainer &transmitterStatus)
// {
//     if (transmitterStatus.getCode() == RESPONSE_STATUS::E220_SUCCESS)
//     {
//         rocketLogger->logInfo(
//             ("LoRa transmitter initialized with configuration: " +
//              static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
//                 .c_str());
//     }
//     else
//     {
//         rocketLogger->logError(
//             ("Failed to initialize LoRa transmitter with error: " +
//              transmitterStatus.getDescription() +
//              " (" + String(transmitterStatus.getCode()) + ")")
//                 .c_str());
//         rocketLogger->logInfo(("Current configuration: " +
//                                static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
//                                   .c_str());
//     }
// }

// Log data transmission response
// void logTransmissionResponse(ResponseStatusContainer &response)
// {
//     response.getCode() != RESPONSE_STATUS::E220_SUCCESS
//         ? rocketLogger->logError(("Failed to transmit data with error: " + response.getDescription() + " (" + String(response.getCode()) + ")").c_str())
//         : rocketLogger->logInfo("Data transmitted successfully.");
// }

// Log a sensor initialization status
void logInitializationResult(const std::string &sensorName, const std::optional<int> &address, bool success)
{
    std::string addressInfo = 
    address.has_value() ? " on address " + std::to_string(address.value())
                        : "";

    success ? rocketLogger->logInfo(sensorName + " sensor initialized" + addressInfo)
            : rocketLogger->logError("Failed to initialize " + sensorName + " sensor" + addressInfo);
}

// Initialize a sensor
bool initSensor(ISensor *sensor, const std::string &name, const std::optional<int> &address)
{
    bool initSuccess = sensor->init();
    logInitializationResult(name, address, initSuccess);
    return initSuccess;
}

// Initialize the sensors inside the sensors vector and log the initialization status
void initAllSensorsAndLogStatus()
{
    for (const auto &[sensor, name, address] : sensors)
    {
        initSensor(sensor, name, address);
    }
}

void selectDevice(uint8_t bus)
{
    Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}
