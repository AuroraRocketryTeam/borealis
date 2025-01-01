#include <gtest/gtest.h>
#include "BME680Sensor.hpp"
#include <optional>

// Mock class for Adafruit_BME680
class MockAdafruit_BME680 {
public:
    bool begin(uint8_t addr) { return true; }
    bool performReading() { return true; }
    float temperature = 25.0;
    float pressure = 1013.25;
    float humidity = 50.0;
    float gas_resistance = 1000.0;
    void setGasHeater(uint16_t heaterTemp, uint16_t heaterTime) {}
    void setTemperatureOversampling(uint8_t os) {}
    void setHumidityOversampling(uint8_t os) {}
    void setPressureOversampling(uint8_t os) {}
};

// Test fixture for BME680Sensor
class BME680SensorTest : public ::testing::Test {
protected:
    BME680Sensor* sensor;
    MockAdafruit_BME680* mockBME;

    void SetUp() override {
        mockBME = new MockAdafruit_BME680();
        sensor = new BME680Sensor(0x76);
        sensor->bme = *mockBME;
    }

    void TearDown() override {
        delete sensor;
        delete mockBME;
    }
};

TEST_F(BME680SensorTest, InitializationSuccess) {
    EXPECT_TRUE(sensor->init());
}

TEST_F(BME680SensorTest, InitializationFailure) {
    mockBME->begin = [](uint8_t addr) { return false; };
    EXPECT_FALSE(sensor->init());
}

TEST_F(BME680SensorTest, GetDataSuccess) {
    sensor->init();
    auto data = sensor->getData();
    ASSERT_TRUE(data.has_value());
    EXPECT_EQ(data->getData("temperature"), 25.0);
    EXPECT_EQ(data->getData("pressure"), 1013.25);
    EXPECT_EQ(data->getData("humidity"), 50.0);
    EXPECT_EQ(data->getData("gas_resistance"), 1000.0);
}

TEST_F(BME680SensorTest, GetDataFailure) {
    mockBME->performReading = []() { return false; };
    sensor->init();
    auto data = sensor->getData();
    EXPECT_FALSE(data.has_value());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}