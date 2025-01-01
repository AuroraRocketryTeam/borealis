#include <gtest/gtest.h>
#include "BNO055Sensor.hpp"
#include <optional>

// Mock class for Adafruit_BNO055
class MockAdafruit_BNO055 {
public:
    bool begin() { return true; }
    void getEvent(sensors_event_t* event, Adafruit_BNO055::adafruit_vector_type_t type) {
        event->orientation.x = 0.0;
        event->orientation.y = 0.0;
        event->orientation.z = 0.0;
        event->gyro.x = 0.0;
        event->gyro.y = 0.0;
        event->gyro.z = 0.0;
        event->acceleration.x = 0.0;
        event->acceleration.y = 0.0;
        event->acceleration.z = 0.0;
        event->magnetic.x = 0.0;
        event->magnetic.y = 0.0;
        event->magnetic.z = 0.0;
    }
    imu::Quaternion getQuat() {
        return imu::Quaternion(1.0, 0.0, 0.0, 0.0);
    }
    void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
        *sys = 3;
        *gyro = 3;
        *accel = 3;
        *mag = 3;
    }
    int8_t getTemp() { return 25; }
};

// Test fixture for BNO055Sensor
class BNO055SensorTest : public ::testing::Test {
protected:
    BNO055Sensor* sensor;
    MockAdafruit_BNO055* mockBNO;

    void SetUp() override {
        mockBNO = new MockAdafruit_BNO055();
        sensor = new BNO055Sensor();
        sensor->bno055 = *mockBNO;
    }

    void TearDown() override {
        delete sensor;
        delete mockBNO;
    }
};

TEST_F(BNO055SensorTest, InitializationSuccess) {
    EXPECT_TRUE(sensor->init());
}

TEST_F(BNO055SensorTest, InitializationFailure) {
    mockBNO->begin = []() { return false; };
    EXPECT_FALSE(sensor->init());
}

TEST_F(BNO055SensorTest, GetDataSuccess) {
    sensor->init();
    auto data = sensor->getData();
    ASSERT_TRUE(data.has_value());
    EXPECT_EQ(data->getData("system_calibration"), 3);
    EXPECT_EQ(data->getData("gyro_calibration"), 3);
    EXPECT_EQ(data->getData("accel_calibration"), 3);
    EXPECT_EQ(data->getData("mag_calibration"), 3);
    EXPECT_EQ(data->getData("board_temperature"), 25);
}

TEST_F(BNO055SensorTest, GetDataFailure) {
    sensor->init();
    sensor->setInitialized(false);
    auto data = sensor->getData();
    EXPECT_FALSE(data.has_value());
}

TEST_F(BNO055SensorTest, CalibrationSuccess) {
    sensor->init();
    EXPECT_TRUE(sensor->calibrate());
}

TEST_F(BNO055SensorTest, CalibrationFailure) {
    sensor->setInitialized(false);
    EXPECT_FALSE(sensor->calibrate());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}