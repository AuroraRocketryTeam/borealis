// filepath: src/sensors/MPRLS/test_MPRLSSensor.cpp
#include <gtest/gtest.h>
#include "MPRLSSensor.hpp"
#include <optional>

// Mock class for Adafruit_MPRLS
class MockAdafruit_MPRLS {
public:
    bool begin() { return true; }
    float readPressure() { return 1013.25; }
};

// Test fixture for MPRLSSensor
class MPRLSSensorTest : public ::testing::Test {
protected:
    MPRLSSensor* sensor;
    MockAdafruit_MPRLS* mockMPRLS;

    void SetUp() override {
        mockMPRLS = new MockAdafruit_MPRLS();
        sensor = new MPRLSSensor();
        sensor->mprls = *mockMPRLS;
    }

    void TearDown() override {
        delete sensor;
        delete mockMPRLS;
    }
};

TEST_F(MPRLSSensorTest, InitializationSuccess) {
    EXPECT_TRUE(sensor->init());
}

TEST_F(MPRLSSensorTest, InitializationFailure) {
    mockMPRLS->begin = []() { return false; };
    EXPECT_FALSE(sensor->init());
}

TEST_F(MPRLSSensorTest, GetDataSuccess) {
    sensor->init();
    auto data = sensor->getData();
    ASSERT_TRUE(data.has_value());
    EXPECT_EQ(data->getData("pressure"), 1013.25);
}

TEST_F(MPRLSSensorTest, GetDataFailure) {
    sensor->init();
    sensor->setInitialized(false);
    auto data = sensor->getData();
    EXPECT_FALSE(data.has_value());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}