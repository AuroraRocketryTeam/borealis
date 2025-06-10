#ifndef KALMANFILTER1D_HPP
#define KALMANFILTER1D_HPP

// Macros needed for a conflict between similar macro variables names of Arduino.h and Eigen.h
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif
#ifdef B0
#undef B0
#endif

#define EKF_N 6 // Size of state space [3-positions, 3-velocities, 3-accelerations, 4-quaternion_rot] 
#define EKF_M 7 // Size of observation (measurement) space [3-positions, 3-accelerations, 4-quaternion_rot]

#include <tinyekf.h>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <random>
#include <vector>
#include <ArduinoEigen.h>
#include "esp_task_wdt.h"

class KalmanFilter1D {
public:
    KalmanFilter1D(Eigen::Vector3f gravity_value, Eigen::Vector3f magnometer_value);
    std::vector<std::vector<float>> step(float dt, float omega[3], float accel[3], float pressure);
    float* state();

private:
    ekf_t ekf;

    // initial covariances of state noise, measurement noise
    // Q matrix (model) // We obtain this value from a comparison between a model and the real data.
    const float P0 = 1e-4; 
    const float V0 = 1e-4;
    const float q_a = 1e-8;
    const float b_a = 1e-8;
    const float b_g = 1e-8;

    // Bias of the sensors (initialized in the constructor functino with calibration data)
    Eigen::Vector3f bias_a; // Bias vector for accelerometer
    Eigen::Vector3f bias_g; // Bias vector for gyroscope

    // R matrix (measurements)
    const float A0 = 1e-3;
    const float G0 = 1e-6;
    const float Z0 = 1;
    const float R0 = 1; // TODO: This is a placeholder for barometer variance, should be estimated based on barometer readings.

    // TODO Estimate barometer variance:!!!
    float estimateBaroVar(float v) {
        float std = (std::abs(v) / 300.0f) * 29.0f + 1.0f;
        return std * std;
    }

    // Process noise covariance
    float Q_diag[EKF_N] = {
        P0,
        V0,
        q_a, 
        q_a, 
        q_a, 
        q_a
    };

    // Measurement noise covariance
    float R[EKF_M*EKF_M] = {
        A0, 0, 0, 0, 0, 0, 0,
        0, A0, 0, 0, 0, 0, 0,
        0, 0, A0, 0, 0, 0, 0,
        0, 0, 0, G0, 0, 0, 0,
        0, 0, 0, 0, G0, 0, 0,
        0, 0, 0, 0, 0, G0, 0,
        0, 0, 0, 0, 0, 0, R0
    };

    // Initially, the acceleration is constantly zero, so it won't change
    float H[EKF_M*EKF_N] = {
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0,
    };
    
    // Measurement Jacobian with input/output relations
    float F[EKF_N*EKF_N];

    // Gravity vector in ENU coordinates
    const Eigen::Vector3f gravity{0, 0, -9.81};

    /**
     * @brief This function must run when the Launcher is still in the launching position.
     * It will also use the magnetometer to align the Z axis with the North, we might not get exact Norht since the readings might be modified by the presence of the aluminum frame, it is just to get a rough idea of the North.
     * 
     * @param gravity_readings A vector of gravity reading samples (a good amount is 200)
     * @return std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f>: the good approximations of quaternions, ???TODO
     */
    std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> calibration(Eigen::Vector3f gravity_value, Eigen::Vector3f magnetometer_value);

    Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world);

    // Compute H_q^{(a)} numerically
    Eigen::Matrix<float, 3, 4> computeHqAccelJacobian(const float dt, const Eigen::Quaternionf& q_nominal, const Eigen::Vector3f& accel_world, const Eigen::Vector3f& omega, float epsilon = 1e-5);

    void run_model(ekf_t * ekf, float dt, float fx[EKF_N], float hx[EKF_M], float omega_x, float omega_y, float omega_z, float accel_z[3], float pressure);

    void computeJacobianF_tinyEKF(ekf_t* ekf, float dt, float omega_x, float omega_y, float omega_z, float accel_z[3], float F_out[EKF_N * EKF_N], float h_pressure_sensor);

    void quaternionToEulerAngles(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw);
};

#endif // KALMANFILTER1D_HPP