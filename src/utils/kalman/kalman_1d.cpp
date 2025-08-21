#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <random>  // Include for random number generation
#include "../lib/Eigen/Geometry"
#include "../lib/Eigen/Dense"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "csv_utils.h"
#include "parameters_ekf.h"

//#define TINYEKF_IMPLEMENTATION
#include "../lib/TinyEKF-master/src/tinyekf.h"

#include "math_utils.h"

static void initialize_ekf(ekf_t *ekf) {
    // Position
    ekf -> x[0] = 0;

    // Velocity
    ekf -> x[1] = 0;

    // Quaternion: received from calibration phase
    ekf -> x[2] = 0.25871;
    ekf -> x[3] = 0.011783;
    ekf -> x[4] = -0.044004;
    ekf -> x[5] = 0.96488;

}

void run_model(ekf_t * ekf, float dt, float fx[EKF_N], float hx[EKF_M], float omega_x, float omega_y, float omega_z, float accel_z[3], float h_pressure_sensor, float z_gps, int save) {

    // Update quaternion; omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen(omega_x - bias_g[0], omega_y - bias_g[1], omega_z - bias_g[2]);
    float theta = omega_eigen.norm() * dt;
    Eigen::Vector3f axis = omega_eigen.normalized();
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)
    Eigen::Quaternionf q_nominal(ekf->x[2], ekf->x[3], ekf->x[4], ekf->x[5]);
    Eigen::Quaternionf q_rot =  q_nominal*delta_q;
    Eigen::Vector3f omega_abs = q_rot * omega_eigen; // Rotate angular velocity to body frame
    q_rot.normalize();

    Eigen::Vector3f acc_body(accel_z[0], accel_z[1], accel_z[2]);
    Eigen::Vector3f accel_abs = (q_rot * (acc_body - bias_a)) + gravity;  // Equivalent to q * a * q.inverse()
 
    // Velocities
    fx[1] = (float)(ekf->x[1] + accel_abs[2]*dt);
    
    // Position
    fx[0] = (float)(ekf->x[0] + ekf->x[1]*dt + 0.5f*accel_abs[2]*dt*dt);

    // Quaternion
    fx[2] = q_rot.w();
    fx[3] = q_rot.x();
    fx[4] = q_rot.y();
    fx[5] = q_rot.z();

    hx[0] = acc_body[0];
    hx[1] = acc_body[1];
    hx[2] = acc_body[2];
    hx[3] = omega_x;
    hx[4] = omega_y;
    hx[5] = omega_z;
    hx[6] = fx[0];
    hx[7] = fx[0];

    if (save == 1) {
        // Save acc_abs to csv
        saveToCSV({accel_abs[0], accel_abs[1], accel_abs[2]}, "../csv_measurements/Sensor Simulation/EKF Results/accel_abs_EKF.csv");
        saveToCSV({omega_abs[0], omega_abs[1], omega_abs[2]}, "../csv_measurements/Sensor Simulation/EKF Results/omega_abs.csv");
    }
}


int main() {
    initializeCSVFiles();

    int lineNum = 1;
    int totalLines = countLinesInFile("../csv_measurements/Sensor Simulation/Sensors/acc_sensor.csv");

    ekf_t ekf = {0};
    const float pdiag[EKF_N] = {P0, V0, q_a, q_a, q_a, q_a};

    ekf_initialize(&ekf, pdiag);
    initialize_ekf(&ekf);

    float accX;
    float accY;
    float accZ;
    float omega_x;
    float omega_y;
    float omega_z;
    float h_pressure_sensor;
    float z_gps; // GPS altitude, can be adjusted if needed
    float z_gps_updated;
    float gps_prev_val;

    while (lineNum <= totalLines) {
        std::vector<float> acc = getAcc(lineNum);           // Get the accelerometer data
        std::vector<float> omega = getOmega(lineNum);       // Get the Angular velocity
        std::vector<float> pressure_sensor_vec = getPressure(lineNum);    // Get the pressure sensor data
        std::vector<float> gps = getGPS(lineNum); // Get the GPS data

        accX = acc[0];
        accY = acc[1];
        accZ = acc[2];
        
        Eigen::Vector3f accel_z(accX, accY, accZ);
        omega_x = omega[0]*M_PI/180;
        omega_y = omega[1]*M_PI/180;
        omega_z = omega[2]*M_PI/180;
        h_pressure_sensor = pressureToAltitude(pressure_sensor_vec[0]) - h_bias_pressure_sensor - SeaLevel;
        z_gps = gps[0] - SeaLevel - gps_bias;

        if (z_gps == gps_prev_val){
            z_gps_updated = z_gps_updated + ekf.x[1]*dt; // If the GPS value is the same as the previous one, add a delay
        }  
        else {
            z_gps_updated = z_gps;
        }
        gps_prev_val = z_gps;
        z_gps = z_gps_updated;

        float fx[EKF_N] = {0};
        float hx[EKF_M] = {0};

        computeFullHJacobian(
            Eigen::Quaternionf(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]),
            Eigen::Vector3f(accX, accY, accZ),
            Eigen::Vector3f(omega_x, omega_y, omega_z),
            H
        );

        R[EKF_M*EKF_M - 1] = estimateBaroVar(ekf.x[1]); // Update the last element of R with the barometer variance
 
        computeJacobianF_tinyEKF(&ekf, dt, omega_x, omega_y, omega_z, accel_z.data(), F, h_pressure_sensor, z_gps);

        // Set the observation vector z
        float z[EKF_M] = {accX, accY, accZ, omega_x, omega_y, omega_z, h_pressure_sensor, z_gps};

        run_model(&ekf, dt, fx, hx, omega_x, omega_y, omega_z, acc.data(), h_pressure_sensor, z_gps, 1);
        
        ekf_predict(&ekf, fx, F, Q);

        ekf_update(&ekf, z, hx, H, R);

        Eigen::Quaternionf q(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);        

        std::vector<float> Altitude_EKF = {ekf.x[0]};
        std::vector<float> Vel_Z_EKF = {ekf.x[1]};
        std::vector<float> Quat_EKF = {ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]};
        saveToCSV(Altitude_EKF, "../csv_measurements/Sensor Simulation/EKF Results/altitude_EKF.csv");
        saveToCSV(Vel_Z_EKF, "../csv_measurements/Sensor Simulation/EKF Results/vel_Z_EKF.csv");
        saveToCSV(Quat_EKF, "../csv_measurements/Sensor Simulation/EKF Results/quat_EKF.csv");

        lineNum++;
    }

    std::cout << "EKF completed for " << totalLines << " lines." << std::endl;
    

    return 0;
}