#ifndef CEKF_H
#define CEKF_H

#include "cmatrix.h"
#include "cquaternion.h"
#include <stdint.h>
#include <math.h>

#define		MAG_UPDATE			0
#define		ACCEL_UPDATE		1

typedef struct {
     int16_t gyro_x;
     int16_t gyro_y;
     int16_t gyro_z;

     int16_t new_gyro_data;

     int16_t accel_x;
     int16_t accel_y;
     int16_t accel_z;

     // Flag specifies whether there is new accel data in the sensor data structure
     int16_t new_accel_data;

     int16_t mag_x;
     int16_t mag_y;
     int16_t mag_z;

     // Flag specifies whether there is new magnetometer data in the sensor data structure
     int16_t new_mag_data;

      // Rate gyro temperature measurement
      int16_t temperature;
} tRawSensorData;


typedef struct sStateData {

     // Orientation states
     union {
          float heading;
          float yaw;
          float psi;
     };
     union {
          float pitch;
          float theta;
     };
     union {
          float roll;
          float phi;
     };

     // Orientation rate states
     union {
          float heading_rate;
          float yaw_rate;
          float psi_dot;
     };

     union {
          float pitch_rate;
          float theta_dot;
     };

     union {
          float roll_rate;
          float phi_dot;
     };

     // Quaternion states "qib" = Quaternion from Inertial to Body
     cQuaternion qib;

     // Gyro biases
     int16_t beta_p,beta_q,beta_r;

      // Gyro temperature compensation terms
      float beta_p0, beta_p1, beta_p2, beta_p3;
      float beta_q0, beta_q1, beta_q2, beta_q3;
      float beta_r0, beta_r1, beta_r2, beta_r3;

     // Accelerometer biases
     int16_t beta_acc_x, beta_acc_y, beta_acc_z;

     // Magnetometer biases
     int16_t beta_mag_x, beta_mag_y, beta_mag_z;

     // Process noise matrix
     cMatrix<4,4> R;

     // Accelerometer alignment matrix
     cMatrix<3,3> accel_cal;

     // Gyro alignment matrix
     cMatrix<3,3> gyro_cal;

     // Magnetometer calibration matrix
     cMatrix<3,3> mag_cal;

     // EKF covariance
     cMatrix<4,4> Sigma;

     // Magnetic field reference vector
     float mag_ref_x;
     float mag_ref_y;
     float mag_ref_z;

     // Accelerometer	reference vector
     float accel_ref_x;
     float accel_ref_y;
     float accel_ref_z;

     // accelerometer measurement variance
     float accel_var;

     // Magnetometer variance
     float mag_var;

     // Process variance
     float process_var;

     // Entries for storing processed sensor data
     float gyro_x;
     float gyro_y;
     float gyro_z;

     float accel_x;
     float accel_y;
     float accel_z;

     float mag_x;
     float mag_y;
     float mag_z;

     float temperature;

} tStateData;

class cEKF
{
public:
    cEKF();


    void init(tStateData &);

    void convertRawSensorData(tStateData &state, tRawSensorData &sensor);

    void estimateStates(tStateData &, tRawSensorData &);
    void predict(tStateData &);
    void update(tStateData &);

    void correct(cMatrix<1,4> &C, float sensor_data, float sensor_hat, float sensor_covariance, tStateData &state);

    void dirtyConvert(tStateData &state, tRawSensorData &sensor);
private:
    bool mInitialized = false;

};

#endif // CEKF_H
