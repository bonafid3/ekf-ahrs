#include "cekf.h"
#include "cTC.h"



cEKF::cEKF()
{
}

void cEKF::estimateStates(tStateData &state, sSensorData &sensor)
{
    if( !mInitialized ) {
        init( state );
        mInitialized = true;
    }

    // First, convert raw sensor data to actual data (acceleration to gravities, gyro data
    // to angular rates, magnetometer to unit-norm data
    //convertRawSensorData( state, sensor );
    dirtyConvert(state, sensor);

    // Run EKF prediction step
    predict( state );

    // Run EKF update step
    update( state );
}

void cEKF::dirtyConvert(tStateData &state, sSensorData &sensor)
{
	float length;
    state.gyro_x = sensor.mGyroX;
    state.gyro_y = sensor.mGyroY;
    state.gyro_z = sensor.mGyroZ;

	state.accel_x = sensor.mAccX / 8096.0;
	state.accel_y = sensor.mAccY / 8096.0;
	state.accel_z = sensor.mAccZ / 8096.0;
	
	

	cMatrix<3,1> svec;
	
	sensor.mMagX -= state.beta_mag_x;
	sensor.mMagY -= state.beta_mag_y;
	sensor.mMagZ -= state.beta_mag_z;
	
    svec.data[0][0] = (float)sensor.mMagX;
    svec.data[1][0] = (float)sensor.mMagY;
    svec.data[2][0] = (float)sensor.mMagZ;

    svec = state.mag_cal * svec;

	sensor.mMagX = svec.data[0][0];
	sensor.mMagY = svec.data[1][0];
	sensor.mMagZ = svec.data[2][0];

	length = sqrt(svec.data[0][0]*svec.data[0][0] + svec.data[1][0]*svec.data[1][0] + svec.data[2][0]*svec.data[2][0]);

    state.mag_x = svec.data[0][0] / length;
    state.mag_y = svec.data[1][0] / length;
    state.mag_z = svec.data[2][0] / length;
}

void cEKF::convertRawSensorData(tStateData &state, sSensorData &sensor)
{
    return;

    cMatrix<3,1> svec;

    // Convert temperature data
    float temp = sensor.mTemperature*0.00357143 + 70.00;
    float temp2 = temp*temp;
    float temp3 = temp2*temp;

    state.temperature = temp;

    // Rate gyros
    svec.data[0][0] = (float)((float)sensor.mGyroX - (float)state.beta_p - (state.beta_p0 + state.beta_p1*temp + state.beta_p2*temp2 + state.beta_p3*temp3) );
    svec.data[1][0] = (float)((float)sensor.mGyroY - (float)state.beta_q - (state.beta_q0 + state.beta_q1*temp + state.beta_q2*temp2 + state.beta_q3*temp3) );
    svec.data[2][0] = (float)((float)sensor.mGyroZ - (float)state.beta_r - (state.beta_r0 + state.beta_r1*temp + state.beta_r2*temp2 + state.beta_r3*temp3) );

    // Multiply gyro measurements by alignment matrix (fixes cross-axis alignment)
    svec = state.gyro_cal * svec;

    // Copy new gyro data to state structure
    state.gyro_x = svec.data[0][0];
    state.gyro_y = svec.data[1][0];
    state.gyro_z = svec.data[2][0];

    // Now for accelerometers
    svec.data[0][0] = ((float)(sensor.mAccX - state.beta_acc_x));
    svec.data[1][0] = ((float)(sensor.mAccY - state.beta_acc_y));
    svec.data[2][0] = ((float)(sensor.mAccZ - state.beta_acc_z));

    svec = state.accel_cal * svec;

    state.accel_x = svec.data[0][0];
    state.accel_y = svec.data[1][0];
    state.accel_z = svec.data[2][0];

    // Now the magnetometer
    svec.data[0][0] = ((float)(sensor.mMagX - state.beta_mag_x));
    svec.data[1][0] = ((float)(sensor.mMagY - state.beta_mag_y));
    svec.data[2][0] = ((float)(sensor.mMagZ - state.beta_mag_z));

    svec = state.mag_cal * svec;

    state.mag_x = svec.data[0][0];
    state.mag_y = svec.data[1][0];
    state.mag_z = svec.data[2][0];
}

// it requires global state data only!
void cEKF::predict(tStateData &gState)
{
    float T,p,q,r;
	
	cTC tc1(TC0, 1);
	
	const float invClock = 3.125e-7;
    T = tc1.counterValue() * invClock;
	tc1.start();
	
	T = 0.0138;

    // Copy body frame angular rates to local variables for convenience
    p = gState.gyro_x;
    q = gState.gyro_y;
    r = gState.gyro_z;

    cMatrix<4,4> A, Atranspose;

    float a, b, c, d;

    // Make local copies of the current attitude quaternion for convenience
    a = gState.qib.a;
    b = gState.qib.b;
    c = gState.qib.c;
    d = gState.qib.d;

    // Convert p, q, and r to rad/s already converted
    //p = p*3.14159/180;
    //q = q*3.14159/180;
    //r = r*3.14159/180;

    // Create a quaternion to represent rotation rate
    cQuaternion pqr_quat;
    pqr_quat.a = 0;
    pqr_quat.b = p;
    pqr_quat.c = q;
    pqr_quat.d = r;

    // Predict new quaternion state based on gyro data
    cQuaternion temp_quat;
    temp_quat = gState.qib * pqr_quat;

    temp_quat = temp_quat * 0.5 * T;

    gState.qib = gState.qib + temp_quat;

    // Normalize new predicted state
    gState.qib.normalize();

    // PROPAGATE COVARIANCE
    // Compute linearized state transition matrix
    /*
          [       1, -(T*p)/2, -(T*q)/2, -(T*r)/2]
          [ (T*p)/2,        1,  (T*r)/2, -(T*q)/2]
          [ (T*q)/2, -(T*r)/2,        1,  (T*p)/2]
          [ (T*r)/2,  (T*q)/2, -(T*p)/2,        1]
    */
    A.data[0][0] = 1;
    A.data[0][1] = -(T*p)/2;
    A.data[0][2] = -(T*q)/2;
    A.data[0][3] = -(T*r)/2;

    A.data[1][0] = (T*p)/2;
    A.data[1][1] = 1;
    A.data[1][2] = (T*r)/2;
    A.data[1][3] = -(T*q)/2;

    A.data[2][0] = (T*q)/2;
    A.data[2][1] = -(T*r)/2;
    A.data[2][2] = 1;
    A.data[2][3] = (T*p)/2;

    A.data[3][0] = (T*r)/2;
    A.data[3][1] = (T*q)/2;
    A.data[3][2] = -(T*p)/2;
    A.data[3][3] = 1;

    // Project the error covariance ahead
    // Compute the new covariance estimate (discrete update: Sigma = A*Sigma*Atranspose + R
    Atranspose = A.transposed();
    gState.Sigma = A * gState.Sigma * Atranspose + gState.R;

}

void cEKF::update(tStateData &gState)
{

    float a,b,c,d;

    // Do accelerometer update if enabled and if there is new accelerometer sensor data available
    if(1)
    {
        float sensor_norm = sqrt(gState.accel_x*gState.accel_x + gState.accel_y*gState.accel_y + gState.accel_z*gState.accel_z);
        cMatrix<1,4> C;

        // Make sure this accelerometer measurement is close to 1 G.  If not, ignore it.
        if( jani::abs(sensor_norm) < 1)
        {
            float ax_hat, ay_hat, az_hat;
            float ax_ref, ay_ref, az_ref;

            // Make local copies of the current quaternion state estimate for convenience
            a = gState.qib.a;
            b = gState.qib.b;
            c = gState.qib.c;
            d = gState.qib.d;

            // Make local copy of accelerometer reference vector for convenience
            ax_ref = gState.accel_ref_x;
            ay_ref = gState.accel_ref_y;
            az_ref = gState.accel_ref_z;

            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            ax_hat = ay_ref*(2*a*d + 2*b*c) - az_ref*(2*a*c - 2*b*d) + ax_ref*(a*a + b*b - c*c - d*d);

            // Create linearized update matrix for x-axis accelerometer
            /* For all axes, C is given by:
               [ 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d, 2*ay*b - 2*a*az - 2*ax*c, 2*a*ay + 2*az*b - 2*ax*d]
               [ 2*a*ay + 2*az*b - 2*ax*d, 2*a*az - 2*ay*b + 2*ax*c, 2*ax*b + 2*ay*c + 2*az*d, 2*az*c - 2*a*ax - 2*ay*d]
               [ 2*a*az - 2*ay*b + 2*ax*c, 2*ax*d - 2*az*b - 2*a*ay, 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d]
               */
            C.data[0][0] = 2*a*ax_ref - 2*az_ref*c + 2*ay_ref*d;
            C.data[0][1] = 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d;
            C.data[0][2] = 2*ay_ref*b - 2*a*az_ref - 2*ax_ref*c;
            C.data[0][3] = 2*a*ay_ref + 2*az_ref*b - 2*ax_ref*d;

            // Do correction
            correct( C, gState.accel_x, ax_hat, gState.accel_var, gState );

            // REPEAT FOR Y-AXIS
            // Make local copies of the current quaternion state estimate for convenience
            a = gState.qib.a;
            b = gState.qib.b;
            c = gState.qib.c;
            d = gState.qib.d;

            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            ay_hat = az_ref*(2*a*b + 2*c*d) - ax_ref*(2*a*d - 2*b*c) + ay_ref*(a*a - b*b + c*c - d*d);
            az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);

            C.data[0][0] = 2*a*ay_ref + 2*az_ref*b - 2*ax_ref*d;
            C.data[0][1] = 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c;
            C.data[0][2] = 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d;
            C.data[0][3] = 2*az_ref*c - 2*a*ax_ref - 2*ay_ref*d;

            // Do correction
            correct( C, gState.accel_y, ay_hat, gState.accel_var, gState );

            // REPEAT FOR Z-AXIS
            // Make local copies of the current quaternion state estimate for convenience
            a = gState.qib.a;
            b = gState.qib.b;
            c = gState.qib.c;
            d = gState.qib.d;

            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);

            C.data[0][0] = 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c;
            C.data[0][1] = 2*ax_ref*d - 2*az_ref*b - 2*a*ay_ref;
            C.data[0][2] = 2*a*ax_ref - 2*az_ref*c + 2*ay_ref*d;
            C.data[0][3] = 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d;

            // Do correction
            correct( C, gState.accel_z, az_hat, gState.accel_var, gState );
        }
    }

    // MAGNETOMETER UPDATE PART
    if(1)
    {
        cMatrix<1,4> C;

        float mx_hat, my_hat, mz_hat;
        float mx_ref, my_ref, mz_ref;

        // Make local copies of the current quaternion state estimate for convenience
        a = gState.qib.a;
        b = gState.qib.b;
        c = gState.qib.c;
        d = gState.qib.d;

        // Make local copy of accelerometer reference vector for convenience
        mx_ref = gState.mag_ref_x;
        my_ref = gState.mag_ref_y;
        mz_ref = gState.mag_ref_z;

        // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
        mx_hat = my_ref*(2*a*d + 2*b*c) - mz_ref*(2*a*c - 2*b*d) + mx_ref*(a*a + b*b - c*c - d*d);

        // Create linearized update matrix for x-axis magnetometer
        /* For all axes, C is given by:
          [ 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d, 2*ay*b - 2*a*az - 2*ax*c, 2*a*ay + 2*az*b - 2*ax*d]
          [ 2*a*ay + 2*az*b - 2*ax*d, 2*a*az - 2*ay*b + 2*ax*c, 2*ax*b + 2*ay*c + 2*az*d, 2*az*c - 2*a*ax - 2*ay*d]
          [ 2*a*az - 2*ay*b + 2*ax*c, 2*ax*d - 2*az*b - 2*a*ay, 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d]
          */
        C.data[0][0] = 2*a*mx_ref - 2*mz_ref*c + 2*my_ref*d;
        C.data[0][1] = 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d;
        C.data[0][2] = 2*my_ref*b - 2*a*mz_ref - 2*mx_ref*c;
        C.data[0][3] = 2*a*my_ref + 2*mz_ref*b - 2*mx_ref*d;

        // Do correction
        correct( C, gState.mag_x, mx_hat, gState.mag_var, gState );

        // REPEAT FOR Y-AXIS
        // Make local copies of the current quaternion state estimate for convenience
        a = gState.qib.a;
        b = gState.qib.b;
        c = gState.qib.c;
        d = gState.qib.d;

        // Compute expected magnetometer measurements based on the current attitude quaternion and the accel reference vector
        my_hat = mz_ref*(2*a*b + 2*c*d) - mx_ref*(2*a*d - 2*b*c) + my_ref*(a*a - b*b + c*c - d*d);

        C.data[0][0] = 2*a*my_ref + 2*mz_ref*b - 2*mx_ref*d;
        C.data[0][1] = 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c;
        C.data[0][2] = 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d;
        C.data[0][3] = 2*mz_ref*c - 2*a*mx_ref - 2*my_ref*d;

        // Do correction
        correct( C, gState.mag_y, my_hat, gState.mag_var, gState );

        // REPEAT FOR Z-AXIS
        // Make local copies of the current quaternion state estimate for convenience
        a = gState.qib.a;
        b = gState.qib.b;
        c = gState.qib.c;
        d = gState.qib.d;

        // Compute expected magnetometer measurements based on the current attitude quaternion and the accel reference vector
        mz_hat = mx_ref*(2*a*c + 2*b*d) - my_ref*(2*a*b - 2*c*d) + mz_ref*(a*a - b*b - c*c + d*d);

        C.data[0][0] = 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c;
        C.data[0][1] = 2*mx_ref*d - 2*mz_ref*b - 2*a*my_ref;
        C.data[0][2] = 2*a*mx_ref - 2*mz_ref*c + 2*my_ref*d;
        C.data[0][3] = 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d;

        // Do correction
        correct( C, gState.mag_z, mz_hat, gState.mag_var, gState );
    }
}

void cEKF::correct( cMatrix<1,4> &C, float sensor_data, float sensor_hat, float sensor_covariance, tStateData &state)
{
    cMatrix<4,1> Ctranspose, L;
    cMatrix<4,4> I;
    float gain_scale=0, error=0;

    Ctranspose = C.transposed();
    // sigma = error in the estimate?
    // Compute Kalman Gain (L = Sigma*C'*(C*Sigma*C' + Q)^-1 )
    /* G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */

    //cMatrix<1,1> Q;
    //Q.data[0][0] = sensor_covariance;
    //L = state.Sigma * Ctranspose * (C * state.Sigma * Ctranspose + Q).inv();

    cMatrix<1,1> temp2;
    temp2 = (C * state.Sigma * Ctranspose);

    gain_scale = 1/(temp2.data[0][0] + sensor_covariance);

    L = state.Sigma * Ctranspose * gain_scale;

    // Update state estimates
    error = sensor_data - sensor_hat;
    //qd << "error" << error;

    // current state corrected by error
    state.qib.a += L.data[0][0]*error;
    state.qib.b += L.data[1][0]*error;
    state.qib.c += L.data[2][0]*error;
    state.qib.d += L.data[3][0]*error;

    // process covariance matrix
    // Now update the covariance estimate (Sigma = (I - L*C)*Sigma
    state.Sigma = (I - L * C) * state.Sigma;
}

void cEKF::init(tStateData &state)
{
    state.R.setToZero();
    state.Sigma.setToZero();

    // Process variance is scaled here so that the performance in Euler Angle mode and Quaternion mode is comparable
    state.Sigma.data[0][0] = state.process_var*0.00001;
    state.Sigma.data[1][1] = state.process_var*0.00001;
    state.Sigma.data[2][2] = state.process_var*0.00001;
    state.Sigma.data[3][3] = state.process_var*0.00001;

    state.R.data[0][0] = state.process_var*0.00001;
    state.R.data[1][1] = state.process_var*0.00001;
    state.R.data[2][2] = state.process_var*0.00001;
    state.R.data[3][3] = state.process_var*0.00001;
}
