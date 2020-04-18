//#include "IMU.h"
#include "Board.h"
TM_AHRSIMU_t AHRS;

att_t att;

float Pre_IMU[3] = {0, 0, 0};
float Pre_gyro = 0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
/////////////////////////////////////////////////////////////////

/* Calculate 1/sqrt(x) with magic number support */
//static float oneOverSqrt(float x) {
//    return 1.0f / (float) sqrt(x);
//}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//float invSqrt(float x){
//   unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
//   float tmp = *(float*)&i;
//   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
//}


void calculateAngles(TM_AHRSIMU_t* AHRS) {
    AHRS->Roll = (float) atan2f(AHRS->_q0 * AHRS->_q1 + AHRS->_q2 * AHRS->_q3, 0.5f - AHRS->_q1 * AHRS->_q1 - AHRS->_q2 * AHRS->_q2);
    AHRS->Pitch = (float) asinf(-2.0f * (AHRS->_q1 * AHRS->_q3 - AHRS->_q0 * AHRS->_q2));
    AHRS->Yaw = (float) atan2f(AHRS->_q1 * AHRS->_q2 + AHRS->_q0 * AHRS->_q3, 0.5f - AHRS->_q2 * AHRS->_q2 - AHRS->_q3 * AHRS->_q3);

    /* Calculate degrees and remove inclination */
    AHRS->Roll *= R2D;
    AHRS->Pitch *= R2D;
    AHRS->Yaw *= R2D;// - AHRS->Inclination;

    /* Check values because of inclination */
//    if (AHRS->Yaw < -180) {
//        AHRS->Yaw = 180.0f - (-180.0f - AHRS->Yaw);
//    } else if (AHRS->Yaw > 180) {
//        AHRS->Yaw = -180.0f - (180.0f - AHRS->Yaw);
//    }
//
    if (AHRS->Yaw >= 180) {
        AHRS->Yaw -= 360.0f;
    } else if (AHRS->Yaw < -180) {
        AHRS->Yaw += 360.0f;
    }
//        if (AHRS->Yaw >= 0) {
//            AHRS->Yaw = 360.0f - AHRS->Yaw;
//        } else {
//            AHRS->Yaw = - AHRS->Yaw;
//        }

  AHRS->Roll  = (0.95 * (Pre_IMU[ROLL]  + (imu.gyroRaw[ROLL] * 0.004)))  + (0.05 * AHRS->Roll);
  AHRS->Pitch = (0.95 * (Pre_IMU[PITCH] + (imu.gyroRaw[PITCH] * 0.004))) + (0.05 * AHRS->Pitch);
  AHRS->Yaw   = (0.95 * (Pre_IMU[YAW]   + (imu.gyroRaw[YAW] * 0.004)))   + (0.05 * AHRS->Yaw);
		
		imu.AHRS[ROLL] = AHRS->Roll + 0.0f;
		imu.AHRS[PITCH] = AHRS->Pitch + 0.0f;
		imu.AHRS[YAW] = AHRS->Yaw;

		att.angle[ROLL] = (int16_t) imu.AHRS[ROLL] * 10;
    att.angle[PITCH] = (int16_t) imu.AHRS[PITCH] * 10;
    att.heading = (int16_t) imu.AHRS[YAW];
		
	Pre_IMU[ROLL]  = AHRS->Roll;
	Pre_IMU[PITCH] = AHRS->Pitch;
	Pre_IMU[YAW]   = AHRS->Yaw;
	}

void TM_AHRSIMU_Init(TM_AHRSIMU_t* AHRS, float sampleRate, float beta, float inclination) {
    AHRS->_beta = beta;
    AHRS->_sampleRate = 1 / sampleRate;
    AHRS->Inclination = inclination;

    AHRS->_q0 = 1.0f;
    AHRS->_q1 = 0.0f;
    AHRS->_q2 = 0.0f;
    AHRS->_q3 = 0.0f;
}

void TM_AHRSIMU_UpdateAHRS(TM_AHRSIMU_t* AHRS, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    /* Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation) */
    if ((mx != 0.0f) || (my != 0.0f) || (mz != 0.0f)) {

        // Convert gyroscope degrees/sec to radians/sec
        gx = AHRSIMU_DEG2RAD(gx);
        gy = AHRSIMU_DEG2RAD(gy);
        gz = AHRSIMU_DEG2RAD(gz);

        /* Rate of change of quaternion from gyroscope */
        qDot1 = 0.5f * (-AHRS->_q1 * gx - AHRS->_q2 * gy - AHRS->_q3 * gz);
        qDot2 = 0.5f * (AHRS->_q0 * gx + AHRS->_q2 * gz - AHRS->_q3 * gy);
        qDot3 = 0.5f * (AHRS->_q0 * gy - AHRS->_q1 * gz + AHRS->_q3 * gx);
        qDot4 = 0.5f * (AHRS->_q0 * gz + AHRS->_q1 * gy - AHRS->_q2 * gx);

        /* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            /* Normalise accelerometer measurement */
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            /* Normalise magnetometer measurement */
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            /* Auxiliary variables to avoid repeated arithmetic */
            _2q0mx = 2.0f * AHRS->_q0 * mx;
            _2q0my = 2.0f * AHRS->_q0 * my;
            _2q0mz = 2.0f * AHRS->_q0 * mz;
            _2q1mx = 2.0f * AHRS->_q1 * mx;
            _2q0 = 2.0f * AHRS->_q0;
            _2q1 = 2.0f * AHRS->_q1;
            _2q2 = 2.0f * AHRS->_q2;
            _2q3 = 2.0f * AHRS->_q3;
            _2q0q2 = 2.0f * AHRS->_q0 * AHRS->_q2;
            _2q2q3 = 2.0f * AHRS->_q2 * AHRS->_q3;
            q0q0 = AHRS->_q0 * AHRS->_q0;
            q0q1 = AHRS->_q0 * AHRS->_q1;
            q0q2 = AHRS->_q0 * AHRS->_q2;
            q0q3 = AHRS->_q0 * AHRS->_q3;
            q1q1 = AHRS->_q1 * AHRS->_q1;
            q1q2 = AHRS->_q1 * AHRS->_q2;
            q1q3 = AHRS->_q1 * AHRS->_q3;
            q2q2 = AHRS->_q2 * AHRS->_q2;
            q2q3 = AHRS->_q2 * AHRS->_q3;
            q3q3 = AHRS->_q3 * AHRS->_q3;

            /* Reference direction of Earth's magnetic field */
            hx = mx * q0q0 - _2q0my * AHRS->_q3 + _2q0mz * AHRS->_q2 + mx * q1q1 + _2q1 * my * AHRS->_q2 + _2q1 * mz * AHRS->_q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * AHRS->_q3 + my * q0q0 - _2q0mz * AHRS->_q1 + _2q1mx * AHRS->_q2 - my * q1q1 + my * q2q2 + _2q2 * mz * AHRS->_q3 - my * q3q3;
            _2bx = (float) sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * AHRS->_q2 + _2q0my * AHRS->_q1 + mz * q0q0 + _2q1mx * AHRS->_q3 - mz * q1q1 + _2q2 * my * AHRS->_q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            /* Gradient decent algorithm corrective step */
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * AHRS->_q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * AHRS->_q3 + _2bz * AHRS->_q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * AHRS->_q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * AHRS->_q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * AHRS->_q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * AHRS->_q2 + _2bz * AHRS->_q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * AHRS->_q3 - _4bz * AHRS->_q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * AHRS->_q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * AHRS->_q2 - _2bz * AHRS->_q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * AHRS->_q1 + _2bz * AHRS->_q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * AHRS->_q0 - _4bz * AHRS->_q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * AHRS->_q3 + _2bz * AHRS->_q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * AHRS->_q0 + _2bz * AHRS->_q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * AHRS->_q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            /* normalise step magnitude */
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            /* Apply feedback step */
            qDot1 -= AHRS->_beta * s0;
            qDot2 -= AHRS->_beta * s1;
            qDot3 -= AHRS->_beta * s2;
            qDot4 -= AHRS->_beta * s3;
        }

        /* Integrate rate of change of quaternion to yield quaternion */
        AHRS->_q0 += qDot1 * AHRS->_sampleRate;
        AHRS->_q1 += qDot2 * AHRS->_sampleRate;
        AHRS->_q2 += qDot3 * AHRS->_sampleRate;
        AHRS->_q3 += qDot4 * AHRS->_sampleRate;

        /* Normalise quaternion */
        recipNorm = invSqrt(AHRS->_q0 * AHRS->_q0 + AHRS->_q1 * AHRS->_q1 + AHRS->_q2 * AHRS->_q2 + AHRS->_q3 * AHRS->_q3);
        AHRS->_q0 *= recipNorm;
        AHRS->_q1 *= recipNorm;
        AHRS->_q2 *= recipNorm;
        AHRS->_q3 *= recipNorm;
    } else {
        /* Update IMU algorithm */
        TM_AHRSIMU_UpdateIMU(AHRS, gx, gy, gz, ax, ay, az);
    }

    /* Calculate new angles */
    calculateAngles(AHRS);
}

void TM_AHRSIMU_UpdateIMU(TM_AHRSIMU_t* AHRS, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	
		// Convert gyroscope degrees/sec to radians/sec
	  gx = AHRSIMU_DEG2RAD(gx);
	  gy = AHRSIMU_DEG2RAD(gy);
	  gz = AHRSIMU_DEG2RAD(gz);

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-AHRS->_q1 * gx - AHRS->_q2 * gy - AHRS->_q3 * gz);
    qDot2 = 0.5f * (AHRS->_q0 * gx + AHRS->_q2 * gz - AHRS->_q3 * gy);
    qDot3 = 0.5f * (AHRS->_q0 * gy - AHRS->_q1 * gz + AHRS->_q3 * gx);
    qDot4 = 0.5f * (AHRS->_q0 * gz + AHRS->_q1 * gy - AHRS->_q2 * gx);

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {

        /* Normalise accelerometer measurement */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        _2q0 = 2.0f * AHRS->_q0;
        _2q1 = 2.0f * AHRS->_q1;
        _2q2 = 2.0f * AHRS->_q2;
        _2q3 = 2.0f * AHRS->_q3;
        _4q0 = 4.0f * AHRS->_q0;
        _4q1 = 4.0f * AHRS->_q1;
        _4q2 = 4.0f * AHRS->_q2;
        _8q1 = 8.0f * AHRS->_q1;
        _8q2 = 8.0f * AHRS->_q2;
        q0q0 = AHRS->_q0 * AHRS->_q0;
        q1q1 = AHRS->_q1 * AHRS->_q1;
        q2q2 = AHRS->_q2 * AHRS->_q2;
        q3q3 = AHRS->_q3 * AHRS->_q3;

        /* Gradient decent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * AHRS->_q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * AHRS->_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * AHRS->_q3 - _2q1 * ax + 4.0f * q2q2 * AHRS->_q3 - _2q2 * ay;

        /* Normalise step magnitude */
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot1 -= AHRS->_beta * s0;
        qDot2 -= AHRS->_beta * s1;
        qDot3 -= AHRS->_beta * s2;
        qDot4 -= AHRS->_beta * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    AHRS->_q0 += qDot1 * AHRS->_sampleRate;
    AHRS->_q1 += qDot2 * AHRS->_sampleRate;
    AHRS->_q2 += qDot3 * AHRS->_sampleRate;
    AHRS->_q3 += qDot4 * AHRS->_sampleRate;

    /* Normalise quaternion */
    recipNorm = invSqrt(AHRS->_q0 * AHRS->_q0 + AHRS->_q1 * AHRS->_q1 + AHRS->_q2 * AHRS->_q2 + AHRS->_q3 * AHRS->_q3);
    AHRS->_q0 *= recipNorm;
    AHRS->_q1 *= recipNorm;
    AHRS->_q2 *= recipNorm;
    AHRS->_q3 *= recipNorm;

    /* Calculate new angles */
    calculateAngles(AHRS);
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - 0.9 * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - 0.9 * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - 0.9 * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - 0.9 * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * 0.004;
  q2 += qDot2 * 0.004;
  q3 += qDot3 * 0.004;
  q4 += qDot4 * 0.004;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;


  imu.AHRS[YAW]   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  imu.AHRS[PITCH] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  imu.AHRS[ROLL]  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  imu.AHRS[PITCH] *= 180.0f / AHRSIMU_PI;
  imu.AHRS[YAW]   *= 180.0f / AHRSIMU_PI;
  imu.AHRS[YAW]   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  imu.AHRS[ROLL]  *= 180.0f / AHRSIMU_PI;

}

void computeIMU(void)
{
	Gyro_getADC();  //240us
  ACC_getADC(); //240us
	Temp_getADC(); //140us
#ifdef IMU_NORMAL
    imu.accx = atan2(imu.accRaw[PITCH], imu.accRaw[YAW] + abs(imu.accRaw[ROLL])) * R2D;
    imu.accy = atan2(imu.accRaw[ROLL], imu.accRaw[YAW] + abs(imu.accRaw[PITCH])) * R2D;
	
		imu.gyrox += imu.gyroRaw[ROLL] * 0.004;
	  imu.gyroy += imu.gyroRaw[PITCH] * 0.004;
	  imu.gyroz += imu.gyroRaw[YAW] * 0.004;
	
		imu.Roll = (0.98f * (imu.Roll + imu.gyroRaw[ROLL] * 0.004)) + (0.02f * imu.accx);
	  imu.Pitch = (0.98f * (imu.Pitch + imu.gyroRaw[PITCH] * 0.004)) + (0.02f * imu.accy);
	  imu.Yaw = imu.gyroz;
		#endif
	
#ifdef IMU_AHRS
    /* Call update function */
    /* This function must be called periodically in inteervals set by sample rate on initialization process */

	TM_AHRSIMU_UpdateIMU(&AHRS, imu.gyroRaw[ROLL], imu.gyroRaw[PITCH], imu.gyroRaw[YAW], imu.accSmooth[ROLL], imu.accSmooth[PITCH], imu.accSmooth[YAW]);

	//TM_AHRSIMU_UpdateAHRS(&AHRS, imu.gyroRaw[ROLL], imu.gyroRaw[PITCH], imu.gyroRaw[YAW], imu.accRaw[ROLL], imu.accRaw[PITCH], imu.accRaw[YAW], imu.magSmooth[ROLL], imu.magSmooth[PITCH], imu.magSmooth[YAW]);
#endif
}
