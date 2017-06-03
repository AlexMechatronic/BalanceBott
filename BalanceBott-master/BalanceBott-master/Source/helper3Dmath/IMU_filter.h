/*
 * IMU_filter.h
 *
 * Created: 23/04/2016 02:18:45 p. m.
 *  Author: aleja_000
 */ 


#ifndef IMU_FILTER_H_
#define IMU_FILTER_H_

// Math library required for `sqrt'
#include <math.h>

// System constants
//#define deltat 0.001f // sampling period in seconds (shown as 1 ms)
#define gyroDegresError 1.0f //5.0f// in deg/s
#define gyroMeasError 3.14159265358979f * (gyroDegresError / 180.0f) // gyroscope measurement error in rad/s (shown as 1 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta

typedef struct {
	float w;
	float x;
	float y;
	float z;
} QuaternionStruct;

typedef struct {
	float roll;     /* rotation around x axis in degrees */
	float pitch;    /* rotation around y axis in degrees */
	float yaw;      /* rotation around z axis in degrees */
} EulerAnglesStruct;


// Global system variables
//float a_x, a_y, a_z; // accelerometer measurements
//float w_x, w_y, w_z; // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
QuaternionStruct lastPosition;

/* detlat is sampling in seconds
 * w_ is the components of the gyroscope in deg/s
 * a_ is the components of the acelerometer in m/s^2 */
QuaternionStruct filterUpdate(float deltat, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);
EulerAnglesStruct getEulerAngles(QuaternionStruct quaternion);
float radiansToDegrees (float radians);

//devuelve la ultima referencia de posicion en quaternion
QuaternionStruct getLastPosition(void);

#endif /* IMU_FILTER_H_ */