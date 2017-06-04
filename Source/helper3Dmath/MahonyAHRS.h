#ifndef MAHONYAHRS_
#define MAHONYAHRS_
/*http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
based on: https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU.git
https://www.luisllamas.es/usar-arduino-con-los-imu-de-9dof-mpu-9150-y-mpu-9250/

https://github.com/kriswiner/MPU-9250
https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf

If you are going to compile a C program with math.h library in LINUX using GCC or G++ you will have to use –lm option after the compile command.

gcc xyz.c -o xyz -lm
Here,

gcc is compiler command (compiler name)
xyz.c is a source file name.
-o is an option to create objcect file.
xyz is the name of object (binary) file.
-lm is an option for math.h 
*/

#include <math.h>

enum _QUATERNION_AXIS_ROTATION {
	AXIS_X,
	AXIS_Y,
	AXIS_Z
};

typedef union {
	struct {
		float x;     /* rotation around x axis in degrees */
		float y;    /* rotation around y axis in degrees */
		float z;      /* rotation around z axis in degrees */
	}Vector;
	float vector[3];
} Vector3F;

typedef union {
	struct {
		short x;     /* rotation around x axis in degrees */
		short y;    /* rotation around y axis in degrees */
		short z;      /* rotation around z axis in degrees */
	}Vector;
	short vector[3];
} Vector3I16;

typedef union {
	struct {
		float w;
		float x;
		float y;
		float z;
	}Quaternion;
	float quaternion[4];
	long quaternionL[4];
} QuaternionStruct;

typedef union {
	struct {
		short w;
		short x;
		short y;
		short z;
	}Quaternion;
	short quaternion[4];
} QuaternionStructI16;


typedef union {
	struct {
		long w;
		long x;
		long y;
		long z;
	}Quaternion;
	long quaternion[4];
} QuaternionStructI32;

typedef union {
	struct {
		float roll;     /* rotation around x axis in degrees */
		float pitch;    /* rotation around y axis in degrees */
		float yaw;      /* rotation around z axis in degrees */
	}Angles;
	float angles[3];
} EulerAnglesStruct;

typedef union {
	struct {
		short roll;     /* rotation around x axis in degrees */
		short pitch;    /* rotation around y axis in degrees */
		short yaw;      /* rotation around z axis in degrees */
	}Angles;
	short angles[3];
} EulerAnglesStructI16;

//funciones para la fusion de los sensores
static void initAHRS(float samplePeriod, float kp, float ki); //solo usar si se quiere cambiar
static QuaternionStruct updateAHRS(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

//funciones de operacion cinematica con quaterniones
static QuaternionStruct quaternionRotation(float theta_rads, QuaternionStruct vct, enum _QUATERNION_AXIS_ROTATION dir);
static Vector3F quaternionRotation3f(float theta_rads, Vector3F vct, enum _QUATERNION_AXIS_ROTATION dir);
static Vector3F quaternionRotTras3f(float theta_rads, Vector3F vct_to_rot, Vector3F traslation, enum _QUATERNION_AXIS_ROTATION dir);

//operaciones basicas de cuaterniones
static QuaternionStruct quaternionDot(QuaternionStruct q1, QuaternionStruct q2);
static QuaternionStruct quaternionProduct(QuaternionStruct q1, QuaternionStruct q2);
static QuaternionStruct quaternionConjugate(QuaternionStruct q);

//operaciones basicas de vectores
static Vector3F vector3fAdd(Vector3F vct1, Vector3F vct2);

static EulerAnglesStruct getEulerAngles(QuaternionStruct quaternion);
static inline float radiansToDegrees(float radians);
static inline float degreesToRadians(float degrees);

static QuaternionStruct getLastPosition(void);

#endif