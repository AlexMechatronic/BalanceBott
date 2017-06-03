#include "MahonyAHRS.h"

void initAHRS(float samplePeriod, float kp, float ki)
{
	SamplePeriod = samplePeriod;
	Kp = kp;
	Ki = ki;
}

QuaternionStruct updateAHRS(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
	float q1 = Quaternion.quaternion[0], q2 = Quaternion.quaternion[1], q3 = Quaternion.quaternion[2], q4 = Quaternion.quaternion[3];   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
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
	norm = (float)sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = (float)sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0 / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = (float)sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * SamplePeriod);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * SamplePeriod);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * SamplePeriod);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * SamplePeriod);

	// Normalise quaternion
	norm = (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	Quaternion.quaternion[0] = q1 * norm;
	Quaternion.quaternion[1] = q2 * norm;
	Quaternion.quaternion[2] = q3 * norm;
	Quaternion.quaternion[3] = q4 * norm;
	
	return Quaternion;
}

QuaternionStruct quaternionRotation(float theta_rads, QuaternionStruct vct, enum _QUATERNION_AXIS_ROTATION dir)
{

	return vct;
}

Vector3F quaternionRotation3f(float theta_rads, Vector3F vct, enum _QUATERNION_AXIS_ROTATION dir)
{
	/*
	Qrot = sin(theta_rads)/2 + cos(theta_rads)/2 i + cos(theta_rads)/2 j + cos(theta_rads)/2 k
	pero en la rotacion de un eje unitario x, y ó z solo el eje escogido vale 1 los demas son 0
	*/
	float tht_h = theta_rads / 2.0f;//var help para no repetir la operacion
	float q_w = sin(tht_h);
	float q_comp = cos(tht_h);
	Vector3F res;

	if (dir == AXIS_X)
	{
		
	}
	else if (dir == AXIS_Y)
	{

	}
	else if (dir == AXIS_Z)
	{

	}

	return res;
}

Vector3F quaternionRotTras3f(float theta_rads, Vector3F vct_to_rot, Vector3F traslation, enum _QUATERNION_AXIS_ROTATION dir)
{
	return vector3fAdd(quaternionRotation3f(theta_rads, vct_to_rot, dir), traslation);
}

QuaternionStruct quaternionDot(QuaternionStruct q1, QuaternionStruct q2)
{
	q1.Quaternion.w *= q2.Quaternion.w;
	q1.Quaternion.x *= q2.Quaternion.x;
	q1.Quaternion.y *= q2.Quaternion.y;
	q1.Quaternion.z *= q2.Quaternion.z;
	return q1;
}

QuaternionStruct quaternionProduct(QuaternionStruct q1, QuaternionStruct q2)
{
	QuaternionStruct res;

	res.Quaternion.w = q1.quaternion[0] * q2.quaternion[0] - q1.quaternion[1] * q2.quaternion[1] - q1.quaternion[2] * q2.quaternion[2] - q1.quaternion[3] * q2.quaternion[3];
	res.Quaternion.x = q1.quaternion[0] * q2.quaternion[1] - q1.quaternion[1] * q2.quaternion[0] - q1.quaternion[2] * q2.quaternion[3] - q1.quaternion[3] * q2.quaternion[2];
	res.Quaternion.y = q1.quaternion[0] * q2.quaternion[2] - q1.quaternion[1] * q2.quaternion[3] - q1.quaternion[2] * q2.quaternion[0] - q1.quaternion[3] * q2.quaternion[1];
	res.Quaternion.z = q1.quaternion[0] * q2.quaternion[3] - q1.quaternion[1] * q2.quaternion[2] - q1.quaternion[2] * q2.quaternion[1] - q1.quaternion[3] * q2.quaternion[0];

	return res;
}

QuaternionStruct quaternionConjugate(QuaternionStruct q)
{
	q.Quaternion.x *= -1;
	q.Quaternion.y *= -1;
	q.Quaternion.z *= -1;

	return q;
}

Vector3F vector3fAdd(Vector3F vct1, Vector3F vct2)
{
	vct1.Vector.x += vct2.Vector.x;
	vct1.Vector.y += vct2.Vector.y;
	vct1.Vector.z += vct2.Vector.z;

	return vct1;
}

EulerAnglesStruct getEulerAngles(QuaternionStruct quaternion)
{
	EulerAnglesStruct eulerAnglesStruct;
	eulerAnglesStruct.Angles.roll = radiansToDegrees(atan2(2.0f * (quaternion.Quaternion.y * quaternion.Quaternion.z - quaternion.Quaternion.w * quaternion.Quaternion.x), 2.0f * quaternion.Quaternion.w * quaternion.Quaternion.w - 1.0f + 2.0f * quaternion.Quaternion.z * quaternion.Quaternion.z));
	eulerAnglesStruct.Angles.pitch = radiansToDegrees(-atan((2.0f * (quaternion.Quaternion.x * quaternion.Quaternion.z + quaternion.Quaternion.w * quaternion.Quaternion.y)) / sqrt(1.0f - pow((2.0f * quaternion.Quaternion.x * quaternion.Quaternion.z + 2.0f * quaternion.Quaternion.w * quaternion.Quaternion.y), 2.0f))));
	eulerAnglesStruct.Angles.yaw = radiansToDegrees(atan2(2.0f * (quaternion.Quaternion.x * quaternion.Quaternion.y - quaternion.Quaternion.w * quaternion.Quaternion.z), 2.0f * quaternion.Quaternion.w * quaternion.Quaternion.w - 1.0f + 2.0f * quaternion.Quaternion.x * quaternion.Quaternion.x));
	return eulerAnglesStruct;
}

float radiansToDegrees(float radians)
{
	return 57.2957795130823f * radians;
}

float degreesToRadians(float degrees)
{
	return degrees / 57.2957795130823f;
}

QuaternionStruct getLastPosition(void)
{
	return Quaternion;
}
