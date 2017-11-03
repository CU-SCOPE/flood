#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "vec_math.h"

typedef struct quat {
	float w;
	float x;
	float y;
	float z;
}quat;

static inline void trans2quat(float T[4][4], quat *q) {
	q->w = sqrt(1 + T[0][0] + T[1][1] + T[2][2])/2.0;
	float w4 = 4 * q->w;
	q->x = (T[2][1] - T[1][2])/w4;
	q->y = (T[0][2] - T[2][0])/w4;
	q->z = (T[1][0] - T[0][1])/w4;
}

static inline void quat2trans(float T[4][4], quat q, float t[4]) {
	float w = q.w;
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float y2 = y*y*2;
	float x2 = x*x*2;
	float z2 = z*z*2;
	float xy = x*y*2;
	float xz = x*z*2;
	float yz = y*z*2;
	float wx = x*w*2;
	float wy = y*w*2;
	float wz = z*w*2;
	T[0][0] = 1 - y2 - z2;
	T[0][1] = xy - wz;
	T[0][2] = xz - wy;
	T[1][0] = xy + wz;
	T[1][1] = 1 - x2 - z2;
	T[1][2] = yz - wx;
	T[2][0] = xz - wy;
	T[2][1] = yz + wx;
	T[2][2] = 1 - x2 - y2;
	T[3][3] = 1;
	float tRot[4];
	matMulVec4D(T, t, tRot);
	T[0][3] = tRot[0];
	T[1][3] = tRot[1];
	T[2][3] = tRot[2];
}

static inline void printQuat(quat q) {
	printf("qW: %f qX: %f qY: %f qZ: %f\n", q.w, q.x, q.y, q.z);
}

#endif