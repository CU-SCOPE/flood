#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef __cplusplus
extern "C" {
#include <stdio.h>
#endif

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
	float w2 = w*w;
	float x2 = x*x;
	float y2 = y*y;
	float z2 = z*z;
	float xy = x*y*2;
	float xz = x*z*2;
	float yz = y*z*2;
	float wx = x*w*2;
	float wy = y*w*2;
	float wz = z*w*2;
	T[0][0] = w2 + x2 - y2 - z2;
	T[0][1] = xy - wz;
	T[0][2] = xz + wy;
	T[1][0] = xy + wz;
	T[1][1] = w2 - x2 + y2 - z2;
	T[1][2] = yz - wx;
	T[2][0] = xz - wy;
	T[2][1] = wx + yz;
	T[2][2] = w2 - x2 - y2 + z2;
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

static inline void printTrans(float T[4][4]) {
	quat q;
	trans2quat(T, &q);
	printQuat(q);
	printf("tX: %f tY: %f tZ: %f\n", T[0][3], T[1][3], T[2][3]);
}

#ifdef __cplusplus
}
#endif

#endif