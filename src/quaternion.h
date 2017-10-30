#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

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

static inline void printQuat(quat q) {
	printf("qW: %f qX: %f qY: %f qZ: %f\n", q.w, q.x, q.y, q.z);
}

#endif