#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef __cplusplus
extern "C" {
#include <stdio.h>
#endif

#include <math.h>
#include "vec_math.h"

#define TO_FILE		0

typedef struct quat {
	float w;
	float x;
	float y;
	float z;
}quat;

static inline void normalizeQuat(quat *q) {
	float W = q->w;
	float X = q->x;
	float Y = q->y;
	float Z = q->z;

	float val = 1/sqrtf(X*X + Y*Y + Z*Z + W*W);
	q->w *= val;
	q->x *= val;
	q->y *= val;
	q->z *= val;
}

static inline void trans2quat(float T[4][4], quat *q) {

	/*
	Taken from Irrlicht quaternion class
	*/
	float diag = T[0][0] + T[1][1] + T[2][2] + 1;

	if( diag > 0.0f )
	{
		const float scale = sqrtf(diag) * 2.0f; // get scale from diagonal

		// TODO: speed this up
		q->x = (T[1][2] - T[2][1]) / scale;
		q->y = (T[2][0] - T[0][2]) / scale;
		q->z = (T[0][1] - T[1][0]) / scale;
		q->w = 0.25f * scale;
	}
	else
	{
		if (T[0][0]>T[1][1] && T[0][0]>T[2][2])
		{
			// 1st element of diag is greatest value
			// find scale according to 1st element, and double it
			const float scale = sqrtf(1.0f + T[0][0] - T[1][1] - T[2][2]) * 2.0f;

			// TODO: speed this up
			q->x = 0.25f * scale;
			q->y = (T[1][0] + T[0][1]) / scale;
			q->z = (T[0][2] + T[2][0]) / scale;
			q->w = (T[1][2] - T[2][1]) / scale;
		}
		else if (T[1][1]>T[2][2])
		{
			// 2nd element of diag is greatest value
			// find scale according to 2nd element, and double it
			const float scale = sqrtf(1.0f + T[1][1] - T[0][0] - T[2][2]) * 2.0f;

			// TODO: speed this up
			q->x = (T[1][0] + T[0][1]) / scale;
			q->y = 0.25f * scale;
			q->z = (T[2][1] + T[1][2]) / scale;
			q->w = (T[2][0] - T[0][2]) / scale;
		}
		else
		{
			// 3rd element of diag is greatest value
			// find scale according to 3rd element, and double it
			const float scale = sqrtf(1.0f + T[2][2] - T[0][0] - T[1][1]) * 2.0f;

			// TODO: speed this up
			q->x = (T[2][0] + T[0][2]) / scale;
			q->y = (T[2][1] + T[1][2]) / scale;
			q->z = 0.25f * scale;
			q->w = (T[0][1] - T[1][0]) / scale;
		}
	}
	normalizeQuat(q);
}

static inline quat multQuat(quat q1, quat q2) {
	quat result;
	result.w = (q2.w * q1.w) - (q2.x * q1.x) - (q2.y * q1.y) - (q2.z * q1.z);
	result.x = (q2.w * q1.x) + (q2.x * q1.w) + (q2.y * q1.z) - (q2.z * q1.y);
	result.y = (q2.w * q1.y) + (q2.y * q1.w) + (q2.z * q1.x) - (q2.x * q1.z);
	result.z = (q2.w * q1.z) + (q2.z * q1.w) + (q2.x * q1.y) - (q2.y * q1.x);
	return result;
}

static inline void quat2trans(float T[4][4], quat q, float t[4]) {
	float W = q.w;
	float X = q.x;
	float Y = q.y;
	float Z = q.z;
	
	T[0][0] = 1.0f - 2.0f*Y*Y - 2.0f*Z*Z;;
	T[0][1] = 2.0f*X*Y + 2.0f*Z*W;
	T[0][2] = 2.0f*X*Z - 2.0f*Y*W;
	T[0][3] = 0.0;

	T[1][0] = 2.0f*X*Y - 2.0f*Z*W;
	T[1][1] = 1.0f - 2.0f*X*X - 2.0f*Z*Z;
	T[1][2] = 2.0f*Z*Y + 2.0f*X*W;
	T[1][3] = 0.0;

	T[2][0] = 2.0f*X*Z + 2.0f*Y*W;
	T[2][1] = 2.0f*Z*Y - 2.0f*X*W;
	T[2][2] = 1.0f - 2.0f*X*X - 2.0f*Y*Y;
	T[2][3] = 0.0;

	float tRot[4];
	matMulVec4D(T, t, tRot);
	T[0][3] = tRot[0];
	T[1][3] = tRot[1];
	T[2][3] = tRot[2];
	T[3][3] = 1;
}


#if TO_FILE
static inline void printQuat(quat q, FILE *f) {
	fprintf(f, "%f  %f  %f  %f\n", q.w, q.x, q.y, q.z);
}

static inline void printTrans(float T[4][4], float translation[3], FILE *pos, FILE *rot) {
	quat q;
	trans2quat(T, &q);
	printQuat(q, rot);
	fprintf(pos, "%f  %f  %f\n", translation[0], translation[1], translation[2]);
}
#else
static inline void printQuat(quat q) {
	printf("qW: %f qX: %f qY: %f qZ: %f\n", q.w, q.x, q.y, q.z);
}

static inline void printTrans(float T[4][4], float translation[3]) {
	quat q;
	trans2quat(T, &q);
	printQuat(q);
	printf("tX: %f tY: %f tZ: %f\n", translation[0], translation[1], translation[2]);
}
#endif

#ifdef __cplusplus
}
#endif

#endif