#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "kd_tree.h"
#include "svd.h"
#include "vec_math.h"

void transform(float T[4][4], point4D *points, uint32_t numPts) {
	uint32_t i;
	point4D tmp[numPts];
	memcpy(tmp, points, numPts*sizeof(point4D));
	for(i=0; i<numPts; i++) {
		matMulVec4D(mat, tmp[i].point, points[i].point);
	}
}

static inline void meanBoth(point4D *points1, point4D *points2, point3D *mean1, point3D *mean2, uint32_t numPts) {
	uint32_t i;
	for(i=0; i<numPts; i++) {
		mean1[0] += points1[i].point[0];
		mean1[1] += points1[i].point[1];
		mean1[2] += points1[i].point[2];
		mean2[0] += points2[i].point[0];
		mean2[1] += points2[i].point[1];
		mean2[2] += points2[i].point[2];
	}
	mean1[0] /= numPts;
	mean1[1] /= numPts;
	mean1[2] /= numPts;
	mean2[0] /= numPts;
	mean2[1] /= numPts;
	mean2[2] /= numPts;
	return mean;
}

static inline void getMat(point4D *scan, point4D *model, float W[3][3], uint32_t numPts) {
	uint32_t i;
	for(i=0; i<numPts; i++) {
		W[0][0] += scan[i].point4D[0] * model[i].point[0];
		W[0][1] += scan[i].point4D[0] * model[i].point[1];
		W[0][2] += scan[i].point4D[0] * model[i].point[2];
		W[1][0] += scan[i].point4D[1] * model[i].point[0];
		W[1][1] += scan[i].point4D[1] * model[i].point[1];
		W[1][2] += scan[i].point4D[1] * model[i].point[2];
		W[2][0] += scan[i].point4D[2] * model[i].point[0];
		W[2][1] += scan[i].point4D[2] * model[i].point[1];
		W[2][2] += scan[i].point4D[2] * model[i].point[2];
	}
}

static inline void runSVD(float W[3][3], float U[3][3], float V[3][3]) {
	float 	s11, s12, s13, 
			s21, s22, s23, 
			s31, s32, s33;

	svd(W[0][0], W[0][1], W[0][2], W[1][0], W[1][1], W[1][2], W[2][0], W[2][1], W[2][2],
	    U[0][0], U[0][1], U[0][2], U[1][0], U[1][1], U[1][2], U[2][0], U[2][1], U[2][2],
	    s11, s12, s13, s21, s22, s23, s31, s32, s33,
	    V[0][0], V[0][1], V[0][2], V[1][0], V[1][1], V[1][2], V[2][0], V[2][1], V[2][2],);
}

void calcTransform(point4D *scan, point4D *model,float T[4][4], uint32_t numPts) {
	uint32_t i;
	float W[3][3] = {0};
	point3D centScan, centModel;
	point4D tempScan[numPts], tempModel[numPts];
	meanBoth(scan, model, &centScan, &centModel, numPts);
	
	for(i=0; i<numPts; i++) {
		tempScan[0] = scan[0] - centScan[0];
		tempScan[1] = scan[1] - centScan[1];
		tempScan[2] = scan[2] - centScan[2];
		tempModel[0] = model[0] - centModel[0];
		tempModel[1] = model[1] - centModel[1];
		tempModel[2] = model[2] - centModel[2];
	}

	float U[3][3], V[3][3], UT[3][3], VT[3][3], R[3][3], t[3];
	getMat(tempScan, tempModel, W, numPts);
	runSVD(W, U, V);
	transpose(V, VT);
	laderman_mul(U, VT, R);
	matMulVec3D(R, centScan, t);
	T[0][0] = R[0][0];
	T[0][1] = R[0][1];
	T[0][2] = R[0][2];
	T[1][0] = R[1][0];
	T[1][1] = R[1][1];
	T[1][2] = R[1][2];
	T[2][0] = R[2][0];
	T[2][1] = R[2][1];
	T[2][2] = R[2][2];
	T[0][3] = t[0];
	T[1][3] = t[1];
	T[2][3] = t[2];
}

void icp(point4D *scan, node *root, float T[4][4], uint32_t numPts) {
	uint8_t i;
	point4D initState[numPts];
	memcpy(initState, scan, numPts*sizeof(point4D));
	transform(T, scan, numPts);
	float minDists[numPts], error;
	point4D closestPts[numPts];
	for(i=0; i<MAX_ITERATIONS; i++) {
		error = 0;
		runSearch(scan, closestPts, minDists, root, numPts);
		calcTransform(scan, closestPts,T, numPts);
		transform(T, scan, numPts);
		meanVec(minDists, &error, numPts);
		printf("%f\n", error);
		if(error <= 0.001) {
			break;
		}
	}
	calcTransform(initState, scan,T, numPts);
}