#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "kd_tree.h"
#include "svd3.h"
#include "vec_math.h"
#include "icp.h"

void transform(float T[4][4], point4D *points, uint32_t numPts) {
	uint32_t i;
	point4D tmp[numPts];
	memcpy(tmp, points, numPts*sizeof(point4D));
	for(i=0; i<numPts; i++) {
		matMulVec4D(T, tmp[i].point, points[i].point);
	}
}

void printMat3(float a11, float a12, float a13, float a21, float a22, float a23, float a31, float a32, float a33) {
    printf("%f %f %f \n", a11, a12, a13);
    printf("%f %f %f \n", a21, a22, a23);
    printf("%f %f %f \n", a31, a32, a33);
}

static inline void meanBoth(point4D *points1, point4D *points2, point3D *mean1, point3D *mean2, uint32_t numPts) {
	uint32_t i;
	for(i=0; i<numPts; i++) {
		mean1->point[0] += points1[i].point[0];
		mean1->point[1] += points1[i].point[1];
		mean1->point[2] += points1[i].point[2];
		mean2->point[0] += points2[i].point[0];
		mean2->point[1] += points2[i].point[1];
		mean2->point[2] += points2[i].point[2];
	}
	mean1->point[0] /= numPts;
	mean1->point[1] /= numPts;
	mean1->point[2] /= numPts;
	mean2->point[0] /= numPts;
	mean2->point[1] /= numPts;
	mean2->point[2] /= numPts;
}



void calcTransform(point4D *scan, point4D *model,float T[4][4], uint32_t numPts) {
	uint32_t i;
	float W[3][3] = {0};
	point3D centScan, centModel;
	point4D tempScan[numPts], tempModel[numPts];
	meanBoth(scan, model, &centScan, &centModel, numPts);
	
	for(i=0; i<numPts; i++) {
		tempScan[i].point[0] = scan[i].point[0] - centScan.point[0];
		tempScan[i].point[1] = scan[i].point[1] - centScan.point[1];
		tempScan[i].point[2] = scan[i].point[2] - centScan.point[2];
		tempModel[i].point[0] = model[i].point[0] - centModel.point[0];
		tempModel[i].point[1] = model[i].point[1] - centModel.point[1];
		tempModel[i].point[2] = model[i].point[2] - centModel.point[2];
	}

	float U[3][3], V[3][3], UT[3][3], R[3][3], t[3], newCent[3];
	getMat(tempScan, tempModel, W, numPts);
	runSVD(W, U, V);
	transpose(U, UT);
	matMul3D(V, UT, R);
	matMulVec3D(R, centScan.point, newCent);
	sub3D(centModel.point, newCent, t);
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
		calcTransform(scan, closestPts, T, numPts);
		transform(T, scan, numPts);
		meanVec(minDists, &error, numPts);
		printf("%f\n", error);
		if(error <= 0.001) {
			break;
		}
	}
	calcTransform(initState, scan,T, numPts);
}