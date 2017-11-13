#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "kd_tree.h"
#include "vec_math.h"
#include "icp.h"
#include "svdcmp.h"


#if DEBUG
void printMat3(float a[3][3]) {
    printf("%f %f %f \n", a[0][0], a[0][1], a[0][2]);
    printf("%f %f %f \n", a[1][0], a[1][1], a[1][2]);
    printf("%f %f %f \n", a[2][0], a[2][1], a[2][2]);
}

void printMat4(float a[4][4]) {
    printf("%f %f %f %f\n", a[0][0], a[0][1], a[0][2], a[0][3]);
    printf("%f %f %f %f\n", a[1][0], a[1][1], a[1][2], a[1][3]);
    printf("%f %f %f %f\n", a[2][0], a[2][1], a[2][2], a[2][3]);
    printf("%f %f %f %f\n", a[3][0], a[3][1], a[3][2], a[3][3]);
}
#endif

void transform(float T[4][4], point4D *points, uint32_t numPts) {
	/*
	transform - Function to apply current transformation to all points in scan
	*/
	uint32_t i;
	point4D tmp[numPts];
	memcpy(tmp, points, numPts*sizeof(point4D));
	for(i=0; i<numPts; i++) {
		matMulVec4D(T, tmp[i].point, points[i].point);
	}
}

static inline void meanBoth(point4D *points1, point4D *points2, float *mean1, float *mean2, uint32_t numPts) {
	/*
	meanBoth - Function used to calculate the centroids of scan and model
	*/
	uint32_t i;
	float pt1[3] = {0.0};
	float pt2[3] = {0.0};
	for(i=0; i<numPts; i++) {
		pt1[0] += points1[i].point[0];
		pt1[1] += points1[i].point[1];
		pt1[2] += points1[i].point[2];
		pt2[0] += points2[i].point[0];
		pt2[1] += points2[i].point[1];
		pt2[2] += points2[i].point[2];
	}
	pt1[0] /= numPts;
	pt1[1] /= numPts;
	pt1[2] /= numPts;
	pt2[0] /= numPts;
	pt2[1] /= numPts;
	pt2[2] /= numPts;
	memcpy(mean1, pt1, 3*sizeof(float));
	memcpy(mean2, pt2, 3*sizeof(float));
}



void calcTransform(point4D *scan, point4D *model, float T[4][4], uint32_t numPts) {
	/*
	calcTransform - Function to calculate the current optimal transformation to minimize distance
	Inputs:
		scan 	- Input frame from flash lidar
		model 	- Closest points found on model
		T 		- Current transformation matrix
		numPts	- Number of points in scan/model
	*/
	uint32_t i;
	float W[3][3] = {0.0};
	point3D centScan, centModel;
	point4D tempScan[numPts], tempModel[numPts];
	meanBoth(scan, model, centScan.point, centModel.point, numPts);
	for(i=0; i<numPts; i++) {
		tempScan[i].point[0] = scan[i].point[0] - centScan.point[0];
		tempScan[i].point[1] = scan[i].point[1] - centScan.point[1];
		tempScan[i].point[2] = scan[i].point[2] - centScan.point[2];
		tempModel[i].point[0] = model[i].point[0] - centModel.point[0];
		tempModel[i].point[1] = model[i].point[1] - centModel.point[1];
		tempModel[i].point[2] = model[i].point[2] - centModel.point[2];
	}
	float V[3][3], UT[3][3], R[3][3], t[3], newCent[3];
	getMat(tempScan, tempModel, W, numPts);
	svdcmp(W, V);
	transpose(W, UT);
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

void icp(point4D *scan, node *root, float T[4][4], uint32_t numPts, uint8_t iterations) {
	/*
	icp - Function to control iterative closest point calculation
	Inputs:
		scan 		- input frame from flash lidar
		root 		- the root node of the K-D tree
		T	  		- Initial transformation matrix
		numPts	- Number of points in the flash lidar point cloud
	*/
	uint8_t i;
	point4D initState[numPts];
	memcpy(initState, scan, numPts*sizeof(point4D));
	transform(T, scan, numPts);
	float minDists[numPts], error;
	point4D closestPts[numPts];
	for(i=0; i<iterations; i++) {
		error = 0.0;
		runSearch(scan, closestPts, minDists, root, numPts);
		calcTransform(scan, closestPts, T, numPts);
		transform(T, scan, numPts);
#if DEBUG
		meanVec(minDists, &error, numPts);
		printf("%f\n", error);
#endif
	}
	calcTransform(initState, scan, T, numPts);
}