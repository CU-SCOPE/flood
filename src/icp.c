#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "kd_tree.h"
#include "svd.h"
#include "vec_math.h"

void transform(float T[4][4], point4D *points, uint32_t numPts) {
	uint32_t i;
	float tmp[numPts];
	memcpy(tmp, points, 4*numPts*sizeof(float));
	for(i=0; i<numPts; i++) {
		matMulVec4D(mat, tmp, points[i].point);
	}
}

void icp(point4D *scan, node *root, float initPose[4][4], uint32_t numPts) {
	uint8_t i;
	transform(initPose, scan, numPts);
	float minDists[numPts];
	point3D closestPts[numPts];
	runSearch(scan, closestPts, minDists, root, numPts)
}