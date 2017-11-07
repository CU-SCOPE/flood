#include <stdio.h>
#include <stdlib.h>
#include "kd_tree.h"
#include "stl.h"
#include "frames.h"
#include "icp.h"
#include "vec_math.h"
#include "quaternion.h"


void main() {
	face *faces;
	float T[4][4] = {0.0}, closestPt[3], dist=100;
	eye4D(T);
	T[2][3] = 10;
	point4D scan[MAX_POINTS];
	
	uint32_t numFaces = loadSTL(&faces);
	node *root = initTree(faces, numFaces);
	FILE *f = fopen("trajectory1/trajectory_noisy00001.pcd","r");
	uint32_t numPts = readFrame(scan, f);
	scan[1].point[2] += 10;
	icp(scan, root, T, numPts);
	quat q;
	trans2quat(T, &q);
	printQuat(q);


	freeModel(faces);
	deleteTree(root,0);
	fclose(f);
}