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
	float T[4][4] = {0.0};
	eye4D(T);
	T[2][3] = 9.9;
	point4D scan[MAX_POINTS];
	
	uint32_t numFaces = loadSTL(&faces);
	node *root = initTree(faces, numFaces);

	FILE *f = fopen("trajectory1_processed/test_0.csv","r");
	uint32_t numPts = readFrame(scan, f);
	icp(scan, root, T, numPts);

	quat q;
	trans2quat(T, &q);
	printQuat(q);


	freeModel(faces);
	deleteTree(root,0);
	fclose(f);
}