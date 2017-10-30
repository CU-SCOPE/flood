#include <stdio.h>
#include <stdlib.h>
#include "kd_tree.h"
#include "stl.h"
#include "frames.h"
#include "icp.h"
#include "vec_math.h"


void main() {
	face *faces;
	float T[4][4] = {0};
	eye4D(T);
	T[2][3] = 10;
	point4D scan[MAX_POINTS];
	int i;
	
	uint32_t numFaces = loadSTL(&faces);
	node *root = initTree(faces, numFaces);

	FILE *f = fopen("trajectory1_processed/trajectory_1.csv","r");
	uint32_t numPts = readFrame(scan, f);
	icp(scan, root, T, numPts);


	freeModel(faces);
	deleteTree(root,0);
}