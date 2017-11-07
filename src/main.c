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
	printf("%f   %f   %f\n", faces[numFaces-1].v1.x,faces[numFaces-1].v1.y,faces[numFaces-1].v1.z);
	printf("%f   %f   %f\n", faces[numFaces-1].v2.x,faces[numFaces-1].v2.y,faces[numFaces-1].v2.z);
	printf("%f   %f   %f\n", faces[numFaces-1].v3.x,faces[numFaces-1].v3.y,faces[numFaces-1].v3.z);
	FILE *f = fopen("trajectory1/trajectory_noisy00001.pcd","r");
	uint32_t numPts = readFrame(scan, f);
	scan[1].point[2] += 10;
	// kd_search(scan[0].point, closestPt, &dist, root);
	// icp(scan, root, T, numPts);
	triangleDist(faces[0], scan[1].point, &dist, closestPt);
	printf("%f  %f   %f\n", closestPt[0], closestPt[1], closestPt[2]);
	printf("%f\n", dist);
	quat q;
	trans2quat(T, &q);
	printQuat(q);


	freeModel(faces);
	deleteTree(root,0);
	fclose(f);
}