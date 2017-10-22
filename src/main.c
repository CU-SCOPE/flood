#include <stdio.h>
#include <stdlib.h>
#include "kd_tree.h"
#include "stl.h"


void main() {
	face *faces;
	float dist = 100;
	float closestPt[3];
	float query[3];
	int i;
	uint32_t numFaces = loadSTL(&faces);
	node *root = initTree(faces, numFaces);
	for(i=0; i<numFaces; i++) {
		query[0] = faces[i].v1.x;
		query[1] = faces[i].v1.y;
		query[2] = faces[i].v1.z;
		kd_search(query, closestPt, &dist, root);
	}
	printf("%f\n", dist);
	freeModel(faces);
	deleteTree(root,0);
}