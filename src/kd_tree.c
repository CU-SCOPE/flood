#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "stl.h"
#include "kd_tree.h"

int floatcomp(const void* elem1, const void* elem2) {
    if(*(const float*)elem1 < *(const float*)elem2)
        return -1;
    return *(const float*)elem1 > *(const float*)elem2;
}

void buildTree(node **current, node *parent, face *faces, uint32_t numFaces) {
	float points[numFaces*3], median;
	uint8_t dim, i;
	uint32_t numPoints= numFaces*3;

	if(!parent) {
		dim = 0;
	} else {
		dim = (parent->dim + 1) % 3;
	}

	for(i=0; i<numPoints; i++) {
		switch(dim){
			case 0:
				points[i*3] = faces[i].v1.x;
				points[i*3+1] = faces[i].v2.x;
				points[i*3+2] = faces[i].v3.x;
			case 1:
				points[i*3] = faces[i].v1.y;
				points[i*3+1] = faces[i].v2.y;
				points[i*3+2] = faces[i].v3.y;
			case 2:
				points[i*3] = faces[i].v1.z;
				points[i*3+1] = faces[i].v2.z;
				points[i*3+2] = faces[i].v3.z;
		}
	}

	qsort(points, numFaces*3, sizeof(float), floatcomp);
	median = points[numPoints/2];
}