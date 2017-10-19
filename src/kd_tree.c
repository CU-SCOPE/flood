#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "stl.h"
#include "kd_tree.h"

int floatcomp(const void* elem1, const void* elem2) {
    if(*(const float*)elem1 < *(const float*)elem2)
        return -1;
    return *(const float*)elem1 > *(const float*)elem2;
}


void buildTree(node **current, node *parent, face *faces, uint32_t numFaces, uint8_t level) {
	float points[numFaces*3], median;
	uint8_t dim;
	uint32_t numPoints= numFaces*3, numLeft = 0, numRight = 0, i;

	dim = level % 3;
	for(i=0; i<numFaces; i++) {
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
	(*current) = malloc(sizeof(node));
	// Create node
	(*current)->val = median;
	(*current)->parent = parent;
	(*current)->dim = dim;
	face tmpLeft[numFaces], tmpRight[numFaces];
	for(i=0; i<numFaces; i++) {
		if((points[i*3] < median) && (points[i*3+1] < median) && (points[i*3+2] < median)) {
			tmpLeft[numLeft] = faces[i];
			numLeft++;
		}else if((points[i*3] > median) && (points[i*3+1] > median) && (points[i*3+2] > median)) {
			tmpRight[numRight] = faces[i];
			numRight++;
		} else {
			tmpLeft[numLeft] = faces[i];
			numLeft++;
			tmpRight[numRight] = faces[i];
			numRight++;
		}
	}
	(*current)->binLeft = malloc(sizeof(face)*numLeft);
	(*current)->binRight = malloc(sizeof(face)*numRight);
	memcpy((*current)->binLeft, tmpLeft, sizeof(face)*numLeft);
	memcpy((*current)->binRight, tmpRight, sizeof(face)*numRight);
	(*current)->numLeft = numLeft;
	(*current)->numRight = numRight;

	if(level < KD_DEPTH-1) {
		buildTree(&((*current)->lChild), (*current), (*current)->binLeft, numLeft, level+1);
		buildTree(&((*current)->rChild), (*current), (*current)->binRight, numRight, level+1);
		free((*current)->binLeft);
		free((*current)->binRight);
	}
}

node *initTree(face *faces, uint32_t numFaces) {
	node *root;
	buildTree(&root, NULL, faces, numFaces, 0);
	return root;
}

void deleteTree(node *root, uint8_t counter) {
	if(counter < KD_DEPTH-1) {
		deleteTree(root->lChild, counter+1);
		deleteTree(root->rChild, counter+1);
	} else {
		free(root->binLeft);
		free(root->binRight);
	}
	free(root);
}
