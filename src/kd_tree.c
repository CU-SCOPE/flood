#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "stl.h"
#include "kd_tree.h"
#include "vec_math.h"

int floatcomp(const void* elem1, const void* elem2) {
    if(*(const float*)elem1 < *(const float*)elem2)
        return -1;
    return *(const float*)elem1 > *(const float*)elem2;
}


void buildTree(node **current, node *parent, face *faces, uint32_t numFaces, uint8_t level, uint16_t ind) {
	/*
	buildTree - Recursive function to construct k-d tree from a 3-D model
	Inputs:
		current		- current node being cunstructed
		parent		- parent node to current node. Null if current is root of tree
		faces 		- list of all faces that fall into the bin described by the current node
		numFaces	- number of faces in current bin
		level		- level of tree current node falls in
		ind 		- index of current node
	*/
	float points[numFaces*3], temp[numFaces*3], median;
	uint8_t dim;
	uint32_t numPoints= numFaces*3, numLeft = 0, numRight = 0, i;
	dim = level % 3;

	// Get location of all vertices in current dimension
	switch(dim){
		case 0:
			for(i=0; i<numFaces; i++) {
				points[i*3] = faces[i].v1.x;
				points[i*3+1] = faces[i].v2.x;
				points[i*3+2] = faces[i].v3.x;
				temp[i*3] = faces[i].v1.x;
				temp[i*3+1] = faces[i].v2.x;
				temp[i*3+2] = faces[i].v3.x;
			}
			break;
		case 1:
			for(i=0; i<numFaces; i++) {
				points[i*3] = faces[i].v1.y;
				points[i*3+1] = faces[i].v2.y;
				points[i*3+2] = faces[i].v3.y;
				temp[i*3] = faces[i].v1.y;
				temp[i*3+1] = faces[i].v2.y;
				temp[i*3+2] = faces[i].v3.y;
			}
			break;
		case 2:
			for(i=0; i<numFaces; i++) {
				points[i*3] = faces[i].v1.z;
				points[i*3+1] = faces[i].v2.z;
				points[i*3+2] = faces[i].v3.z;
				temp[i*3] = faces[i].v1.z;
				temp[i*3+1] = faces[i].v2.z;
				temp[i*3+2] = faces[i].v3.z;
			}
			break;
	}
	// Sort all points to split on median
	qsort(points, numFaces*3, sizeof(float), floatcomp);
	median = points[numPoints/2];
	// Create node
	(*current) = malloc(sizeof(node));
	(*current)->val = median;
	(*current)->parent = parent;
	(*current)->dim = dim;
	(*current)->ind = ind;
	face tmpLeft[numFaces], tmpRight[numFaces];
	// Seperate all faces into two bins created by node
	for(i=0; i<numFaces; i++) {
		if((temp[i*3] < median) && (temp[i*3+1] < median) && (temp[i*3+2] < median)) {
			tmpLeft[numLeft] = faces[i];
			numLeft++;
		} else if((temp[i*3] >= median) && (temp[i*3+1] >= median) && (temp[i*3+2] >= median)) {
			tmpRight[numRight] = faces[i];
			numRight++;
		} else {
			tmpLeft[numLeft] = faces[i];
			numLeft++;
			tmpRight[numRight] = faces[i];
			numRight++;
		}
	}
	// Point node bins to the temporary separated bins
	(*current)->binLeft = malloc(sizeof(face)*numLeft);
	(*current)->binRight = malloc(sizeof(face)*numRight);
	memcpy((*current)->binLeft, tmpLeft, sizeof(face)*numLeft);
	memcpy((*current)->binRight, tmpRight, sizeof(face)*numRight);
	(*current)->numLeft = numLeft;
	(*current)->numRight = numRight;
	
	// Set parent child pointers
	if(level) {
		if(ind % 2) {
			parent->lChild = (*current);
		} else {

			parent->rChild = (*current);
		}
	}

	// Recursively call buildTree and delete bins if node is not a leaf node
	if(level < KD_DEPTH-1) {
		buildTree(&((*current)->lChild), (*current), (*current)->binLeft, numLeft, level+1, 2*ind+1);
		buildTree(&((*current)->rChild), (*current), (*current)->binRight, numRight, level+1, 2*ind+2);
		free((*current)->binLeft);
		free((*current)->binRight);
	} else {
		(*current)->lChild = NULL;
		(*current)->rChild = NULL;
	}
}

node *initTree(face *faces, uint32_t numFaces) {
	/*
	initTree - driver function start building tree
	*/
	node *root;
	buildTree(&root, NULL, faces, numFaces, 0, 0);
	return root;
}

void checkFaces(face *faces, uint32_t numFaces, float *query, float *closestPt, float *dist) {
	/*
	checkFaces - Function in search to check all bins in current leaf node
	Inputs:
		faces 		- Bin of faces to check for node
		numFaces	- Number of faces in bin
		query		- 3-D query point
		closestPts 	- Current closest 3-D point
		dist 		- Current minimum distance of closest point
	*/
	uint32_t i;
	float tmpPt[3], tmpDist, curDist = *dist;
	for(i=0; i<numFaces; i++) {
		triangleDist(faces[i], query, &tmpDist, tmpPt);
		if(tmpDist < curDist) {
			curDist = tmpDist;
			memcpy(closestPt, tmpPt, sizeof(float)*3);
		}
	}
	*dist = curDist;
}

node *traverse(node *root, float *query, float *closestPt, float *dist){
	/*
	traverse - Function to traverse down a subtree
	Inputs:
		root		- root node of subtree
		query		- 3-D query point
		closestPt 	- Current closest 3-D point
		dist 		- Current minimum distance of closestPt
	*/
	node *current = root;
	face *faceBin;
	uint32_t numFaces;
	bool left;
	while(current->rChild) {
		if(query[current->dim] < current->val) {
			current = current->lChild;
		} else {
			current = current->rChild;
		}
	}
	if(query[current->dim] < current->val) {
		faceBin = current->binLeft;
		numFaces = current->numLeft;
		left = true;
	} else {
		faceBin = current->binRight;
		numFaces = current->numRight;
		left = false;
	}
	checkFaces(faceBin, numFaces, query, closestPt, dist);
	float plneDist = query[current->dim] - current->val;
	plneDist *= plneDist;
	if(plneDist < *dist) {
		if(left) {
			faceBin = current->binRight;
			numFaces = current->numRight;
		} else {
			faceBin = current->binLeft;
			numFaces = current->numLeft;
		}
		checkFaces(faceBin, numFaces, query, closestPt, dist);
	}
	return current;
}

void kd_search(float *query, float *closestPt, float *dist, node *root) {
	/*
	kd_search - Function to search through kd-tree for the closest point on the model to an arbitrary query point
	Inputs:
		query	- 3-D query point
	*/
	node *current = root;
	uint16_t checked[KD_DEPTH] = {0}, counter = 0;
	bool goLeft;
	float plneDist;
	current = traverse(current, query, closestPt, dist);
	checked[0] = current->ind;
	checked[KD_DEPTH-1] = -1;
	do {
		goLeft = current == current->rChild;
		current = current->parent;
		counter++;
		if(checked[counter] != current->ind) {
			plneDist = query[current->dim] - current->val;
			plneDist *= plneDist;
			if(plneDist < *dist) {
				checked[counter] = current->ind;
				counter = 0;
				if(goLeft) {
					current = current->rChild;
				} else {
					current = current->lChild;
				}
				traverse(current, query, closestPt, dist);
			}
		}
	} while(current->parent);
	*dist = sqrt((*dist));
}

float runSearch(point4D *points, point4D *closestPts, float *minDists, node *root, uint32_t numPts) {
	uint32_t i;
	float mean;
	for(i=0; i<numPts; i++) {
		minDists[i] = 100.0;
		closestPts[i].point[3] = 1.0;
		kd_search(points[i].point, closestPts[i].point, &(minDists[i]), root);
		mean += minDists[i];
	}
	mean /= numPts;
	return mean;
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
