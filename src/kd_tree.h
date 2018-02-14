#ifndef KD_TREE_H
#define KD_TREE_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stl.h"

// Define Macros
#define KD_DEPTH     9
#define KD_NODES     (1 << KD_DEPTH) - 1
#define KD_BINS      KD_NODES + 1
#define MAX_POINTS	 23232

typedef struct node{
	float val;
	unsigned int ind;
	unsigned int numLeft;
	unsigned int numRight;
	unsigned int dim;
	face *binLeft;
	face *binRight;
	struct node *parent;
	struct node *lChild;
	struct node *rChild;
}node;

typedef struct point3D {
	float point[3];
}point3D;

typedef struct point4D {
	float point[4];
}point4D;

node *initTree(face *faces, unsigned int numFaces);
void deleteTree(node *root, unsigned int counter);
void kd_search(float *query, float *closestPt, float *dist, node *root);
float runSearch(point4D *points, point4D *closestPts, float *minDists, node *root, unsigned int numPts);

#ifdef __cplusplus
}
#endif

#endif