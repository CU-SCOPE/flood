#ifndef KD_TREE_H
#define KD_TREE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stl.h"

// Define Macros
#define KD_DEPTH     9
#define KD_NODES     (1 << KD_DEPTH) - 1
#define KD_BINS      KD_NODES + 1
#define MAX_POINTS	 5000

typedef struct node{
	float val;
	uint16_t ind;
	uint16_t numLeft;
	uint16_t numRight;
	uint8_t dim;
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

node *initTree(face *faces, uint32_t numFaces);
void deleteTree(node *root, uint8_t counter);
void kd_search(float *query, float *closestPt, float *dist, node *root);
float runSearch(point4D *points, point4D *closestPts, float *minDists, node *root, uint32_t numPts);

#ifdef __cplusplus
}
#endif

#endif