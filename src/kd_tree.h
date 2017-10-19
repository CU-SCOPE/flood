#ifndef KD_TREE_H
#define KD_TREE_H

#include <stdint.h>
#include "stl.h"

// Define Macros
#define KD_DEPTH     6
#define KD_NODES     (1 << KD_DEPTH) - 1
#define KD_BINS      KD_NODES + 1

typedef struct node{
	float val;
	uint16_t numLeft;
	uint16_t numRight;
	uint8_t dim;
	face *binLeft;
	face *binRight;
	struct node *parent;
	struct node *lChild;
	struct node *rChild;
}node;

void buildTree(node **root, node *parent, face *faces, uint32_t numFaces, uint8_t level);
node *initTree(face *faces, uint32_t numFaces);
void deleteTree(node *root, uint8_t counter);

#endif