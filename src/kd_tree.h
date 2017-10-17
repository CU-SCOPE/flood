#ifndef KD_TREE_H
#define KD_TREE_H

#include <stdint.h>
#include "stl.h"

// Define Macros
#define KD_DEPTH     6
#define KD_NODES     (1 << KD_DEPTH) - 1
#define KD_BINS      KD_NODES + 1

typedef struct node{
	float level;
	uint16_t numLeft;
	uint16_t numRight;
	uint8_t level;
	face *binLeft;
	face *binRight;
	node *parent;
	node *lChild;
	node *rChild;
}node;

void buildTree();

#endif