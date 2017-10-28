#ifndef ICP_H
#define ICP_H

#include "kd_tree.h"

#define MAX_ITERATIONS		5

void icp(point4D *scan, node *root, float T[4][4], uint32_t numPts);

#endif