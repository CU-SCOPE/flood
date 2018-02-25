#ifndef ICP_H
#define ICP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kd_tree.h"

#define MAX_ITERATIONS_FIND		5
#define MAX_ITERATIONS_KNOWN    5
#define NUM_STANDARD_DEVS		2

float icp(point4D *scan, node *root, float T[4][4], unsigned int numPts, unsigned int iterations);

#ifdef __cplusplus
}
#endif

#endif