#ifndef ICP_H
#define ICP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kd_tree.h"

#define MAX_ITERATIONS_FIND		20
#define MAX_ITERATIONS_KNOWN    5
#define NUM_STANDARD_DEVS		1

float icp(point4D *scan, node *root, float T[4][4], uint32_t numPts, uint8_t iterations);

#ifdef __cplusplus
}
#endif

#endif