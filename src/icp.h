#ifndef ICP_H
#define ICP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kd_tree.h"

#define MAX_ITERATIONS_FIND		10
#define MAX_ITERATIONS_KNOWN    2

void icp(point4D *scan, node *root, float T[4][4], uint32_t numPts, uint8_t iterations);

#ifdef __cplusplus
}
#endif

#endif