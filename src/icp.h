#ifndef VEC_MATH_H
#define VEC_MATH_H

#include "kd_tree.h"

#define MAX_ITERATIONS		5

void icp(float **scan, node *root, uint32_t numPts);

#endif