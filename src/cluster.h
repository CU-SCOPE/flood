#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include <iostream>
#include "kd_tree.h"

#define CLUSTER_THRESH		0.08

std::vector<point4D> hcluster(std::vector<point4D> v);
static inline float calc_dist(point4D a, point4D b);

#endif