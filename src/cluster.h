#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include <iostream>
#include "kd_tree.h"

#define CLUSTER_THRESH		0.06

/*typedef struct cluster_node{
	point3D point;
	int clusterID;
	cluster_node *next;
	cluster_node *previous;
}cluster_node;

typedef struct cluster{
	cluster_node *root;
	cluster_node *end;
	int numNodes;
}cluster;*/

std::vector<point4D> hcluster(std::vector<point4D> v);
static inline float calc_dist(point4D a, point4D b);

#endif