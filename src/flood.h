#ifndef FLOOD_H
#define FLOOD_H

#include <stdint.h>
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"


#define NUM_FILES    60


class FLOOD {
public:
	void run();
private:
	void getFrame();
	void runICP();
	node *root;
	face *faces;
	point4D scan[MAX_POINTS];
	float T[4][4];
	uint32_t numPts;
	uint32_t numFaces;
};

#endif