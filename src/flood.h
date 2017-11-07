#ifndef FLOOD_H
#define FLOOD_H

#include <stdint.h>
#include <stdbool.h>
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include "stl.h"


#define NUM_FILES			60
#define FRAME_DIRECTORY		"trajectory1/"

class FLOOD {
public:
	FLOOD();
	~FLOOD();
	void run();
	void initializePose(quat q, float t[4]);
private:
	void getFrame(uint8_t fileNum);
	node *root;
	face *faces;
	point4D scan[MAX_POINTS];
	float T[4][4];
	uint32_t numPts;
	uint32_t numFaces;
	bool finding;
};

#endif