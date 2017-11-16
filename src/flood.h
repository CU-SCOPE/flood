#ifndef FLOOD_H
#define FLOOD_H

#include <stdint.h>
#include <stdbool.h>
#include <string>
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include "stl.h"

#if DEBUG
#define NUM_FILES			1
#else
#define NUM_FILES			60
#endif
#define FRAME_DIRECTORIES	"trajectories.txt"

class FLOOD {
public:
	FLOOD();
	~FLOOD();
	void run();
	void initializePose(quat q, float t[4]);
private:
	void getFrame(uint8_t fileNum, std::string dir);
	node *root;
	face *faces;
	point4D scan[MAX_POINTS];
	float T[4][4] = {{0}};
	uint32_t numPts;
	uint32_t numFaces;
	bool finding;
};

#endif