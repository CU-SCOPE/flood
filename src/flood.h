#ifndef FLOOD_H
#define FLOOD_H


#include <stdbool.h>
#include <string>
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include "stl.h"

#if DEBUG
#define NUM_FILES			10
#else
#define NUM_FILES			179
#endif
#define FRAME_DIRECTORIES	"trajectories.txt"
#define THRESH				0.06

class FLOOD {
public:
	FLOOD();
	~FLOOD();
	void run();
	void initializePose(quat q, float t[4]);
private:
	void getFrame(unsigned int fileNum, std::string dir);
	void getPosition(std::string dir);
	node *root;
	face *faces;
	point4D scan[MAX_POINTS];
	float T[4][4] = {{0}};
	quat q;
	float translation[4];
	unsigned int numPts;
	unsigned int numFaces;
	bool finding;
};

#endif