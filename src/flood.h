#ifndef FLOOD_H
#define FLOOD_H


#include <stdbool.h>
#include <string>
#include <thread>
#include <atomic>
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include "stl.h"
#include "image.h"
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"

#if DEBUG
#define NUM_FILES			10
#else
#define NUM_FILES			100
#endif
#define FRAME_DIRECTORIES	"./"
#define THRESH				0.012

class FLOOD {
public:
	FLOOD();
	~FLOOD();
	void run();
	void getPosition(float position);
private:
	void calcPose();
	void getFrame();
	void initializePose(quat q, float t[4], float Temp[4][4]);
	node *root;
	face *faces;
	point4D scan[MAX_POINTS];
	float T[4][4] = {{0}};
	quat q, rotx, roty, rotz;
	float translation[4];
	std::atomic<int> numPts;
	unsigned int numFaces;
	bool finding;
	std::atomic<bool> read;
};

#endif