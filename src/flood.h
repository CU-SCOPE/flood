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
#define NUM_FILES			1
#endif
#define FRAME_DIRECTORIES	"test/"
#define THRESH				0.04

class FLOOD {
public:
	FLOOD();
	~FLOOD();
	void run();
	void initializePose(quat q, float t[4]);
private:
	void calcPose();
	void getFrame();
	void getPosition(std::string dir);
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