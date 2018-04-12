#ifndef FLOOD_H
#define FLOOD_H


#include <stdbool.h>
#include <string>
#include <thread>
#include <semaphore.h>
#include <atomic>
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include "cluster.h"
#include "stl.h"
#include "image.h"
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"

#if DEBUG
#define NUM_FILES			10
#else
#define NUM_FILES			179
#endif
#define FRAME_DIRECTORIES	"test/"
#define THRESH				0.025

class FLOOD {
public:
	FLOOD();
	~FLOOD();
	void run();
	void initializePose(quat qInit, float t[4], float Temp[4][4]);
	void getPosition(float position);
private:
	void calcPose();
	void getFrame();
#if RENDER
	void render();
#endif
	void printQuat(quat q, FILE *f);
	void printTrans(float T[4][4], float translation[3], FILE *pos, FILE *rot);
	node *root;
	face *faces;
	point4D scan[MAX_POINTS];
	float T[4][4] = {{0}};
	quat q, rotx, roty, rotz;
	float translation[4];
	std::atomic<int> numPts;
	unsigned int numFaces;
	bool finding;
	pthread_mutex_t lock, sa_lock;
	sem_t frame1;
	std::atomic<bool> done, exit;
	float shared_array[7];
};

#endif
