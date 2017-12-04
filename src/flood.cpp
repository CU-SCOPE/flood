#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdbool.h>
#include <time.h>
#include <fstream>
#include "flood.h"
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include "frames.h"

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	eye4D(T);
	finding = true;
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	freeModel(faces);
	deleteTree(root,0);
};

void FLOOD::run() {
	uint8_t i, j, k, l;
	quat current, temp;
	quat rotx, roty, rotz;
	// 45 degree rotation about each axis
	rotx.w = 0.924; roty.w = 0.924; rotz.w = 0.924;
	rotx.x = 0.383; roty.x = 0.0; rotz.x = 0.0;
	rotx.y = 0.0; roty.y = 0.383; rotz.y = 0.0;
	rotx.z = 0.0; roty.z = 0.0; rotz.z = 0.383;
	clock_t start, end, looking, found;
	float error;
	double time = 0, acq_time;
	std::ifstream file(FRAME_DIRECTORIES);
    std::string dir, pos, rot; 
    while (std::getline(file, dir)) {
#if TO_FILE
    	pos = dir + "position_act.txt";
    	rot = dir + "rotation_act.txt";
    	FILE *fpos = std::fopen(pos.c_str(), "w");
    	FILE *frot = std::fopen(rot.c_str(), "w");
#endif
    	q.w = 1; q.x = 0; q.y = 0; q.z = 0;
    	current = q;
    	printf("%s\n", dir.c_str());
    	getPosition(dir);
		for(i=1; i<=NUM_FILES; i++) {
			getFrame(i, dir);
			if(!finding) {
				start = clock();
				error = icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
				end = clock();
				time += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
			}
			else {
				looking = clock();
				point4D initState[numPts];
				memcpy(initState, scan, numPts*sizeof(point4D));
				error = icp(initState, root, T, numPts, MAX_ITERATIONS_FIND);
				for(j=0; j<7; j++) {
					for(k=0; k<7; k++) {
						for(l=0; l<7; l++) {
							memcpy(initState, scan, numPts*sizeof(point4D));
							temp = multQuat(rotz, current);
							current = temp;
							initializePose(current, translation);
							error = icp(initState, root, T, numPts, MAX_ITERATIONS_FIND);
							if(error < 0.02) {
								k = 7; j = 7;
								break;
							}
						}
						temp = multQuat(roty, current);
						current = temp;
					}
					temp = multQuat(rotx, current);
					current = temp;
				}
				if(error > 0.02) {
					printf("DID NOT CONVERGE!!!\n");
					return;
				}
				found = clock();
				acq_time = (double) (found-looking) / CLOCKS_PER_SEC;
				finding = false;
			}
#if TO_FILE
			printf("%f\n", error);
			printTrans(T, fpos, frot);
#else
			printf("%f\n", error);
			printTrans(T);
#endif
		}
#if TO_FILE
		std::fclose(fpos);
		std::fclose(frot);
#endif
		time /= (NUM_FILES - 2);
		printf("Time to acquire: %fs\n", acq_time);
		printf("Average time: %fms\n", time);
	}
}

void FLOOD::initializePose(quat qInit, float t[4]) {
	quat2trans(T,qInit,t);
}

void FLOOD::getFrame(uint8_t fileNum, std::string dir) {
	std::string filename, prefix, num, sufix;
	num = std::to_string(fileNum);
#if OLD
	if(fileNum < 10) {
		prefix = "trajectory_noisy0000";
	} else {
		prefix = "trajectory_noisy000";
	}
	sufix = ".pcd";
#else
	prefix = "test";
	sufix = "_noisy00000.pcd";
#endif
	filename = dir + prefix + num + sufix;
	FILE *f = std::fopen(filename.c_str(),"r");
	printf("%s\n", filename.c_str());
	numPts = readFrame(scan, f);
	std::fclose(f);
}

void FLOOD::getPosition(std::string dir) {
	std::string filename = dir + "position.txt";
	FILE *f = std::fopen(filename.c_str(), "r");
	fscanf(f,"%f  %f  %f", &translation[0], &translation[1], &translation[2]);
	translation[2] = 10 - translation[2];
	translation[3] = 1;
	printf("%f %f %f\n", translation[0], translation[1], translation[2]);
	std::fclose(f);
}