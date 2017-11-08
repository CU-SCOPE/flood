#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdbool.h>
#include <time.h>
#include "flood.h"
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include "frames.h"

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	finding = 1;
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	freeModel(faces);
	deleteTree(root,0);
};

void FLOOD::run() {
	uint8_t i;
	quat q;
	clock_t start, end;
	double time = 0;
	for(i=1; i<=NUM_FILES; i++) {
		getFrame(i);
		if(i > 1) {
			start = clock();
			icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
			end = clock();
			time += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
		}
		else {
			icp(scan, root, T, numPts, MAX_ITERATIONS_FIND);
		}
		trans2quat(T, &q);
		printQuat(q);
	}
	time /= 58;
	printf("Average time: %fms\n", time);
}

void FLOOD::initializePose(quat q, float t[4]) {
	quat2trans(T,q,t);
}

void FLOOD::getFrame(uint8_t fileNum) {
	std::string filename, prefix, num, sufix;
	std::string dir = FRAME_DIRECTORY;
	if(fileNum < 10) {
		prefix = "trajectory_noisy0000";
	} else {
		prefix = "trajectory_noisy000";
	}
	num = std::to_string(fileNum);
	sufix = ".pcd";
	filename = dir + prefix + num + sufix;
	FILE *f = std::fopen(filename.c_str(),"r");
	printf("%s\n", filename.c_str());
	numPts = readFrame(scan, f);
	std::fclose(f);
}
