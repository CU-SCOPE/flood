#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdbool.h>
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
	freeModel(faces);
	deleteTree(root,0);
};

void FLOOD::run() {
	uint8_t i;
	quat q;
	for(i=1; i<=NUM_FILES; i++) {
		getFrame(i);
		if(i > 0)
			icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
		else
			icp(scan, root, T, numPts, MAX_ITERATIONS_FIND);
		trans2quat(T, &q);
		printQuat(q);
	}
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
	FILE *f = fopen(filename.c_str(),"r");
	printf("%s\n", filename.c_str());
	numPts = readFrame(scan, f);
	fclose(f);
}
