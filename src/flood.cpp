#include "flood.h"
#include "kd_tree.h"
#include "icp.h"
#include "quaternion.h"
#include <stdlib.h>

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
};

FLOOD::~FLOOD() {
	freeModel(faces);
	deleteTree(root,0);
};

void FLOOD::run() {
	uint8_t i;
	for(i=0; i<=NUM_FILES; i++) {

	}
}