#include <stdio.h>
#include <stdlib.h>
#include "stl.h"


void main() {
	face *faces;
	uint32_t numFaces = loadSTL(&faces);
	face *root = initTree(faces, numFaces);
	freeModel(faces);
	deleteTree(root,0);
}