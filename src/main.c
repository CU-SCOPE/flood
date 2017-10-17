#include <stdio.h>
#include <stdlib.h>
#include "stl.h"


void main() {
	printf("plswork\n");
	face *faces;
	FILE *f = fopen("models/galileo_small.stl", "r");
	loadSTL(&faces, f);
	printf("%f\n", faces[12425].v1.x);
	freeModel(faces);
}