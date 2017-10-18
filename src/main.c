#include <stdio.h>
#include <stdlib.h>
#include "stl.h"


void main() {
	face *faces;
	loadSTL(&faces);
	printf("%f\n", faces[12425].v1.x);
	freeModel(faces);
}