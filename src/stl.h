#ifndef STL_H
#define STL_H

#include <stdint.h>

typedef struct point{
	float x;
	float y;
	float z;
}point;

typedef struct face {
	point v1;
	point v2;
	point v3;
}face;

void loadSTL(face **faces, FILE *f);
face createFace(char *buff, uint32_t ind);
void freeModel(face *faces);

#endif