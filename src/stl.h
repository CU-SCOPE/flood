#ifndef STL_H
#define STL_H

#include <stdint.h>

#define STL_FILE_NAME    "models/galileo_small.stl"

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

uint32_t loadSTL(face **faces);
face createFace(char *buff, uint32_t ind);
void freeModel(face *faces);

#endif