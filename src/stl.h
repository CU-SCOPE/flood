#ifndef STL_H
#define STL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define STL_FILE_NAME    "models/galileo_red.stl"

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

#ifdef __cplusplus
}
#endif

#endif