#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stl.h"

uint32_t loadSTL(face **faces){
	FILE *f = fopen(STL_FILE_NAME, "r");
	char header[80];
	char buffNtri[4];
	uint32_t numTri, i, ind, size = 50, offset = 12, read;

	read = fread(header, 80*sizeof(char), 1, f);
	read = fread(buffNtri, 4*sizeof(char), 1, f);

	numTri = *((uint32_t*) buffNtri);
	char buff[numTri*size];
	read = fread(buff, sizeof(char), numTri*size, f);

	(*faces) = malloc(numTri * sizeof(face));
	for(i=0; i<numTri; i++) {
		ind = i * size;	
		(*faces)[i] = createFace(buff, ind);	
	}
	fclose(f);
	return numTri;
}

face createFace(char *buff, uint32_t ind) {
	face newFace;
	char normal[12];
	memcpy(normal, buff+ind, 12);

	ind += 12;
	char x1[4] = {buff[ind], buff[ind + 1], buff[ind + 2], buff[ind + 3]};
	char y1[4] = {buff[(ind + 4)], buff[(ind + 4) + 1], buff[(ind + 4) + 2], buff[(ind + 4) + 3]};
	char z1[4] = {buff[(ind + 8)], buff[(ind + 8) + 1], buff[(ind + 8) + 2], buff[(ind + 8) + 3]};
	
	ind += 12;
	char x2[4] = {buff[ind], buff[ind + 1], buff[ind + 2], buff[ind + 3]};
	char y2[4] = {buff[(ind + 4)], buff[(ind + 4) + 1], buff[(ind + 4) + 2], buff[(ind + 4) + 3]};
	char z2[4] = {buff[(ind + 8)], buff[(ind + 8) + 1], buff[(ind + 8) + 2], buff[(ind + 8) + 3]};
	
	ind += 12;
	char x3[4] = {buff[ind], buff[ind + 1], buff[ind + 2], buff[ind + 3]};
	char y3[4] = {buff[(ind + 4)], buff[(ind + 4) + 1], buff[(ind + 4) + 2], buff[(ind + 4) + 3]};
	char z3[4] = {buff[(ind + 8)], buff[(ind + 8) + 1], buff[(ind + 8) + 2], buff[(ind + 8) + 3]};

	newFace.normal.x = *((float*) &normal[0] );
	newFace.normal.y = *((float*) &normal[4] );
	newFace.normal.z = *((float*) &normal[8] );

	newFace.v1.x = *((float*) x1 );
	newFace.v1.y = *((float*) y1 );
	newFace.v1.z = *((float*) z1 );

	newFace.v2.x = *((float*) x2 );
	newFace.v2.y = *((float*) y2 );
	newFace.v2.z = *((float*) z2 );

	newFace.v3.x = *((float*) x3 );	
	newFace.v3.y = *((float*) y3 );	
	newFace.v3.z = *((float*) z3 );	

	return newFace;
}

void freeModel(face *faces) {
	free(faces);
}