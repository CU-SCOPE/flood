#ifndef CSV_H
#define CSV_H

#include <stdio.h>
#include <kd_tree.h>

#define MAX_POINTS		500

static inline uint32_t readFrame(point4D *scan, FILE *f) {
	uint8_t numVals = 3;
	uint32_t i = 0;
	while(numVals == 3) {
		numVals = fscanf(f, "%f   %f   %f", &(scan[i].point[0]), &(scan[i].point[1]), &(scan[i].point[2]));
		scan[i].point[3] = 1.0;
		i++;
		if(i == MAX_POINTS)
			return i-1;
	}
	return i-1;
}


#endif