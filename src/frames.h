#ifndef CSV_H
#define CSV_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "kd_tree.h"


static inline uint32_t readFrame(point4D *scan, FILE *f) {
	uint8_t numVals = 3;
	uint32_t i;
	float tmp[3];
	char buff[1024];
	for(i=0; i<11; i++) {
		fgets(buff, 1024, f);
	}
	i=0;
	do {
		numVals = fscanf(f, "%f %f %f %*f %*d", &(tmp[0]), &(tmp[1]), &(tmp[2]));
		if(isfinite(tmp[0]) && isfinite(tmp[1]) && isfinite(tmp[2])) {
			memcpy(scan[i].point, tmp, 3*sizeof(float));
			scan[i].point[3] = 1.0;
			i++;
			if(i == MAX_POINTS)
				return i;
		}
	} while(!feof(f));
	return i;
}


#endif