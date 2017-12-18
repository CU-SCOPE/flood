#ifndef FRAME_H
#define FRAME_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "kd_tree.h"


static inline unsigned int readFrame(point4D *scan, FILE *f) {
	unsigned int numVals = 3;
	unsigned int i;
	float tmp[3];
	char buff[1024];
	char *garbage;
	for(i=0; i<11; i++) {
		garbage = fgets(buff, 1024, f);
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