#include "cluster.h"
#include <stdio.h>

int main() {
	int i;
	std::vector<point4D> v, out;
	FILE *f = fopen("../build/exe/pc_target/clouds/cloud_big_3_4-320.txt", "r");
	do {
		point4D current;
		fscanf(f, "%f  %f  %f", &current.point[0], &current.point[1], &current.point[2]);
		current.point[0] /= 1000; current.point[1] /= 1000; current.point[2] /= 1000;
		v.push_back(current);
	} while(!feof(f));
	out = hcluster(v);
	fclose(f);
	f = fopen("../build/exe/pc_target/new0.txt", "w"); 
	for(i = 0; i < out.size(); i++) {
		fprintf(f, "%f  %f  %f\n", out[i].point[0], out[i].point[1], out[i].point[2]);
	}
	fclose(f);
}