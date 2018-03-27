#include "flood.h"


int main(int argc, char *argv[]) {
	if(argc < 2) {
		fprintf(stderr, "Did not pass initial position\n");
		exit(1);
	}
	FLOOD driver;
	driver.getPosition(atof(argv[1]));
	driver.run();
	return 0;
}