#include <stdio.h>
#include "flood.h"
#include "quaternion.h"


int main() {
	FLOOD driver;
	quat q;
	q.w = 1;
	q.x = 0;
	q.y = 0;
	q.z = 0;
	driver.initializePose(q,t);
	driver.run();
	return 0;
}