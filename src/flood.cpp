#include <fstream>
#include "flood.h"
#include "frames.h"

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	finding = true;
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	freeModel(faces);
	deleteTree(root,0);
};

void FLOOD::run() {
	unsigned int i, j, k, l;
	quat current, temp;
	quat rotx, roty, rotz;
	// Define quaternion for 45 degree rotation about each axis
	rotx.w = 0.924; roty.w = 0.924; rotz.w = 0.924;
	rotx.x = 0.383; roty.x = 0.0;   rotz.x = 0.0;
	rotx.y = 0.0;   roty.y = 0.383; rotz.y = 0.0;
	rotx.z = 0.0;   roty.z = 0.0;   rotz.z = 0.383;
	// Initialize timer
	clock_t start, end, looking, found;
	float error;
	double time = 0, acq_time;
	// Read all trajectories being tested
	std::ifstream file(FRAME_DIRECTORIES);
    std::string dir, pos, rot; 
    while (std::getline(file, dir)) {
#if TO_FILE
    	// Output results
    	pos = dir + "position_act.txt";
    	rot = dir + "rotation_act.txt";
    	FILE *fpos = std::fopen(pos.c_str(), "w");
    	FILE *frot = std::fopen(rot.c_str(), "w");
#endif
    	// Initialize Pose to no rotation
    	q.w = 1; q.x = 0; q.y = 0; q.z = 0;
    	current = q;
    	printf("%s\n", dir.c_str());
    	// Find the initial position
    	getPosition(dir);
    	// Analyze trajectory
		for(i=1; i<=NUM_FILES; i++) {
			// If the pose is known
			if(!finding) {
				// Run ICP
				start = clock();
				// Get current frame
				getFrame(i, dir);
				error = icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
				end = clock();
				time += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
			}
			// If pose is unknown
			else {
				// Get current frame
				getFrame(i, dir);
				looking = clock();
				// Remember the initial state of the LiDAR scan
				point4D initState[numPts];
				memcpy(initState, scan, numPts*sizeof(point4D));
				initializePose(current, translation);
				// First test
				error = icp(initState, root, T, numPts, MAX_ITERATIONS_FIND);
				//Rotate about each access until a solution converges
				if(error < THRESH) {
					for(j=0; j<7; j++) {
						for(k=0; k<7; k++) {
							for(l=0; l<7; l++) {
								memcpy(initState, scan, numPts*sizeof(point4D));
								// rotate about z
								temp = multQuat(rotz, current);
								current = temp;
								initializePose(current, translation);
								// Perform icp
								error = icp(initState, root, T, numPts, MAX_ITERATIONS_FIND);
								// Check for convergance
								if(error < THRESH) {
									k = 7; j = 7;
									break;
								}
							}
							// Rotate about y
							temp = multQuat(roty, current);
							current = temp;
						}
						// Rotate about x
						temp = multQuat(rotx, current);
						current = temp;
					}
				}
				// If it didn't converge send error
				if(error > THRESH) {
					printf("DID NOT CONVERGE!!!\n");
					return;
				}
				found = clock();
				acq_time = (double) (found-looking) / CLOCKS_PER_SEC;
				finding = false;
			}
			// Print results
#if TO_FILE
			printf("%f\n", error);
			printTrans(T, fpos, frot);
#else
			printf("%f\n", error);
			printTrans(T);
#endif
		}
#if TO_FILE
		std::fclose(fpos);
		std::fclose(frot);
#endif
		time /= (NUM_FILES - 2);
		printf("Time to acquire: %fs\n", acq_time);
		printf("Average time: %fms\n", time);
	}
}

void FLOOD::initializePose(quat qInit, float t[4]) {
	quat2trans(T,qInit,t);
}

void FLOOD::getFrame(unsigned int fileNum, std::string dir) {
	std::string filename, prefix, num, sufix;
	num = std::to_string(fileNum);
	prefix = "test";
	sufix = "_noisy00000.pcd";
	filename = dir + prefix + num + sufix;
	FILE *f = std::fopen(filename.c_str(),"r");
	printf("%s\n", filename.c_str());
	numPts = readFrame(scan, f);
	std::fclose(f);
}

void FLOOD::getPosition(std::string dir) {
	unsigned int vals;
	std::string filename = dir + "position.txt";
	FILE *f = std::fopen(filename.c_str(), "r");
	vals = fscanf(f,"%f  %f  %f", &translation[0], &translation[1], &translation[2]);
	translation[0] = -translation[0]; translation[1] = -translation[1];
	translation[2] = 10 - translation[2];
	translation[3] = 1;
	printf("%f %f %f\n", translation[0], translation[1], translation[2]);
	std::fclose(f);
}