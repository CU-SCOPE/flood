#include <fstream>
#include <cstdlib>
#include <ctime>
#include "flood.h"
#include "frames.h"

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	// POSE is unkown at start
	finding = true;
	read = true;
	// Define quaternion for 45 degree rotation about each axis
	rotx.w = 0.924; roty.w = 0.924; rotz.w = 0.924;
	rotx.x = 0.383; roty.x = 0.0;   rotz.x = 0.0;
	rotx.y = 0.0;   roty.y = 0.383; rotz.y = 0.0;
	rotx.z = 0.0;   roty.z = 0.0;   rotz.z = 0.383;
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	freeModel(faces);
	deleteTree(root,0);
};

void FLOOD::run() {
	std::thread frames (&FLOOD::getFrame, this); // Thread to read frames in
	std::thread icp (&FLOOD::calcPose, this); // Thread to calculate POSE
	frames.join(); // Cleanup threads
	icp.join();
}

void FLOOD::calcPose() {
	unsigned int i, j, num = 0;
	quat current, temp;
	// Initialize timer
	clock_t start, end, looking, found;
	float error, RT[3][3], t[3];
	double tm = 0, acq_time;
	// Read all trajectories being tested
    std::string dir = FRAME_DIRECTORIES, pos, rot;
    std::string filename = dir + "position.txt";
	FILE *f = std::fopen(filename.c_str(), "r"); 
#if TO_FILE
    // Output results
    pos = dir + "position_act.txt";
    rot = dir + "rotation_act.txt";
    FILE *fpos = std::fopen(pos.c_str(), "w");
    FILE *frot = std::fopen(rot.c_str(), "w");
#endif
	q.w = 1; q.x = 0; q.y = 0; q.z = 0;
	for(i=1; i<=NUM_FILES; i++) {
		while(read) {std::this_thread::yield();} // Wait for new frame
		if(!finding) {
			start = clock();
			error = icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
			end = clock();
			tm += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
		}
		else {
			current = q;
			getPosition(f);
			looking = clock();
			point4D initState[numPts];
			memcpy(initState, scan, numPts*sizeof(point4D));
			initializePose(current, translation, T);
			error = icp(initState, root, T, numPts, MAX_ITERATIONS_FIND);
			for(int j=0; j<7; j++) {
				for(int k=0; k<7; k++) {
					for(int l=0; l<7; l++) {
						if(error < THRESH) {
							k = 7; j = 7;
							break;
						}
						memcpy(initState, scan, numPts*sizeof(point4D));
						temp = multQuat(rotz, current);
						current = temp;
						initializePose(current, translation, T);
						error = icp(initState, root, T, numPts, MAX_ITERATIONS_FIND);
					}
					temp = multQuat(roty, current);
					current = temp;
				}
				temp = multQuat(rotx, current);
				current = temp;
			}
			if(error > THRESH) {
				printf("DID NOT CONVERGE!!!\n");
				continue;
			}
			found = clock();
			acq_time = (double) (found-looking) / CLOCKS_PER_SEC;
			finding = false;
		}
		// Rotate position vector
		RT[0][0] = T[0][0]; RT[0][1] = T[1][0]; RT[0][2] = T[2][0];
		RT[1][0] = T[0][1]; RT[1][1] = T[1][1]; RT[1][2] = T[2][1];
		RT[2][0] = T[0][2]; RT[2][1] = T[1][2]; RT[2][2] = T[2][2];
		t[0] = T[0][3]; t[1] = T[1][3]; t[2] = T[2][3];
		matMulVec3D(RT, t, translation);
		// Print results
#if TO_FILE
		printf("%f\n", error);
		printTrans(T, translation, fpos, frot);
#else
		printf("%f\n", error);
		printTrans(T, translation);
#endif
		read = true; // Tell frame reader to copy in next frame
	}
#if TO_FILE
	std::fclose(fpos);
	std::fclose(frot);
#endif
	std::fclose(f);
	tm /= num;
	printf("Time to acquire: %fs\n", acq_time);
	printf("Average time: %fms\n", tm);
}

void FLOOD::initializePose(quat qInit, float t[4], float Temp[4][4]) {
	quat2trans(Temp,qInit,t);
}

void FLOOD::getFrame() {
	std::string dir = FRAME_DIRECTORIES, filename, prefix, num, sufix;
	printf("%s\n", dir.c_str());
	prefix = "test";
	sufix = "_noisy00000.pcd";
	unsigned int fileNum = 1;
	FILE *f;
	point4D temp[MAX_POINTS];
	int points;
	while(fileNum <= NUM_FILES) {
		num = std::to_string(fileNum);
		filename = dir + prefix + num + sufix;
		f = std::fopen(filename.c_str(),"r");
		printf("%s\n", filename.c_str());
		points = readFrame(temp, f);
		std::fclose(f);
		while(!read) {std::this_thread::yield();}
		memcpy(scan, temp, points*sizeof(point4D));
		numPts = points;
		++fileNum;
		read = false;
	}
}

void FLOOD::getPosition(FILE *f) {
	unsigned int vals;
	vals = fscanf(f,"%f  %f  %f", &translation[0], &translation[1], &translation[2]);
	translation[0] = -translation[0]; translation[1] = -translation[1];
	translation[2] = 10 - translation[2];
	translation[3] = 1;
	printf("%f %f %f\n", translation[0], translation[1], translation[2]);
}