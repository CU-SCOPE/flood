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
    // Analyze trajectory
	for(i=1; i<=NUM_FILES; i++) {
		while(read) {std::this_thread::yield();} // Wait for new frame
		// If the pose is known
		if(!finding) {
			// Run ICP
			start = clock();
			// Get current frame
			error = icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
			end = clock();
			tm += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
			num++;
			// Restart if error is drifting off too much; tune threshold value here
			if(error > 0.1)
				finding = true;
		}
		// If pose is unknown
		else {
			point4D initState[numPts];
			float Temp[4][4] = {0}, best = 100;
			srand(time(NULL));
			// Get current frame
			looking = clock();
			for(j=0; j<10; j++) {
				memcpy(initState, scan, numPts*sizeof(point4D));
				current.w = ((double) rand() / (RAND_MAX)); current.x = ((double) rand() / (RAND_MAX));
				current.y = ((double) rand() / (RAND_MAX)); current.z = ((double) rand() / (RAND_MAX));
				initializePose(current, translation, Temp);
				// First test
				error = icp(initState, root, Temp, numPts, MAX_ITERATIONS_FIND);
				if(error < best) {
					best = error;
					memcpy(T, Temp, 16*sizeof(float));
				}
			}	
			error = best;
			// If it didn't converge send error
			if(error > THRESH) {
				printf("%f\n", error);
				read = true; // Tell frame reader to copy in next frame
				printf("DID NOT CONVERGE!!!\n");
				continue;
			}
			found = clock();
			acq_time = (double) (found-looking) / CLOCKS_PER_SEC;
			finding = false;
		}
		// Print results
#if TO_FILE
		printf("%f\n", error);
		printTrans(T, translation, fpos, frot);
#else
		printf("%f\n", error);
		printTrans(T, translation);
#endif
		// Rotate position vector
		RT[0][0] = T[0][0]; RT[0][1] = T[1][0]; RT[0][2] = T[2][0];
		RT[1][0] = T[0][1]; RT[1][1] = T[1][1]; RT[1][2] = T[2][1];
		RT[2][0] = T[0][2]; RT[2][1] = T[1][2]; RT[2][2] = T[2][2];
		t[0] = T[0][3]; t[1] = T[1][3]; t[2] = T[2][3];
		matMulVec3D(RT, t, translation);
		read = true; // Tell frame reader to copy in next frame
	}
#if TO_FILE
	std::fclose(fpos);
	std::fclose(frot);
#endif
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
	while(fileNum <= NUM_FILES) {
		while(!read) {std::this_thread::yield();}
		num = std::to_string(fileNum);
		filename = dir + prefix + num + sufix;
		f = std::fopen(filename.c_str(),"r");
		printf("%s\n", filename.c_str());
		numPts = readFrame(scan, f);
		std::fclose(f);
		++fileNum;
		read = false;
	}
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