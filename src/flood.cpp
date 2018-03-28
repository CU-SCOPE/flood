#include <fstream>
#include <cstdlib>
#include <ctime>
#include "flood.h"
#include "frames.h"
#include "render.h"

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	// POSE is unkown at start
	finding = true;
	done = false;
	// Define quaternion for 45 degree rotation about each axis
	rotx.w = 0.924; roty.w = 0.924; rotz.w = 0.924;
	rotx.x = 0.383; roty.x = 0.0;   rotz.x = 0.0;
	rotx.y = 0.0;   roty.y = 0.383; rotz.y = 0.0;
	rotx.z = 0.0;   roty.z = 0.0;   rotz.z = 0.383;
	pthread_mutex_init(&lock, NULL);
	pthread_mutex_init(&sa_lock, NULL);
	sem_init(&frame1, 0, 0);
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	freeModel(faces);
	deleteTree(root,0);
	pthread_mutex_destroy(&lock);
	pthread_mutex_destroy(&sa_lock);
	sem_destroy(&frame1);
};

void FLOOD::run() {
	std::thread frames(&FLOOD::getFrame, this); // Thread to read frames in
	std::thread icp(&FLOOD::calcPose, this); // Thread to calculate POSE
	std::thread animate(&FLOOD::render, this);
	frames.join(); // Cleanup threads
	icp.join();
	animate.join();
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
		sem_wait(&frame1);
		pthread_mutex_lock(&lock);
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
			float mindist = 100, Transf[4][4] = {{0}};
			memcpy(initState, scan, numPts*sizeof(point4D));
			initializePose(current, translation, Transf);
			error = icp(initState, root, Transf, numPts, MAX_ITERATIONS_FIND);
			for(int j=0; j<6; j++) {
				if(error < mindist) {
					mindist = error;
					memcpy(T, Transf, 16*sizeof(float));
				}
				memcpy(initState, scan, numPts*sizeof(point4D));
				temp = multQuat(rotx, current);
				current = temp;
				initializePose(current, translation, Transf);
				error = icp(initState, root, Transf, numPts, MAX_ITERATIONS_FIND);
			}
			// if(error > THRESH) {
			// 	printf("DID NOT CONVERGE!!!\n");
			// 	pthread_mutex_unlock(&lock);
			// 	continue;
			// }
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
		printTrans(T, translation, NULL, NULL);
#endif
		pthread_mutex_unlock(&lock);
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
		pthread_mutex_lock(&lock);
		memcpy(scan, temp, points*sizeof(point4D));
		numPts = points;
		++fileNum;
		sem_post(&frame1);
		pthread_mutex_unlock(&lock);
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


void FLOOD::printQuat(quat qt, FILE *f) {
	if(f) {
		fprintf(f, "%f  %f  %f  %f\n", qt.w, qt.x, qt.y, qt.z);
	} else {
		printf("%f  %f  %f  %f\n", qt.w, qt.x, qt.y, qt.z);
	}
}

void FLOOD::printTrans(float T[4][4], float translation[3], FILE *pos, FILE *rot) {
	quat qt;
	trans2quat(T, &qt);
	pthread_mutex_lock(&sa_lock);
	shared_array[0] = qt.w; shared_array[1] = qt.x;
	shared_array[2] = qt.y; shared_array[3] = qt.z;
	shared_array[4] = translation[0];
	shared_array[5] = translation[1];
	shared_array[6] = translation[2];
	done = true;
	pthread_mutex_unlock(&sa_lock);
	printQuat(qt, rot);
	if(pos) {
		fprintf(pos, "%f  %f  %f\n", translation[0], translation[1], translation[2]);
	} else {
		printf("%f  %f  %f\n", translation[0], translation[1], translation[2]);
	}
}

void FLOOD::render() {
	Render scene(shared_array, &done, &sa_lock);
	scene.run();
}