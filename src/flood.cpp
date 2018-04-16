#include <fstream>
#include <cstdlib>
#include <ctime>
#include "flood.h"
#include "frames.h"
#include <unistd.h>
#if RENDER
#include "render.h"
#endif

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	// POSE is unkown at start
	finding = true;
	done = false;
	exit = false;
	// Define quaternion for 45 degree rotation about each axis
	rotx.w = 0.985; roty.w = 0.924; rotz.w = 0.924;
	rotx.x = 0.383; roty.x = 0.0;   rotz.x = 0.0;
	rotx.y = 0.0;   roty.y = 0.383; rotz.y = 0.0;
	rotx.z = 0.0;   roty.z = 0.0;   rotz.z = 0.383;
	pthread_mutex_init(&lock, NULL);
	pthread_mutex_init(&sa_lock, NULL);
	sem_init(&frame1, 0, 0);
	sem_init(&new_frame, 0, 1);
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	freeModel(faces);
	deleteTree(root,0);
	pthread_mutex_destroy(&lock);
	pthread_mutex_destroy(&sa_lock);
	sem_destroy(&frame1);
	sem_destroy(&new_frame);
};

void FLOOD::run() {
	std::thread frames(&FLOOD::getFrame, this); // Thread to read frames in
	std::thread icp(&FLOOD::calcPose, this); // Thread to calculate POSE
#if RENDER
	std::thread animate(&FLOOD::render, this);
#endif
	frames.join(); // Cleanup threads
	icp.join();
#if RENDER
		animate.join();
#endif
}

void FLOOD::calcPose() {
	unsigned int i, j, num = 0;
	quat current, temp;
	// Initialize timer
	clock_t start, end, looking, found;
	float error, prev_error, RT[3][3], t[3];
	double tm = 0, acq_time;
	// Read all trajectories being tested
    std::string dir = FRAME_DIRECTORIES, pos, rot;
#if TO_FILE
    // Output results
    pos = dir + "position_big_1_3.txt";
    rot = dir + "orientation_big_1_3.txt";
    FILE *fpos = std::fopen(pos.c_str(), "w");
    FILE *frot = std::fopen(rot.c_str(), "w");
#endif
	q.w = 1; q.x = 0; q.y = 0; q.z = 0;
	while(!exit) {
		sem_wait(&frame1);
		pthread_mutex_lock(&lock);
		if(!finding) {
			start = clock();
			error = icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
			end = clock();
			tm += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
			num++;
			if(error > 0.03){
				finding = true;
			}
		}
		else {
			float best = 100;
			float Temp[4][4] = {{0}};
			current = q;
			looking = clock();
			point4D initState[numPts];
			memcpy(initState, scan, numPts*sizeof(point4D));
			initializePose(current, translation, Temp);
			error = icp(initState, root, Temp, numPts, MAX_ITERATIONS_FIND);
			for(int j=0; j<7; j++) {
				if(error < best) {
					best = error;
					memcpy(T, Temp, 16*sizeof(float));
				}
				memcpy(initState, scan, numPts*sizeof(point4D));
				temp = multQuat(rotx, current);
				current = temp;
				initializePose(current, translation, Temp);
				error = icp(initState, root, Temp, numPts, MAX_ITERATIONS_FIND);
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
		prev_error = error;
		// Print results
#if TO_FILE
		printf("%f\n", error);
		printTrans(T, translation, fpos, frot);
#else
		printf("%f\n", error);
		printTrans(T, translation, NULL, NULL);
#endif
		sem_post(&new_frame);
		pthread_mutex_unlock(&lock);
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
	int num = 0, val = 0;
	float temp[3];
	while(!exit) {
		std::string fname = "clouds2/cloud_big_1_3-" + std::to_string(num) + "_test.txt";
		if(access(fname.c_str(), F_OK) < 0){
			exit = true;
			break;
		}
		num++;
		FILE *f = fopen(fname.c_str(), "r");
		std::vector<point4D> v;
		v.reserve(10000);
		while(!feof(f)) {
			point4D point;
			if(fscanf(f, "%f  %f  %f", &temp[0], &temp[1], &temp[2]) != 3)
				continue;
			point.point[0] = temp[0]/1000.0f; point.point[1] = temp[1]/1000.0f; point.point[2] = temp[2]/1000.0f;
			point.point[3] = 1.0f;
			v.push_back(point);
		}
		fclose(f);
		v.pop_back();
		val = v.size();
		if(val > 5000) {
			std::vector<point4D> temp_points;
			temp_points.reserve(val);
			if(val < 10000) {
				for(std::size_t i = 0; i < val; i+=2) {
					temp_points.push_back(v[i]);
				}
			} else {
				for(std::size_t i = 0; i < val; i+=3) {
					temp_points.push_back(v[i]);
				}
			}
			v = temp_points;
		}
		v = hcluster(v);
		sem_wait(&new_frame);
		pthread_mutex_lock(&lock);
		numPts = v.size();
		std::copy(v.begin(), v.end(), scan); // Copy new frame to image buffer
		sem_post(&frame1);
		pthread_mutex_unlock(&lock);
	}
}

void FLOOD::getPosition(float position) {
	translation[0] = -position; translation[1] = 0; translation[2] = 0; translation[3] = 1;
	pthread_mutex_lock(&sa_lock);
	shared_array[0] = 1; shared_array[1] = 0;
	shared_array[2] = 0; shared_array[3] = 0;
	shared_array[4] = translation[0];
	shared_array[5] = translation[1];
	shared_array[6] = translation[2];
	done = true;
	pthread_mutex_unlock(&sa_lock);
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

#if RENDER
void FLOOD::render() {
	Render scene(shared_array, &done, &sa_lock, &exit);
	scene.run();
}
#endif