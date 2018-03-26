#include <fstream>
#include <cstdlib>
#include <ctime>
#include "flood.h"
// #include "frames.h"

FLOOD::FLOOD() {
	numFaces = loadSTL(&faces);
	root = initTree(faces, numFaces);
	// POSE is unkown at start
	finding = true;
	// Define quaternion for 45 degree rotation about each axis
	rotx.w = 0.924; roty.w = 0.924; rotz.w = 0.924;
	rotx.x = 0.383; roty.x = 0.0;   rotz.x = 0.0;
	rotx.y = 0.0;   roty.y = 0.383; rotz.y = 0.0;
	rotx.z = 0.0;   roty.z = 0.0;   rotz.z = 0.383;
	pthread_mutex_init(&lock, NULL);
};

FLOOD::~FLOOD() {
	printf("Exiting\n");
	freeModel(faces);
	deleteTree(root,0);
	pthread_mutex_destroy(&lock);
};

void FLOOD::run() {
	std::thread frames(&FLOOD::getFrame, this); // Thread to read frames in
	std::thread icp(&FLOOD::calcPose, this); // Thread to calculate POSE
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
    pos = dir + "position_small.txt";
    rot = dir + "rotation_small.txt";
    FILE *fpos = std::fopen(pos.c_str(), "w");
    FILE *frot = std::fopen(rot.c_str(), "w");
#endif
    // Initialize Pose to no rotation
    q.w = 1; q.x = 0; q.y = 0; q.z = 0;
    current = q;
    // Analyze trajectory
	for(i=1; i<=NUM_FILES; i++) {
		pthread_mutex_lock(&lock);
		if(!finding) {
			start = clock();
			error = icp(scan, root, T, numPts, MAX_ITERATIONS_KNOWN);
			end = clock();
			tm += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
		}
		else {
			current = q;
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
			if(error > THRESH) {
				printf("DID NOT CONVERGE!!!\n");
				pthread_mutex_unlock(&lock);
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
	o3d3xx::Logging::Init();
	o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
	// Start framegrabber
	o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
	o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
	int fileNum = 1, val = 0;
	float t[3], dims[3] = {0.25, 0.7, 0.7};
	t[0] = -translation[0]; t[1] = -translation[1]; t[2] = -translation[2];
	img->setPosition(t, dims);
	std::vector<point4D> v;
	while(fileNum <= NUM_FILES) {
		if (! fg->WaitForFrame(img.get(), 1000)) {
			std::cerr << "Timeout waiting for camera!" << std::endl;
			continue;
		}
		v = img->XYZImage();
		printf("%d\n", v.size());
		v = hcluster(v);
		pthread_mutex_lock(&lock);
		numPts = v.size();
		val = numPts;
		printf("%d\n", val);
		std::copy(v.begin(), v.end(), scan); // Copy new frame to image buffer
		++fileNum;
		t[0] = -translation[0]; t[1] = -translation[1]; t[2] = -translation[2];
		img->setPosition(t, dims);
		pthread_mutex_unlock(&lock);
	}
}

void FLOOD::getPosition(float position) {
	translation[0] = -position; translation[1] = 0; translation[2] = 0; translation[3] = 1;
}