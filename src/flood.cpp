#include <fstream>
#include "flood.h"
// #include "frames.h"

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
	// Get initial position
	// getPosition(FRAME_DIRECTORIES);
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
	unsigned int i, j, k, l;
	quat current, temp;
	// Initialize timer
	clock_t start, end, looking, found;
	float error;
	double time = 0, acq_time;
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
			time += (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
		}
		// If pose is unknown
		else {
			int num = numPts;
			// Get current frame
			looking = clock();
			initializePose(current, translation);
			// First test
			point4D initState[num];
			memcpy(initState, scan, num*sizeof(point4D)); // Copy frame from buffer
			error = icp(initState, root, T, num, MAX_ITERATIONS_FIND);
			//Rotate about each access until a solution converges
			if(error > THRESH) {
				for(j=0; j<1; j++) {
					for(k=0; k<1; k++) {
						for(l=0; l<1; l++) {
							memcpy(initState, scan, num*sizeof(point4D));
							// rotate about z
							temp = multQuat(rotz, current);
							current = temp;
							initializePose(current, translation);
							// Perform icp
							error = icp(initState, root, T, num, MAX_ITERATIONS_FIND);
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
				continue;
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
		read = true; // Tell frame reader to copy in next frame
	}
#if TO_FILE
	std::fclose(fpos);
	std::fclose(frot);
#endif
	time /= (NUM_FILES - 2);
	printf("Time to acquire: %fs\n", acq_time);
	printf("Average time: %fms\n", time);
}

void FLOOD::initializePose(quat qInit, float t[4]) {
	quat2trans(T,qInit,t);
}

void FLOOD::getFrame() {
	o3d3xx::Logging::Init();
	o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
	// Start framegrabber
	o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
	o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
	int fileNum = 1;
	std::vector<point4D> v;
	while(fileNum <= NUM_FILES) {
		if (! fg->WaitForFrame(img.get(), 1000)) {
			std::cerr << "Timeout waiting for camera!" << std::endl;
			continue;
		}
		v = img->XYZImage();
		while(!read) {std::this_thread::yield();} // Wait for current frame to be read
		numPts = v.size();
		std::copy(v.begin(), v.end(), scan); // Copy new frame to image buffer
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