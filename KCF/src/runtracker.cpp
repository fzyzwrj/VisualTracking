#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

//#include <dirent.h>

using namespace std;
using namespace cv;

// 初始目标选框
Rect initRect;
// 视频帧
Mat frame;
// 初始化标记
bool initFlag = true;
void onMouse(int event, int x, int y, int flag, void *)
{
	static Point prePoint;
	static Point curPoint;
	static Mat imgTmp;
	// 鼠标左键下
	if (event == CV_EVENT_LBUTTONDOWN) {
		imgTmp = frame.clone();
		prePoint = Point(x, y);
		initFlag = true;
	}	// 鼠标左键按下并移动
	else if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_FLAG_LBUTTON)) {
		imgTmp = frame.clone();	// 每次移动都要拷贝原图用于画图
		curPoint = Point(x, y);
		rectangle(imgTmp, prePoint, curPoint, Scalar(255, 255, 0), 1, 8);
		imshow("Image", imgTmp);
	}	// 松开左键，选框完毕
	else if (event == CV_EVENT_LBUTTONUP) {
		initRect = Rect(prePoint, curPoint);
		initFlag = false;
		if (initRect.width < 2 || initRect.height < 2)
			initFlag = true;
	}
	else if (event == CV_EVENT_LBUTTONDBLCLK) {
		std::cout << "Closed." << std::endl;
		exit(0);
	}
}

int main(int argc, char* argv[])
{
	if (argc > 5) return -1;

	///parameter
	bool HOG = true;
	bool FIXEDWINDOW = true;
	bool MULTISCALE = false;
	bool SILENT = false;
	bool LAB = false;

	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "hog") == 0)
			HOG = true;
		if (strcmp(argv[i], "fixed_window") == 0)
			FIXEDWINDOW = true;
		if (strcmp(argv[i], "singlescale") == 0)
			MULTISCALE = false;
		if (strcmp(argv[i], "show") == 0)
			SILENT = false;
		if (strcmp(argv[i], "lab") == 0) {
			LAB = true;
			HOG = true;
		}
		if (strcmp(argv[i], "gray") == 0)
			HOG = false;
	}

	// Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	const int img_width = 4096;
	const int img_height = 2160;
	const int fScale = 4.0;

	VideoCapture cap("G:\\resources\\videos\\DJI_0001.MOV");
	assert(cap.isOpened());
	//if (!cap.isOpened()) {
	//	std::cerr << "Error: open video failed." << std::endl;
	//	return -1;
	//}
	int nFrames = 0;
	Rect result;
	int frameStartIndex = 0;
	std::cout << "frames passed number: " << std::endl;
	std::cin >> frameStartIndex;
	cap.set(CV_CAP_PROP_POS_FRAMES, frameStartIndex);
	namedWindow("Image", 0);
	resizeWindow("Image", img_width / fScale, img_height / fScale);
	moveWindow("Image", 0, 0);
	setMouseCallback("Image", onMouse);

	static int filenameIndex = 0;
	const std::string filenamePrefix = "./res/";
	std::ofstream fout;
	int frameIndex = frameStartIndex;
	while (true) {
		cap >> frame;
		++frameIndex;

		// 初始化选定目标
		if (nFrames == 0 || initFlag) {
			imshow("Image", frame);
			waitKey(0);	// 选框
			tracker.init(initRect, frame);
			rectangle(frame, initRect, Scalar(0, 255, 255), 1, 8);

			if (fout.is_open())
				fout.close();

			++filenameIndex;
			fout.open(filenamePrefix + std::to_string(filenameIndex) + ".txt");
			assert(fout.is_open());
			fout << frameIndex << " " << initRect << std::endl;
		}
		// 更新
		else {
			result = tracker.update(frame);
			rectangle(frame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 255), 1, 8);
			//cout << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
			fout << frameIndex << " " << result << std::endl;
		}

		nFrames++;

		if (!SILENT) {
			imshow("Image", frame);
			int ch = waitKey(1);
		}
	}
	return 0;
}


//int main(int argc, char* argv[]){
//
//	if (argc > 5) return -1;
//
//	bool HOG = true;
//	bool FIXEDWINDOW = true;
//	bool MULTISCALE = false;
//	bool SILENT = false;
//	bool LAB = false;
//
//	for(int i = 0; i < argc; i++){
//		if ( strcmp (argv[i], "hog") == 0 )
//			HOG = true;
//		if ( strcmp (argv[i], "fixed_window") == 0 )
//			FIXEDWINDOW = true;
//		if ( strcmp (argv[i], "singlescale") == 0 )
//			MULTISCALE = false;
//		if ( strcmp (argv[i], "show") == 0 )
//			SILENT = false;
//		if ( strcmp (argv[i], "lab") == 0 ){
//			LAB = true;
//			HOG = true;
//		}
//		if ( strcmp (argv[i], "gray") == 0 )
//			HOG = false;
//	}
//	
//	// Create KCFTracker object
//	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
//
//	// Frame readed
//	Mat frame;
//
//	// Tracker results
//	Rect result;
//
//	// Path to list.txt
//	ifstream listFile;
//	string fileName = "images.txt";
//  	listFile.open(fileName);
//
//  	// Read groundtruth for the 1st frame
//  	ifstream groundtruthFile;
//	string groundtruth = "region.txt";
//  	groundtruthFile.open(groundtruth);
//  	string firstLine;
//  	getline(groundtruthFile, firstLine);
//	groundtruthFile.close();
//  	
//  	istringstream ss(firstLine);
//
//  	// Read groundtruth like a dumb
//  	float x1, y1, x2, y2, x3, y3, x4, y4;
//  	char ch;
//	ss >> x1;
//	ss >> ch;
//	ss >> y1;
//	ss >> ch;
//	ss >> x2;
//	ss >> ch;
//	ss >> y2;
//	ss >> ch;
//	ss >> x3;
//	ss >> ch;
//	ss >> y3;
//	ss >> ch;
//	ss >> x4;
//	ss >> ch;
//	ss >> y4; 
//
//	// Using min and max of X and Y for groundtruth rectangle
//	float xMin =  min(x1, min(x2, min(x3, x4)));
//	float yMin =  min(y1, min(y2, min(y3, y4)));
//	float width = max(x1, max(x2, max(x3, x4))) - xMin;
//	float height = max(y1, max(y2, max(y3, y4))) - yMin;
//
//	
//	// Read Images
//	ifstream listFramesFile;
//	string listFrames = "images.txt";
//	listFramesFile.open(listFrames);
//	string frameName;
//
//
//	// Write Results
//	ofstream resultsFile;
//	string resultsPath = "output.txt";
//	resultsFile.open(resultsPath);
//
//	// Frame counter
//	int nFrames = 0;
//
//
//	while ( getline(listFramesFile, frameName) ){
//		frameName = frameName;
//
//		// Read each frame from the list
//		frame = imread(frameName, CV_LOAD_IMAGE_COLOR);
//
//		// First frame, give the groundtruth to the tracker
//		if (nFrames == 0) {
//			tracker.init( Rect(xMin, yMin, width, height), frame );
//			rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );
//			resultsFile << xMin << "," << yMin << "," << width << "," << height << endl;
//		}
//		// Update
//		else{
//			result = tracker.update(frame);
//			rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 255, 255 ), 1, 8 );
//			resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
//		}
//
//		nFrames++;
//
//		if (!SILENT){
//			imshow("Image", frame);
//			waitKey(1);
//		}
//	}
//	resultsFile.close();
//
//	listFile.close();
//	return 0;
//}
