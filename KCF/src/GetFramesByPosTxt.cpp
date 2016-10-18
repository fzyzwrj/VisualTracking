#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

//#include <dirent.h>
#include "GetTrackingPosTxt.h";

using namespace std;
using namespace cv;

// 初始目标选框
static Rect initRect;
// 视频帧
static Mat frame;
// 初始化标记
static bool initFlag = true;
static void onMouse(int event, int x, int y, int flag, void *)
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

int GetFramesByPosTxt(int argc, char* argv[], const std::string &videoFilename, const std::vector<std::string > &vecPosFilename)
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

	VideoCapture cap(videoFilename);
	assert(cap.isOpened());

	int nFrames = 0;
	Rect result;
	int frameStartIndex = 0;

	cap.set(CV_CAP_PROP_POS_FRAMES, frameStartIndex);
	namedWindow("Image", 0);
	resizeWindow("Image", img_width / fScale, img_height / fScale);
	moveWindow("Image", 0, 0);
	setMouseCallback("Image", onMouse);

	static int filenameIndex = 0;
	//const std::string filenamePrefix = "./res/";
	//std::ofstream fout;
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

			//if (fout.is_open())
			//	fout.close();

			++filenameIndex;
			//fout.open(filenamePrefix + std::to_string(filenameIndex) + ".txt");
			//assert(fout.is_open());
			//fout << frameIndex << " " << initRect << std::endl;
		}
		// 更新
		else {
			result = tracker.update(frame);
			rectangle(frame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 255), 1, 8);
			//cout << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
			//fout << frameIndex << " " << result << std::endl;
		}

		nFrames++;

		if (!SILENT) {
			imshow("Image", frame);
			int ch = waitKey(1);
		}
	}
	return 0;
}