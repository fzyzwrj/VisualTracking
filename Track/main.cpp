#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common.h"
#include "kcftracker.hpp"

static void initKCF()
{

}
// 初始目标选框
static cv::Rect initRect;
// 视频帧
static cv::Mat frame;
// 初始化标记
static bool initFlag = true;

static const std::string winName = "SHOW";

static void onMouse(int event, int x, int y, int flag, void *)
{
	static cv::Point prePoint;
	static cv::Point curPoint;
	static cv::Mat imgTmp;
	// 鼠标左键下
	if (event == CV_EVENT_LBUTTONDOWN) {
		imgTmp = frame.clone();
		prePoint = cv::Point(x, y);
		initFlag = true;
	}	// 鼠标左键按下并移动
	else if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_FLAG_LBUTTON)) {
		imgTmp = frame.clone();	// 每次移动都要拷贝原图用于画图
		curPoint = cv::Point(x, y);
		cv::rectangle(imgTmp, prePoint, curPoint, RED, 1, 8);
		cv::imshow(winName, imgTmp);
	}	// 松开左键，选框完毕
	else if (event == CV_EVENT_LBUTTONUP) {
		initRect = cv::Rect(prePoint, curPoint);
		initFlag = false;
		if (initRect.width < 2 || initRect.height < 2)
			initFlag = true;
	}
	else if (event == CV_EVENT_LBUTTONDBLCLK) {
		std::cout << "Closed." << std::endl;
		exit(0);
	}
}

int main(int argc, char *argv[])
{
	const std::string videoFilename = "G:\\resources\\videos\\DJI_0001.MOV";
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	cv::namedWindow(winName, 1);
	cv::setMouseCallback(winName, onMouse);

	// init KCF
	bool HOG = true;
	bool FIXEDWINDOW = true;
	bool MULTISCALE = false;
	bool LAB = false;
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	int frameIndex = 0;
	for (;;) {
		++frameIndex;
		cap >> frame;
		cv::resize(frame, frame, cv::Size(1024, 540));
		if (frameIndex == 0 || initFlag) {
			cv::imshow(winName, frame);
			cv::waitKey(0);
			tracker.init(initRect, frame);
			rectangle(frame, initRect, RED, 1, 8);
		}
		else {
			float peak_value = 0.0f;
			cv::Rect resRect = tracker.update(frame, peak_value);
			cv::rectangle(frame, resRect, GREEN);
		}
		cv::imshow(winName, frame);
		char ch = cv::waitKey(10);
		if (toupper(ch) == 'Q')
			break;
	}

	return 0;
}