#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common.h"
#include "kcftracker.hpp"
#include "KalmanFilter.h"

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

template<typename T, typename U>
static double calcDist(cv::Point_<T> pt1, cv::Point_<U> pt2)
{
	double x = pt1.x - pt2.x;
	double y = pt1.y - pt2.y;
	return sqrt(x * x + y * y);
}

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
	const std::string videoFilename = "G:\\resources\\videos\\DJI_0002.MOV";
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
	CKalManFilter KF;

	int frameIndex = 0;
	cap.set(CV_CAP_PROP_POS_FRAMES, 2500);
	float dx = 0.0f, dy = 0.0f;
	//bool initVelocity = true;
	int framesCalcForVelocityTotal = 0;	// 使用多少帧来初始化速度参数，达到十帧后，标记initVelocity，同时初始化卡尔曼滤波器
	int frameIndexForInit = 0;
	bool initKF = false;

	cv::Rect lastRect;	 // 上一帧的跟踪结果
	for (;;) {
		++frameIndex;
		cap >> frame;
		cv::resize(frame, frame, cv::Size(1024, 540));
		if (frameIndex == 0 || initFlag) {
			cv::imshow(winName, frame);
			cv::waitKey(0);
			tracker.init(initRect, frame);
			
			//KF.init(initRect.x, initRect.y, -5 / 4.0, 12 / 4.0);
			//framesCalcForVelocityTotal = 0;
			//initVelocity = false;
			frameIndexForInit = frameIndex;
			dx = 0.0f;
			dy = 0.0f;
			initKF = false;
			
			rectangle(frame, initRect, RED, 1, 8);

			lastRect = initRect;
		}
		else {
			float peak_value = 0.0f;
			cv::Rect resRect = tracker.updateWithoutTrain(frame, peak_value);
			cv::Rect searchROI = tracker._extracted_roi;
			std::cout << peak_value << std::endl;

			cv::Point2f KFPt;
			if (peak_value >= 0.45f) {
				tracker.updateTrain(frame);
				KFPt = KF.predict(lastRect.x, lastRect.y);
			}
			else if (peak_value < 0.45f && peak_value > 0.35f) {
				KFPt = KF.predict(lastRect.x, lastRect.y);
			}
			else {
				if (initKF) {
					KFPt = KF.predict();
					resRect = cv::Rect(KFPt.x, KFPt.y, initRect.width, initRect.height);
					tracker.setROI(KFPt.x, KFPt.y, frame);
				}
				else {
					std::cout << "NO INIT KF YET." << std::endl;
				}

			}
			std::cout << KFPt << " " << resRect.tl() << " " << calcDist(KFPt, resRect.tl()) << std::endl;

			if (frameIndex - frameIndexForInit < 5) {
				dx += resRect.x - lastRect.x;
				dy += resRect.y - lastRect.y;
			}
			else if (frameIndex - frameIndexForInit == 5) {
				dx /= 5;
				dy /= 5;
				KF.init(resRect.x, resRect.y, dx, dy);
				initKF = true;
				std::cout << "###################### INIT KF" << dx << " " << dy << std::endl;
			}

			lastRect = resRect;

			cv::rectangle(frame, resRect, GREEN);
			cv::rectangle(frame, searchROI, PINK);
			if (initKF)
				cv::rectangle(frame, cv::Rect(KFPt.x, KFPt.y, resRect.width, resRect.height), YELLOW);
		}
		cv::imshow(winName, frame);
		char ch = cv::waitKey(10);
		if (toupper(ch) == 'Q')
			break;
	}

	return 0;
}