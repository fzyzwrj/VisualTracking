#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "kcftracker.hpp"
#include "GetFramesByPos.h"
#include "common.h"
#include "kcftracker.hpp"


//class KCFTrackerWrapper
//{
//public:
//	const static int m_trackerNum = 8;
//	KCFTrackerWrapper(const cv::Mat &img, const cv::Rect &initRect)
//	{
//		float angle = 360.0f;
//		cv::Rect scaleInitRect = initRect;
//
//		for (int i = 0; i < m_trackerNum; ++i) {
//			float anglePer = angle / m_trackerNum * i;
//
//		}
//	}
//private:
//	// 当前选择的跟踪器，是对vecTracker里面的引用
//	KCFTracker curTracker;
//	// 备用的跟踪器
//	std::vector<KCFTracker> vecTracker;
//	cv::Rect m_initRect;
//	
//
//};

// 初始目标选框
static cv::Rect initRect;
// 视频帧
static cv::Mat frame;
// 初始化标记
static bool readyInitKCF = true;

static const std::string frameWinName = "SHOW";

static void onMouse(int event, int x, int y, int flag, void *)
{
	static cv::Point prePoint;
	static cv::Point curPoint;
	static cv::Mat imgTmp;
	// 鼠标左键下
	if (event == CV_EVENT_LBUTTONDOWN) {
		imgTmp = frame.clone();
		prePoint = cv::Point(x, y);
		readyInitKCF = true;
	}	// 鼠标左键按下并移动
	else if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_FLAG_LBUTTON)) {
		imgTmp = frame.clone();	// 每次移动都要拷贝原图用于画图
		curPoint = cv::Point(x, y);
		cv::rectangle(imgTmp, prePoint, curPoint, RED, 1, 8);
		cv::imshow(frameWinName, imgTmp);
	}	// 松开左键，选框完毕
	else if (event == CV_EVENT_LBUTTONUP) {
		initRect = cv::Rect(prePoint, curPoint);
		readyInitKCF = false;
		if (initRect.width < 2 || initRect.height < 2)
			readyInitKCF = true;
	}
	else if (event == CV_EVENT_LBUTTONDBLCLK) {
		std::cout << "Closed." << std::endl;
		exit(0);
	}
}

int main(int argc, char *argv[])
{
	KCFTracker tracker(true, true, false, false);
	const std::string dirFilename("C:\\resources\\res\\rotate_2\\");
	const int imgTotal = 20;


	cv::namedWindow(frameWinName);
	cv::moveWindow(frameWinName, 0, 0);
	cv::setMouseCallback(frameWinName, onMouse);


	for (int i = 0; i < imgTotal; ++i) {
		std::string imgFilename = dirFilename + std::to_string(i) + "_rotate.jpg";
		frame = cv::imread(imgFilename);
		assert(frame.data);
		cv::resize(frame, frame, cv::Size(2048, 1080));

		// 是否重新跟踪目标
		if (i == 0) {
			cv::imshow(frameWinName, frame);
			cv::waitKey(0);
			tracker.init(initRect, frame);
			cv::rectangle(frame, initRect, RED, 1, 8);
		}
		else {
			float peak_value = 0.0f;
			tracker.setROI(initRect.x, initRect.y, frame);
			cv::Rect resRect = tracker.updateWithoutTrain(frame, peak_value);
			tracker.setROI(initRect.x, initRect.y, frame);
			std::cout << peak_value << std::endl;
			//tracker.updateTrain(frame);
			cv::rectangle(frame, resRect, GREEN);
		}
		cv::imshow(frameWinName, frame);
		cv::waitKey(0);
	}
}

int main1(int argc, char *argv[])
{
	/// 获取图片帧
	//const std::string videoName = "G:\\resources\\videos\\DJI_0001.MOV";

	//for (size_t i = 1; i <= 77; ++i) {
	//	const std::string dirName = "video" + std::to_string(i);
	//	const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\" + std::to_string(i) + ".txt";
	//	std::vector<cv::Rect> vecTrackPos;
	//	std::vector<int> vecFrameIndex;
	//	getTrackPos(posFilename, vecTrackPos, vecFrameIndex);

	//	getFramesByPos(videoName, vecTrackPos, vecFrameIndex, dirName);
	//}

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

	const std::string videoName = "G:\\resources\\videos\\DJI_0001.MOV";
	for (size_t i = 1; i <= 77; ++i) {
		const std::string dirName = "video" + std::to_string(i);
		const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\" + std::to_string(i) + ".txt";
		std::vector<cv::Rect> vecTrackPos;
		std::vector<int> vecFrameIndex;
		getTrackPos(posFilename, vecTrackPos, vecFrameIndex);
		scaleTrackPos(vecTrackPos, 4);

		//getFramesByPos(videoName, vecTrackPos, vecFrameIndex, dirName);
		cv::VideoCapture cap(videoName);
		assert(cap.isOpened());
		cap.set(CV_CAP_PROP_POS_FRAMES, vecFrameIndex[0]);
		cv::Mat frame;

		cv::namedWindow("DEMO");
		//cv::resizeWqindow("DEMO", 1024, 540);
		cv::moveWindow("DEMO", 0, 0);


		//std::ofstream fout("4k_peak_" + std::to_string(i) + ".txt");
		std::ofstream fout("1k_peak_" + std::to_string(i) + ".txt");// 1024 x 540

		assert(fout.is_open());
		for (size_t j = 0; j < vecTrackPos.size(); ++j) {
			cap >> frame;
			cv::resize(frame, frame, cv::Size(1024, 540));

			if (j == 0) {
				tracker.init(vecTrackPos[0], frame);
				cv::rectangle(frame, vecTrackPos[0], RED, 1, 8);
			}
			else {
				float peak_value = 0.0f;
				cv::Rect res = tracker.update(frame, peak_value);
				cv::rectangle(frame, res, GREEN, 1, 8);
				
				char buf[256];
				sprintf(buf, "%-8d%.3f", vecFrameIndex[j], peak_value);
				fout << buf;
				//fout << vecFrameIndex[j] << " " << peak_value;
				if (peak_value < 0.4f)
					fout << " ###";
				fout << std::endl;
			}
			SHOW_WAIT("DEMO", frame);
		}
		fout.close();
	}
	return 0;
}
