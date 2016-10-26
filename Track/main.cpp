#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common.h"
#include "kcftracker.hpp"
#include "KalmanFilter.h"
#include "TreeDetect.h"
#include "LineDetect.h"

//// 给定点，转换成GPS点
//double convertToGSP(const cv::Point2d &centerGPS, cv::Point2d pt, const cv::Size &sz)
//{
//	// 帧中心点的GPS的坐标，由字幕得到 centerGPS
//	
//	// 转换pt到跟踪图中心距离
//	pt.x -= sz.width / 2;
//	pt.y -= sz.height / 2;
//
//
//}
#define SCALE_PIXEL(W, H, f)

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
static double calcDist(const cv::Point_<T> &pt1, const cv::Point_<U> &pt2)
{
	double x = pt1.x - pt2.x;
	double y = pt1.y - pt2.y;
	return sqrt(x * x + y * y);
}


// 直线pt到直线pt2,pt3的距离
template<typename T, typename U>
static double calcDist(const cv::Point_<T> &P, const cv::Point_<U> &A, const cv::Point_<U> &B)
{
	double dAP = sqrt((P.x - A.x) * (P.x - A.x) + (P.y - A.y) * (P.y - A.y));
	double dAB = sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));

	double cross = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);

	double cosTheta = cross / dAB / dAP;
	return dAP * sqrt(1 - cosTheta * cosTheta);
}

// 返回周围4个点的颜色，如果不一样，返回白色
cv::Scalar roundingScalar(const cv::Mat &img, const cv::Point &pt)
{
	const cv::Scalar left(img.at<cv::Vec3b>(pt.y, pt.x - 1)[0], img.at<cv::Vec3b>(pt.y, pt.x - 1)[1], img.at<cv::Vec3b>(pt.y, pt.x - 1)[2]);
	const cv::Scalar right(img.at<cv::Vec3b>(pt.y, pt.x + 1)[0], img.at<cv::Vec3b>(pt.y, pt.x + 1)[1], img.at<cv::Vec3b>(pt.y, pt.x + 1)[2]);
	const cv::Scalar top(img.at<cv::Vec3b>(pt.y - 1, pt.x)[0], img.at<cv::Vec3b>(pt.y - 1, pt.x)[1], img.at<cv::Vec3b>(pt.y - 1, pt.x)[2]);
	const cv::Scalar bottom(img.at<cv::Vec3b>(pt.y + 1, pt.x)[0], img.at<cv::Vec3b>(pt.y + 1, pt.x)[1], img.at<cv::Vec3b>(pt.y + 1, pt.x)[2]);
	if ((left == right) && (left == top) && (left == bottom))
		return left;
	else
		return cv::Scalar(255, 255, 255);
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
	cv::namedWindow(winName);
	cv::setMouseCallback(winName, onMouse);

	// init KCF
	bool HOG = true;
	bool FIXEDWINDOW = true;
	bool MULTISCALE = false;
	bool LAB = false;
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	CKalManFilter KF;

	int frameIndex = 1400;
	cap.set(CV_CAP_PROP_POS_FRAMES, frameIndex - 1);
	float dx = 0.0f, dy = 0.0f;
	//bool initVelocity = true;
	int framesCalcForVelocityTotal = 0;	// 使用多少帧来初始化速度参数，达到十帧后，标记initVelocity，同时初始化卡尔曼滤波器
	int frameIndexForInit = 0;
	bool initKF = false;

	cv::Rect lastRect;	 // 上一帧的跟踪结果
	cv::moveWindow(winName, 0, 0);

	////////////////////////////////////////////////////////////////////////// MAP
	const std::string mapFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.png";
	const std::string coordinateFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.txt";
	const std::string SRTFilename = "G:\\resources\\videos\\DJI_0002.SRT";
	const std::string markedMapFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\mark.png";

	CMap m(mapFilename, coordinateFilename, 1 << 1, markedMapFilename);
	std::vector<cv::Point2d> vecGPS;
	std::vector<double> vecHigh;
	parseGPSAndHighFromSRT(SRTFilename, vecGPS, vecHigh);
	cv::Mat mapROI;
	cv::Mat mapROIMarked;

	bool inRedArea = false;
	bool inBlueArea = false;
	bool inGreenArea = false;
	bool inBlackArea = false;

	//////////////////////////////////////////////////////////////////////////


	for (;;) {
		++frameIndex;
		cap >> frame;
		cv::resize(frame, frame, cv::Size(2048, 1080));
		//cv::resize(frame, frame, cv::Size(1024, 540));



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
			//tracker.updateTrain(frame);
			cv::Rect searchROI = tracker._extracted_roi; // 上一帧的跟踪框
			std::cout << peak_value << std::endl;

			// 树木检测，在原图中直接标记出来
			cv::Mat searchROIImg = frame(searchROI & cv::Rect(0, 0, frame.cols, frame.rows));
			//cv::Mat searchROIImg2 = searchROIImg.clone();
			cv::Mat colorFilteredImg;
			colorFilter(searchROIImg, colorFilteredImg);
			cv::Mat colorFilteredBGRImg;
			cv::cvtColor(colorFilteredImg, colorFilteredBGRImg, CV_GRAY2BGR);
			add(searchROIImg, colorFilteredBGRImg, searchROIImg);
			//addWeighted(searchROIImg, 0.2, colorFilteredImg, 0.8, 0, searchROIImg);
			//SHOW(searchROIImg);
			//SHOW(colorFilteredImg);


			cv::Rect searchROILine = searchROI;
			searchROILine.x -= searchROILine.width / 2;
			searchROILine.width += searchROILine.width;
			searchROILine &= cv::Rect(0, 0, frame.cols, frame.rows);
			cv::Mat searchROILImg = frame(searchROILine);
			//searchROIImg = frame(cv::Rect(searchROI.x - searchROI.width / 2, searchROI.y, searchROI.width * 2, searchROI.height));

			cv::Mat lineDetectedImg;
			std::vector<cv::Vec4i> lines = lineDetect(searchROIImg, lineDetectedImg);
			for (size_t i = 0; i < lines.size(); ++i) {
				cv::Vec4i l = lines[i];
				cv::Point pt1(l[0], l[1]);
				cv::Point pt2(l[2], l[3]);
				cv::line(searchROIImg, pt1, pt2, LIGHTBLUE, 4);
			}

			// searchROIImg就是中心点
			cv::Point2f lineDetectCenterPt(searchROIImg.cols / 2.0f, searchROIImg.rows / 2.0f);
			bool occlused = false;
			int occusedNum = 0;
			for (size_t i = 0; i < lines.size(); ++i) {
				const cv::Vec4i &l = lines[i];
				double dist = calcDist(lineDetectCenterPt, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]));
				if (dist < lineDetectCenterPt.x) {
					++occusedNum;
				}
			}
			if (lines.size() * 0.67 < occusedNum)
				occlused = true;




			//SHOW(lineDetectedImg);

			cv::Point2f KFPt;
			
			if (peak_value >= 0.45f) {
				tracker.updateTrain(frame);
				KFPt = KF.predict(lastRect.x, lastRect.y);
			}
			else if (peak_value < 0.45f && peak_value > 0.35f) {


				KFPt = KF.predict(lastRect.x, lastRect.y);
			}
			else {
				if (occlused) {
					std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
					if (inRedArea)
						std::cout << "RED" << std::endl;
					else if (inBlueArea)
						std::cout << "BLUE" << std::endl;
					else if (inBlackArea)
						std::cout << "BLACK" << std::endl;
					else if (inGreenArea)
						std::cout << "GREEN" << std::endl;
					else
						std::cout << "NOOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;
				}


				if (initKF) {
					KFPt = KF.predict();
					resRect = cv::Rect(KFPt.x, KFPt.y, initRect.width, initRect.height);
					tracker.setROI(KFPt.x, KFPt.y, frame);
		
					////////////////////////////////////////////////////////////////////////// 增加直线判断
					int x = resRect.x - lastRect.x;
					int y = resRect.y - lastRect.y;
					for (size_t i = 0; i < lines.size(); ++i) {

					}
					//////////////////////////////////////////////////////////////////////////

				}
				else {
					std::cerr << "Warning: Having no init Kalman Filter Yes." << std::endl;
				}

			}

			// std::cout << KFPt << " " << resRect.tl() << " " << calcDist(KFPt, resRect.tl()) << std::endl;

			if (frameIndex - frameIndexForInit < 5) {
				dx += resRect.x - lastRect.x;
				dy += resRect.y - lastRect.y;
			}
			else if (frameIndex - frameIndexForInit == 5) {
				dx /= 5;
				dy /= 5;
				KF.init(resRect.x, resRect.y, dx, dy);
				initKF = true;
			}

			lastRect = resRect;


			//////////////////////////////////////////////////////////////////////////
			int secondIndex = (frameIndex / 25 + 1);
			if (frameIndex % 25 == 0) {
				m.calcFrame(vecGPS[secondIndex - 1], cv::Mat(cv::Size(4096, 2160), 0), vecHigh[secondIndex - 1], 729, 117);
				//m.drawMapROI(mapROI);
				m.getMapROI(mapROI, mapROIMarked);


			}
			if (mapROI.data) {
				cv::Point2f targerPt(lastRect.x + lastRect.width / 2, lastRect.y + lastRect.height / 2);
				targerPt.x /= (frame.cols * 1.0f / mapROI.cols);
				targerPt.y /= (frame.rows * 1.0f / mapROI.rows);
				DRAW_CROSS(mapROI, targerPt, RED, 1);
				SHOW(mapROI);
				SHOW(mapROIMarked);

				const cv::Scalar &color = roundingScalar(mapROIMarked, targerPt);
				if (color == RED)
					inRedArea = true;
				else
					inRedArea = false;

				if (color == GREEN)
					inGreenArea = true;
				else
					inGreenArea = false;

				if (color == BLUE)
					inBlueArea = true;
				else
					inBlueArea = false;

				if (color == BLACK)
					inBlackArea = true;
				else
					inBlackArea = false;


			}

			//////////////////////////////////////////////////////////////////////////



			//// 传感器给定
			//float angle = (28 + 90) / CV_PI;
			//const float height = 100.0f;	// 飞行高度
			//cv::Point2d center(frame.cols / 2, frame.rows / 2);
			//cv::Point resPt(resRect.x + resRect.width / 2, resRect.y + resRect.height / 2);
			//cv::Point2d ptInFrame;
			//// 用图片大小来归一化
			//ptInFrame.x = (resPt.x - center.x) / frame.cols;
			//ptInFrame.y = (resPt.y - center.y) / frame.rows;
			//cv::Point2d ptInMap;
			


			cv::rectangle(frame, resRect, GREEN);
			cv::rectangle(frame, searchROI, PINK);

			if (initKF)
				cv::rectangle(frame, cv::Rect(KFPt.x, KFPt.y, resRect.width, resRect.height), YELLOW);
		}
		cv::imshow(winName, frame);
		char ch = cv::waitKey(10);
		if (toupper(ch) == 'Q')
			break;
		else if (toupper(ch) == 'P')
			cv::waitKey(0);
	}

	return 0;
}