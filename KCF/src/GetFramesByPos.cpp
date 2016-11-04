#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

#include <direct.h>
#include "utils.h"
#include "utils_opencv.h"
#include "GetTrackingPosTxt.h"


cv::Rect rectAddBorder(const cv::Rect &rect, int borderWidth)
{
	const cv::Point pt(borderWidth, borderWidth);
	return cv::Rect(rect.tl() - pt, rect.br() + pt);
}

// 将矩形变成正方形
cv::Rect rectSetToSquare(const cv::Rect &rect)
{
	cv::Rect square = rect;
	int width = rect.width;
	int high = rect.height;
	if (width > high) {
		int gap = width - high;
		square.y -= gap / 2;
		square.height += gap;
	}
	else if (width < high) {
		int gap = high - width;
		square.x -= gap / 2;
		square.width += gap;
	}
	return square;
}

// 缩放到1.4倍，用于裁剪
cv::Rect scaleRect(const cv::Rect &rect)
{
	float scale = sqrt(2.0f);
	cv::Rect resRect = rect;
	//resRect.width *= scale;
	//resRect.height *= scale;
	{
		int gap = (scale - 1) * resRect.width;
		resRect.width += gap;
		resRect.x -= gap / 2;
	}
	{
		int gap = (scale - 1) * resRect.height;
		resRect.height += gap;
		resRect.y -= gap / 2;
	}
	return resRect;
}

// rotate and crop
void rotateAndCrop(const cv::Mat &img, int width, int height, std::vector<cv::Mat> &vecImg)
{
	// square
	assert(width > 0);
	assert(height > 0);
	assert(width - height > -5);
	assert(width - height < 5);
	cv::Mat srcImg = img.clone();
	cv::Size sz(srcImg.cols, srcImg.rows);
	cv::Size szRes(width, height);
	cv::Point2f center(img.cols / 2.0f, img.rows / 2.0f);
	double angle = 360.0;
	double anglePerImg = angle / 360;
	//vecImg.reserve(360);
	//vecImg.reserve(angle);
	for (int i = 0; i < 360; ++i)
	{
		double angleCurrentImg = i * anglePerImg;
		cv::Mat rotMat = cv::getRotationMatrix2D(center, angleCurrentImg, 1);
		cv::Mat rotImg;
		cv::warpAffine(srcImg, rotImg, rotMat, sz);
		cv::Mat cropImg;
		cv::getRectSubPix(rotImg, szRes, center, cropImg);
		//cv::imshow("RES", cropImg);
		//cv::waitKey(0);
		vecImg.push_back(cropImg);
		//vecImg[i] = cropImg.clone();
		//cv::imshow("CLONE", vecImg[i]);
		//cv::waitKey(0);
	}
}

using namespace std;
using namespace cv;

int getFramesByPos(const std::string &videoFilename, const std::vector<cv::Rect> &vecTrackPos, const std::vector<int> &vecFrameIndex, const std::string &dirName)
{
	//std::cout << "start getFramesByPos" << std::endl;
	assert(vecTrackPos.size() == vecFrameIndex.size());
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	_mkdir(dirName.c_str());

	cap.set(CV_CAP_PROP_POS_FRAMES, vecFrameIndex[0]);
	cv::Mat frame;
	static int cnt = 1;
	if (cnt == 1) {
		srand(unsigned(time(NULL)));
		++cnt;
	}

	for (size_t i = 0; i < vecTrackPos.size(); ++i) {
		cap >> frame;
		const cv::Rect rectBG(0, 0, frame.cols, frame.rows);
		//cv::Rect trackRect = rectAddBorder(vecTrackPos[i], 40);
		cv::Rect trackRect = vecTrackPos[i];
		trackRect = rectSetToSquare(trackRect);
		cv::Rect largeRect = scaleRect(trackRect);

		const cv::Rect ROIRect = largeRect & rectBG;
		if (ROIRect == largeRect) {
			// 边缘足够扩展才可以旋转
			cv::Mat scaleImg = frame(ROIRect).clone();
			std::vector<cv::Mat> vecImg;
			//std::cout << "start rotate and crop" << std::endl;
			rotateAndCrop(scaleImg, trackRect.width, trackRect.height, vecImg);
			char buf[256];
			//std::cout << "start write file" << std::endl;
			for (int j = 0; j < vecImg.size(); ++j) {
				sprintf(buf, "%s/%d_%d.jpg", dirName.c_str(), i, j);
				cv::imwrite(buf, vecImg[j]);
			}
		}

		//cv::Mat ROIImg = frame(ROIRect);
		//		
		//std::string ROIFilename = dirName + "/" + std::to_string(i) + ".jpg";
		//cv::imwrite(ROIFilename, ROIImg);
	}
	return 0;
}


// 随机截取
int getFramesByRandom(const std::string &videoFilename, const std::vector<cv::Rect> &vecTrackPos, const std::vector<int> &vecFrameIndex, const std::string &dirName)
{
	assert(vecTrackPos.size() == vecFrameIndex.size());
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	_mkdir(dirName.c_str());

	cap.set(CV_CAP_PROP_POS_FRAMES, vecFrameIndex[0]);
	cv::Mat frame;
	for (size_t i = 0; i < vecTrackPos.size(); ++i) {
		cap >> frame;
		const cv::Rect rectBG(0, 0, frame.cols, frame.rows);
		//cv::Rect trackRect = rectAddBorder(vecTrackPos[i], 40);
		cv::Rect trackRect = vecTrackPos[i];
		
		// random
		trackRect.x = rand() % rectBG.width;
		trackRect.y = rand() % rectBG.height;

		trackRect = rectSetToSquare(trackRect);
		cv::Rect largeRect = scaleRect(trackRect);

		const cv::Rect ROIRect = largeRect & rectBG;
		if (ROIRect == largeRect) {
			//cv::Mat scaleImg = frame(ROIRect).clone();
			std::vector<cv::Mat> vecImg;
			char buf[256];
			for (int j = 0; j < 360; ++j) {
				largeRect.x = rand() % rectBG.width;
				largeRect.y = rand() % rectBG.height;
				cv::Rect newRect = largeRect & rectBG;
				if (newRect == largeRect) {
					cv::Mat img = frame(largeRect);
					sprintf(buf, "%s/%d_%d.jpg", dirName.c_str(), i, j);
					cv::imwrite(buf, img);
				}
			}
		}
	}
	return 0;
}