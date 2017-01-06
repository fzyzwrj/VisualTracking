#include <cmath>
#include <cmath>
#include <iostream>
#include <assert.h>
#include <fstream>
#include <string>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>

#include "utils.h"
#include "utils_opencv.h"
#include "LineDetect.h"

#define SHOW(img) \
	do { \
		cv::imshow(#img, img);\
	} while (0)

#define MOVE_WIN(img) \
	do {\
		/*cv::moveWindow(#img, 400, 400);*/\
	}while (0)

static int g_edgeThresh = 30;	// 这个参数一般70比较适合
static cv::Mat g_blurImg;
static cv::Mat g_cannyImg;
static cv::Mat g_houghPImg;

static cv::Mat g_sobelxImg;
static cv::Mat g_sobelyImg;

cv::Mat getHistImg(const cv::MatND& hist, int &maxLoc)
{
	double maxVal = 0;
	double minVal = 0;

	//找到直方图中的最大值和最小值
	cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
	int histSize = hist.rows;
	cv::Mat histImg(histSize, histSize, CV_8U, cv::Scalar(255));
	// 设置最大峰值为图像高度的90%
	int hpt = static_cast<int>(0.9*histSize);

	for (int h = 0; h < histSize; h++)
	{
		float binVal = hist.at<float>(h);
		if (fabs(binVal - maxVal) < EPS)
			maxLoc = h;
		int intensity = static_cast<int>(binVal*hpt / maxVal);
		line(histImg, cv::Point(h, histSize), cv::Point(h, histSize - intensity), cv::Scalar::all(0));
	}

	return histImg;
}

// 返回灰度值
int calcHist(const cv::Mat &grayImg)
{
	const int channels[1] = { 0 };
	const int histSize[1] = { 256 };
	float hranges[2] = { 0, 255 };
	const float *ranges[1] = { hranges };
	cv::MatND hist;
	cv::calcHist(&grayImg, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
	int maxLoc = 0;
	cv::Mat histImg = getHistImg(hist, maxLoc);
	std::cout << maxLoc << std::endl;
	cv::Mat roadImg = cv::Mat::zeros(grayImg.size(), CV_8UC1);
	for (int r = 0; r < roadImg.rows; ++r) {
		for (int c = 0; c < roadImg.cols; ++c) {
			if (abs(maxLoc - grayImg.at<uchar>(r, c)) < 25)
				roadImg.at<uchar>(r, c) = 255;
		}
	}
	SHOW(roadImg);

	SHOW(histImg);
	return 0;
}

void onCanny(int, void *)
{
	cv::Canny(g_blurImg, g_cannyImg, g_edgeThresh, 3 * g_edgeThresh);

	/*cv::Mat exImg;
	cv::Mat se(3, 3, CV_8U, cv::Scalar(1));
	cv::morphologyEx(g_cannyImg, exImg, CV_MOP_CLOSE, se);
	cv::morphologyEx(exImg, exImg, CV_MOP_OPEN, se);
	cv::morphologyEx(exImg, exImg, CV_MOP_OPEN, se);
	SHOW(exImg);*/
	cv::Mat cannyContours;
	//std::vector<cv::Vec4i> contours;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(g_cannyImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point());

	auto it = contours.begin();
	while (it != contours.end()) {
		auto rRect = cv::minAreaRect(*it);
		double ratio = 1.0;
		if (rRect.size.width < rRect.size.height)
			ratio = rRect.size.height * 1.0 / rRect.size.width;
		else
			ratio = rRect.size.width * 1.0 / rRect.size.height;

		if (ratio < 20 && rRect.size.area() < 25 * 50)
			it = contours.erase(it);
		else
			++it;
	}

	cv::Mat drawImg = cv::Mat::zeros(g_cannyImg.size(), CV_8UC3);
	cv::RNG rng(12345);
	for (int i = 0; i < contours.size(); ++i) {
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::drawContours(drawImg, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
	}

	SHOW(drawImg);
	SHOW(g_cannyImg); 
	MOVE_WIN(g_cannyImg);
	/// houghlines
	//std::vector<cv::Vec2f> lines;
	//cv::HoughLines(srcImg, lines, 1, cv::CV_PI / 180, 150, 0, 0);	//adjust parameter
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(g_cannyImg, lines, 1, CV_PI / 180, 80, 80, 10); //adjust parameter
	cv::Mat houghPImg(g_cannyImg.size(), g_cannyImg.type(), cv::Scalar::all(0));

	// 删除角度过大的线段
	//for (auto it = lines.begin(); it != lines.end(); /**/) {
	//	const cv::Point pt1((*it)[0], (*it)[1]);
	//	const cv::Point pt2((*it)[2], (*it)[3]);
	//	const cv::Point pt = pt2 - pt1;
	//	double angle = atan(pt.y * 1.0 / pt.x);
	//	angle = angle / CV_PI * 180;
	//	angle = fabs(angle);
	//	if (angle >= 20.0 && angle <= 70.0)
	//		it = lines.erase(it);
	//	else
	//		++it;
	//}
	g_houghPImg.create(g_cannyImg.size(), CV_8U);
	g_houghPImg = cv::Scalar::all(0);
	for (size_t i = 0; i < lines.size(); ++i) {
		float rho = lines[i][0], theta = lines[i][1];
		const cv::Vec4i &l = lines[i];
		const cv::Point pt1(l[0], l[1]);
		const cv::Point pt2(l[2], l[3]);
		cv::line(g_houghPImg, pt1, pt2, cv::Scalar(255, 0, 255));
	}
	//SHOW(g_houghPImg);
	cv::moveWindow("g_houghPImg", 0, 0);
}

void lineDetectDetailed(const cv::Mat &srcImg, cv::Mat &houghPImg)
{
	g_houghPImg = houghPImg;
	cv::Mat grayImg;
	cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);

	//cv::GaussianBlur
	cv::blur(grayImg, g_blurImg, cv::Size(3, 3));
	//g_blurImg = grayImg.clone();

	static const int edgeThreshMax = 255;
	g_cannyImg = g_blurImg.clone();
	g_sobelxImg = g_blurImg.clone();
	g_sobelyImg = g_blurImg.clone();

	// OSTU
	/*cv::Mat tmpImg;
	cv::threshold(g_blurImg, tmpImg, 200, 255, CV_THRESH_OTSU);
	SHOW(tmpImg);*/
	cv::Sobel(g_blurImg, g_sobelxImg, CV_8UC1, 1, 0, 3, 1, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(g_sobelxImg, g_sobelxImg);
	cv::Sobel(g_blurImg, g_sobelyImg, CV_8UC1, 0, 1, 3, 1, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(g_sobelyImg, g_sobelyImg);
	SHOW(g_sobelxImg);
	SHOW(g_sobelyImg);

	cv::imshow("cannyImg", g_cannyImg);
	cv::moveWindow("cannyImg", 0, 0);
	cv::createTrackbar("cannyBar", "cannyImg", &g_edgeThresh, edgeThreshMax, onCanny);

	cv::imshow("srcImg", srcImg);

	calcHist(g_cannyImg);

	cv::waitKey(0);
	houghPImg = g_houghPImg;
}

std::vector<cv::Vec4i> lineDetect(const cv::Mat &srcImg, cv::Mat &houghPImg)
{
	cv::Mat grayImg;
	cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
	cv::Mat cannyImg;
	const int edgeThresh = 50;
	cv::Canny(grayImg, cannyImg, edgeThresh, 3 * edgeThresh);

	std::vector<cv::Vec4i> lines;
	//double minLineLength = srcImg.rows < srcImg.cols ? srcImg.rows : srcImg.cols;
	//minLineLength -= 10;
	cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 80, 0, 10); //adjust parameter
	//cv::houghPImg(cannyImg.size(), cannyImg.type(), cv::Scalar::all(0));

	// 删除角度过大的线段
	//for (auto it = lines.begin(); it != lines.end(); /**/) {
	//	const cv::Point pt1((*it)[0], (*it)[1]);
	//	const cv::Point pt2((*it)[2], (*it)[3]);
	//	const cv::Point pt = pt2 - pt1;
	//	double angle = atan(pt.y * 1.0 / pt.x);
	//	angle = angle / CV_PI * 180;
	//	angle = fabs(angle);
	//	if (angle >= 20.0 && angle <= 70.0)
	//		it = lines.erase(it);
	//	else
	//		++it;
	//}
	//houghPImg.create(cannyImg.size(), CV_8U);
	//houghPImg = cv::Scalar::all(0);
	//for (size_t i = 0; i < lines.size(); ++i) {
	//	//float rho = lines[i][0], theta = lines[i][1];
	//	const cv::Vec4i &l = lines[i];
	//	const cv::Point pt1(l[0], l[1]);
	//	const cv::Point pt2(l[2], l[3]);
	//	cv::line(houghPImg, pt1, pt2, cv::Scalar(255, 0, 255));
	//}
	return lines;
}

size_t lineDetect(const cv::Mat &srcImg, cv::vector<cv::Vec4i> &lines)
{
	cv::Mat grayImg;
	cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
	cv::Mat cannyImg;
	const int edgeThresh = 50;
	cv::Canny(grayImg, cannyImg, edgeThresh, 3 * edgeThresh);

	assert(lines.empty());
	//std::vector<cv::Vec4i> lines;
	//double minLineLength = srcImg.rows < srcImg.cols ? srcImg.rows : srcImg.cols;
	//minLineLength -= 10;
	cv::HoughLinesP(cannyImg, lines, 1, CV_PI / 180, 80, 0, 10); //adjust parameter
																 //cv::houghPImg(cannyImg.size(), cannyImg.type(), cv::Scalar::all(0));
	return lines.size();
}

void drawLines(cv::Mat &img, const cv::vector<cv::Vec4i> &lines)
{
	for (size_t i = 0; i < lines.size(); ++i) {
		const cv::Vec4i &l = lines[i];
		const cv::Point pt1(l[0], l[1]);
		const cv::Point pt2(l[2], l[3]);
		cv::line(img, pt1, pt2, RED);
	}
}


void parseGPSAndHighFromSRT(const std::string &SRTFilename, std::vector<cv::Point2d> &vecPos, std::vector<double> &vecHigh, const std::string &saveFilename)
{
	assert(vecPos.empty());
	//std::vector<cv::Point2d> vecPos;
	std::fstream fin(SRTFilename);
	assert(fin.is_open());
	std::string line;
	while (std::getline(fin, line)) {
		double latitude = 0;
		double longitude = 0;
		double high = 0;
		int satelliteNum = 0;
		if (line[0] == 'G' && line[1] == 'P' && line[2] == 'S') {
			sscanf_s(line.c_str(), "GPS(%lf,%lf, %d) Hb:%lf", &longitude, &latitude, &satelliteNum, &high);
			vecPos.push_back(cv::Point2d(longitude, latitude));
			vecHigh.push_back(high);
		}
	}

	if (!saveFilename.empty()) {
		std::ofstream fout(saveFilename);
		assert(fout.is_open());
		for (size_t i = 0; i < vecPos.size(); ++i) {
			char str[256];
			sprintf_s(str, "%4.6lf ,%4.6lf, %4.6lf", vecPos[i].x, vecPos[i].y, vecHigh[i]);
			fout << str << std::endl;
		}
		fout.close();
	}
}