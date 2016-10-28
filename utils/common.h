#pragma once
#ifndef COMMON_H__
#define COMMON_H__

#include <iostream>
#include <fstream>
#include <string>
#include <fstream>
#include <opencv2\opencv.hpp>
#include "common.h"


#define MY_EXPORT __declspec(dllexport)

#define SHOW(img) \
	do { \
		cv::imshow(#img, img);\
	} while (0)

#define SHOW_WAIT(winName, img) \
	do {\
		cv::imshow(winName, img);\
		char ch = cv::waitKey(50);\
		if (toupper(ch) == 'Q')\
			exit(0);\
	} while (0)

#define DRAW_CROSS(image, center, color, d)\
	do{\
		cv::line(image, cv::Point(center.x - d, center.y - d), \
			cv::Point(center.x + d, center.y + d), color, 4, CV_AA, 0); \
		cv::line(image, cv::Point(center.x - d, center.y + d), \
			cv::Point(center.x + d, center.y - d), color, 4, CV_AA, 0); \
	} while (0)

// 对函数调用计时
#define TEST_TIME(func)\
	do {\
		cv::TickMeter tm;\
		tm.start();\
		func;\
		tm.stop();\
		/*std::cout << "########## TIME: " << #func << " " << tm.getTimeMilli() << "ms" << std::endl;*/\
	} while (0)


MY_EXPORT void getTrackPos(const std::string &posFilename, std::vector<cv::Rect> &vecTrackPos, std::vector<int> &vecFrameIndex);

MY_EXPORT void scaleTrackPos(std::vector<cv::Rect> &vecTrackPos, const int scale);

const cv::Scalar RED(0, 0, 255);
const cv::Scalar PINK(230, 130, 255);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar LIGHTBLUE(255, 255, 160);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar PURPLE(205, 0, 205);
const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar BLACK(0, 0, 0);


// 模板只能放在H文件中了，如果放在common.h无法特化
// 计算点P到直线AB的距离，通过矢量方向得到
template<typename T, typename U>
float calcDist(const cv::Point_<T> &P, const cv::Point_<U> &A, const cv::Point_<U> &B)
{
	float dAP = sqrt((P.x - A.x) * (P.x - A.x) + (P.y - A.y) * (P.y - A.y));
	float dAB = sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));

	float cross = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);

	float cosTheta = cross / dAB / dAP;
	return dAP * sqrt(1 - cosTheta * cosTheta);
}

// 计算两点之间的距离
template<typename T, typename U>
float calcDist(const cv::Point_<T> &pt1, const cv::Point_<U> &pt2)
{
	float x = pt1.x - pt2.x;
	float y = pt1.y - pt2.y;
	return sqrt(x * x + y * y);
}

// 改写成Vec2f即可
// 计算速度与直线的夹角
template<typename T, typename U>
float calcAngle(const cv::Point_<T> &V, const cv::Point_<U> &A, const cv::Point_<U> &B)
{
	auto V2 = B - A;
	float dV = sqrt(V.x * V.x + V.y * V.y);
	float dV2 = sqrt(V2.x * V2.x + V2.y * V2.y);
	
	float cross = V.x * V2.x + V.y * V2.y;

	float cosTheta = cross / dV / dV2;
	float angle = acos(cosTheta);
	return angle;
}



#endif /* COMMON_H__ */
