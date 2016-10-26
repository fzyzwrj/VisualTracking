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



#endif /* COMMON_H__ */
