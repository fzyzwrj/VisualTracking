#pragma once
#ifndef COMMON_H__
#define COMMON_H__

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <assert.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <stack>
#include <queue>
#include <set>
#include <functional>
#include <algorithm>
#include <iomanip>
#include <iterator>

#include <opencv2\opencv.hpp>

#include <direct.h>
// ��������
#define MY_EXPORT __declspec(dllexport)

// ASSERT
#define MY_ASSERT(expr)	(void)((expr) || (My_assert(#expr, __FUNCTION__, __FILE__, __LINE__), 0))

inline void My_assert(const char *expr, const char *function, const char *file, int line)
{
	printf("Assertion failed: %s, function %s, file %s, line %d\n", expr, function, file, line);
	abort();
}

// ��ʾͼƬ������waitKey
#define SHOW(img) \
	do { \
		cv::imshow(#img, img);\
	} while (0)


// ��ʾͼƬ����waitKey
#define SHOW_WAIT(img) \
	do {\
		cv::imshow(#img, img);\
		cv::waitKey(0);\
	} while(0)


// ��ʾ��Ƶ֡��30FPS��Q�˳����ո���ͣ
#define SHOW_FRAME(winName, img) \
	do {\
		cv::imshow(winName, img);\
		char ch = cv::waitKey(33);\
		if (toupper(ch) == 'Q')\
			exit(0);\
		else if (toupper(ch) == ' ')\
			cv::waitKey(0);\
	} while (0)


// ����X����
#define DRAW_CROSS(image, center, color, d)\
	do{\
		cv::line(image, cv::Point(center.x - d, center.y - d), \
			cv::Point(center.x + d, center.y + d), color, 4, CV_AA, 0); \
		cv::line(image, cv::Point(center.x - d, center.y + d), \
			cv::Point(center.x + d, center.y - d), color, 4, CV_AA, 0); \
	} while (0)

// �Ժ������ü�ʱ����ʾ����ʱ�䣨���룩
#define TEST_TIME(func)\
	do {\
		cv::TickMeter tm;\
		tm.start();\
		func;\
		tm.stop();\
		std::cout << "TIME: " << #func << " " << tm.getTimeMilli() << "ms" << std::endl;\
	} while (0)


MY_EXPORT void getTrackPos(const std::string &posFilename, std::vector<cv::Rect> &vecTrackPos, std::vector<int> &vecFrameIndex);

MY_EXPORT void scaleTrackPos(std::vector<cv::Rect> &vecTrackPos, const int scale);


// ������ɫ
const cv::Scalar RED(0, 0, 255);
const cv::Scalar PINK(230, 130, 255);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar LIGHT_BLUE(255, 255, 160);
const cv::Scalar LIGHT_GREEN(144, 238, 144);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar PURPLE(205, 0, 205);
const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar BLACK(0, 0, 0);
const cv::Scalar GRAY(190, 190, 190);


// ���ñ���
typedef std::vector<int> vecI;
typedef std::vector<float> vecF;
typedef std::vector<double> vecD;
typedef std::vector<std::string> vecS;
typedef std::vector<cv::Mat> vecM;
typedef const std::string CStr;
typedef const cv::Mat CMat;



// ģ��ֻ�ܷ���H�ļ����ˣ��������common.h�޷��ػ�
// �����P��ֱ��AB�ľ��룬ͨ��ʸ������õ�
template<typename T, typename U>
float calcDist(const cv::Point_<T> &P, const cv::Point_<U> &A, const cv::Point_<U> &B)
{
	float dAP = sqrt((P.x - A.x) * (P.x - A.x) + (P.y - A.y) * (P.y - A.y));
	float dAB = sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));

	float cross = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);

	float cosTheta = cross / dAB / dAP;
	return dAP * sqrt(1 - cosTheta * cosTheta);
}

// ��������֮��ľ���
template<typename T, typename U>
float calcDist(const cv::Point_<T> &pt1, const cv::Point_<U> &pt2)
{
	float x = pt1.x - pt2.x;
	float y = pt1.y - pt2.y;
	return sqrt(x * x + y * y);
}

// ��д��Vec2f����
// �����ٶ���ֱ�ߵļн�
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
