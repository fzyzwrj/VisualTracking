#pragma once
#ifndef LINE_DETECT_H__
#define LINE_DETECT_H__
#include <opencv2\opencv.hpp>

void lineDetect(const cv::Mat &srcImg, cv::Mat &houghPImg);


#endif /* LINE_DETECT_H__ */
