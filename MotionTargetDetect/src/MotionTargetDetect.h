#pragma once
#ifndef MOTION_TARGET_DETECT_H__
#define MOTION_TARGET_DETECT_H__

#include <opencv2\opencv.hpp>

// 对img1, img2做帧间差分，返回二值图像
 void frameDiff(const cv::Mat &img1, const cv::Mat &dimg2, cv::Mat &res);

#endif /* MOTION_TARGET_DETECT_H__ */