#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <opencv2/opencv.hpp>

#include "common.h"
#include "KalmanFilter.h"

int main()
{
	const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\4.txt";
	const int scale = 4;
	std::vector<cv::Rect> vecTrackPos;	// ���ٵĹ켣
	std::vector<int> vecFrameIndex;
	getTrackPos(posFilename, vecTrackPos, vecFrameIndex);
	scaleTrackPos(vecTrackPos, 4);

	cv::Mat imgBG(540, 1024, CV_8UC3);
	cv::namedWindow("imgBG");
	cv::resizeWindow("imgBG", 1024, 540);
	CKalManFilter KF;

	cv::Rect initTrack = vecTrackPos[0];
	KF.init(initTrack.x, initTrack.y, -5 / scale, 12 / scale);

	for (auto it = vecTrackPos.cbegin() + 1; it != vecTrackPos.cend(); ++it) {
		cv::Point measurePt(it->x, it->y);
		cv::Point2f filterPt = KF.predict(it->x, it->y);

		imgBG = cv::Scalar::all(0);
		DRAW_CROSS(imgBG, measurePt, cv::Scalar(0, 255, 0), 3);
		DRAW_CROSS(imgBG, filterPt, cv::Scalar(255, 0, 0), 3);

		SHOW_WAIT("imgBG", imgBG);
	}

	for (;;) {
		cv::Point2f filterPt = KF.predict();
		imgBG = cv::Scalar::all(0);
		DRAW_CROSS(imgBG, filterPt, cv::Scalar(255, 0, 0), 3);
		SHOW_WAIT("imgBG", imgBG);
	}

	return 0;
}