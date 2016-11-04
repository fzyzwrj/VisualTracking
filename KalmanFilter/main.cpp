#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <opencv2/opencv.hpp>

#include "KalmanFilter.h"

#include "utils.h"
#include "utils_opencv.h"

int main()
{
	const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\24.txt";
	const int scale = 4;
	std::vector<cv::Rect> vecTrackPos;	// ¸ú×ÙµÄ¹ì¼£
	std::vector<int> vecFrameIndex;
	getTrackPos(posFilename, vecTrackPos, vecFrameIndex);
	scaleTrackPos(vecTrackPos, 4);

	cv::Mat imgBG(540, 1024, CV_8UC3);
	cv::namedWindow("imgBG");
	cv::resizeWindow("imgBG", 1024, 540);
	CKalmanFilter KF;

	cv::Rect initTrack = vecTrackPos[0];
	KF.init(initTrack.x, initTrack.y, -5.0 / scale, 12.0 / scale);  

	for (auto it = vecTrackPos.cbegin() + 1; it != vecTrackPos.cend(); ++it) {
		cv::Point measurePt(it->x, it->y);
		cv::Mat predictMat;
		KF.predictAndCorrect(it->x, it->y, it->x - (it - 1)->x, it->y - (it - 1)->y, predictMat);
		cv::Point2f filterPt(predictMat.at<float>(0), predictMat.at<float>(1));

		std::cout << predictMat << std::endl;
		std::cout << "### " << *it << std::endl;
		//cv::Point2f statePt = KF.m_statePt;
		//std::cout << std::endl;
		//std::cout << measurePt << std::endl;
		//std::cout << filterPt << std::endl;
		//std::cout << statePt << std::endl;

		imgBG = cv::Scalar::all(0);
		DRAW_CROSS(imgBG, measurePt, GREEN, 3);
		DRAW_CROSS(imgBG, filterPt, BLUE, 3);
		//DRAW_CROSS(imgBG, statePt, YELLOW, 3);
		SHOW_FRAME("bg", imgBG);
		//SHOW_WAIT("imgBG", imgBG);
	}

	for (;;) {
		cv::Mat predictMat;
		KF.predict(predictMat);
		cv::Point2f filterPt(predictMat.at<float>(0), predictMat.at<float>(1));
		std::cout << predictMat << std::endl;

		imgBG = cv::Scalar::all(0);
		DRAW_CROSS(imgBG, filterPt, cv::Scalar(255, 0, 0), 3);
		//SHOW_WAIT("imgBG", imgBG);
		SHOW_FRAME("bg", imgBG);
	}

	return 0;
}
