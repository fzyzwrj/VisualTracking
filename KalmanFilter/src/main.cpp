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
	std::vector<cv::Rect> vecTrackPos;	// 跟踪的轨迹
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


//int main()
//{
//	// freopen("output2.txt", "w", stdout);
//	const int fScale = 4.0f;	// 图像和轨迹需要缩放的倍数，4
//
//	const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\63.txt";
//	std::vector<cv::Rect> vecTrackPos;	// 跟踪的轨迹
//	std::vector<int> vecFrameIndex;
//	getTrackPos(posFilename, vecTrackPos, vecFrameIndex);
//	scaleTrackPos(vecTrackPos, 4);
//
//	assert(vecTrackPos.size() > 10);
//	//cv::Mat imgBG(4096, 2160, CV_8UC3);	// image background
//	cv::Mat imgBG(540, 1024, CV_8UC3);
//	cv::namedWindow("imgBG");
//	cv::resizeWindow("imgBG", 1024, 540);
//	
//	// draw track
//	//for (const auto &track : vecTrack) {
//	//	imgBG = cv::Scalar::all(0);
//	//	cv::Point center(track.x + track.width / 2, track.y + track.height / 2);
//	//	drawCross(imgBG, center, cv::Scalar(255, 0, 0), 3);
//	//	//SHOW(imgBG);
//	//	cv::imshow("imgBG", imgBG);
//	//	char ch = cv::waitKey(100);
//	//	if (toupper(ch) == 'Q' || ch == 27)
//	//		break;
//	//}
//
//	const int stateTotal = 4; // x, y, dx, dy
//	const int measurementTotal = 2; // x, y
//	cv::KalmanFilter KF(stateTotal, measurementTotal, 0);
//
//	//cv::Mat processNoise(measurementTotal, 1, CV_32F);
//	cv::Mat state(stateTotal, 1, CV_32F);
//	cv::Mat measurement = cv::Mat::zeros(measurementTotal, 1, CV_32F);
//	cv::Mat processNoise(stateTotal, 1, CV_32F);
//	
//	KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, \
//		0, 1, 0, 1, \
//		0, 0, 1, 0, \
//		0, 0, 0, 1);
//	cv::setIdentity(KF.measurementMatrix);
//	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-3));	// 由噪声矩阵可以产生噪声processNoise
//	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1));
//	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
//
//	cv::Rect initTrack = vecTrackPos[0];
//
//	// 参数需要细调
//	state = *(cv::Mat_<float>(stateTotal, 1) << initTrack.x, initTrack.y, -5 / fScale, 12 / fScale);
//	KF.statePost = *(cv::Mat_<float>(stateTotal, 1) << initTrack.x, initTrack.y, -5 / fScale, 12 / fScale);
//	cv::randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
//
//
//	//std::cout << state << std::endl;
//	for (auto it = vecTrackPos.cbegin() + 1; it != vecTrackPos.cend(); ++it) {
//		cv::Point2f statePt(state.at<float>(0), state.at<float>(1));
//
//		// predict
//		cv::Mat prediction = KF.predict();
//		cv::Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1));
//
//		// measure z(n) = a * x(n) + v(n)
//		cv::randn(measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0))); // measure noise
//		//measurement += KF.transitionMatrix * state;
//		measurement.at<float>(0) += it->x;
//		measurement.at<float>(1) += it->y;
//		cv::Point2f measurePt(measurement.at<float>(0), measurement.at<float>(1));
//
//		imgBG = cv::Scalar::all(0);
//		//std::cout << statePt << std::endl;
//		drawCross(imgBG, statePt, cv::Scalar(0, 0, 255), 3);
//		drawCross(imgBG, predictPt, cv::Scalar(0, 255, 0), 3);
//		drawCross(imgBG, measurePt, cv::Scalar(255, 0, 0), 3);
//
//		cv::imshow("imgBG", imgBG);
//		if (cv::theRNG().uniform(0, 4))
//			KF.correct(measurement);
//		// update
//		cv::randn(processNoise, cv::Scalar::all(0), cv::Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//
//		state = KF.transitionMatrix * state + processNoise;
//		char ch = cv::waitKey(100);
//		if (toupper(ch) == 'Q')
//			break;
//	}
//
//	for (;;) {
//		cv::Point2f statePt(state.at<float>(0), state.at<float>(1));
//
//		cv::Mat prediction = KF.predict();
//		// cv::Point2f改成cv::Point会出错!!!
//		cv::Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1));
//
//		cv::randn(measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0)));
//		measurement.at<float>(0) += predictPt.x;
//		measurement.at<float>(1) += predictPt.y;
//		std::cout << measurement.at<float>(0) << " " << measurement.at<float>(1) << std::endl;
//
//		cv::Point2f measurePt(measurement.at<float>(0), measurement.at<float>(1));
//
//		imgBG = cv::Scalar::all(0);
//		drawCross(imgBG, predictPt, cv::Scalar(0, 0, 255), 3);
//		drawCross(imgBG, measurePt, cv::Scalar(0, 255, 0), 3);
//
//		cv::imshow("imgBG", imgBG);
//		if (cv::theRNG().uniform(0, 4))
//			KF.correct(measurement);
//
//		cv::randn(processNoise, cv::Scalar::all(0), cv::Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//
//		state = KF.transitionMatrix * state + processNoise;
//		char ch = cv::waitKey(100);
//		if (toupper(ch) == 'Q')
//			break;
//	}
//
//	return 0;
//}
