#pragma once
#ifndef KALMAN_FILTER_H__
#define KALMAN_FILTER_H__

#include "common.h"
class CKalManFilter
{
public:
	CKalManFilter() : KF(stateTotal, measurementTotal, 0), state(stateTotal, 1, CV_32F), measurement(measurementTotal, 1, CV_32F)
	{
		KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, \
			0, 1, 0, 1, \
			0, 0, 1, 0, \
			0, 0, 0, 1);
		cv::setIdentity(KF.measurementMatrix);
		cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));	// 由噪声矩阵可以产生噪声processNoise
		cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
	}

	void init(int x, int y, float dx, float dy)
	{
		// 参数需要细调，尤其dx, dy
		state = *(cv::Mat_<float>(stateTotal, 1) << x, y, dx, dy);
		//std::cout << state << " " << "INIT ############" << std::endl;
		//KF.statePost = (cv::Mat_<float>(stateTotal, 1) << x, y, dx, dy);
		cv::randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(1e-1));
		KF.statePost += state;
	}

	cv::Point2f predict(int x = -1, int y = -1)
	{
		//std::cout << state << " " << "############" << std::endl;
		//状态变量好像只是跟初始值有关，感觉没有什么用

		cv::Point2f statePt(state.at<float>(0), state.at<float>(1));
		//m_statePt = statePt;
		// predict
		cv::Mat prediction = KF.predict();
		cv::Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1));

		// measure z(n) = a * x(n) + v(n)
		cv::randn(measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0))); // measure noise,measurement += KF.transitionMatrix * state;																								  
		if (x == -1 && y == -1) {
			measurement.at<float>(0) += predictPt.x;
			measurement.at<float>(1) += predictPt.y;
		}
		else {
			measurement.at<float>(0) += x;
			measurement.at<float>(1) += y;
		}

		cv::Point2f measurePt(measurement.at<float>(0), measurement.at<float>(1));

		// update
		//if (cv::theRNG().uniform(0, 4))
			KF.correct(measurement);
		cv::randn(processNoise, cv::Scalar::all(0), cv::Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));

		state = KF.transitionMatrix * state + processNoise;
		return predictPt;
	}
	//cv::Point2f m_statePt;

private:
	static const int stateTotal = 4; // x, y, dx, dy
	static const int measurementTotal = 2; // x, y
	cv::KalmanFilter KF;
	cv::Mat state;		// 状态变量
	cv::Mat measurement;	// 观测变量
	cv::Mat processNoise;	// 噪声变量
};

#define KALMAN_FILTER_H__
#endif /* KALMAN_FILTER_H__ */