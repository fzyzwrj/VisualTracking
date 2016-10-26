#pragma once
#ifndef KALMAN_FILTER_H__
#define KALMAN_FILTER_H__

class CKalmanFilter
{
public:
	CKalmanFilter();

	// 初始化位置与速度，最好用几帧的速度来预测
	void init(float x, float y, float dx, float dy);

	// 返回Kalman Filter预测的结果，直接使用观测变量来校正
	void predictAndCorrect(const cv::Mat &measureMat, cv::Mat &predictMat);

	void predictAndCorrect(float x, float y, float dx, float dy, cv::Mat &predictMat);

	cv::Point2f predictAndCorrect(float x, float y);

	// 轨迹和速度预测，返回Kalman Filter预测的结果，使用Kalman Filter预测的变量来校正Kalman Filter
	void predict(cv::Mat &predictMat);

	cv::Point2f predict(void);

private:
	static const int stateTotal = 4;		// x, y, dx, dy
	static const int measurementTotal = 4;	// x, y, dx, dy
	static const int defaultMeasureValue = 0XFFFFF;
	cv::KalmanFilter KF;
	cv::Mat state;			// 状态变量
	cv::Mat measurement;	// 观测变量
	cv::Mat processNoise;	// 噪声变量
};

#define KALMAN_FILTER_H__
#endif /* KALMAN_FILTER_H__ */