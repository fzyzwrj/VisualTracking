#pragma once
#ifndef KALMAN_FILTER_H__
#define KALMAN_FILTER_H__

class CKalmanFilter
{
public:
	CKalmanFilter();

	// ��ʼ��λ�����ٶȣ�����ü�֡���ٶ���Ԥ��
	void init(float x, float y, float dx, float dy);

	// ����Kalman FilterԤ��Ľ����ֱ��ʹ�ù۲������У��
	void predictAndCorrect(const cv::Mat &measureMat, cv::Mat &predictMat);

	void predictAndCorrect(float x, float y, float dx, float dy, cv::Mat &predictMat);

	cv::Point2f predictAndCorrect(float x, float y);

	// �켣���ٶ�Ԥ�⣬����Kalman FilterԤ��Ľ����ʹ��Kalman FilterԤ��ı�����У��Kalman Filter
	void predict(cv::Mat &predictMat);

	cv::Point2f predict(void);

private:
	static const int stateTotal = 4;		// x, y, dx, dy
	static const int measurementTotal = 4;	// x, y, dx, dy
	static const int defaultMeasureValue = 0XFFFFF;
	cv::KalmanFilter KF;
	cv::Mat state;			// ״̬����
	cv::Mat measurement;	// �۲����
	cv::Mat processNoise;	// ��������
};

#define KALMAN_FILTER_H__
#endif /* KALMAN_FILTER_H__ */