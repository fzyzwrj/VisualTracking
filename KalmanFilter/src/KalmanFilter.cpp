#include <opencv2\opencv.hpp>

#include "KalmanFilter.h"

#include "common.h"

CKalmanFilter::CKalmanFilter() : KF(stateTotal, measurementTotal, 0), state(stateTotal, 1, CV_32F), measurement(measurementTotal, 1, CV_32F)
{
	KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, \
		0, 1, 0, 1, \
		0, 0, 1, 0, \
		0, 0, 0, 1);

	cv::setIdentity(KF.measurementMatrix);
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));	// ������������Բ�������processNoise
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
}

// ��ʼ��λ�����ٶȣ�����ü�֡���ٶ���Ԥ��
void CKalmanFilter::init(float x, float y, float dx, float dy)
{
	state = *(cv::Mat_<float>(stateTotal, 1) << x, y, dx, dy);
	measurement = *(cv::Mat_<float>(measurementTotal, 1) << x, y, dx, dy);
	// �Գ�ʼ״̬��������
	cv::randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(1e-1));
	KF.statePost += state;
}

// ����Kalman FilterԤ��Ľ����ֱ��ʹ�ù۲������У��
void CKalmanFilter::predictAndCorrect(const cv::Mat &measureMat, cv::Mat &predictMat)
{
	assert(measureMat.rows == measurementTotal && measureMat.cols == 1);
	
	predictMat = KF.predict();
	
	cv::randn(measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0)));
	measurement += measureMat;	//  Ԥ���������
	KF.correct(measurement);
	cv::randn(processNoise, cv::Scalar::all(0), cv::Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
	state = KF.transitionMatrix * state + processNoise;
}

void CKalmanFilter::predictAndCorrect(float x, float y, float dx, float dy, cv::Mat &predictMat)
{
	cv::Mat measureMat = *(cv::Mat_<float>(measurementTotal, 1) << x, y, dx, dy);
	predictAndCorrect(measureMat, predictMat);
}

cv::Point2f CKalmanFilter::predictAndCorrect(float x, float y)
{
	//const cv::Mat measureMat = *(cv::Mat_<float>(measurementTotal, 1) << x, y, measurement.at<float>(2), measurement.at<float>(3));
	std::cout << measurement << std::endl;
	const cv::Mat measureMat = *(cv::Mat_<float>(measurementTotal, 1) << x, y, measurement.at<float>(2), measurement.at<float>(3));
	cv::Mat predictMat;
	predictAndCorrect(measureMat, predictMat);
	cv::Point2f predictPt(predictMat.at<float>(0), predictMat.at<float>(1));
	return predictPt;
}

// �켣���ٶ�Ԥ�⣬����Kalman FilterԤ��Ľ����ʹ��Kalman FilterԤ��ı�����У��Kalman Filter
void CKalmanFilter::predict(cv::Mat &predictMat)
{
	predictMat = KF.predict();
	cv::randn(measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0)));
	// measure z(n) = a * x(n) + v(n)
	measurement += predictMat;	//  Ԥ���������
	KF.correct(measurement);
	cv::randn(processNoise, cv::Scalar::all(0), cv::Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
	state = KF.transitionMatrix * state + processNoise;
}

cv::Point2f CKalmanFilter::predict(void)
{
	cv::Mat predictMat;
	predict(predictMat);
	cv::Point2f predictPt(predictMat.at<float>(0), predictMat.at<float>(1));
	return predictPt;
}


//// ���ع۲����������в����۲��������ʹ�ò�����Ϊ�۲����������Kalman Filter������ֱ����Kalman Filter��Ԥ��Ԥ�������
//cv::Point2f predict(int x = defaultMeasureValue, int y = defaultMeasureValue, int dx = defaultMeasureValue, int dy = defaultMeasureValue)
//{
//	cv::Point2f statePt(state.at<float>(0), state.at<float>(1));
//	//m_statePt = statePt;
//	// predict
//	cv::Mat prediction = KF.predict();
//	cv::Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1));
//	cv::Point2f predictVPt(prediction.at<float>(2), prediction.at<float>(3));


//	// measure z(n) = a * x(n) + v(n)
//	cv::randn(measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0))); // measure noise,measurement += KF.transitionMatrix * state;																								  
//	if (x == defaultMeasureValue && y == defaultMeasureValue) {
//		measurement.at<float>(0) += predictPt.x;
//		measurement.at<float>(1) += predictPt.y;
//		measurement.at<float>(2) += predictVPt.x;
//		measurement.at<float>(3) += predictVPt.y;
//	}
//	else {
//		measurement.at<float>(0) += x;
//		measurement.at<float>(1) += y;
//		measurement.at<float>(2) += dx;
//		measurement.at<float>(3) += dy;
//	}

//	cv::Point2f measurePt(measurement.at<float>(0), measurement.at<float>(1));

//	KF.correct(measurement);
//	cv::randn(processNoise, cv::Scalar::all(0), cv::Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));

//	state = KF.transitionMatrix * state + processNoise;
//	std::cout << measurement << std::endl;
//	return predictPt;
//}
////cv::Point2f m_statePt;