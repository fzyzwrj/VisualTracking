#include "TreeDetect.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils.h"
#include "utils_opencv.h "

const bool adaptive_minsv = true;  // ����Ӧ��ɫ������ֵ

// ��HSV��ɫ�ռ��жϸ��������ص��Ƿ�����ɫ������Ӧ���ͶȺ�����
inline bool checkGreenPixelHSV(const cv::Vec3b &color) {
	// const Vec3b target(15, 18, 152)
	// S��V����Сֵ��adaptive_minsv���boolֵ�ж�
	// ���Ϊtrue������Сֵȡ����Hֵ��������˥��
	// ���Ϊfalse����������Ӧ��ʹ�ù̶�����Сֵminabs_sv
	const float max_sv = 255;
	const float minref_sv = 35;
	const float minabs_sv = 95;
	const int min_h = 40;
	const int max_h = 80;

	const float diff_h = float((max_h - min_h) / 2);
	const float avg_h = min_h + diff_h;

	const int H = int(color[0]);  // 0-180
	const int S = int(color[1]);  // 0-255
	const int V = int(color[2]);  // 0-255

	if (H > min_h && H < max_h) {
		const float Hdiff = fabs(H - avg_h);
		const float Hdiff_p = float(Hdiff) / diff_h;	// ��һ��

		// ����ɫ���Ĵ�������Ӧ�������Ⱥͱ��Ͷ�
		float min_sv = 0.0f;
		if (true == adaptive_minsv)
			min_sv = minref_sv - minref_sv / 2 * (1 - Hdiff_p);
		else
			min_sv = minabs_sv;

		if ((S > min_sv + 30 && S < max_sv) && (V > min_sv && V < max_sv))
			return true;
	}
	return false;
}

// ��RGB��ɫ�ռ��жϸ��������ص��Ƿ�����ɫ
inline bool checkGreenPixelBGR(const cv::Vec3b &color)
{
	const int b = color[0];
	const int g = color[1];
	const int r = color[2];
	return (g > r + 18) && (g > b + 18);
}

// ��ʱû�õ���������ɫ���������������ٲ�ɫ�����࣬div�ǵ���������������
static void colorReduce(const cv::Mat &srcImg, cv::Mat &dstImg, int div) {
	MY_ASSERT(srcImg.data);

	dstImg = srcImg.clone();
	const int rows = dstImg.rows; // number of lines
	const int cols = dstImg.cols * dstImg.channels(); // elem per line

	for (int r = 0; r < dstImg.rows; ++r) {
		uchar *p = dstImg.ptr<uchar>(r);
		for (int c = 0; c < dstImg.cols; ++c)
			p[c] = p[c] / div * div + div / 2;
	}
}

// ����BGRԭͼ��������ɫ���dstImg���ض�ֵ�����ͼ�񣬺���������ɫ�ĸ���
int colorFilter(const cv::Mat &srcImg, cv::Mat &dstImg) {
	MY_ASSERT(srcImg.data && srcImg.channels() == 3);	// RGB picture

	int greenTotal = 0;
	/// HSV������ɫ
	cv::Mat hsvImg;
	cv::cvtColor(srcImg, hsvImg, CV_BGR2HSV);
	cv::Mat colorFiltedHSVImg(hsvImg.size(), CV_8UC1);
	colorFiltedHSVImg = cv::Scalar::all(0);

	for (int r = 0; r != hsvImg.rows; ++r) {
		for (int c = 0; c != hsvImg.cols; ++c) {
			const cv::Vec3b color = hsvImg.at<cv::Vec3b>(r, c);
			if (checkGreenPixelHSV(color)) {
				colorFiltedHSVImg.at<uchar>(r, c) = 255;
				++greenTotal;
			}
		}
	}
	D_SHOW(colorFiltedHSVImg);

	/// ����Ӧ��̬ѧ�����ĳߴ��С
	int filterSize = 30;	// 4k
	if (dstImg.rows <= 256)
		filterSize /= 10;
	else if (dstImg.rows <= 512)
		filterSize /= 8;
	else if (dstImg.rows <= 1024)
		filterSize /= 6;
	else if (srcImg.rows <= 2048)
		filterSize /= 1.5;

	// �ߴ���ҪΪ����
	if ((filterSize & 0x1) == 0)
		filterSize += 1;

	const int morphW = filterSize;
	const int morphH = filterSize;
	const int blurW = filterSize;
	const int blurH = filterSize;

	/// ��̬ѧ���������бղ�����ͨͼ��
	cv::Mat openEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));
	cv::Mat closeEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));
	cv::Mat dilateEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));

	cv::Mat morphImg;	// ��̬ѧ�������ͼ��
	cv::morphologyEx(colorFiltedHSVImg, morphImg, cv::MORPH_CLOSE, closeEleMat);
	D_SHOW(morphImg);

	/// ��˹ģ������ֵ����ƽ��ͼ��
	cv::Mat blurImg;
	cv::Size blurSize(blurW, blurH);
	cv::GaussianBlur(morphImg, blurImg, blurSize, 0);
	cv::threshold(blurImg, dstImg, 128, 255, cv::THRESH_BINARY);
	D_SHOW(dstImg);

	return greenTotal;
}