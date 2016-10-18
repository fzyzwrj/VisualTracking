#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define SHOW(img) \
//	do { \
//		cv::imshow(#img, img);\
//	} while (0)
#include "common.h"

const bool adaptive_minsv = true;  // ����Ӧ��ɫ������ֵ

// const Vec3b target(15, 18, 152)
// ������ɫ
bool checkGreenPixelHSV(const cv::Vec3b &color) {
	// S��V����Сֵ��adaptive_minsv���boolֵ�ж�
	// ���Ϊtrue������Сֵȡ����Hֵ��������˥��
	// ���Ϊfalse����������Ӧ��ʹ�ù̶�����Сֵminabs_sv
	// Ĭ��Ϊfalse
	const float max_sv = 255;
	//const float minref_sv = 64;
	const float minref_sv = 45;

	const float minabs_sv = 95;
	//int min_h = 45;
	//int max_h = 75;
	int min_h = 35;
	int max_h = 85;

	float diff_h = float((max_h - min_h) / 2);
	float avg_h = min_h + diff_h;

	int H = int(color[0]);  // 0-180
	int S = int(color[1]);  // 0-255
	int V = int(color[2]);  // 0-255

	if (H > min_h && H < max_h) {
		float Hdiff = fabs(H - avg_h);

		float Hdiff_p = float(Hdiff) / diff_h;

		// S��V����Сֵ��adaptive_minsv���boolֵ�ж�
		// ���Ϊtrue������Сֵȡ����Hֵ��������˥��
		// ���Ϊfalse����������Ӧ��ʹ�ù̶�����Сֵminabs_sv

		float min_sv = 0.0f;
		if (true == adaptive_minsv)
			min_sv = minref_sv - minref_sv / 2 * (1 - Hdiff_p);  // inref_sv - minref_sv / 2 * (1 - Hdiff_p)
		else
			min_sv = minabs_sv;  // add

		// ԭ��ȡֵ40
		if ((S > min_sv + 30 && S < max_sv) && (V > min_sv && V < max_sv))
			return true;
	}
	return false;
}

bool checkGreenPixelBGR(const cv::Vec3b &color)
{
	const int b = color[0];
	const int g = color[1];
	const int r = color[2];
	if (g > r + 18 && g > b + 18)
		return true;
	else
		return false;
}

// div �ֿ��С
void colorReduce(const cv::Mat &srcImg, cv::Mat &dstImg, int div) {
	assert(srcImg.data);

	dstImg = srcImg.clone();
	int rows = dstImg.rows; // number of lines
	int cols = dstImg.cols * dstImg.channels(); // elem per line
	for (int r = 0; r < dstImg.rows; ++r) {
		uchar *p = dstImg.ptr<uchar>(r);
		for (int c = 0; c < dstImg.cols; ++c)
			p[c] = p[c] / div * div + div / 2;
	}
}

void colorFilter(const cv::Mat &srcImg, cv::Mat &dstImg) {
	// RGB picture
	assert(srcImg.data && srcImg.channels() == 3);

	cv::Mat blurImg = srcImg.clone();
	//cv::resize(blurImg, blurImg, cv::Size(512, 270));

	//cv::GaussianBlur(srcImg, blurImg, cv::Size(9, 9), 0);

	/// HSV������ɫ
	cv::Mat hsvImg;
	cv::cvtColor(blurImg, hsvImg, CV_BGR2HSV);
	cv::Mat colorFiltedHSVImg(hsvImg.size(), CV_8UC1);
	colorFiltedHSVImg = cv::Scalar::all(0);

	for (int r = 0; r != hsvImg.rows; ++r) {
		for (int c = 0; c != hsvImg.cols; ++c) {
			cv::Vec3b color = blurImg.at<cv::Vec3b>(r, c);
			if (checkGreenPixelHSV(color))
				colorFiltedHSVImg.at<uchar>(r, c) = 255;
		}
	}

	/// BGR������ɫ
	cv::Mat colorFiltedRGBImg(srcImg.size(), CV_8UC1);
	colorFiltedRGBImg = cv::Scalar::all(0);
	for (int r = 0; r != srcImg.rows; ++r) {
		for (int c = 0; c != srcImg.cols; ++c) {
			cv::Vec3b color = srcImg.at<cv::Vec3b>(r, c);
			if (checkGreenPixelBGR(color))
				colorFiltedRGBImg.at<uchar>(r, c) = 255;
		}
	}

	//SHOW(colorFiltedHSVImg);
	//SHOW(colorFiltedRGBImg);
	/// ���������ɫ���˵ķ���
	cv::Mat maskImg = colorFiltedHSVImg | colorFiltedRGBImg;

	/// ����Ӧ��̬ѧ�����ĳߴ��С
	int filterSize = 30;	// 4k
	if (blurImg.rows <= 256)
		filterSize /= 10;
	else if (blurImg.rows <= 512)
		filterSize /= 5;
	else if (blurImg.rows <= 1024)
		filterSize /= 2;
	else if (srcImg.rows <= 2048)
		filterSize /= 1.5;

	// �ߴ���ҪΪ����
	if ((filterSize & 0x1) == 0)
		filterSize += 1;

	const int morphW = filterSize;
	const int morphH = filterSize;
	const int blurW = filterSize;
	const int blurH = filterSize;

	/// ƽ��
	cv::Mat openEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));
	cv::Mat closeEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));
	cv::Mat dilateEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));

	cv::Mat morphImg;
	//SHOW(maskImg);
	cv::morphologyEx(maskImg, morphImg, cv::MORPH_CLOSE, closeEleMat);
	
	//SHOW(morphImg);
	//cv::morphologyEx(morphImg, morphImg, cv::MORPH_OPEN, openEleMat);

	blurImg = morphImg.clone();
	cv::Size blurSize(blurW, blurH);

	//cv::blur(blurImg, blurImg, blurSize);
	cv::GaussianBlur(blurImg, blurImg, blurSize, 0);

	/// �˲���Ķ�ֵ��
	cv::threshold(blurImg, blurImg, 128, 255, cv::THRESH_BINARY);
	dstImg = blurImg.clone();
}