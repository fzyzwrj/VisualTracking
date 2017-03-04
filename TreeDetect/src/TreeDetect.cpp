#include "TreeDetect.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils.h"
#include "utils_opencv.h "

const bool adaptive_minsv = true;  // 自适应颜色过滤阈值

// 在HSV颜色空间判断给定的像素点是否是绿色，自适应饱和度和亮度
inline bool checkGreenPixelHSV(const cv::Vec3b &color) {
	// const Vec3b target(15, 18, 152)
	// S和V的最小值由adaptive_minsv这个bool值判断
	// 如果为true，则最小值取决于H值，按比例衰减
	// 如果为false，则不再自适应，使用固定的最小值minabs_sv
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
		const float Hdiff_p = float(Hdiff) / diff_h;	// 归一化

		// 根据色调的纯度自适应降低亮度和饱和度
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

// 在RGB颜色空间判断给定的像素点是否是绿色
inline bool checkGreenPixelBGR(const cv::Vec3b &color)
{
	const int b = color[0];
	const int g = color[1];
	const int r = color[2];
	return (g > r + 18) && (g > b + 18);
}

// 暂时没用到：增大颜色的量化步长，较少彩色的种类，div是调整到的量化步长
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

// 输入BGR原图，进行绿色检测dstImg返回二值化后的图像，函数返回绿色的个数
int colorFilter(const cv::Mat &srcImg, cv::Mat &dstImg) {
	MY_ASSERT(srcImg.data && srcImg.channels() == 3);	// RGB picture

	int greenTotal = 0;
	/// HSV过滤绿色
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

	/// 自适应形态学操作的尺寸大小
	int filterSize = 30;	// 4k
	if (dstImg.rows <= 256)
		filterSize /= 10;
	else if (dstImg.rows <= 512)
		filterSize /= 8;
	else if (dstImg.rows <= 1024)
		filterSize /= 6;
	else if (srcImg.rows <= 2048)
		filterSize /= 1.5;

	// 尺寸需要为奇数
	if ((filterSize & 0x1) == 0)
		filterSize += 1;

	const int morphW = filterSize;
	const int morphH = filterSize;
	const int blurW = filterSize;
	const int blurH = filterSize;

	/// 形态学操作，进行闭操作连通图像
	cv::Mat openEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));
	cv::Mat closeEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));
	cv::Mat dilateEleMat = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphW, morphH));

	cv::Mat morphImg;	// 形态学操作后的图像
	cv::morphologyEx(colorFiltedHSVImg, morphImg, cv::MORPH_CLOSE, closeEleMat);
	D_SHOW(morphImg);

	/// 高斯模糊并二值化，平滑图像
	cv::Mat blurImg;
	cv::Size blurSize(blurW, blurH);
	cv::GaussianBlur(morphImg, blurImg, blurSize, 0);
	cv::threshold(blurImg, dstImg, 128, 255, cv::THRESH_BINARY);
	D_SHOW(dstImg);

	return greenTotal;
}