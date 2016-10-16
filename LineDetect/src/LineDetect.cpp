#include <cmath>
#include <cmath>
#include <iostream>
#include <assert.h>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>

#define SHOW(img) \
	do { \
		cv::imshow(#img, img);\
	} while (0)


#define MOVE_WIN(img) \
	do {\
		cv::moveWindow(#img, 400, 400);\
	}while (0)

static int g_edgeThresh;
static cv::Mat g_blurImg;
static cv::Mat g_cannyImg;
static cv::Mat g_houghPImg;
void onCanny(int, void *)
{
	cv::Canny(g_blurImg, g_cannyImg, g_edgeThresh, 3 * g_edgeThresh);
	SHOW(g_cannyImg);
	MOVE_WIN(g_cannyImg);
	/// houghlines
	//std::vector<cv::Vec2f> lines;
	//cv::HoughLines(srcImg, lines, 1, cv::CV_PI / 180, 150, 0, 0);	//adjust parameter
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(g_cannyImg, lines, 1, CV_PI / 180, 80, 80, 10); //adjust parameter
	cv::Mat houghPImg(g_cannyImg.size(), g_cannyImg.type(), cv::Scalar::all(0));

	// 删除角度过大的线段
	//for (auto it = lines.begin(); it != lines.end(); /**/) {
	//	const cv::Point pt1((*it)[0], (*it)[1]);
	//	const cv::Point pt2((*it)[2], (*it)[3]);
	//	const cv::Point pt = pt2 - pt1;
	//	double angle = atan(pt.y * 1.0 / pt.x);
	//	angle = angle / CV_PI * 180;
	//	angle = fabs(angle);
	//	if (angle >= 20.0 && angle <= 70.0)
	//		it = lines.erase(it);
	//	else
	//		++it;
	//}
	g_houghPImg.create(g_cannyImg.size(), CV_8U);
	g_houghPImg = cv::Scalar::all(0);
	for (size_t i = 0; i < lines.size(); ++i) {
		float rho = lines[i][0], theta = lines[i][1];
		const cv::Vec4i &l = lines[i];
		const cv::Point pt1(l[0], l[1]);
		const cv::Point pt2(l[2], l[3]);
		cv::line(g_houghPImg, pt1, pt2, cv::Scalar(255, 0, 255));
	}
	SHOW(g_houghPImg);
}

void lineDetect(const cv::Mat &srcImg, cv::Mat &houghPImg)
{
	g_houghPImg = houghPImg;
	cv::Mat grayImg;
	cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);

	//cv::GaussianBlur
	//cv::blur(grayImg, g_blurImg, cv::Size(3, 3));
	g_blurImg = grayImg.clone();

	static const int edgeThreshMax = 255;
	g_cannyImg = g_blurImg.clone();
	cv::imshow("cannyImg", g_cannyImg);
	cv::moveWindow("cannyImg", 0, 0);
	cv::createTrackbar("cannyBar", "cannyImg", &g_edgeThresh, edgeThreshMax, onCanny);

	cv::imshow("srcImg", srcImg);
	cv::waitKey(0);
	houghPImg = g_houghPImg;
}