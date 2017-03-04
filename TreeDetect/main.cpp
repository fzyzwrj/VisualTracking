#include <cstdio>
#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TreeDetect.h"

#include "utils.h"
#include "utils_opencv.h"
// 测试单张图片，给定给定图片缩放到1024 x 540，显示绿色检测的结果二值图
int testPicture()
{
	const std::string filename = "F:\\resources\\videos\\foot_bridge\\2897.jpg";
	//const std::string filename = "G:\\resources\\videos\\foot_bridge_next_pa\\1.jpg";
	//const std::string cfilename = "C:/resources/6770.jpg";

	cv::Mat srcImg = cv::imread(filename);
	MY_ASSERT(srcImg.data);

	cv::resize(srcImg, srcImg, cv::Size(1024, 540));
	//srcImg = srcImg(cv::Rect(1, 1, 767, 539));	// 只显示特定的区域

	cv::Mat colorFilterImg;
	const int greenPixelTotal = colorFilter(srcImg, colorFilterImg);
	const int pixelTotal = srcImg.rows * srcImg.cols;
	const float ratio = greenPixelTotal * 1.0f / pixelTotal;
	printf("%s, total pixels: %d, green pixels: %d, ratio: %.2f\n", filename.c_str(), pixelTotal, greenPixelTotal, ratio);

	SHOW(srcImg);
	SHOW(colorFilterImg);
	cv::waitKey(0);
	
	return 0;
}

// 测试视频序列
// arg1: 视频路径
// arg2: 视频需要缩放的比例
int testVideo()
{
	const std::string videoFilename = "F:\\resources\\videos\\DJI_0003.MOV";
	cv::VideoCapture cap(videoFilename);
	MY_ASSERT(cap.isOpened());
	cv::Mat frame;
	cv::Mat framePrev;
	
	cap.set(CV_CAP_PROP_POS_FRAMES, 1500);	// 从1500帧开始
	cap >> frame;

	for (;;) {
		cap >> frame;
		cv::resize(frame, frame, cv::Size(1024, 540));

		cv::Mat colorFilterImg;
		TIME(colorFilter(frame, colorFilterImg));

		cv::imshow("SRC", frame);
		cv::imshow("DST", colorFilterImg);
		int ch = cv::waitKey(1);

		if (toupper(ch) == 'Q')
			break;
		else if (toupper(ch) == 'P')
			char ch = cv::waitKey(0);
		else
			continue;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	testVideo();
}