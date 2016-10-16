#include <iostream>
#include <vector>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "TreeDetect.h"

#define SHOW(img) \
	do { \
		cv::namedWindow(#img);\
		cv::moveWindow(#img, 0, 0);\
		cv::imshow(#img, img);\
	} while (0)

int main()
{
	const std::string filename = "G:\\resources\\videos\\foot_bridge\\2100.jpg";
	//const std::string filename = "G:\\resources\\videos\\foot_bridge_next_pa\\1.jpg";
	//const std::string cfilename = "C:/resources/6770.jpg";
	cv::Mat srcImg = cv::imread(filename);
	assert(srcImg.data);
	cv::resize(srcImg, srcImg, cv::Size(1024, 540));

	cv::Mat colorFilterImg;
	colorFilter(srcImg, colorFilterImg);
	SHOW(srcImg);
	SHOW(colorFilterImg);

	cv::waitKey(0);
	return 0;
}