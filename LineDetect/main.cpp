/************************************************************************/
/* 	2016.10.9 by astratwu
	V1.0
	备注：对车道、主干道等航拍图像进行直线检测，分别出主要干道 
*/
/************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <functional>

#include <opencv2/opencv.hpp>

#include "LineDetect.h"

#define SHOW(img) \
	do { \
		cv::imshow(#img, img);\
	} while (0)

int main(int argc, char *argv[])
{
	//const std::string filename = "G:\\resources\\videos\\foot_bridge\\2100.jpg";
	//const std::string filename = "G:\\resources\\videos\\foot_bridge_next_pa\\1.jpg";
	const std::string filename = "C:\\Users\\Administrator\\Documents\\Visual Studio 2015\\Projects\\BingObjectnessCVPR14\\BingObjectnessCVPR14\\captureOcclusionVideo\\occlusion_video_42\\_add_border_40_105.jpg";
	cv::Mat srcImg = cv::imread(filename);
	assert(srcImg.data);
	//cv::resize(srcImg, srcImg, cv::Size(1024, 540));

	cv::Mat houghPImg;
	lineDetect(srcImg, houghPImg);

	return 0;
}