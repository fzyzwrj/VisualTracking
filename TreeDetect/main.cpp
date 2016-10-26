#include <iostream>
#include <vector>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "common.h"

#include "TreeDetect.h"


int detectImg()
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

int main(int argc, char *argv[])
{
	const std::string videoFilename = "G:\\resources\\videos\\DJI_0003.MOV";
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	cv::Mat frame;
	cv::Mat framePrev;
	cap.set(CV_CAP_PROP_POS_FRAMES, 1500);
	cap >> frame;
	//cv::namedWindow("SHOW");
	//cv::moveWindow("SHOW", 0, 0);

	cv::TickMeter tm;
	for (;;) {
		cap >> frame;
		cv::resize(frame, frame, cv::Size(1024, 540));
		tm.reset();
		tm.start();
		//cv::resize(frame, frame, cv::Size(256, 135));
		tm.stop();
		std::cout << tm.getTimeSec() << " ######## " << std::endl;
		cv::Mat colorFilterImg;

		tm.reset();
		tm.start();
		colorFilter(frame, colorFilterImg);
		tm.stop();
		std::cout << tm.getTimeSec() << std::endl;
		cv::imshow("SRC", frame);
		cv::imshow("DST", colorFilterImg);
		char ch = cv::waitKey(1);

		if (toupper(ch) == 'Q')
			break;
		else if (toupper(ch) == 'P')
			char ch = cv::waitKey(0);
	}

	return 0;
}