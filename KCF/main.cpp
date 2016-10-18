#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "kcftracker.hpp"
#include "GetFramesByPos.h"
#include "common.h"
#include "kcftracker.hpp"

int main(int argc, char *argv[])
{
	/// ��ȡͼƬ֡
	//const std::string videoName = "G:\\resources\\videos\\DJI_0001.MOV";

	//for (size_t i = 1; i <= 77; ++i) {
	//	const std::string dirName = "video" + std::to_string(i);
	//	const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\" + std::to_string(i) + ".txt";
	//	std::vector<cv::Rect> vecTrackPos;
	//	std::vector<int> vecFrameIndex;
	//	getTrackPos(posFilename, vecTrackPos, vecFrameIndex);

	//	getFramesByPos(videoName, vecTrackPos, vecFrameIndex, dirName);
	//}

	if (argc > 5) return -1;

	///parameter
	bool HOG = true;
	bool FIXEDWINDOW = true;
	bool MULTISCALE = false;
	bool SILENT = false;
	bool LAB = false;

	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "hog") == 0)
			HOG = true;
		if (strcmp(argv[i], "fixed_window") == 0)
			FIXEDWINDOW = true;
		if (strcmp(argv[i], "singlescale") == 0)
			MULTISCALE = false;
		if (strcmp(argv[i], "show") == 0)
			SILENT = false;
		if (strcmp(argv[i], "lab") == 0) {
			LAB = true;
			HOG = true;
		}
		if (strcmp(argv[i], "gray") == 0)
			HOG = false;
	}

	// Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	const std::string videoName = "G:\\resources\\videos\\DJI_0001.MOV";
	for (size_t i = 1; i <= 77; ++i) {
		const std::string dirName = "video" + std::to_string(i);
		const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\" + std::to_string(i) + ".txt";
		std::vector<cv::Rect> vecTrackPos;
		std::vector<int> vecFrameIndex;
		getTrackPos(posFilename, vecTrackPos, vecFrameIndex);
		scaleTrackPos(vecTrackPos, 4);

		//getFramesByPos(videoName, vecTrackPos, vecFrameIndex, dirName);
		cv::VideoCapture cap(videoName);
		assert(cap.isOpened());
		cap.set(CV_CAP_PROP_POS_FRAMES, vecFrameIndex[0]);
		cv::Mat frame;

		cv::namedWindow("DEMO");
		//cv::resizeWqindow("DEMO", 1024, 540);
		cv::moveWindow("DEMO", 0, 0);


		//std::ofstream fout("4k_peak_" + std::to_string(i) + ".txt");
		std::ofstream fout("1k_peak_" + std::to_string(i) + ".txt");// 1024 x 540

		assert(fout.is_open());
		for (size_t j = 0; j < vecTrackPos.size(); ++j) {
			cap >> frame;
			cv::resize(frame, frame, cv::Size(1024, 540));

			if (j == 0) {
				tracker.init(vecTrackPos[0], frame);
				cv::rectangle(frame, vecTrackPos[0], RED, 1, 8);
			}
			else {
				float peak_value = 0.0f;
				cv::Rect res = tracker.update(frame, peak_value);
				cv::rectangle(frame, res, GREEN, 1, 8);
				
				char buf[256];
				sprintf(buf, "%-8d%.3f", vecFrameIndex[j], peak_value);
				fout << buf;
				//fout << vecFrameIndex[j] << " " << peak_value;
				if (peak_value < 0.4f)
					fout << " ###";
				fout << std::endl;
			}
			SHOW_WAIT("DEMO", frame);
		}
		fout.close();
	}
	return 0;
}
