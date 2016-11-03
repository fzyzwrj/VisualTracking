#include <iostream>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "common.h"


MY_EXPORT void getTrackPos(const std::string &posFilename, std::vector<cv::Rect> &vecTrackPos, std::vector<int> &vecFrameIndex)
{
	assert(vecTrackPos.empty());
	assert(vecFrameIndex.empty());
	std::ifstream fin(posFilename);
	std::cout << posFilename << std::endl;
	assert(fin.is_open());
	std::string line;

	while (std::getline(fin, line)) {
		int frameIndex, width, height, x, y;
		sscanf_s(line.c_str(), "%d [%d x %d from (%d, %d)]", &frameIndex, &width, &height, &x, &y);

		cv::Rect track(x, y, width, height);
		vecTrackPos.push_back(track);
		vecFrameIndex.push_back(frameIndex);
	} 
}

MY_EXPORT void scaleTrackPos(std::vector<cv::Rect> &vecTrackPos, const int scale)
{
	for (auto &trackPos : vecTrackPos) {
		trackPos.width /= scale;
		trackPos.height /= scale;
		trackPos.x /= scale;
		trackPos.y /= scale;
	}
}
