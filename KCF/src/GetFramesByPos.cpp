#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

#include <direct.h>
#include "common.h"
#include "GetTrackingPosTxt.h"


cv::Rect rectAddBorder(const cv::Rect &rect, int borderWidth)
{
	const cv::Point pt(borderWidth, borderWidth);
	return cv::Rect(rect.tl() - pt, rect.br() + pt);
}

using namespace std;
using namespace cv;

int getFramesByPos(const std::string &videoFilename, const std::vector<cv::Rect> &vecTrackPos, const std::vector<int> &vecFrameIndex, const std::string &dirName)
{
	assert(vecTrackPos.size() == vecFrameIndex.size());
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	_mkdir(dirName.c_str());

	cap.set(CV_CAP_PROP_POS_FRAMES, vecFrameIndex[0]);
	cv::Mat frame;
	for (size_t i = 0; i < vecTrackPos.size(); ++i) {
		cap >> frame;
		const cv::Rect rectBG(0, 0, frame.cols, frame.rows);
		cv::Rect trackRect = rectAddBorder(vecTrackPos[i], 40);
		const cv::Rect ROIRect = trackRect & rectBG;
		cv::Mat ROIImg = frame(ROIRect);
				
		std::string ROIFilename = dirName + "/add_border_40_" + std::to_string(i) + ".jpg";
		cv::imwrite(ROIFilename, ROIImg);
	}
	return 0;
}