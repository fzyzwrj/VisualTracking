#ifndef LINE_DETECT_H__
#define LINE_DETECT_H__
#include <opencv2\opencv.hpp>

void lineDetectDetailed(const cv::Mat &srcImg, cv::Mat &houghPImg);
std::vector<cv::Vec4i> lineDetect(const cv::Mat &srcImg, cv::Mat &houghPImg);

// ½âÎö×ÖÄ»
void parseGPSAndHighFromSRT(const std::string &SRTFilename, std::vector<cv::Point2d> &vecPos, std::vector<double> &vecHigh, const std::string &saveFilename = "");

#endif /* LINE_DETECT_H__ */
