#ifndef LINE_DETECT_H__
#define LINE_DETECT_H__
#include <opencv2\opencv.hpp>
#include <string>
#include <vector>

void lineDetectDetailed(const cv::Mat &srcImg, cv::Mat &houghPImg);
size_t lineDetect(const cv::Mat &srcImg, cv::vector<cv::Vec4i> &lines);
void drawLines(cv::Mat &img, const cv::vector<cv::Vec4i> &lines);

// ½âÎö×ÖÄ»
void parseGPSAndHighFromSRT(const std::string &SRTFilename, std::vector<cv::Point2d> &vecPos, std::vector<double> &vecHigh, const std::string &saveFilename = "");

#endif /* LINE_DETECT_H__ */
