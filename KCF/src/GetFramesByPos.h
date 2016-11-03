#ifndef GET_FRAMES_BY_POS_TXT__
#define GET_FRAMES_BY_POS_TXT__

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
// 通过跟踪轨迹的文本文件，获得对应的视频帧，并输出对应跟踪峰值
int getFramesByPos(const std::string &videoFilename, const std::vector<cv::Rect> &vecTrackPos, const std::vector<int> &vecFrameIndex, const std::string &dirName);

int getFramesByRandom(const std::string &videoFilename, const std::vector<cv::Rect> &vecTrackPos, const std::vector<int> &vecFrameIndex, const std::string &dirName);

#endif /* GET_FRAMES_BY_POS_TXT__ */
