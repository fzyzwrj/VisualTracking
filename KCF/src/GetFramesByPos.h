#ifndef GET_FRAMES_BY_POS_TXT__
#define GET_FRAMES_BY_POS_TXT__

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
// ͨ�����ٹ켣���ı��ļ�����ö�Ӧ����Ƶ֡���������Ӧ���ٷ�ֵ
int getFramesByPos(const std::string &videoFilename, const std::vector<cv::Rect> &vecTrackPos, const std::vector<int> &vecFrameIndex, const std::string &dirName);

int getFramesByRandom(const std::string &videoFilename, const std::vector<cv::Rect> &vecTrackPos, const std::vector<int> &vecFrameIndex, const std::string &dirName);

#endif /* GET_FRAMES_BY_POS_TXT__ */
