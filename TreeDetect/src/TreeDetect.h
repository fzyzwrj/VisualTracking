#ifndef TREE_DETECT_H__
#define TREE_DETECT_H__

#include <opencv2\opencv.hpp>

// ��������DEBUG_SHOW������D_SHOWʱ����ʾ�м�ͼƬ
#ifndef DEBUG_SHOW
#define DEBUG_SHOW
#endif

// ������ͨ����ԭͼ��������ɫ��ⷵ�ض�ֵ�����ͼ��
int colorFilter(const cv::Mat &srcImg, cv::Mat &dstImg);

#endif // TREE_DETECT_H__