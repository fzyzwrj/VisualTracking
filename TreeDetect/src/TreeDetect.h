#ifndef TREE_DETECT_H__
#define TREE_DETECT_H__

#include <opencv2\opencv.hpp>

// 定义宏变量DEBUG_SHOW，调用D_SHOW时会显示中间图片
#ifndef DEBUG_SHOW
#define DEBUG_SHOW
#endif

// 输入三通道的原图，进行绿色检测返回二值化后的图像
int colorFilter(const cv::Mat &srcImg, cv::Mat &dstImg);

#endif // TREE_DETECT_H__