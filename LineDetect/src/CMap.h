#ifndef CMAP_H__
#define CMAP_H__
#include <opencv2\opencv.hpp>

class CMap
{
public:
	// 从全能地图下载器中加载地图
	// 
	CMap(const std::string &mapFilename, const std::string &coordinateFilename, const std::string &markedMapFilename);

	// scale，未避免地图过大，先进行缩放
	CMap(const cv::Mat &img, const cv::Point2d lb, const cv::Point2d rt);

	// 在地图中标记GPS点
	void markGPSPt(const cv::Point2d &GPSPt);

	// 根据GPS和帧在地图上保存并标记，使用RotateRect标记出区域
	// high是获取帧时的高度，f是比例尺（f即图中1mm和实际1m的关系），通过相似三角形计算得出，angle是旋转角度，可飞机的姿态信息得到
	void calcFrame(const cv::Point2d &centerGPS, const cv::Mat &frame, double high, double f, float angle = 0);

	// 得到当前帧在地图中的图片，经过旋转校正
	void getMapROI(cv::Mat &img, cv::Mat &imgMarked) const;

private:
	// 从全能电子地图下载器图像拼接行中提取坐标，格式如下：
	//:108.9117944240, 34.2332927949 
	cv::Point2d parsePt(const std::string &line) const;

	// 将经纬度转换成地图上的点
	cv::Point parseGPSPt(const cv::Point2d &GPSPos) const;

	// 获取旋转矩形校正裁剪后的图片，主要用于获取旋转后的视野内的地图
	void getCorrectedImgFromRotateRect(const cv::Mat &img, const cv::RotatedRect &rRect, cv::Mat &cropImg) const;

	// 判断经纬度点是否在给定的地图内
	bool checkInMap(const cv::Point2d &GPSPos) const;

	// 在图中画点，仅用于内部调用
	void markPt(const cv::Point &pt, const cv::Scalar &color = cv::Scalar(205, 0, 205));

	cv::Mat m_mapImg; // 大地图
	cv::Point2d m_lbGPSPt; // left bottom
	cv::Point2d m_rtGPSPt; // right top
	cv::RotatedRect m_mapROIRRect;	// 将当前帧所在区域在地图上标记，用于找出跟踪目标的位置，经过旋转后的
								//cv::Point2d centerPt; // 中心点的GPS坐标，x为纵坐标，y为横坐标
	cv::Point2d m_centerPt; // 中心点的GPS坐标，x为纵坐标，y为横坐标
	std::vector<cv::Point2d> vecCenterPt; // 从字幕中得到中心点的坐标，暂存，后期实时得到
	cv::Mat m_markedMapImg;	// 标记出遮挡区域的地图
};


#endif /* CMAP_H__ */
