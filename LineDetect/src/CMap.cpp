#include <iostream>
#include <string>
#include <fstream>
#include <opencv2\opencv.hpp>
#include <assert.h>

#include "CMap.h"

// 从全能地图下载器中加载地图
CMap::CMap(const std::string &mapFilename, const std::string &coordinateFilename, const std::string &markedMapFilename)
{
	m_mapImg = cv::imread(mapFilename);
	m_mapMarkedImg = cv::imread(markedMapFilename);
	std::fstream fin(coordinateFilename);

	assert(m_mapImg.data);
	assert(m_mapMarkedImg.data);
	assert(fin.is_open());

	// 跳过坐标文件前6行，无效数据
	std::string line;
	for (int i = 0; i < 6; ++i)
		std::getline(fin, line);

	std::getline(fin, line);		m_lbGPSPt = parsePt(line);	// 左下角
	std::getline(fin, line);								// 左上角
	std::getline(fin, line);		m_rtGPSPt = parsePt(line);	// 右上角
	std::getline(fin, line);								// 右下角
}

// scale，未避免地图过大，先进行缩放
CMap::CMap(const cv::Mat &img, const cv::Point2d lb, const cv::Point2d rt) : m_mapImg(img.clone()), m_lbGPSPt(lb), m_rtGPSPt(rt)
{

}

// 在地图中标记GPS点
void CMap::markGPSPt(const cv::Point2d &GPSPt)
{
	cv::Point pt = parseGPSPt(GPSPt);
	markPt(pt);
}

// 根据GPS和帧在地图上保存并标记，使用RotateRect标记出区域
// high是获取帧时的高度，f是比例尺（f即图中1mm和实际1m的关系），通过相似三角形计算得出，angle是旋转角度，可飞机的姿态信息得到
void CMap::calcFrame(const cv::Point2d &centerGPS, const cv::Mat &frame, double high, double f, float angle)
{
	/*
	1个经度在不同纬线上为111km x cos(theta)，1个纬度在经度上为111km
	4k为4096 x 2160个pixel，而屏幕分辨率为96DPI，即96 pixel/inch
	先转换成毫米，然后再根据f计算
	*/

	// 计算像素宽高在GPS中的差
	double dist_x = frame.cols / 2 / 96.0 * 2.54 * 10 * high / f / 111000 / cos(centerGPS.y / 180.0 * CV_PI);
	double dixt_y = frame.rows / 2 / 96.0 * 2.54 * 10 * high / f / 111000;

	// 由中心GPS点，计算出左下角和右上角
	m_centerPt = centerGPS;
	cv::Point lbPt = parseGPSPt(cv::Point2d(centerGPS.x - dist_x, centerGPS.y - dixt_y));
	cv::Point rtPt = parseGPSPt(cv::Point2d(centerGPS.x + dist_x, centerGPS.y + dixt_y));

	cv::Rect rect = cv::Rect(lbPt, rtPt);
	m_mapROIRRect = cv::RotatedRect(parseGPSPt(centerGPS), cv::Size2f(rect.width, rect.height), angle);
}

// 得到当前帧在地图中的图片，经过旋转校正
void CMap::getMapROI(cv::Mat &img, cv::Mat &imgMarked) const
{
	getCorrectedImgFromRotateRect(m_mapImg, m_mapROIRRect, img);
	getCorrectedImgFromRotateRect(m_mapMarkedImg, m_mapROIRRect, imgMarked);
}

// 从全能电子地图下载器图像拼接行中提取坐标，格式如下：
//:108.9117944240, 34.2332927949 
cv::Point2d CMap::parsePt(const std::string &line) const
{
	// 可能性能有些低，可以用C优化
	std::string::size_type pos = line.find(":");
	assert(pos != std::string::npos);
	const std::string &str = line.substr(pos + 1);
	double longitude = std::stod(str, &pos);
	double latitude = std::stod(str.substr(pos + 1));
	return cv::Point2d(longitude, latitude);
}

// 将经纬度转换成地图上的点
cv::Point CMap::parseGPSPt(const cv::Point2d &GPSPos) const
{
	assert(checkInMap(GPSPos));
	// 归一化
	double longitudeNormal = (GPSPos.x - m_lbGPSPt.x) / (m_rtGPSPt.x - m_lbGPSPt.x);
	double latitudeNormal = (GPSPos.y - m_lbGPSPt.y) / (m_rtGPSPt.y - m_lbGPSPt.y);

	// GPS坐标与OpenCV坐标起始点不同，GPS纵坐标大的在图片上方，而OPENCV坐标大的在图片下方，需要求补
	latitudeNormal = 1 - latitudeNormal;
	return cv::Point((int)(longitudeNormal * m_mapImg.cols), (int)(latitudeNormal * m_mapImg.rows));
}

// 获取旋转矩形校正裁剪后的图片，主要用于获取旋转后的视野内的地图
void CMap::getCorrectedImgFromRotateRect(const cv::Mat &img, const cv::RotatedRect &rRect, cv::Mat &cropImg) const
{
	// 得到外接矩形的图
	cv::Rect rect = rRect.boundingRect();
	cv::Mat minAreaImg = img(rect);
	cv::Point2f center(minAreaImg.cols / 2.0f, minAreaImg.rows / 2.0f);

	// 得到旋转矩阵，angle为逆时针
	cv::Mat rotImg = cv::getRotationMatrix2D(center, rRect.angle, 1.0);

	// 对源图旋转
	cv::warpAffine(minAreaImg, rotImg, rotImg, minAreaImg.size());

	// 裁剪图
	center = cv::Point2f(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
	cv::getRectSubPix(rotImg, rRect.size, center, cropImg);
}

// 判断经纬度点是否在给定的地图内
bool CMap::checkInMap(const cv::Point2d &GPSPos) const
{
	assert(m_lbGPSPt.x < GPSPos.x);
	assert(GPSPos.x < m_rtGPSPt.x);
	assert(m_lbGPSPt.y < GPSPos.y);
	assert(GPSPos.y < m_rtGPSPt.y);
	return (m_lbGPSPt.x < GPSPos.x) && (m_lbGPSPt.y < GPSPos.y) && (GPSPos.x < m_rtGPSPt.x) && (GPSPos.y < m_rtGPSPt.y);
}

// 在图中画点，仅用于内部调用
void CMap::markPt(const cv::Point &pt, const cv::Scalar &color)
{
	const int d = 2;
	cv::line(m_mapImg, cv::Point(pt.x - d, pt.y - d), cv::Point(pt.x + d, pt.y + d), color, 4, CV_AA, 0);
	cv::line(m_mapImg, cv::Point(pt.x - d, pt.y + d), cv::Point(pt.x + d, pt.y - d), color, 4, CV_AA, 0);
}
