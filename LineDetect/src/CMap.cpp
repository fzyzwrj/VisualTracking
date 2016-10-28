#include <iostream>
#include <string>
#include <fstream>
#include <opencv2\opencv.hpp>
#include <assert.h>

#include "CMap.h"

// ��ȫ�ܵ�ͼ�������м��ص�ͼ
CMap::CMap(const std::string &mapFilename, const std::string &coordinateFilename, const std::string &markedMapFilename)
{
	m_mapImg = cv::imread(mapFilename);
	m_mapMarkedImg = cv::imread(markedMapFilename);
	std::fstream fin(coordinateFilename);

	assert(m_mapImg.data);
	assert(m_mapMarkedImg.data);
	assert(fin.is_open());

	// ���������ļ�ǰ6�У���Ч����
	std::string line;
	for (int i = 0; i < 6; ++i)
		std::getline(fin, line);

	std::getline(fin, line);		m_lbGPSPt = parsePt(line);	// ���½�
	std::getline(fin, line);								// ���Ͻ�
	std::getline(fin, line);		m_rtGPSPt = parsePt(line);	// ���Ͻ�
	std::getline(fin, line);								// ���½�
}

// scale��δ�����ͼ�����Ƚ�������
CMap::CMap(const cv::Mat &img, const cv::Point2d lb, const cv::Point2d rt) : m_mapImg(img.clone()), m_lbGPSPt(lb), m_rtGPSPt(rt)
{

}

// �ڵ�ͼ�б��GPS��
void CMap::markGPSPt(const cv::Point2d &GPSPt)
{
	cv::Point pt = parseGPSPt(GPSPt);
	markPt(pt);
}

// ����GPS��֡�ڵ�ͼ�ϱ��沢��ǣ�ʹ��RotateRect��ǳ�����
// high�ǻ�ȡ֡ʱ�ĸ߶ȣ�f�Ǳ����ߣ�f��ͼ��1mm��ʵ��1m�Ĺ�ϵ����ͨ�����������μ���ó���angle����ת�Ƕȣ��ɷɻ�����̬��Ϣ�õ�
void CMap::calcFrame(const cv::Point2d &centerGPS, const cv::Mat &frame, double high, double f, float angle)
{
	/*
	1�������ڲ�ͬγ����Ϊ111km x cos(theta)��1��γ���ھ�����Ϊ111km
	4kΪ4096 x 2160��pixel������Ļ�ֱ���Ϊ96DPI����96 pixel/inch
	��ת���ɺ��ף�Ȼ���ٸ���f����
	*/

	// �������ؿ����GPS�еĲ�
	double dist_x = frame.cols / 2 / 96.0 * 2.54 * 10 * high / f / 111000 / cos(centerGPS.y / 180.0 * CV_PI);
	double dixt_y = frame.rows / 2 / 96.0 * 2.54 * 10 * high / f / 111000;

	// ������GPS�㣬��������½Ǻ����Ͻ�
	m_centerPt = centerGPS;
	cv::Point lbPt = parseGPSPt(cv::Point2d(centerGPS.x - dist_x, centerGPS.y - dixt_y));
	cv::Point rtPt = parseGPSPt(cv::Point2d(centerGPS.x + dist_x, centerGPS.y + dixt_y));

	cv::Rect rect = cv::Rect(lbPt, rtPt);
	m_mapROIRRect = cv::RotatedRect(parseGPSPt(centerGPS), cv::Size2f(rect.width, rect.height), angle);
}

// �õ���ǰ֡�ڵ�ͼ�е�ͼƬ��������תУ��
void CMap::getMapROI(cv::Mat &img, cv::Mat &imgMarked) const
{
	getCorrectedImgFromRotateRect(m_mapImg, m_mapROIRRect, img);
	getCorrectedImgFromRotateRect(m_mapMarkedImg, m_mapROIRRect, imgMarked);
}

// ��ȫ�ܵ��ӵ�ͼ������ͼ��ƴ��������ȡ���꣬��ʽ���£�
//:108.9117944240, 34.2332927949 
cv::Point2d CMap::parsePt(const std::string &line) const
{
	// ����������Щ�ͣ�������C�Ż�
	std::string::size_type pos = line.find(":");
	assert(pos != std::string::npos);
	const std::string &str = line.substr(pos + 1);
	double longitude = std::stod(str, &pos);
	double latitude = std::stod(str.substr(pos + 1));
	return cv::Point2d(longitude, latitude);
}

// ����γ��ת���ɵ�ͼ�ϵĵ�
cv::Point CMap::parseGPSPt(const cv::Point2d &GPSPos) const
{
	assert(checkInMap(GPSPos));
	// ��һ��
	double longitudeNormal = (GPSPos.x - m_lbGPSPt.x) / (m_rtGPSPt.x - m_lbGPSPt.x);
	double latitudeNormal = (GPSPos.y - m_lbGPSPt.y) / (m_rtGPSPt.y - m_lbGPSPt.y);

	// GPS������OpenCV������ʼ�㲻ͬ��GPS����������ͼƬ�Ϸ�����OPENCV��������ͼƬ�·�����Ҫ��
	latitudeNormal = 1 - latitudeNormal;
	return cv::Point((int)(longitudeNormal * m_mapImg.cols), (int)(latitudeNormal * m_mapImg.rows));
}

// ��ȡ��ת����У���ü����ͼƬ����Ҫ���ڻ�ȡ��ת�����Ұ�ڵĵ�ͼ
void CMap::getCorrectedImgFromRotateRect(const cv::Mat &img, const cv::RotatedRect &rRect, cv::Mat &cropImg) const
{
	// �õ���Ӿ��ε�ͼ
	cv::Rect rect = rRect.boundingRect();
	cv::Mat minAreaImg = img(rect);
	cv::Point2f center(minAreaImg.cols / 2.0f, minAreaImg.rows / 2.0f);

	// �õ���ת����angleΪ��ʱ��
	cv::Mat rotImg = cv::getRotationMatrix2D(center, rRect.angle, 1.0);

	// ��Դͼ��ת
	cv::warpAffine(minAreaImg, rotImg, rotImg, minAreaImg.size());

	// �ü�ͼ
	center = cv::Point2f(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
	cv::getRectSubPix(rotImg, rRect.size, center, cropImg);
}

// �жϾ�γ�ȵ��Ƿ��ڸ����ĵ�ͼ��
bool CMap::checkInMap(const cv::Point2d &GPSPos) const
{
	assert(m_lbGPSPt.x < GPSPos.x);
	assert(GPSPos.x < m_rtGPSPt.x);
	assert(m_lbGPSPt.y < GPSPos.y);
	assert(GPSPos.y < m_rtGPSPt.y);
	return (m_lbGPSPt.x < GPSPos.x) && (m_lbGPSPt.y < GPSPos.y) && (GPSPos.x < m_rtGPSPt.x) && (GPSPos.y < m_rtGPSPt.y);
}

// ��ͼ�л��㣬�������ڲ�����
void CMap::markPt(const cv::Point &pt, const cv::Scalar &color)
{
	const int d = 2;
	cv::line(m_mapImg, cv::Point(pt.x - d, pt.y - d), cv::Point(pt.x + d, pt.y + d), color, 4, CV_AA, 0);
	cv::line(m_mapImg, cv::Point(pt.x - d, pt.y + d), cv::Point(pt.x + d, pt.y - d), color, 4, CV_AA, 0);
}
