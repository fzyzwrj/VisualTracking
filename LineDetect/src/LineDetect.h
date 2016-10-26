#ifndef LINE_DETECT_H__
#define LINE_DETECT_H__
#include <opencv2\opencv.hpp>

void lineDetectDetailed(const cv::Mat &srcImg, cv::Mat &houghPImg);
std::vector<cv::Vec4i> lineDetect(const cv::Mat &srcImg, cv::Mat &houghPImg);

class CMap
{
public:
	// 从全能地图下载器中加载地图
	CMap(const std::string &mapFilename, const std::string &coordinateFilename, int scale = 1, const std::string &markedMapFilename = "")
	{
		m_img = cv::imread(mapFilename);
		assert(m_img.data);
		cv::resize(m_img, m_img, cv::Size(m_img.cols / scale, m_img.rows / scale));

		std::fstream fin(coordinateFilename);
		assert(fin.is_open());
		std::string line;

		// 跳过文件头
		for (int i = 0; i < 6; ++i)
			getline(fin, line);
		
		getline(fin, line);		m_lb = parsePt(line);	// 左下角
		getline(fin, line);								// 左上角
		getline(fin, line);		m_rt = parsePt(line);	// 右上角
		getline(fin, line);								// 右下角
		if (!markedMapFilename.empty()) {
			m_ImgMarked = cv::imread(markedMapFilename);
			assert(m_ImgMarked.data);
			cv::resize(m_ImgMarked, m_ImgMarked, cv::Size(m_ImgMarked.cols / scale, m_ImgMarked.rows / scale));
		}		
	}

	// scale，未避免地图过大，先进行缩放
	CMap(const cv::Mat &img, const cv::Point2d lb, const cv::Point2d rt, int scale = 1) : m_img(img.clone()), m_lb(lb), m_rt(rt)
	{
		cv::resize(m_img, m_img, cv::Size(m_img.cols / scale, m_img.rows / scale));
	}

	// 在地图中标记GPS点
	void mark(const cv::Point2d &GPSPt)
	{
		cv::Point pt = parseGPSPt(GPSPt);
		mark(pt);
	}

	void show(void)
	{
		cv::imshow("MAP", m_imgDraw);
	}

	// 根据GPS和帧在地图上保存并标记，使用RotateRect标记出区域
	// high是获取帧时的高度，f是比例尺（f即图中1mm和实际1m的关系），通过相似三角形计算得出，angle是旋转角度，可飞机的姿态信息得到
	/*
		1个经度在不同纬线上为111km x cos(theta)，1个纬度在经度上为111km
		4k为4096 x 2160个pixel，而屏幕分辨率为96DPI，即96 pixel/inch
		先转换成毫米，然后再根据f计算
	*/
	void calcFrame(const cv::Point2d &centerGPS, const cv::Mat &frame, double high, double f, float angle = 0)
	{
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
	void getMapROI(cv::Mat &img, cv::Mat &imgMarked)
	{
		cv::Rect rect = m_mapROIRRect.boundingRect();
		cv::Mat minAreaMat = m_img(rect);
		cv::Point2f minAreaCenterPt(minAreaMat.cols / 2.0f, minAreaMat.rows / 2.0f);
		// 得到旋转矩阵，逆时针方向
		cv::Mat rotMat = cv::getRotationMatrix2D(minAreaCenterPt, m_mapROIRRect.angle, 1);

		// 对源图旋转
		cv::Mat rotImg;
		cv::warpAffine(minAreaMat, rotImg, rotMat, minAreaMat.size());

		// 裁剪
		cv::Point2f newCenterPt(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
		cv::getRectSubPix(rotImg, m_mapROIRRect.size, newCenterPt, img);

		// 对mark图片采取同样的操作
		// 这一部分传参有些奇怪
		if (m_ImgMarked.data) {
			cv::Mat minAreaMat = m_ImgMarked(rect);

			cv::Point2f minAreaCenterPt(minAreaMat.cols / 2.0f, minAreaMat.rows / 2.0f);
			// 得到旋转矩阵，逆时针方向
			cv::Mat rotMat = cv::getRotationMatrix2D(minAreaCenterPt, m_mapROIRRect.angle, 1);

			// 对源图旋转
			cv::Mat rotImg;
			cv::warpAffine(minAreaMat, rotImg, rotMat, minAreaMat.size());
			// 裁剪
			cv::Point2f newCenterPt(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
			cv::getRectSubPix(rotImg, m_mapROIRRect.size, newCenterPt, imgMarked);
		}
	}

	//void drawMapROI(cv::Mat &img, double angle = 0)
	//{
	//	cv::Rect rect = m_mapROIRRect.boundingRect();
	//	img = m_img(rect).clone();
	//	// 可能会有些计算偏移，除以2的时候
	//	cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(img.cols / 2.0, img.rows / 2.0), m_mapROIRRect.size, m_mapROIRRect.angle);
	//	cv::Point2f vertices[4];
	//	rRect.points(vertices);
	//	for (int i = 0; i < 4; i++)
	//		line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));

	//	// 还是逆时针旋转
	//	cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point2f(img.cols / 2.0, img.rows / 2.0), m_mapROIRRect.angle, 1);
	//	cv::Mat imgRot = cv::Mat::zeros(rRect.size, img.type());
	//	cv::warpAffine(img, imgRot, rotMat, img.size());
	//	cv::Point2f centerPt(img.cols / 2.0f, img.rows / 2.0f);
	//	cv::getRectSubPix(imgRot, rRect.size, centerPt, imgRot);
	//	cv::imshow("IMG", imgRot);
	//	cv::waitKey(0);
	//	//rRect = cv::RotatedRect(rRect.center, rRect.size, -m_mapROIRRect.angle);
	//}

private:
	// 从全能电子地图下载器图像拼接行中提取坐标，格式如下：
	//:108.9117944240, 34.2332927949 
	cv::Point2d parsePt(const std::string line)
	{
		// 可能性能有些低，可以用C优化
		std::string::size_type pos = line.find(":");
		assert(pos != std::string::npos);
		const std::string &str = line.substr(pos + 1);
		double longitude = std::stod(str, &pos);
		double latitude = std::stod(str.substr(pos + 1));
		return cv::Point2d(longitude, latitude);
	}

	// 将经纬度（纬度在前）转换成地图上的点
	cv::Point parseGPSPt(const cv::Point2d &GPSPos)
	{
		assert(checkInMap(GPSPos));
		// 归一化
		double longitudeNormal = (GPSPos.x - m_lb.x) / (m_rt.x - m_lb.x);
		double latitudeNormal = (GPSPos.y - m_lb.y) / (m_rt.y - m_lb.y);

		// GPS坐标与OpenCV坐标起始点不同，纵向需要反方向计算
		latitudeNormal = 1 - latitudeNormal;
		return cv::Point((int)(longitudeNormal * m_img.cols), (int)(latitudeNormal * m_img.rows));
	}

	// 检查经纬度点是否在给定的地图内
	bool checkInMap(const cv::Point2d &GPSPos)
	{
		assert(m_lb.x < GPSPos.x);
		assert(GPSPos.x < m_rt.x);
		assert(m_lb.y < GPSPos.y);
		assert(GPSPos.y < m_rt.y);
		return (m_lb.x < GPSPos.x) && (m_lb.y < GPSPos.y) && (GPSPos.x < m_rt.x) && (GPSPos.y < m_rt.y);
	}

	// 在图中画点，仅用于内部调用
	void mark(const cv::Point pt, const cv::Scalar &color = cv::Scalar(205, 0, 205))
	{
		const int d = 3;
		cv::line(m_img, cv::Point(pt.x - d, pt.y - d), cv::Point(pt.x + d, pt.y + d), color, 4, CV_AA, 0);
		cv::line(m_img, cv::Point(pt.x - d, pt.y + d), cv::Point(pt.x + d, pt.y - d), color, 4, CV_AA, 0);
	}

	cv::Mat m_imgDraw;	// 备份大地图，用于时时刷新
	cv::Mat m_img; // 大地图
	cv::Point2d m_lb; // left bottom
	cv::Point2d m_rt; // right top
	cv::RotatedRect m_mapROIRRect;	// 将当前帧所在区域在地图上标记，用于找出跟踪目标的位置，经过旋转后的
								//cv::Point2d centerPt; // 中心点的GPS坐标，x为纵坐标，y为横坐标
	cv::Point2d m_centerPt; // 中心点的GPS坐标，x为纵坐标，y为横坐标
	std::vector<cv::Point2d> vecCenterPt; // 从字幕中得到中心点的坐标，暂存，后期实时得到
	cv::Mat m_ImgMarked;	// 
};

// 解析字幕
void parseGPSAndHighFromSRT(const std::string &SRTFilename, std::vector<cv::Point2d> &vecPos, std::vector<double> &vecHigh, const std::string &saveFilename = "");

#endif /* LINE_DETECT_H__ */
