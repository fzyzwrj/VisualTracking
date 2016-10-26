#ifndef LINE_DETECT_H__
#define LINE_DETECT_H__
#include <opencv2\opencv.hpp>

void lineDetectDetailed(const cv::Mat &srcImg, cv::Mat &houghPImg);
std::vector<cv::Vec4i> lineDetect(const cv::Mat &srcImg, cv::Mat &houghPImg);

class CMap
{
public:
	// ��ȫ�ܵ�ͼ�������м��ص�ͼ
	CMap(const std::string &mapFilename, const std::string &coordinateFilename, int scale = 1, const std::string &markedMapFilename = "")
	{
		m_img = cv::imread(mapFilename);
		assert(m_img.data);
		cv::resize(m_img, m_img, cv::Size(m_img.cols / scale, m_img.rows / scale));

		std::fstream fin(coordinateFilename);
		assert(fin.is_open());
		std::string line;

		// �����ļ�ͷ
		for (int i = 0; i < 6; ++i)
			getline(fin, line);
		
		getline(fin, line);		m_lb = parsePt(line);	// ���½�
		getline(fin, line);								// ���Ͻ�
		getline(fin, line);		m_rt = parsePt(line);	// ���Ͻ�
		getline(fin, line);								// ���½�
		if (!markedMapFilename.empty()) {
			m_ImgMarked = cv::imread(markedMapFilename);
			assert(m_ImgMarked.data);
			cv::resize(m_ImgMarked, m_ImgMarked, cv::Size(m_ImgMarked.cols / scale, m_ImgMarked.rows / scale));
		}		
	}

	// scale��δ�����ͼ�����Ƚ�������
	CMap(const cv::Mat &img, const cv::Point2d lb, const cv::Point2d rt, int scale = 1) : m_img(img.clone()), m_lb(lb), m_rt(rt)
	{
		cv::resize(m_img, m_img, cv::Size(m_img.cols / scale, m_img.rows / scale));
	}

	// �ڵ�ͼ�б��GPS��
	void mark(const cv::Point2d &GPSPt)
	{
		cv::Point pt = parseGPSPt(GPSPt);
		mark(pt);
	}

	void show(void)
	{
		cv::imshow("MAP", m_imgDraw);
	}

	// ����GPS��֡�ڵ�ͼ�ϱ��沢��ǣ�ʹ��RotateRect��ǳ�����
	// high�ǻ�ȡ֡ʱ�ĸ߶ȣ�f�Ǳ����ߣ�f��ͼ��1mm��ʵ��1m�Ĺ�ϵ����ͨ�����������μ���ó���angle����ת�Ƕȣ��ɷɻ�����̬��Ϣ�õ�
	/*
		1�������ڲ�ͬγ����Ϊ111km x cos(theta)��1��γ���ھ�����Ϊ111km
		4kΪ4096 x 2160��pixel������Ļ�ֱ���Ϊ96DPI����96 pixel/inch
		��ת���ɺ��ף�Ȼ���ٸ���f����
	*/
	void calcFrame(const cv::Point2d &centerGPS, const cv::Mat &frame, double high, double f, float angle = 0)
	{
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
	void getMapROI(cv::Mat &img, cv::Mat &imgMarked)
	{
		cv::Rect rect = m_mapROIRRect.boundingRect();
		cv::Mat minAreaMat = m_img(rect);
		cv::Point2f minAreaCenterPt(minAreaMat.cols / 2.0f, minAreaMat.rows / 2.0f);
		// �õ���ת������ʱ�뷽��
		cv::Mat rotMat = cv::getRotationMatrix2D(minAreaCenterPt, m_mapROIRRect.angle, 1);

		// ��Դͼ��ת
		cv::Mat rotImg;
		cv::warpAffine(minAreaMat, rotImg, rotMat, minAreaMat.size());

		// �ü�
		cv::Point2f newCenterPt(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
		cv::getRectSubPix(rotImg, m_mapROIRRect.size, newCenterPt, img);

		// ��markͼƬ��ȡͬ���Ĳ���
		// ��һ���ִ�����Щ���
		if (m_ImgMarked.data) {
			cv::Mat minAreaMat = m_ImgMarked(rect);

			cv::Point2f minAreaCenterPt(minAreaMat.cols / 2.0f, minAreaMat.rows / 2.0f);
			// �õ���ת������ʱ�뷽��
			cv::Mat rotMat = cv::getRotationMatrix2D(minAreaCenterPt, m_mapROIRRect.angle, 1);

			// ��Դͼ��ת
			cv::Mat rotImg;
			cv::warpAffine(minAreaMat, rotImg, rotMat, minAreaMat.size());
			// �ü�
			cv::Point2f newCenterPt(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
			cv::getRectSubPix(rotImg, m_mapROIRRect.size, newCenterPt, imgMarked);
		}
	}

	//void drawMapROI(cv::Mat &img, double angle = 0)
	//{
	//	cv::Rect rect = m_mapROIRRect.boundingRect();
	//	img = m_img(rect).clone();
	//	// ���ܻ���Щ����ƫ�ƣ�����2��ʱ��
	//	cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(img.cols / 2.0, img.rows / 2.0), m_mapROIRRect.size, m_mapROIRRect.angle);
	//	cv::Point2f vertices[4];
	//	rRect.points(vertices);
	//	for (int i = 0; i < 4; i++)
	//		line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));

	//	// ������ʱ����ת
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
	// ��ȫ�ܵ��ӵ�ͼ������ͼ��ƴ��������ȡ���꣬��ʽ���£�
	//:108.9117944240, 34.2332927949 
	cv::Point2d parsePt(const std::string line)
	{
		// ����������Щ�ͣ�������C�Ż�
		std::string::size_type pos = line.find(":");
		assert(pos != std::string::npos);
		const std::string &str = line.substr(pos + 1);
		double longitude = std::stod(str, &pos);
		double latitude = std::stod(str.substr(pos + 1));
		return cv::Point2d(longitude, latitude);
	}

	// ����γ�ȣ�γ����ǰ��ת���ɵ�ͼ�ϵĵ�
	cv::Point parseGPSPt(const cv::Point2d &GPSPos)
	{
		assert(checkInMap(GPSPos));
		// ��һ��
		double longitudeNormal = (GPSPos.x - m_lb.x) / (m_rt.x - m_lb.x);
		double latitudeNormal = (GPSPos.y - m_lb.y) / (m_rt.y - m_lb.y);

		// GPS������OpenCV������ʼ�㲻ͬ��������Ҫ���������
		latitudeNormal = 1 - latitudeNormal;
		return cv::Point((int)(longitudeNormal * m_img.cols), (int)(latitudeNormal * m_img.rows));
	}

	// ��龭γ�ȵ��Ƿ��ڸ����ĵ�ͼ��
	bool checkInMap(const cv::Point2d &GPSPos)
	{
		assert(m_lb.x < GPSPos.x);
		assert(GPSPos.x < m_rt.x);
		assert(m_lb.y < GPSPos.y);
		assert(GPSPos.y < m_rt.y);
		return (m_lb.x < GPSPos.x) && (m_lb.y < GPSPos.y) && (GPSPos.x < m_rt.x) && (GPSPos.y < m_rt.y);
	}

	// ��ͼ�л��㣬�������ڲ�����
	void mark(const cv::Point pt, const cv::Scalar &color = cv::Scalar(205, 0, 205))
	{
		const int d = 3;
		cv::line(m_img, cv::Point(pt.x - d, pt.y - d), cv::Point(pt.x + d, pt.y + d), color, 4, CV_AA, 0);
		cv::line(m_img, cv::Point(pt.x - d, pt.y + d), cv::Point(pt.x + d, pt.y - d), color, 4, CV_AA, 0);
	}

	cv::Mat m_imgDraw;	// ���ݴ��ͼ������ʱʱˢ��
	cv::Mat m_img; // ���ͼ
	cv::Point2d m_lb; // left bottom
	cv::Point2d m_rt; // right top
	cv::RotatedRect m_mapROIRRect;	// ����ǰ֡���������ڵ�ͼ�ϱ�ǣ������ҳ�����Ŀ���λ�ã�������ת���
								//cv::Point2d centerPt; // ���ĵ��GPS���꣬xΪ�����꣬yΪ������
	cv::Point2d m_centerPt; // ���ĵ��GPS���꣬xΪ�����꣬yΪ������
	std::vector<cv::Point2d> vecCenterPt; // ����Ļ�еõ����ĵ�����꣬�ݴ棬����ʵʱ�õ�
	cv::Mat m_ImgMarked;	// 
};

// ������Ļ
void parseGPSAndHighFromSRT(const std::string &SRTFilename, std::vector<cv::Point2d> &vecPos, std::vector<double> &vecHigh, const std::string &saveFilename = "");

#endif /* LINE_DETECT_H__ */
