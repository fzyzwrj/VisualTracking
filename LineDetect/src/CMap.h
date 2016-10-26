#ifndef CMAP_H__
#define CMAP_H__
#include <opencv2\opencv.hpp>

class CMap
{
public:
	// ��ȫ�ܵ�ͼ�������м��ص�ͼ
	// 
	CMap(const std::string &mapFilename, const std::string &coordinateFilename, const std::string &markedMapFilename);

	// scale��δ�����ͼ�����Ƚ�������
	CMap(const cv::Mat &img, const cv::Point2d lb, const cv::Point2d rt);

	// �ڵ�ͼ�б��GPS��
	void markGPSPt(const cv::Point2d &GPSPt);

	// ����GPS��֡�ڵ�ͼ�ϱ��沢��ǣ�ʹ��RotateRect��ǳ�����
	// high�ǻ�ȡ֡ʱ�ĸ߶ȣ�f�Ǳ����ߣ�f��ͼ��1mm��ʵ��1m�Ĺ�ϵ����ͨ�����������μ���ó���angle����ת�Ƕȣ��ɷɻ�����̬��Ϣ�õ�
	void calcFrame(const cv::Point2d &centerGPS, const cv::Mat &frame, double high, double f, float angle = 0);

	// �õ���ǰ֡�ڵ�ͼ�е�ͼƬ��������תУ��
	void getMapROI(cv::Mat &img, cv::Mat &imgMarked) const;

private:
	// ��ȫ�ܵ��ӵ�ͼ������ͼ��ƴ��������ȡ���꣬��ʽ���£�
	//:108.9117944240, 34.2332927949 
	cv::Point2d parsePt(const std::string &line) const;

	// ����γ��ת���ɵ�ͼ�ϵĵ�
	cv::Point parseGPSPt(const cv::Point2d &GPSPos) const;

	// ��ȡ��ת����У���ü����ͼƬ����Ҫ���ڻ�ȡ��ת�����Ұ�ڵĵ�ͼ
	void getCorrectedImgFromRotateRect(const cv::Mat &img, const cv::RotatedRect &rRect, cv::Mat &cropImg) const;

	// �жϾ�γ�ȵ��Ƿ��ڸ����ĵ�ͼ��
	bool checkInMap(const cv::Point2d &GPSPos) const;

	// ��ͼ�л��㣬�������ڲ�����
	void markPt(const cv::Point &pt, const cv::Scalar &color = cv::Scalar(205, 0, 205));

	cv::Mat m_mapImg; // ���ͼ
	cv::Point2d m_lbGPSPt; // left bottom
	cv::Point2d m_rtGPSPt; // right top
	cv::RotatedRect m_mapROIRRect;	// ����ǰ֡���������ڵ�ͼ�ϱ�ǣ������ҳ�����Ŀ���λ�ã�������ת���
								//cv::Point2d centerPt; // ���ĵ��GPS���꣬xΪ�����꣬yΪ������
	cv::Point2d m_centerPt; // ���ĵ��GPS���꣬xΪ�����꣬yΪ������
	std::vector<cv::Point2d> vecCenterPt; // ����Ļ�еõ����ĵ�����꣬�ݴ棬����ʵʱ�õ�
	cv::Mat m_markedMapImg;	// ��ǳ��ڵ�����ĵ�ͼ
};


#endif /* CMAP_H__ */
