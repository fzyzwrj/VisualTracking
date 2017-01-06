/************************************************************************/
/* 	2016.10.9 by astratwu
	V1.0
	备注：对车道、主干道等航拍图像进行直线检测，分别出主要干道 
*/
/************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <functional>

#include <opencv2/opencv.hpp>

#include "utils.h"
#include "utils_opencv.h"

#include "LineDetect.h"
#include "CMap.h"

void getGPSFromTXT(const std::string &filename, std::vector<cv::Point2d> &vecGPS)
{
	std::fstream fin(filename);
	assert(fin.is_open());
	std::string line;
	while (getline(fin, line)) {
		double longitude = 0;
		double latitude = 0;

		sscanf_s(line.c_str(), "%lf, %lf", &latitude, &longitude);
		//vecGPS.push_back(cv::Point2d(latitude, longitude));
		vecGPS.push_back(cv::Point2d(longitude, latitude));
	}
}


// 通过点来判断是否显示正确
void test_mark_GPS()
{
	const std::string mapFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.png";
	const std::string coordinateFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.txt";
	const std::string SRTFilename = "G:\\resources\\videos\\DJI_0002.SRT";
	CMap m(mapFilename, coordinateFilename, mapFilename);

	cv::Point2d gpsPt(34.2355635246, 108.9165973293);
	m.markGPSPt(gpsPt);
	//m.show();
	cv::waitKey(0);
}
int main1(int argc, char *argv[])
{
	const std::string mapFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.png";
	const std::string coordinateFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.txt";
	const std::string SRTFilename = "G:\\resources\\videos\\DJI_0002.SRT";

	CMap m(mapFilename, coordinateFilename, mapFilename);
	std::vector<cv::Point2d> vecGPS;
	std::vector<double> vecHigh;
	parseGPSAndHighFromSRT(SRTFilename, vecGPS, vecHigh, "output_test.txt");


	for (size_t i = 0; i < vecGPS.size(); ++i) {
		// 跳到1400帧，已升高空
		if (i < 56)
			continue;
		//m.mark(vecGPS[i]);
		cv::Mat img(cv::Size(4096, 2160), 0);
		m.calcFrame(vecGPS[i], img, 100, 729, 117);
		img;
		//m.drawMapROI(img);
		//m.show();
		cv::waitKey(0);
	}
	//m.show();
	cv::waitKey(0);
	return 0;
}


int main(int argc, char *argv[])
{
	//const std::string filename = "G:\\resources\\videos\\foot_bridge\\2100.jpg";
	//const std::string filename = "G:\\resources\\videos\\foot_bridge_next_pa\\1.jpg";
	//const std::string filename = "C:\\Users\\Administrator\\Documents\\Visual Studio 2015\\Projects\\BingObjectnessCVPR14\\BingObjectnessCVPR14\\captureOcclusionVideo\\occlusion_video_42\\_add_border_40_105.jpg";
	//const std::string filename = "C:\\resources\\res\\occlusion_frame\\occlusion_video_18\\_add_border_40_52.jpg";
	//const std::string filename = "C:\\resources\\imgs\\1.jpg";
	CStr filename("G:\\resources\\videos\\overpass\\1563.jpg");
	cv::Mat srcImg = cv::imread(filename, 0);
	assert(srcImg.data);
	cv::resize(srcImg, srcImg, cv::Size(2160, 1080));
	//cv::resize(srcImg, srcImg, cv::Size(1024, 540));

	std::vector<cv::Mat> vecMat;
	srcImg = cv::imread("ori.jpg", 0);
	freopen("res.txt", "a", stdout);
	std::cout << srcImg << std::endl;
	std::cout << "####" << std::endl;
	cv::Mat img;
	cv::cvtColor(srcImg, img, CV_BayerGB2BGR);
	cv::split(img, vecMat);
	for (int i = 0; i < vecMat.size(); ++i) {
		cv::imshow(std::to_string(i), vecMat[i]);
		std::cout << vecMat[i] << std::endl;
		std::cout << "####" << std::endl;

	}
	cv::waitKey(0);

	return 0;
	cv::Mat houghPImg;
	//cv::Mat ROIImg = srcImg(cv::Rect(200, 200, 500, 500));
	cv::Mat ROIImg = srcImg(cv::Rect(1000, 500, 450, 500));
	lineDetectDetailed(ROIImg, houghPImg);

	return 0;
}





//void getMap(const std::string &mapFilename, const std::string &coordinateFilename)
//{
//	std::fstream fin(mapFilename);
//	assert(fin.is_open());
//	std::string line;
//	for (int i = 0; i < 6; ++i)
//		getline(fin, line);
//
//	// 左下角
//	getline(fin, line);
//	cv::Point2d lb = parsePt(line);	 // left bottom
//	// 左上角
//	getline(fin, line);
//	// 右上角
//	getline(fin, line);
//	cv::Point2d rt = parsePt(line);	 // left bottom
//	// 右下角
//	getline(fin, line);
//}
