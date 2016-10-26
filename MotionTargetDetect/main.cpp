////// FeatureTracker.cpp : 定义控制台应用程序的入口点。
//////
////
////#include <opencv2\highgui\highgui.hpp>
////#include <opencv2\imgproc\imgproc.hpp>
////#include <string>
////#include <iomanip>
////
////using namespace cv;
////using namespace std;
////
//////帧处理基类  
////class FrameProcessor
////{
////public:
////	virtual void process(Mat &input, Mat &ouput) = 0;
////};
////
////class BGFGSegmentor :public FrameProcessor
////{
////	Mat gray;//当前帧灰度图  
////	Mat background;//背景图，格式为32位浮点  
////	Mat backImage;//CV_8U格式背景图  
////	Mat foreground;//前景图  
////	double learningRate;//学习率  
////	int Thres;//阈值，滤去扰动
////public:
////	BGFGSegmentor() :Thres(25), learningRate(0.2) {}
////
////	//帧处理函数  
////	void process(Mat &frame, Mat &output)
////	{
////		//转化为灰度图
////		cvtColor(frame, gray, CV_BGR2GRAY);
////		if (background.empty())
////			//第一帧
////			gray.convertTo(background, CV_32F);
////		//背景转为CV_8U格式以便求取和当前帧差的绝对值
////		background.convertTo(backImage, CV_8U);
////		//求当前帧与背景的差别  
////		absdiff(backImage, gray, foreground);
////		//过滤掉前景中与背景差别不大的扰动点
////		threshold(foreground, output, Thres, 255, THRESH_BINARY_INV);
////		//更新背景，output作为掩码  
////		accumulateWeighted(gray, background, learningRate, output);
////	}
////};
////
////
////
////class VideoProcessor
////{
////private:
////	VideoCapture caputure;
////	//写视频流对象  
////	VideoWriter writer;
////	//输出文件名  
////	string Outputfile;
////	int currentIndex;
////	int digits;
////	string extension;
////	FrameProcessor *frameprocessor;
////	//图像处理函数指针  
////	void(*process)(Mat &, Mat &);
////	bool callIt;
////	string WindowNameInput;
////	string WindowNameOutput;
////	//延时  
////	int delay;
////	long fnumber;
////	//第frameToStop停止  
////	long frameToStop;
////	//暂停标志  
////	bool stop;
////	//图像序列作为输入视频流  
////	vector<string> images;
////	//迭代器  
////public:
////	VideoProcessor() :callIt(true), delay(0), fnumber(0), stop(false), digits(0), frameToStop(-1) {}
////
////
////	//设置图像处理函数  
////	void setFrameProcessor(void(*process)(Mat &, Mat &)) {
////		frameprocessor = 0;
////		this->process = process;
////		CallProcess();
////	}
////	//打开视频  
////	bool setInput(string filename) {
////		fnumber = 0;
////		//若已打开，释放重新打开  
////		caputure.release();
////		return caputure.open(filename);
////	}
////	//设置输入视频播放窗口  
////	void displayInput(string wn) {
////		WindowNameInput = wn;
////		namedWindow(WindowNameInput);
////	}
////	//设置输出视频播放窗口  
////	void displayOutput(string wn) {
////		WindowNameOutput = wn;
////		namedWindow(WindowNameOutput);
////	}
////	//销毁窗口  
////	void dontDisplay() {
////		destroyWindow(WindowNameInput);
////		destroyWindow(WindowNameOutput);
////		WindowNameInput.clear();
////		WindowNameOutput.clear();
////	}
////
////	//启动  
////	void run() {
////		Mat frame;
////		Mat output;
////		if (!isOpened())
////			return;
////		stop = false;
////		while (!isStopped()) {
////			//读取下一帧  
////			if (!readNextFrame(frame))
////				break;
////			if (WindowNameInput.length() != 0)
////				imshow(WindowNameInput, frame);
////			//处理该帧  
////			if (callIt) {
////				if (process)
////					process(frame, output);
////				else if (frameprocessor)
////					frameprocessor->process(frame, output);
////			}
////			else {
////				output = frame;
////			}
////			if (Outputfile.length()) {
////				cvtColor(output, output, CV_GRAY2BGR);
////				writeNextFrame(output);
////			}
////			if (WindowNameOutput.length() != 0)
////				imshow(WindowNameOutput, output);
////			//按键暂停，继续按键继续  
////			if (delay >= 0 && waitKey(delay) >= 0)
////				waitKey(0);
////			//到达指定暂停键，退出  
////			if (frameToStop >= 0 && getFrameNumber() == frameToStop)
////				stopIt();
////		}
////	}
////	//暂停键置位  
////	void stopIt() {
////		stop = true;
////	}
////	//查询暂停标志位  
////	bool isStopped() {
////		return stop;
////	}
////	//返回视频打开标志  
////	bool isOpened() {
////		return  caputure.isOpened() || !images.empty();
////	}
////	//设置延时  
////	void setDelay(int d) {
////		delay = d;
////	}
////	//读取下一帧  
////	bool readNextFrame(Mat &frame) {
////		if (images.size() == 0)
////			return caputure.read(frame);
////		else {
////			if (itImg != images.end()) {
////				frame = imread(*itImg);
////				itImg++;
////				return frame.data ? 1 : 0;
////			}
////			else
////				return false;
////		}
////	}
////
////	void CallProcess() {
////		callIt = true;
////	}
////	void  dontCallProcess() {
////		callIt = false;
////	}
////	//设置停止帧  
////	void stopAtFrameNo(long frame) {
////		frameToStop = frame;
////	}
////	// 获得当前帧的位置  
////	long getFrameNumber() {
////		long fnumber = static_cast<long>(caputure.get((CV_CAP_PROP_POS_FRAMES)));
////		return fnumber;
////	}
////
////	//获得帧大小  
////	Size getFrameSize() {
////		if (images.size() == 0) {
////			// 从视频流获得帧大小  
////			int w = static_cast<int>(caputure.get(CV_CAP_PROP_FRAME_WIDTH));
////			int h = static_cast<int>(caputure.get(CV_CAP_PROP_FRAME_HEIGHT));
////			return Size(w, h);
////		}
////		else {
////			//从图像获得帧大小  
////			cv::Mat tmp = cv::imread(images[0]);
////			return (tmp.data) ? (tmp.size()) : (Size(0, 0));
////		}
////	}
////
////	//获取帧率  
////	double getFrameRate() {
////		return caputure.get(CV_CAP_PROP_FPS);
////	}
////	vector<string>::const_iterator itImg;
////	bool setInput(const vector<string> &imgs) {
////		fnumber = 0;
////		caputure.release();
////		images = imgs;
////		itImg = images.begin();
////		return true;
////	}
////
////	void  setFrameProcessor(FrameProcessor *frameprocessor) {
////		process = 0;
////		this->frameprocessor = frameprocessor;
////		CallProcess();
////	}
////
////	//获得编码类型  
////	int getCodec(char codec[4]) {
////		if (images.size() != 0)
////			return -1;
////		union { // 数据结构4-char  
////			int value;
////			char code[4];
////		} returned;
////		//获得编码值  
////		returned.value = static_cast<int>(
////			caputure.get(CV_CAP_PROP_FOURCC));
////		// get the 4 characters  
////		codec[0] = returned.code[0];
////		codec[1] = returned.code[1];
////		codec[2] = returned.code[2];
////		codec[3] = returned.code[3];
////		return returned.value;
////	}
////
////
////	bool setOutput(const string &filename, int codec = 0, double framerate = 0.0, bool isColor = true) {
////		//设置文件名  
////		Outputfile = filename;
////		//清空扩展名  
////		extension.clear();
////		//设置帧率  
////		if (framerate == 0.0) {
////			framerate = getFrameRate();
////		}
////		//获取输入原视频的编码方式  
////		char c[4];
////		if (codec == 0) {
////			codec = getCodec(c);
////		}
////		return writer.open(Outputfile,
////			codec,
////			framerate,
////			getFrameSize(),
////			isColor);
////	}
////
////	//输出视频帧到图片fileme+currentIndex.ext,如filename001.jpg  
////	bool setOutput(const string &filename,//路径  
////		const string &ext,//扩展名  
////		int numberOfDigits = 3,//数字位数  
////		int startIndex = 0) {//起始索引  
////		if (numberOfDigits < 0)
////			return false;
////		Outputfile = filename;
////		extension = ext;
////		digits = numberOfDigits;
////		currentIndex = startIndex;
////		return true;
////	}
////
////	//写下一帧  
////	void writeNextFrame(Mat &frame) {
////		//如果扩展名不为空，写到图片文件中  
////		if (extension.length()) {
////			stringstream ss;
////			ss << Outputfile << setfill('0') << setw(digits) << currentIndex++ << extension;
////			imwrite(ss.str(), frame);
////		}
////		//反之，写到视频文件中  
////		else {
////			writer.write(frame);
////		}
////	}
////};
////
////int main(int argc, char *argv[])
////{
////	VideoProcessor processor;
////	BGFGSegmentor segmentor;
////	//打开输入视频  
////	//processor.setInput("walk.avi");
////	processor.setInput("C:/DJI_0005_Compress.MP4");
////	processor.displayInput("Current Frame");
////	processor.displayOutput("Output Frame");
////	//设置每一帧的延时  
////	processor.setDelay(1000. / processor.getFrameRate());
////	//设置帧处理函数，可以任意  
////	processor.setFrameProcessor(&segmentor);
////	processor.setOutput("walkout", ".avi");
////	processor.setOutput("walkout", ".jpg");
////	processor.run();
////	return 0;
////}
//
////#include <iostream>  
////#include <string>  
////
////#include <opencv2/opencv.hpp>  
//
//
//
////////////////////////////////////////////////////////////////////////////
//
//
////int main(int argc, char** argv)
////{
////	std::string videoFile = "C:/DJI_0005_Compress.MP4";
////
////	cv::VideoCapture capture;
////	capture.open(videoFile);
////
////	if (!capture.isOpened())
////	{
////		std::cout << "read video failure" << std::endl;
////		return -1;
////	}
////
////
////	cv::BackgroundSubtractorMOG2 mog;
////
////	cv::Mat foreground;
////	cv::Mat background;
////
////	cv::Mat frame;
////	long frameNo = 0;
////	while (capture.read(frame))
////	{
////		++frameNo;
////
////		std::cout << frameNo << std::endl;
////
////		// 运动前景检测，并更新背景  
////		mog(frame, foreground, 0.001);
////
////		// 腐蚀  
////		cv::erode(foreground, foreground, cv::Mat());
////
////		// 膨胀  
////		cv::dilate(foreground, foreground, cv::Mat());
////
////		mog.getBackgroundImage(background);   // 返回当前背景图像  
////
////		cv::imshow("video", foreground);
////		cv::imshow("background", background);
////
////
////		if (cv::waitKey(25) > 0)
////		{
////			break;
////		}
////	}
////
////
////
////	return 0;
////}
//
//
//
////////////////////////////////////////////////////////////////////////////
//
//// 运动前景检测——基于自适应背景更新
////Author: www.icvpr.com  
////Blog:   http://blog.csdn.net/icvpr  
//// 效果实在差得很
//
//#include <iostream>  
//#include <string>  
//
//#include <opencv2/opencv.hpp>  
//
//int main(int argc, char** argv)
//{
//	std::string videoFileName = "C:/DJI_0005_Compress.MP4";
//
//	int threshold = 25;    // 二值化阈值  
//	//float alpha = 0.01;     // 更新速度 [0, 1]  
//	float alpha = 0.1;    // 更新速度 [0, 1]  
//
//	cv::VideoCapture capture;
//	capture.open(videoFileName);
//	if (!capture.isOpened())
//	{
//		std::cout << "cannot open video" << std::endl;
//		return -1;
//	}
//
//
//	cv::Mat foregroundImg;
//	cv::Mat foregroundMat;
//
//	cv::Mat backgroundImg;
//	cv::Mat backgroundMat;
//
//	cv::Mat frame;
//	cv::Mat grayImg;
//	cv::Mat grayMat;
//
//	while (capture.read(frame))
//	{
//		cv::cvtColor(frame, grayImg, CV_BGR2GRAY);
//		grayImg.convertTo(grayMat, CV_32FC1);
//
//		if (backgroundMat.empty())
//		{
//			grayImg.copyTo(backgroundImg);
//			grayImg.convertTo(backgroundMat, CV_32FC1);
//		}
//
//		// 背景减除  
//		cv::absdiff(grayMat, backgroundMat, foregroundMat);
//
//		// 自适应背景更新  
//		cv::addWeighted(backgroundMat, alpha, foregroundMat, 1 - alpha, 0, backgroundMat);
//
//		// 二值化，获取前景像素点  
//		cv::threshold(foregroundMat, foregroundMat, threshold, 255, CV_THRESH_BINARY);
//
//
//		// 为了显示用，将CV_32FC1转换为CV_8U  
//		cv::convertScaleAbs(foregroundMat, foregroundImg);
//		cv::convertScaleAbs(backgroundMat, backgroundImg);
//
//		cv::imshow("frame", frame);
//		cv::imshow("foreground", foregroundImg);
//		cv::imshow("background", backgroundImg);
//
//		if (cv::waitKey(25) > 0)
//		{
//			break;
//		}
//	}
//
//	return 0;
//}
//
//
//
#include <opencv2\opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "common.h"

int main(int argc, char *argv[])
{
	const std::string videoFilename = "G:\\resources\\videos\\DJI_0003.MOV";
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	cv::Mat frame;
	cv::Mat framePrev;
	cap.set(CV_CAP_PROP_POS_FRAMES, 1500);
	cap >> frame;
	cv::cvtColor(frame, frame, CV_BGR2GRAY);
	cv::resize(frame, frame, cv::Size(1024, 540));
	cv::namedWindow("SHOW");
	cv::namedWindow("SHOW_ORI");
	cv::moveWindow("SHOW", 0, 0);
	cv::moveWindow("SHOW_ORI", 0, 0);

	for (;;) {
		framePrev = frame;
		cap >> frame;
		cv::resize(frame, frame, cv::Size(1024, 540));
		cv::cvtColor(frame, frame, CV_BGR2GRAY);
		cv::Mat frameDiff;
		cv::absdiff(frame, framePrev, frameDiff);
		cv::threshold(frameDiff, frameDiff, 20, 255, CV_THRESH_BINARY);

		const int erosionSz = 1;
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
			cv::Size(2 * erosionSz + 1, 2 * erosionSz + 1));
		//cv::erode(frameDiff, frameDiff, element);
		cv::morphologyEx(frameDiff, frameDiff, CV_MOP_OPEN, element);
		//cv::dilate(frameDiff, frameDiff, element);
		//cv::dilate(frameDiff, frameDiff, element);
		cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT,
			cv::Size(8 * erosionSz + 1, 8 * erosionSz + 1));
		cv::dilate(frameDiff, frameDiff, elementDilate);
		cv::imshow("SHOW", frameDiff);
		cv::imshow("SHOW_ORI", frame);
		char ch = cv::waitKey(1);
		if (toupper(ch) == 'Q')
			break;
		else if (toupper(ch) == 'P')
			char ch = cv::waitKey(0);
	}

	return 0;
}


//
//#include "highgui.h"  
//#include "cxcore.h"  
//#include "ml.h"  
//#include <opencv2/opencv.hpp>
//#include "cv.h"  
//
//void main()
//{
//	CvCapture* capture;
//	capture = cvCaptureFromFile("G:\\resources\\videos\\DJI_0003.MOV");//获取视频  
//	cvNamedWindow("camera", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("moving area", CV_WINDOW_AUTOSIZE);
//
//
//	IplImage* tempFrame;//用于遍历capture中的帧，通道数为3，需要转化为单通道才可以处理  
//	IplImage* currentFrame;//当前帧  
//	IplImage* previousFrame;//上一帧  
//							/*
//							CvMat结构，本质上和IplImage差不多，但是因为IplImage里的数据只能用uchar的形式存放，当需要这些图像数据看作数据矩阵来运算时，0~255的精度显然满足不了要求；
//							然而CvMat里却可以存放任意通道数、任意格式的数据
//							*/
//	CvMat* tempFrameMat;
//	CvMat* currentFrameMat; //IplImage要转成CvMat进行处理  
//	CvMat* previousFrameMat;
//
//	int frameNum = 0;
//	cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, 1400);
//	while (tempFrame = cvQueryFrame(capture))
//	{
//		CvSize  sz;
//		sz.width = 1024;
//		sz.height = 540;
//		IplImage *dst = cvCreateImage(sz, tempFrame->depth, tempFrame->nChannels);
//		cvResize(tempFrame, dst, 1);
//		//cvCopy(dst, tempFrame);
//		tempFrame = dst;
//		//tempFrame=cvQueryFrame(capture);  
//
//		frameNum++;
//		if (frameNum == 1)
//		{
//			//第一帧先初始化各个结构，为它们分配空间  
//			previousFrame = cvCreateImage(cvSize(tempFrame->width, tempFrame->height), IPL_DEPTH_8U, 1);
//			currentFrame = cvCreateImage(cvSize(tempFrame->width, tempFrame->height), IPL_DEPTH_8U, 1);
//			currentFrameMat = cvCreateMat(tempFrame->height, tempFrame->width, CV_32FC1);
//			previousFrameMat = cvCreateMat(tempFrame->height, tempFrame->width, CV_32FC1);
//			tempFrameMat = cvCreateMat(tempFrame->height, tempFrame->width, CV_32FC1);
//			//此时这些IplImage和CvMat都是空的，没有存有数据  
//		}
//		if (frameNum >= 2)
//		{
//			cvCvtColor(tempFrame, currentFrame, CV_BGR2GRAY);//转化为单通道灰度图，此时currentFrame已经存了tempFrame的内容  
//															 /*
//															 用cvConvert将IplImage转为CvMat，接下来用cvAbsDiff对它们处理
//															 经过转换后，currentFrame没有改变，但是tempFrameMat已经存了currentFrame的内容
//															 */
//			cvConvert(currentFrame, tempFrameMat);
//			cvConvert(previousFrame, previousFrameMat);
//
//			cvAbsDiff(tempFrameMat, previousFrameMat, currentFrameMat);//做差求绝对值  
//																	   /*
//																	   在currentFrameMat中找大于20（阈值）的像素点，把currentFrame中对应的点设为255
//																	   此处阈值可以帮助把车辆的阴影消除掉
//																	   */
//			cvThreshold(currentFrameMat, currentFrame, 20, 255.0, CV_THRESH_BINARY);
//			//cvConvert(currentFrameMat,currentFrame); //观察不二值化的情况  
//
//			cvDilate(currentFrame, currentFrame);  //膨胀  
//			cvErode(currentFrame, currentFrame);   //腐蚀  
//			cvFlip(currentFrame, NULL, 0);  //垂直翻转  
//											//显示图像
//			cvNamedWindow("camera", 1);
//			cvResizeWindow("camera", 1024, 540);
//			cvNamedWindow("moving area", 1);
//			cvResizeWindow("moving area", 1024, 540);
//			cvShowImage("camera", tempFrame);
//			cvShowImage("moving area", currentFrame);
//		}
//		//把当前帧保存作为下一次处理的前一帧  
//		cvCvtColor(tempFrame, previousFrame, CV_BGR2GRAY);
//		cvWaitKey(1);
//
//	}//end while  
//
//	 //释放资源  
//	cvReleaseImage(&tempFrame);
//	cvReleaseImage(&previousFrame);
//	cvReleaseImage(&currentFrame);
//
//	cvReleaseCapture(&capture);
//	cvReleaseMat(&previousFrameMat);
//	cvReleaseMat(&currentFrameMat);
//	cvDestroyWindow("camera");
//	cvDestroyWindow("moving area");
//}