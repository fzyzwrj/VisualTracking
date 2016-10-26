////// FeatureTracker.cpp : �������̨Ӧ�ó������ڵ㡣
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
//////֡�������  
////class FrameProcessor
////{
////public:
////	virtual void process(Mat &input, Mat &ouput) = 0;
////};
////
////class BGFGSegmentor :public FrameProcessor
////{
////	Mat gray;//��ǰ֡�Ҷ�ͼ  
////	Mat background;//����ͼ����ʽΪ32λ����  
////	Mat backImage;//CV_8U��ʽ����ͼ  
////	Mat foreground;//ǰ��ͼ  
////	double learningRate;//ѧϰ��  
////	int Thres;//��ֵ����ȥ�Ŷ�
////public:
////	BGFGSegmentor() :Thres(25), learningRate(0.2) {}
////
////	//֡������  
////	void process(Mat &frame, Mat &output)
////	{
////		//ת��Ϊ�Ҷ�ͼ
////		cvtColor(frame, gray, CV_BGR2GRAY);
////		if (background.empty())
////			//��һ֡
////			gray.convertTo(background, CV_32F);
////		//����תΪCV_8U��ʽ�Ա���ȡ�͵�ǰ֡��ľ���ֵ
////		background.convertTo(backImage, CV_8U);
////		//��ǰ֡�뱳���Ĳ��  
////		absdiff(backImage, gray, foreground);
////		//���˵�ǰ�����뱳����𲻴���Ŷ���
////		threshold(foreground, output, Thres, 255, THRESH_BINARY_INV);
////		//���±�����output��Ϊ����  
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
////	//д��Ƶ������  
////	VideoWriter writer;
////	//����ļ���  
////	string Outputfile;
////	int currentIndex;
////	int digits;
////	string extension;
////	FrameProcessor *frameprocessor;
////	//ͼ������ָ��  
////	void(*process)(Mat &, Mat &);
////	bool callIt;
////	string WindowNameInput;
////	string WindowNameOutput;
////	//��ʱ  
////	int delay;
////	long fnumber;
////	//��frameToStopֹͣ  
////	long frameToStop;
////	//��ͣ��־  
////	bool stop;
////	//ͼ��������Ϊ������Ƶ��  
////	vector<string> images;
////	//������  
////public:
////	VideoProcessor() :callIt(true), delay(0), fnumber(0), stop(false), digits(0), frameToStop(-1) {}
////
////
////	//����ͼ������  
////	void setFrameProcessor(void(*process)(Mat &, Mat &)) {
////		frameprocessor = 0;
////		this->process = process;
////		CallProcess();
////	}
////	//����Ƶ  
////	bool setInput(string filename) {
////		fnumber = 0;
////		//���Ѵ򿪣��ͷ����´�  
////		caputure.release();
////		return caputure.open(filename);
////	}
////	//����������Ƶ���Ŵ���  
////	void displayInput(string wn) {
////		WindowNameInput = wn;
////		namedWindow(WindowNameInput);
////	}
////	//���������Ƶ���Ŵ���  
////	void displayOutput(string wn) {
////		WindowNameOutput = wn;
////		namedWindow(WindowNameOutput);
////	}
////	//���ٴ���  
////	void dontDisplay() {
////		destroyWindow(WindowNameInput);
////		destroyWindow(WindowNameOutput);
////		WindowNameInput.clear();
////		WindowNameOutput.clear();
////	}
////
////	//����  
////	void run() {
////		Mat frame;
////		Mat output;
////		if (!isOpened())
////			return;
////		stop = false;
////		while (!isStopped()) {
////			//��ȡ��һ֡  
////			if (!readNextFrame(frame))
////				break;
////			if (WindowNameInput.length() != 0)
////				imshow(WindowNameInput, frame);
////			//�����֡  
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
////			//������ͣ��������������  
////			if (delay >= 0 && waitKey(delay) >= 0)
////				waitKey(0);
////			//����ָ����ͣ�����˳�  
////			if (frameToStop >= 0 && getFrameNumber() == frameToStop)
////				stopIt();
////		}
////	}
////	//��ͣ����λ  
////	void stopIt() {
////		stop = true;
////	}
////	//��ѯ��ͣ��־λ  
////	bool isStopped() {
////		return stop;
////	}
////	//������Ƶ�򿪱�־  
////	bool isOpened() {
////		return  caputure.isOpened() || !images.empty();
////	}
////	//������ʱ  
////	void setDelay(int d) {
////		delay = d;
////	}
////	//��ȡ��һ֡  
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
////	//����ֹͣ֡  
////	void stopAtFrameNo(long frame) {
////		frameToStop = frame;
////	}
////	// ��õ�ǰ֡��λ��  
////	long getFrameNumber() {
////		long fnumber = static_cast<long>(caputure.get((CV_CAP_PROP_POS_FRAMES)));
////		return fnumber;
////	}
////
////	//���֡��С  
////	Size getFrameSize() {
////		if (images.size() == 0) {
////			// ����Ƶ�����֡��С  
////			int w = static_cast<int>(caputure.get(CV_CAP_PROP_FRAME_WIDTH));
////			int h = static_cast<int>(caputure.get(CV_CAP_PROP_FRAME_HEIGHT));
////			return Size(w, h);
////		}
////		else {
////			//��ͼ����֡��С  
////			cv::Mat tmp = cv::imread(images[0]);
////			return (tmp.data) ? (tmp.size()) : (Size(0, 0));
////		}
////	}
////
////	//��ȡ֡��  
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
////	//��ñ�������  
////	int getCodec(char codec[4]) {
////		if (images.size() != 0)
////			return -1;
////		union { // ���ݽṹ4-char  
////			int value;
////			char code[4];
////		} returned;
////		//��ñ���ֵ  
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
////		//�����ļ���  
////		Outputfile = filename;
////		//�����չ��  
////		extension.clear();
////		//����֡��  
////		if (framerate == 0.0) {
////			framerate = getFrameRate();
////		}
////		//��ȡ����ԭ��Ƶ�ı��뷽ʽ  
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
////	//�����Ƶ֡��ͼƬfileme+currentIndex.ext,��filename001.jpg  
////	bool setOutput(const string &filename,//·��  
////		const string &ext,//��չ��  
////		int numberOfDigits = 3,//����λ��  
////		int startIndex = 0) {//��ʼ����  
////		if (numberOfDigits < 0)
////			return false;
////		Outputfile = filename;
////		extension = ext;
////		digits = numberOfDigits;
////		currentIndex = startIndex;
////		return true;
////	}
////
////	//д��һ֡  
////	void writeNextFrame(Mat &frame) {
////		//�����չ����Ϊ�գ�д��ͼƬ�ļ���  
////		if (extension.length()) {
////			stringstream ss;
////			ss << Outputfile << setfill('0') << setw(digits) << currentIndex++ << extension;
////			imwrite(ss.str(), frame);
////		}
////		//��֮��д����Ƶ�ļ���  
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
////	//��������Ƶ  
////	//processor.setInput("walk.avi");
////	processor.setInput("C:/DJI_0005_Compress.MP4");
////	processor.displayInput("Current Frame");
////	processor.displayOutput("Output Frame");
////	//����ÿһ֡����ʱ  
////	processor.setDelay(1000. / processor.getFrameRate());
////	//����֡����������������  
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
////		// �˶�ǰ����⣬�����±���  
////		mog(frame, foreground, 0.001);
////
////		// ��ʴ  
////		cv::erode(foreground, foreground, cv::Mat());
////
////		// ����  
////		cv::dilate(foreground, foreground, cv::Mat());
////
////		mog.getBackgroundImage(background);   // ���ص�ǰ����ͼ��  
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
//// �˶�ǰ����⡪����������Ӧ��������
////Author: www.icvpr.com  
////Blog:   http://blog.csdn.net/icvpr  
//// Ч��ʵ�ڲ�ú�
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
//	int threshold = 25;    // ��ֵ����ֵ  
//	//float alpha = 0.01;     // �����ٶ� [0, 1]  
//	float alpha = 0.1;    // �����ٶ� [0, 1]  
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
//		// ��������  
//		cv::absdiff(grayMat, backgroundMat, foregroundMat);
//
//		// ����Ӧ��������  
//		cv::addWeighted(backgroundMat, alpha, foregroundMat, 1 - alpha, 0, backgroundMat);
//
//		// ��ֵ������ȡǰ�����ص�  
//		cv::threshold(foregroundMat, foregroundMat, threshold, 255, CV_THRESH_BINARY);
//
//
//		// Ϊ����ʾ�ã���CV_32FC1ת��ΪCV_8U  
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
//	capture = cvCaptureFromFile("G:\\resources\\videos\\DJI_0003.MOV");//��ȡ��Ƶ  
//	cvNamedWindow("camera", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("moving area", CV_WINDOW_AUTOSIZE);
//
//
//	IplImage* tempFrame;//���ڱ���capture�е�֡��ͨ����Ϊ3����Ҫת��Ϊ��ͨ���ſ��Դ���  
//	IplImage* currentFrame;//��ǰ֡  
//	IplImage* previousFrame;//��һ֡  
//							/*
//							CvMat�ṹ�������Ϻ�IplImage��࣬������ΪIplImage�������ֻ����uchar����ʽ��ţ�����Ҫ��Щͼ�����ݿ������ݾ���������ʱ��0~255�ľ�����Ȼ���㲻��Ҫ��
//							Ȼ��CvMat��ȴ���Դ������ͨ�����������ʽ������
//							*/
//	CvMat* tempFrameMat;
//	CvMat* currentFrameMat; //IplImageҪת��CvMat���д���  
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
//			//��һ֡�ȳ�ʼ�������ṹ��Ϊ���Ƿ���ռ�  
//			previousFrame = cvCreateImage(cvSize(tempFrame->width, tempFrame->height), IPL_DEPTH_8U, 1);
//			currentFrame = cvCreateImage(cvSize(tempFrame->width, tempFrame->height), IPL_DEPTH_8U, 1);
//			currentFrameMat = cvCreateMat(tempFrame->height, tempFrame->width, CV_32FC1);
//			previousFrameMat = cvCreateMat(tempFrame->height, tempFrame->width, CV_32FC1);
//			tempFrameMat = cvCreateMat(tempFrame->height, tempFrame->width, CV_32FC1);
//			//��ʱ��ЩIplImage��CvMat���ǿյģ�û�д�������  
//		}
//		if (frameNum >= 2)
//		{
//			cvCvtColor(tempFrame, currentFrame, CV_BGR2GRAY);//ת��Ϊ��ͨ���Ҷ�ͼ����ʱcurrentFrame�Ѿ�����tempFrame������  
//															 /*
//															 ��cvConvert��IplImageתΪCvMat����������cvAbsDiff�����Ǵ���
//															 ����ת����currentFrameû�иı䣬����tempFrameMat�Ѿ�����currentFrame������
//															 */
//			cvConvert(currentFrame, tempFrameMat);
//			cvConvert(previousFrame, previousFrameMat);
//
//			cvAbsDiff(tempFrameMat, previousFrameMat, currentFrameMat);//���������ֵ  
//																	   /*
//																	   ��currentFrameMat���Ҵ���20����ֵ�������ص㣬��currentFrame�ж�Ӧ�ĵ���Ϊ255
//																	   �˴���ֵ���԰����ѳ�������Ӱ������
//																	   */
//			cvThreshold(currentFrameMat, currentFrame, 20, 255.0, CV_THRESH_BINARY);
//			//cvConvert(currentFrameMat,currentFrame); //�۲첻��ֵ�������  
//
//			cvDilate(currentFrame, currentFrame);  //����  
//			cvErode(currentFrame, currentFrame);   //��ʴ  
//			cvFlip(currentFrame, NULL, 0);  //��ֱ��ת  
//											//��ʾͼ��
//			cvNamedWindow("camera", 1);
//			cvResizeWindow("camera", 1024, 540);
//			cvNamedWindow("moving area", 1);
//			cvResizeWindow("moving area", 1024, 540);
//			cvShowImage("camera", tempFrame);
//			cvShowImage("moving area", currentFrame);
//		}
//		//�ѵ�ǰ֡������Ϊ��һ�δ����ǰһ֡  
//		cvCvtColor(tempFrame, previousFrame, CV_BGR2GRAY);
//		cvWaitKey(1);
//
//	}//end while  
//
//	 //�ͷ���Դ  
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