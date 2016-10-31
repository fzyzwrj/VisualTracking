#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common.h"
#include "kcftracker.hpp"
#include "KalmanFilter.h"
#include "TreeDetect.h"
#include "LineDetect.h"
#include "CMap.h"
static float f = 729.0f;



template<typename T>
float calcAngle(const cv::Point_<T> *pt)
{
	return 0.0f;
}


cv::Rect scaleRect(const cv::Rect &rect, float scale)
{
	cv::Point2f center(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f);
	cv::Rect rectScaled = rect;

	rectScaled.width *= scale;
	rectScaled.height *= scale;

	rectScaled.x = center.x - rectScaled.width / 2.0f;
	rectScaled.y = center.y - rectScaled.height / 2.0f;
	return rectScaled;
}

// times���ܹ�Ҫѵ���Ĵ���
void updateKCF(KCFTracker &tracker, int times, const cv::Mat &oriFrameImg, float angle, const cv::Rect &searchROIRect, const cv::Rect &reserveROIRect)
{
	times = 20;
	cv::Mat frame = oriFrameImg.clone();
	assert(angle >= 0.0f);
	float anglePerTrain = angle / times;	// ÿһ֡��Ҫ���ٽǶȵ�
	anglePerTrain = anglePerTrain * 180 / CV_PI;
	//cv::imshow("NO", oriFrameImg(searchROIRect));
	cv::Rect searchROIRectScaled = scaleRect(searchROIRect, 2);
	//cv::imshow("NO2", oriFrameImg(searchROIRectScaled));
	//cv::waitKey(0);

	//cv::Point2f center(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);
	cv::Point2f center(searchROIRectScaled.width / 2.0f, searchROIRectScaled.height / 2.0f);

	cv::Mat tempImg = oriFrameImg.clone();


	//tracker.init(reserveROIRect, tempImg);



	for (int i = 0; i < times; ++i) {
		//std::cout << i << std::endl;
		//tracker.updateTrain(frame);
		// getRotationMatrix2D�Ĳ����У��Ƕ�Ҫ�Ƕ����������ǻ��ȵģ���
		// ע��rotateRect�ĽǶȣ�Ӧ��Ҳ�Ƿ��ػ��ȵģ�
		cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, -((i + 1) * anglePerTrain), 1.0); // Ҫȡ������ĽǶȣ�����ʱ����ת
		//std::cout << rotMatPerFrame << std::endl;

		cv::Mat ROIImg = frame(searchROIRectScaled & cv::Rect(0, 0, frame.cols, frame.rows)).clone();
		//SHOW(ROIImg);
		cv::Mat rotImg;
		cv::warpAffine(ROIImg, rotImg, rotMatPerFrame, ROIImg.size());
		//SHOW(rotImg);
		cv::Point2f newCenter(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
		cv::Mat ROIResImg;
		cv::getRectSubPix(rotImg, searchROIRect.size(), newCenter, ROIResImg);
		SHOW(ROIResImg);
		cv::Mat ROIOriImg = tempImg(searchROIRect);
		assert(ROIOriImg.size() == ROIResImg.size());
		//ROIOriImg = ROIResImg.clone();
		cv::addWeighted(ROIOriImg, 0, ROIResImg, 1, 0, ROIOriImg);
		/*ROIResImg.copyTo()*/
		//SHOW(ROIOriImg);
		//SHOW(frame);
		cv::imshow("FUNC", tempImg);
		//std::cout << tempImg.size << std::endl;

		cv::waitKey(0);
		//tracker.setROI(center.x, center.y, frame);
		tracker.setROI(reserveROIRect.x, reserveROIRect.y, tempImg);
		float peak_value = 0.0f;
		tracker.updateWithoutTrain(tempImg, peak_value);
		std::cout << "PEAK " << peak_value << std::endl;
		tracker.updateTrain(tempImg);

		if (i == times - 1) {
			tracker.init(reserveROIRect, tempImg);
		}


		//float angle = calcAngle(V, cv::Point2f(0, 0), offsetInFrame);	// ����ĽǶȼ��㿼�ǵ���ǰת��
		//angle = fabs(angle);

		//float angle = calcAngle(offsetInFrame);
		//float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// ��Ҫ����֡���ɾ���ֱ�ӳ����ٶ�
		//float anglePerFrame = angle / rotateFramesNum;	// ÿһ֡��Ҫ���ٽǶȵ�

														//cv::Point2f center(lastRect.width / 2.0f, lastRect.height / 2.0f);
		//center = cv::Point2f(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);

		//// Ϊ���ڲ�����������������ֱ�ӽ�searchROI����Ϊ1.41��


		////cv::Rect searchROIRectScaled = scaleRect(searchROIRect, sqrt(2.0f));
		//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // Ҫȡ������ĽǶȣ�����ʱ����ת
		//cv::Mat ROIImg = frame(searchROIRectScaled & cv::Rect(0, 0, frame.cols, frame.rows));
		//cv::Mat rotImg;
		//cv::warpAffine(ROIImg, rotImg, rotMatPerFrame, ROIImg.size());
		//center = cv::Point2f(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
		//cv::Mat resImg;
		//cv::getRectSubPix(rotImg, searchROIRect.size(), center, resImg);
		//std::cout << angle << std::endl;
		//std::cout << anglePerFrame << std::endl;
		//cv::imshow("RES", resImg);
		//cv::waitKey(0);
	}

}






// !!! ������������Ƶ�common.h�У���֪�������д���
// ������Χ4�������ɫ�������һ�������ذ�ɫ
static cv::Scalar roundingScalar(const cv::Mat &img, const cv::Point &pt)
{
	const cv::Scalar left(img.at<cv::Vec3b>(pt.y, pt.x - 1)[0], img.at<cv::Vec3b>(pt.y, pt.x - 1)[1], img.at<cv::Vec3b>(pt.y, pt.x - 1)[2]);
	const cv::Scalar right(img.at<cv::Vec3b>(pt.y, pt.x + 1)[0], img.at<cv::Vec3b>(pt.y, pt.x + 1)[1], img.at<cv::Vec3b>(pt.y, pt.x + 1)[2]);
	const cv::Scalar top(img.at<cv::Vec3b>(pt.y - 1, pt.x)[0], img.at<cv::Vec3b>(pt.y - 1, pt.x)[1], img.at<cv::Vec3b>(pt.y - 1, pt.x)[2]);
	const cv::Scalar bottom(img.at<cv::Vec3b>(pt.y + 1, pt.x)[0], img.at<cv::Vec3b>(pt.y + 1, pt.x)[1], img.at<cv::Vec3b>(pt.y + 1, pt.x)[2]);
	if ((left == right) && (left == top) && (left == bottom))
		return left;
	else
		return WHITE;
}

// ��ʵ�ʾ��뻻������˻������ͼƬ�е����ص���룬�����f�Ǹ���4k�������
cv::Vec2f convertDistIntoFrame(const cv::Vec2f &realVec, float high, float f)
{
	cv::Vec2f inFrameVec;
	float x = realVec[0];
	float y = realVec[1];
	float xx = x * f / high;
	inFrameVec[0] = realVec[0] * f / high;// ת���ɺ���
	inFrameVec[1] = realVec[1] * f / high;

	inFrameVec[0] = inFrameVec[0] / 10 / 2.54 * 96;
	inFrameVec[1] = inFrameVec[1] / 10 / 2.54 * 96;
	return inFrameVec;
}

static void initKCF()
{

}
static bool checkOcclused(const cv::Point2f &center, const std::vector<cv::Vec4i> &lines, float distThreshold)
{
	int occlusedTotal = 0;
	for (const auto &l : lines) {
		cv::Point pt1(l[0], l[1]);
		cv::Point pt2(l[2], l[3]);

		float dist = calcDist(center, pt1, pt2);
		if (dist < distThreshold)
			++occlusedTotal;
	}
	return occlusedTotal > lines.size() * 0.67;
}


// ��ʼĿ��ѡ��
static cv::Rect initRect;
// ��Ƶ֡
static cv::Mat frame;
// ��ʼ�����
static bool readyInitKCF = true;

static const std::string frameWinName = "SHOW";

static void onMouse(int event, int x, int y, int flag, void *)
{
	static cv::Point prePoint;
	static cv::Point curPoint;
	static cv::Mat imgTmp;
	// ��������
	if (event == CV_EVENT_LBUTTONDOWN) {
		imgTmp = frame.clone();
		prePoint = cv::Point(x, y);
		readyInitKCF = true;
	}	// ���������²��ƶ�
	else if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_FLAG_LBUTTON)) {
		imgTmp = frame.clone();	// ÿ���ƶ���Ҫ����ԭͼ���ڻ�ͼ
		curPoint = cv::Point(x, y);
		cv::rectangle(imgTmp, prePoint, curPoint, RED, 1, 8);
		cv::imshow(frameWinName, imgTmp);
	}	// �ɿ������ѡ�����
	else if (event == CV_EVENT_LBUTTONUP) {
		initRect = cv::Rect(prePoint, curPoint);
		readyInitKCF = false;
		if (initRect.width < 2 || initRect.height < 2)
			readyInitKCF = true;
	}
	else if (event == CV_EVENT_LBUTTONDBLCLK) {
		std::cout << "Closed." << std::endl;
		exit(0);
	}
}

int main(int argc, char *argv[])
{
	//freopen("res.txt", "w", stdout);
	const std::string videoFilename = "G:\\resources\\videos\\DJI_0002.MOV";
	const std::string mapFilename = "G:\\MapTileDownload\\OutPut\\�ȸ��ͼ_161020230531_L19\\�ȸ��ͼ_161020230531.png";
	const std::string mapMarkedFilename = "G:\\MapTileDownload\\OutPut\\�ȸ��ͼ_161020230531_L19\\mark.png";
	const std::string coordinateFilename = "G:\\MapTileDownload\\OutPut\\�ȸ��ͼ_161020230531_L19\\�ȸ��ͼ_161020230531.txt";
	const std::string SRTFilename = "G:\\resources\\videos\\DJI_0002.SRT";

	// ��ʼ����Ƶ
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	// ��ʼ��һ֡λ��
	int frameIndex = 1400;
	cap.set(CV_CAP_PROP_POS_FRAMES, frameIndex - 1);
	cv::namedWindow(frameWinName);
	cv::moveWindow(frameWinName, 0, 0);
	cv::setMouseCallback(frameWinName, onMouse);

	// ��ʼ��KCF
	bool HOG = true;
	bool FIXEDWINDOW = true;
	bool MULTISCALE = false;
	bool LAB = false;
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	cv::Rect resRect;	// ��ǰ֡�ĸ��ٽ��
	cv::Rect lastRect;	 // ��һ֡�ĸ��ٽ��
	cv::Rect searchROIRect;	 // ��ǰ֡��������һ����Ŀ��� x 2.5
	float dx = 0.0f;
	float dy = 0.0f;
	cv::Rect reserveRect;	// ����������ת�ĸ��ٽ����һ���ǵ�ǰ֡��ǰ3֡���ң������ڵ�ʱKCF�ĸ��£�ֱ�ӽ�Ŀ����ת��
	bool redetect = false;
	cv::Mat reserveFrame;
	cv::Rect reserveROIRect;
	cv::Rect reserveSearchROIRect;
	cv::Rect redetectRect;

	// ��ʼ��Klaman Filter
	CKalmanFilter KF;
	float dxInitKF = 0.0f, dyInitKF = 0.0f;	// ��ʼ���ٶ�
	const int framesCalcForVelocityTotal = 5;	// ʹ�ö���֡����ʼ���ٶȲ������ﵽ���ǳ�ʼ����ˣ�����ʼ���������˲���
	int frameIndexForInitKF = 0;	// ��һ֡���ڳ�ʼ��KF���ٶȵģ���framesCalcForVelocityTotal��ͬʹ��
	bool KFInited = false;

	// ��ʼ����ͼ
	CMap m(mapFilename, coordinateFilename, mapMarkedFilename);
	std::vector<cv::Point2d> vecGPS;
	std::vector<double> vecHigh;
	parseGPSAndHighFromSRT(SRTFilename, vecGPS, vecHigh);
	cv::Mat mapROI;
	cv::Mat mapMarkedROI;
	bool inRedArea = false;
	bool inBlueArea = false;
	bool inGreenArea = false;
	bool inBlackArea = false;

	bool inRedetect = false;

	for (;;) {
		++frameIndex;
		cap >> frame;
		// ������Ƶ�ֱ��ʹ�����Ҫ�ʵ���С
		TEST_TIME(cv::resize(frame, frame, cv::Size(2048, 1080)));

		// �Ƿ����¸���Ŀ��
		if (frameIndex == 0 || readyInitKCF) {
			cv::imshow(frameWinName, frame);
			cv::waitKey(0);
			tracker.init(initRect, frame);
			reserveFrame = frame.clone();
			cv::rectangle(frame, initRect, RED, 1, 8);
			lastRect = initRect;

			// ��ʼ���ٶ�
			frameIndexForInitKF = frameIndex;
			dxInitKF = 0.0f;
			dyInitKF = 0.0f;
			KFInited = false;

			reserveROIRect = initRect;
			reserveSearchROIRect = tracker._extracted_roi;
		}
		else {
			float peak_value = 0.0f;
			TEST_TIME(resRect = tracker.updateWithoutTrain(frame, peak_value));	// �����٣���ѵ��

			if (redetect) {
				resRect = redetectRect;
				tracker.setROI(resRect.x, resRect.y, frame);
			}
			std::cout << "PEAK VALUE: " << peak_value << std::endl;
			//std::cout << "BEGIN " << resRect << std::endl;

			searchROIRect = tracker._extracted_roi;			// ��һ֡�ĸ��ٿ�

			// ��ľ��⣬��ԭͼ�б��
			cv::Mat searchROIImg = frame(searchROIRect & cv::Rect(0, 0, frame.cols, frame.rows));	// ���ܳ���ԭͼ
			cv::Mat colorFilteredImg;
			TEST_TIME(colorFilter(searchROIImg, colorFilteredImg));	// ʱ����ݸ��ٳߴ������10~60ms��releaseģʽ�£�����5ms
			cv::Mat colorFilteredBGRImg;
			cv::cvtColor(colorFilteredImg, colorFilteredBGRImg, CV_GRAY2BGR);
			// ������ԭͼ��
			//TEST_TIME(add(searchROIImg, colorFilteredBGRImg, searchROIImg));


			// ֱ�߼��
			cv::Mat lineDetectedImg;
			std::vector<cv::Vec4i> lines;
			TEST_TIME(lineDetect(searchROIImg, lines));	// ʱ����ݸ��ٳߴ������10~60ms��releaseģʽ�£�10ms����
			drawLines(searchROIImg, lines);
			//SHOW(lineDetectedImg);

			
			// ��ȡ���˶�����ƽ�л��ߴ�ֱ���߶�
			std::vector<cv::Vec4i> linesParallel;
			std::vector<cv::Vec4i> linesVertical;
			cv::Point2f V(dx, dy);
			for (const auto &l : lines) {
				cv::Point pt1(l[0], l[1]);
				cv::Point pt2(l[2], l[3]);
				float angle = calcAngle(V, pt1, pt2);
				float len = calcDist(pt1, pt2);
				angle = fabs(angle);
				angle *= 180 / CV_PI;
				const float threshold = 0.67f;
				if (angle < 10 && len > threshold * searchROIRect.width)
					linesParallel.push_back(l);
				else if (angle > 80 && len > threshold * searchROIRect.height)
					linesVertical.push_back(l);
				else
					;
			}

			// ����ֱ�߼������ж��ڵ�
			cv::Point2f center(searchROIImg.cols / 2.0f, searchROIImg.rows / 2.0f);
			bool parallelOcclused = checkOcclused(center, linesParallel, initRect.width * 0.5f);	// ��Ϊ���ٿ�һ���Ƚϴ��۰���ټ�СһЩ
			bool verticalOcclused = checkOcclused(center, linesVertical, initRect.height * 0.5f);
			char buf[256];
			
			if (parallelOcclused || verticalOcclused)	// ����Ļ��ӡ�ڵ���Ϣ
				sprintf_s(buf, "Total: %zd, P: %zd, V: %zd, Res: %d, %d", lines.size(), linesParallel.size(), linesVertical.size(), parallelOcclused, verticalOcclused);
			cv::putText(frame, buf, cv::Point(30, 30), cv::FONT_HERSHEY_DUPLEX, 1.0, RED);

			cv::Point2f KFPt;	// Kalman FilterԤ����Ľ��
			// �ж��Ƿ��ڵ���ͬʱ�����Ƿ����KCF
			// ���ڵ�
			std::cout << "redect " << redetect << std::endl;


			if (redetect == true && peak_value >= 0.3f) {
				inRedetect = true;
				redetect = false;
				KFInited = true;
				//resRect = tracker.update(frame);
				float peak_value_redect = 0.0f;

				resRect = tracker.updateWithoutTrain(frame, peak_value_redect);
				std::cout << "resRect " << resRect << std::endl;
				tracker.updateTrain(frame);
				std::cout << peak_value_redect << std::endl;
				parallelOcclused = false;
				verticalOcclused = false;
			}
			if (inRedetect) {
				parallelOcclused = false;
				verticalOcclused = false;
			}


			if (redetect)
				peak_value = 0.30f;

			if (peak_value >= 0.30f) {
				dx = resRect.x - lastRect.x;
				dy = resRect.y - lastRect.y;


				// ��ȫ���ڵ�������KCF
				if (peak_value >= 0.45f) {
					TEST_TIME(tracker.updateTrain(frame));
					redetect = false;
					KFInited = true;
				}
				else {
					// û�б���ȫ�ڵ�����Ϊ��ֹģ��Ư�ƣ�������KCF������ɶ���ת�����ʧЧ
				}

				if (KFInited) {
					cv::Mat resMat;

					KF.predictAndCorrect(lastRect.x * 1.0f, lastRect.y * 1.0f, dx, dy, resMat);
					KFPt.x = resMat.at<float>(0);
					KFPt.y = resMat.at<float>(1);
					//std::cout << "dx, dy " << resMat.at<float>(2) << " " << resMat.at<float>(3) << std::endl;
					//std::cout << "True dx, dy " << dx << " " << dy << std::endl;
				}

			}
			// ��ȫ���ڵ���ʹ��Kalman Filter����Ԥ��
			else {
				// �ж����ڵ�ֱ�ӵľ����С�ķ�������ʻ�ĽǶȵĹ�ϵ�����


				if (parallelOcclused || verticalOcclused) {
					redetect = true;

					KFInited = false;
					std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
					if (inRedArea) {
						std::cout << "RED" << std::endl;
						// ��ָ��������м��
						// �õ�����ʾƫ�ƣ����Ըĳ��ļ�����ʽ����¼���ڵ�ͼ����������ص�ĸ���
						// ����ƫ��
						cv::Point2f offset(15, -16);
						cv::Point2f offsetInFrame = convertDistIntoFrame(offset, vecHigh[frameIndex / 25], f);
						//std::cout << offsetInFrame << std::endl;

						// ���ڹ�һ������Ϊԭͼ����С��2��
						offsetInFrame.x /= 2.0f;
						offsetInFrame.y /= 2.0f;

						// ��������º��λ�ã���Ԥ���λ��
						resRect.x += offsetInFrame.x;
						resRect.y += offsetInFrame.y;
						redetectRect = resRect;

						float angle = calcAngle(V, cv::Point2f(0, 0), offsetInFrame);	// ����ĽǶȼ��㿼�ǵ���ǰת��
						angle = fabs(angle);

						//float angle = calcAngle(offsetInFrame);
						float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// ��Ҫ����֡���ɾ���ֱ�ӳ����ٶ�
						float anglePerFrame = angle / rotateFramesNum;	// ÿһ֡��Ҫ���ٽǶȵ�

																		// ����������KCF
						updateKCF(tracker, rotateFramesNum, reserveFrame, angle, reserveSearchROIRect, reserveROIRect);
						tracker.setROI(resRect.x, resRect.y, frame);

						//cv::Point2f center(lastRect.width / 2.0f, lastRect.height / 2.0f);
						//center = cv::Point2f(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);
						//
						//// Ϊ���ڲ�����������������ֱ�ӽ�searchROI����Ϊ1.41��

						//
						//cv::Rect searchROIRectScaled = scaleRect(searchROIRect, sqrt(2.0f));
						//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // Ҫȡ������ĽǶȣ�����ʱ����ת
						//cv::Mat ROIImg = frame(searchROIRectScaled & cv::Rect(0, 0, frame.cols, frame.rows));
						//cv::Mat rotImg;
						//cv::warpAffine(ROIImg, rotImg, rotMatPerFrame, ROIImg.size());
						//center = cv::Point2f(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
						//cv::Mat resImg;
						//cv::getRectSubPix(rotImg, searchROIRect.size(), center, resImg);
						//std::cout << angle << std::endl;
						//std::cout << anglePerFrame << std::endl;
						//cv::imshow("RES", resImg);
						//cv::waitKey(0);


						//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // Ҫȡ������ĽǶȣ�����ʱ����ת

						//// �����frameӦ�ñ�����
						//cv::Mat ROIImg = frame(searchROIRect & cv::Rect(0, 0, frame.cols, frame.rows)).clone();

						//cv::Mat rotImg;	// trainImg
						//cv::warpAffine(ROIImg, rotImg, rotMatPerFrame, rotMatPerFrame.size());
						//center = cv::Point2f(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
						//cv::getRectSubPix(rotImg, initRect.size, center, initRect.size);
						////tracker.train(rotImg);



					}
					else if (inBlueArea)
						std::cout << "BLUE" << std::endl;
					else if (inBlackArea)
						std::cout << "BLACK" << std::endl;
					else if (inGreenArea)
						std::cout << "GREEN" << std::endl;
					else
						std::cout << "NOOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;
				}

				// �����ڵ��󣬶�Kalman FilterҪ���³�ʼ��
				if (KFInited) {
					cv::Mat predictMat;
					KF.predict(predictMat);
					KFPt = cv::Point2f(predictMat.at<float>(0), predictMat.at<float>(1));
					resRect = cv::Rect(KFPt.x, KFPt.y, initRect.width, initRect.height);

					////////////////////////////////////////////////////////////////////////// ����ֱ���ж�
					dx = resRect.x - lastRect.x;
					dy = resRect.y - lastRect.y;
					//std::cout << "dx, dy " << predictMat.at<float>(2) << " " << predictMat.at<float>(3) << std::endl;
					//std::cout << "True dx, dy " << dx << " " << dy << std::endl;
					//for (size_t i = 0; i < lines.size(); ++i) {

					//////////////////////////////////////////////////////////////////////////
				}

				tracker.setROI(resRect.x, resRect.y, frame);

			}

			// std::cout << KFPt << " " << resRect.tl() << " " << calcDist(KFPt, resRect.tl()) << std::endl;

			if (frameIndex - frameIndexForInitKF < 5) {
				dxInitKF += resRect.x - lastRect.x;
				dyInitKF += resRect.y - lastRect.y;
			}
			else if (frameIndex - frameIndexForInitKF == 5) {
				dxInitKF /= 5;
				dyInitKF /= 5;
				KF.init(resRect.x, resRect.y, dxInitKF, dyInitKF);
				KFInited = true;
			}
			reserveRect = lastRect;
			lastRect = resRect;


			//////////////////////////////////////////////////////////////////////////
			int secondIndex = (frameIndex / 25 + 1);
			if (frameIndex % 25 == 0) {
				m.calcFrame(vecGPS[secondIndex - 1], cv::Mat(cv::Size(4096, 2160), 0), vecHigh[secondIndex - 1], 729, 117);
				//m.drawMapROI(mapROI);
				m.getMapROI(mapROI, mapMarkedROI);


			}
			if (mapROI.data) {
				cv::Point2f targerPt(lastRect.x + lastRect.width / 2, lastRect.y + lastRect.height / 2);
				targerPt.x /= (frame.cols * 1.0f / mapROI.cols);
				targerPt.y /= (frame.rows * 1.0f / mapROI.rows);
				DRAW_CROSS(mapROI, targerPt, RED, 1);
				SHOW(mapROI);
				SHOW(mapMarkedROI);

				const cv::Scalar &color = roundingScalar(mapMarkedROI, targerPt);
				if (color == RED)
					inRedArea = true;
				else
					inRedArea = false;

				if (color == GREEN)
					inGreenArea = true;
				else
					inGreenArea = false;

				if (color == BLUE)
					inBlueArea = true;
				else
					inBlueArea = false;

				if (color == BLACK)
					inBlackArea = true;
				else
					inBlackArea = false;


			}

			//////////////////////////////////////////////////////////////////////////



			//// ����������
			//float angle = (28 + 90) / CV_PI;
			//const float height = 100.0f;	// ���и߶�
			//cv::Point2d center(frame.cols / 2, frame.rows / 2);
			//cv::Point resPt(resRect.x + resRect.width / 2, resRect.y + resRect.height / 2);
			//cv::Point2d ptInFrame;
			//// ��ͼƬ��С����һ��
			//ptInFrame.x = (resPt.x - center.x) / frame.cols;
			//ptInFrame.y = (resPt.y - center.y) / frame.rows;
			//cv::Point2d ptInMap;
			

			//std::cout << "END " << resRect << std::endl;
			cv::rectangle(frame, resRect, GREEN);
			cv::rectangle(frame, searchROIRect, PINK);

			if (KFInited)
				cv::rectangle(frame, cv::Rect(KFPt.x, KFPt.y, resRect.width, resRect.height), YELLOW);
		}
		cv::imshow(frameWinName, frame);
		char ch = cv::waitKey(10);
		if (toupper(ch) == 'Q')
			break;
		else if (toupper(ch) == 'P')
			cv::waitKey(0);
	}

	return 0;
}