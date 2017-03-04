#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils.h"
#include "utils_opencv.h"

#include "kcftracker.hpp"
#include "KalmanFilter.h"
#include "TreeDetect.h"
#include "LineDetect.h"
#include "CMap.h"
#include "linear.h"
#include "MotionTargetDetect.h"

// 无人机飞行高度比例尺因子，在从视频到地图的GPS时，需要用到
const float f = 729.0f;


// 根据LibLinear直接创建图像的灰度特征
void makeFeature(const cv::Mat &img, feature_node *x_space)
{
	cv::Mat tmpImg;
	cv::resize(img, tmpImg, cv::Size(200, 200));
	tmpImg = img.reshape(1, 1);	// reshape成一维
	cv::Mat floatImg;	// 转换成float类型的mat
	tmpImg.convertTo(floatImg, CV_32F);
	const int DIM_FEA = floatImg.cols;	// 特征维度
	const float *xData = floatImg.ptr<float>(0);
	int i = 0;
	for (i = 0; i < DIM_FEA; ++i) {
		x_space[i].index = i + 1;
		x_space[i].value = xData[i];
	}
	// 填充结尾标记
	x_space[i].index = DIM_FEA + 1;
	x_space[i].value = 1;
	x_space[i].index = -1;
}

// 根据缩放比例scale缩放矩形框
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

// times是总共要训练的次数
void updateKCF(KCFTracker &tracker, int times, const cv::Mat &oriFrameImg, float angle, const cv::Rect &searchROIRect, const cv::Rect &reserveROIRect)
{
	times = 20;
	cv::Mat frame = oriFrameImg.clone();
	assert(angle >= 0.0f);
	float anglePerTrain = angle / times;	// 每一帧需要多少角度的
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
		if (i != times - 1)
			continue;

		//std::cout << i << std::endl;
		//tracker.updateTrain(frame);
		// getRotationMatrix2D的参数中，角度要是度数，而不是弧度的！！
		// 注意rotateRect的角度，应该也是返回弧度的！
		cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, -((i + 1) * anglePerTrain), 1.0); // 要取反方向的角度，即逆时针旋转
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
		std::string filename = std::to_string(i) + "_rotate.jpg";
		cv::imwrite(filename, tempImg);
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


		//float angle = calcAngle(V, cv::Point2f(0, 0), offsetInFrame);	// 这里的角度计算考虑到提前转弯
		//angle = fabs(angle);

		//float angle = calcAngle(offsetInFrame);
		//float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// 需要多少帧，由距离直接除以速度
		//float anglePerFrame = angle / rotateFramesNum;	// 每一帧需要多少角度的

														//cv::Point2f center(preRect.width / 2.0f, preRect.height / 2.0f);
		//center = cv::Point2f(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);

		//// 为便于操作与性能上提升，直接将searchROI扩大为1.41倍


		////cv::Rect searchROIRectScaled = scaleRect(searchROIRect, sqrt(2.0f));
		//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // 要取反方向的角度，即逆时针旋转
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


// !!! 这个函数不能移到common.h中，不知道哪里有错误
// 返回周围4个点的颜色，如果不一样，返回白色
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

// 把实际距离换算成无人机拍摄的图片中的像素点距离，这里的f是根据4k来计算的
cv::Vec2f convertDistIntoFrame(const cv::Vec2f &realVec, float high, float f)
{
	cv::Vec2f inFrameVec;
	float x = realVec[0];
	float y = realVec[1];
	float xx = x * f / high;
	inFrameVec[0] = realVec[0] * f / high;// 转换成毫米
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

		double dist = calcDist(center, pt1, pt2);
		if (dist < distThreshold)
			++occlusedTotal;
	}
	return occlusedTotal > lines.size() * 0.67;
}


// 初始目标选框
static cv::Rect initRect;
// 视频帧
static cv::Mat frame;
// 初始化标记，是否准备开始初始化KCF，如果true，则将进入onMouse选定厨师跟踪目标
static bool readyInitKCF = true;

static const std::string frameWinName = "SHOW";

static void onMouse(int event, int x, int y, int flag, void *)
{
	static cv::Point prePoint;
	static cv::Point curPoint;
	static cv::Mat imgTmp;
	// 鼠标左键下
	if (event == CV_EVENT_LBUTTONDOWN) {
		imgTmp = frame.clone();
		prePoint = cv::Point(x, y);
		readyInitKCF = true;
	}	// 鼠标左键按下并移动
	else if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_FLAG_LBUTTON)) {
		imgTmp = frame.clone();	// 每次移动都要拷贝原图用于画图
		curPoint = cv::Point(x, y);
		cv::rectangle(imgTmp, prePoint, curPoint, RED, 1, 8);
		cv::imshow(frameWinName, imgTmp);
	}	// 松开左键，选框完毕
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
	CStr videoFilename = "G:\\resources\\videos\\DJI_0002.MOV";
	CStr mapFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.png";
	CStr mapMarkedFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\mark.png";
	CStr coordinateFilename = "G:\\MapTileDownload\\OutPut\\谷歌地图_161020230531_L19\\谷歌地图_161020230531.txt";
	CStr SRTFilename = "G:\\resources\\videos\\DJI_0002.SRT";

	// 初始化视频
	cv::VideoCapture cap(videoFilename);
	assert(cap.isOpened());
	// 初始第一帧位置
	int frameIndex = 1400;
	cap.set(CV_CAP_PROP_POS_FRAMES, frameIndex - 1);
	cv::namedWindow(frameWinName);
	cv::moveWindow(frameWinName, 0, 0);
	cv::setMouseCallback(frameWinName, onMouse);

	// 初始化KCF
	KCFTracker tracker(true, true, true, false);

	cv::Rect curRect;		// 当前帧的跟踪结果
	cv::Rect preRect;		// 上一帧的跟踪结果
	cv::Rect maxResRect;
	cv::Rect searchROIRect;	// 当前帧的搜索框，一般是目标框 x 2.5
	float dx = 0.0f;
	float dy = 0.0f;
	cv::Rect reserveRect;	// 保存用于旋转的跟踪结果，一般是当前帧的前3帧左右，用于遮挡时KCF的更新（直接将目标旋转）
	bool redetect = false;
	cv::Mat reserveFrame;
	cv::Rect reserveROIRect;
	cv::Rect reserveSearchROIRect;
	cv::Rect redetectRect;
	cv::Mat lastFrame;	// 保存上一帧，用于运动检测

	// 初始化Klaman Filter
	CKalmanFilter KF;
	float dxInitKF = 0.0f, dyInitKF = 0.0f;	// 初始化速度
	const int framesCalcForVelocityTotal = 5;	// 使用多少帧来初始化速度参数，达到后标记初始标记了，随后初始化卡尔曼滤波器
	int frameIndexForInitKF = 0;	// 第一帧用于初始化KF的速度的，与framesCalcForVelocityTotal共同使用
	bool KFInited = false;

	bool noKF = false;

	// 初始化地图
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

	//cv::VideoWriter writer("res.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0f, cv::Size(2048, 1080));
	//assert(writer.isOpened());

	CStr svmPModelFilename("4000.dat");
	CStr svmNModelFilename("5000.dat");

	struct model *svmPModel = load_model(svmPModelFilename.c_str());
	struct model *svmNModel = load_model(svmNModelFilename.c_str());
	const int DIM_FEA = 200 * 200;
	feature_node *x_space = new feature_node[DIM_FEA + 2];


	for (;;) {
		++frameIndex;
		//lastFrame = frame;
		frame.copyTo(lastFrame);
		cap >> frame;
		// 由于视频分辨率过大，需要适当缩小
		TIME(cv::resize(frame, frame, cv::Size(2048, 1080)));

		// 是否重新跟踪目标
		if (frameIndex == 1 || readyInitKCF) {
			cv::imshow(frameWinName, frame);
			cv::waitKey(0);

			tracker.init(initRect, frame);
			reserveFrame = frame.clone();
			cv::rectangle(frame, initRect, RED, 1, 8);
			preRect = initRect;

			// 初始化速度
			frameIndexForInitKF = frameIndex;
			dxInitKF = 0.0f;
			dyInitKF = 0.0f;
			KFInited = false;
			inRedetect = false;
			KFInited = false;
			noKF = false;

			reserveROIRect = initRect;
			reserveSearchROIRect = tracker._extracted_roi;


			//updateKCF(tracker, 180, reserveFrame, CV_PI, reserveSearchROIRect, reserveROIRect);
		}
		else {
			float peak_value = 0.0f;
			TIME(curRect = tracker.updateWithoutTrain(frame, peak_value));	// 仅跟踪，不训练
			makeFeature(frame(curRect & cv::Rect(0, 0, frame.cols, frame.rows)).clone(), x_space);
			double labelP = predict(svmPModel, x_space);
			double labelN = predict(svmNModel, x_space);
			std::cout << labelP << " " << labelN << std::endl;
			if (redetect && !noKF) {
				curRect = redetectRect;
				tracker.setROI(curRect.x, curRect.y, frame);

				float max_peak_value = 0.0f;
				cv::Rect maxcurRect;
				for (int r = -10; r <= 10; ++r) {
					for (int c = 0; c < (10 - abs(r)) / 3; ++c) {
						tracker.setROI(curRect.x + 2 * c, curRect.y + 2 * r, frame);
						curRect = tracker.updateWithoutTrain(frame, peak_value);
						if (peak_value > max_peak_value) {
							max_peak_value = peak_value;
							maxResRect = curRect;
						}
					}
				}
				cv::Rect detectROIRect(curRect.x - curRect.width * 2, curRect.y - curRect.height * 2, curRect.width * 4, curRect.height * 4);
				cv::Mat curDetectFrame, lastDetectFrame;
				curDetectFrame = frame(detectROIRect).clone();
				lastDetectFrame = lastFrame(detectROIRect).clone();
				cv::Mat diffImg;
				frameDiff(curDetectFrame, lastDetectFrame, diffImg);
				SHOW(curDetectFrame); 
				SHOW(diffImg);

				//for (int i = -10; i < 10; ++i) {
				//	tracker.setROI(curRect.x, curRect.y + 2 * i, frame);
				//	curRect = tracker.updateWithoutTrain(frame, peak_value);
				//	if (peak_value > max_peak_value) {
				//		max_peak_value = peak_value;
				//		maxResRect = curRect;
				//	}
				//}
				std::cout << "SEARCH ROI PEAK_VALUE " << peak_value << std::endl;
				curRect = maxResRect;
				peak_value = max_peak_value;

			}
			std::cout << "PEAK VALUE: " << peak_value << std::endl;
			if (noKF)
				peak_value = 0.5;
			//std::cout << "BEGIN " << curRect << std::endl;

			searchROIRect = tracker._extracted_roi;			// 上一帧的跟踪框

			// 树木检测，在原图中标记
			cv::Mat searchROIImg = frame(searchROIRect & cv::Rect(0, 0, frame.cols, frame.rows));	// 不能超出原图
			cv::Mat colorFilteredImg;
			TIME(colorFilter(searchROIImg, colorFilteredImg));	// 时间根据跟踪尺寸决定，10~60ms，release模式下，不到5ms
			cv::Mat colorFilteredBGRImg;
			cv::cvtColor(colorFilteredImg, colorFilteredBGRImg, CV_GRAY2BGR);
			// 叠加在原图中
			TIME(add(searchROIImg, colorFilteredBGRImg, searchROIImg));


			// 直线检测
			cv::Mat lineDetectedImg;
			std::vector<cv::Vec4i> lines;
			TIME(lineDetect(searchROIImg, lines));	// 时间根据跟踪尺寸决定，10~60ms，release模式下，10ms左右
			drawLines(searchROIImg, lines);
			//SHOW(lineDetectedImg);

			
			// 获取与运动方向平行或者垂直的线段
			std::vector<cv::Vec4i> linesParallel;
			std::vector<cv::Vec4i> linesVertical;
			cv::Point2f V(dx, dy);
			for (const auto &l : lines) {
				cv::Point pt1(l[0], l[1]);
				cv::Point pt2(l[2], l[3]);
				float angle = calcAngle(V, pt1 - pt2);
				float len = calcDist(pt1, pt2);
				angle = fabs(angle);
				angle *= 180 / CV_PI;
				const float threshold = 0.67f;
				if (angle < 45 && len > threshold * searchROIRect.width)
					linesParallel.push_back(l);
				else if (angle > 70 && len > threshold * searchROIRect.height)
					linesVertical.push_back(l);
				else
					;
			}

			// 根据直线检测情况判断遮挡
			cv::Point2f center(searchROIImg.cols / 2.0f, searchROIImg.rows / 2.0f);
			bool parallelOcclused = checkOcclused(center, linesParallel, initRect.width * 0.5f);	// 因为跟踪框一般会比较大，折半后再减小一些
			bool verticalOcclused = checkOcclused(center, linesVertical, initRect.height * 0.5f);
			char buf[256];
			
			if (parallelOcclused || verticalOcclused)	// 在屏幕打印遮挡信息
				sprintf_s(buf, "Total: %zd, P: %zd, V: %zd, Res: %d, %d", lines.size(), linesParallel.size(), linesVertical.size(), parallelOcclused, verticalOcclused);
			cv::putText(frame, buf, cv::Point(30, 30), cv::FONT_HERSHEY_DUPLEX, 1.0, RED);

			cv::Point2f KFPt;	// Kalman Filter预测出的结果
			// 判断是否遮挡，同时决定是否更新KCF
			// 无遮挡
			std::cout << "redect " << redetect << std::endl;


			if (redetect == true && peak_value >= 0.35f) {
				inRedetect = true; 
				noKF = true;
				redetect = false;
				KFInited = true;
				//curRect = tracker.update(frame);
				float peak_value_redect = 0.0f;

				curRect = tracker.updateWithoutTrain(frame, peak_value_redect);
				std::cout << "curRect " << curRect << std::endl;
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
				dx = curRect.x - preRect.x;
				dy = curRect.y - preRect.y;


				// 完全无遮挡，更新KCF
				if (peak_value >= 0.45f) {
					TIME(tracker.updateTrain(frame));
					redetect = false;
					KFInited = true;
				}
				else {
					// 没有被完全遮挡，但为防止模型漂移，不更新KCF，但造成对旋转不变等失效
				}

				if (KFInited) {
					cv::Mat resMat;

					KF.predictAndCorrect(preRect.x * 1.0f, preRect.y * 1.0f, dx, dy, resMat);
					KFPt.x = resMat.at<float>(0);
					KFPt.y = resMat.at<float>(1);
					//std::cout << "dx, dy " << resMat.at<float>(2) << " " << resMat.at<float>(3) << std::endl;
					//std::cout << "True dx, dy " << dx << " " << dy << std::endl;
				}

			}
			// 完全被遮挡，使用Kalman Filter进行预测
			else {
				// 判断离遮挡直接的距离减小的方向与行驶的角度的关系，如果


				//if (parallelOcclused || verticalOcclused) {
				if (parallelOcclused) {
					redetect = true;

					KFInited = false;
					std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
					if (inRedArea) {
						std::cout << "RED" << std::endl;
						// 在指定区域进行检测
						// 用点来表示偏移，可以改成文件的形式来记录，在地图中来标记像素点的个数
						// 计算偏移
						cv::Point2f offset(15, -16);
						cv::Point2f offsetInFrame = convertDistIntoFrame(offset, vecHigh[frameIndex / 25], f);
						//std::cout << offsetInFrame << std::endl;

						// 用于归一化，因为原图被缩小了2倍
						offsetInFrame.x /= 2.0f;
						offsetInFrame.y /= 2.0f;

						// 计算出更新后的位置，即预测的位置
						curRect.x += offsetInFrame.x;
						curRect.y += offsetInFrame.y;
						redetectRect = curRect;

						float angle = calcAngle(V, cv::Point2f(0, 0) - offsetInFrame);	// 这里的角度计算考虑到提前转弯
						angle = fabs(angle);

						//float angle = calcAngle(offsetInFrame);
						float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// 需要多少帧，由距离直接除以速度
						float anglePerFrame = angle / rotateFramesNum;	// 每一帧需要多少角度的

																		// 以下来更新KCF
						updateKCF(tracker, rotateFramesNum, reserveFrame, angle, reserveSearchROIRect, reserveROIRect);
						tracker.setROI(curRect.x, curRect.y, frame);

						//cv::Point2f center(preRect.width / 2.0f, preRect.height / 2.0f);
						//center = cv::Point2f(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);
						//
						//// 为便于操作与性能上提升，直接将searchROI扩大为1.41倍

						//
						//cv::Rect searchROIRectScaled = scaleRect(searchROIRect, sqrt(2.0f));
						//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // 要取反方向的角度，即逆时针旋转
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


						//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // 要取反方向的角度，即逆时针旋转

						//// 这里的frame应该保留的
						//cv::Mat ROIImg = frame(searchROIRect & cv::Rect(0, 0, frame.cols, frame.rows)).clone();

						//cv::Mat rotImg;	// trainImg
						//cv::warpAffine(ROIImg, rotImg, rotMatPerFrame, rotMatPerFrame.size());
						//center = cv::Point2f(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
						//cv::getRectSubPix(rotImg, initRect.size, center, initRect.size);
						////tracker.train(rotImg);



					}
					else if (inBlueArea) {
						std::cout << "BLUE" << std::endl;
						// 在指定区域进行检测
						// 用点来表示偏移，可以改成文件的形式来记录，在地图中来标记像素点的个数
						// 计算偏移
						cv::Point2f offset(-15, 13);
						cv::Point2f offsetInFrame = convertDistIntoFrame(offset, vecHigh[frameIndex / 25], f);
						//std::cout << offsetInFrame << std::endl;

						// 用于归一化，因为原图被缩小了2倍
						offsetInFrame.x /= 2.0f;
						offsetInFrame.y /= 2.0f;

						// 计算出更新后的位置，即预测的位置
						curRect.x += offsetInFrame.x;
						curRect.y += offsetInFrame.y;
						redetectRect = curRect;

						float angle = calcAngle(V, cv::Point2f(0, 0) - offsetInFrame);	// 这里的角度计算考虑到提前转弯
						angle = fabs(angle);

						//float angle = calcAngle(offsetInFrame);
						float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// 需要多少帧，由距离直接除以速度
						float anglePerFrame = angle / rotateFramesNum;	// 每一帧需要多少角度的

																		// 以下来更新KCF
						updateKCF(tracker, rotateFramesNum, reserveFrame, angle, reserveSearchROIRect, reserveROIRect);
						tracker.setROI(curRect.x, curRect.y, frame);
					}
					else if (inBlackArea)
						std::cout << "BLACK" << std::endl;
					else if (inGreenArea)
						std::cout << "GREEN" << std::endl;
					else
						std::cout << "NOOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;
				}

				// 发生遮挡后，对Kalman Filter要重新初始化
				if (KFInited) {
					cv::Mat predictMat;
					KF.predict(predictMat);
					KFPt = cv::Point2f(predictMat.at<float>(0), predictMat.at<float>(1));
					curRect = cv::Rect(KFPt.x, KFPt.y, initRect.width, initRect.height);

					////////////////////////////////////////////////////////////////////////// 增加直线判断
					dx = curRect.x - preRect.x;
					dy = curRect.y - preRect.y;
					//std::cout << "dx, dy " << predictMat.at<float>(2) << " " << predictMat.at<float>(3) << std::endl;
					//std::cout << "True dx, dy " << dx << " " << dy << std::endl;
					//for (size_t i = 0; i < lines.size(); ++i) {

					//////////////////////////////////////////////////////////////////////////
				}

				tracker.setROI(curRect.x, curRect.y, frame);

			}

			// std::cout << KFPt << " " << curRect.tl() << " " << calcDist(KFPt, curRect.tl()) << std::endl;

			if (frameIndex - frameIndexForInitKF < 5) {
				dxInitKF += curRect.x - preRect.x;
				dyInitKF += curRect.y - preRect.y;
			}
			else if (frameIndex - frameIndexForInitKF == 5) {
				dxInitKF /= 5;
				dyInitKF /= 5;
				KF.init(curRect.x, curRect.y, dxInitKF, dyInitKF);
				KFInited = true;
			}
			reserveRect = preRect;
			preRect = curRect;


			//////////////////////////////////////////////////////////////////////////
			int secondIndex = (frameIndex / 25 + 1);
			if (frameIndex % 25 == 0) {
				m.calcFrame(vecGPS[secondIndex - 1], cv::Mat(cv::Size(4096, 2160), 0), vecHigh[secondIndex - 1], 729, 117);
				//m.drawMapROI(mapROI);
				m.getMapROI(mapROI, mapMarkedROI);


			}
			if (mapROI.data) {
				cv::Point2f targerPt(preRect.x + preRect.width / 2, preRect.y + preRect.height / 2);
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



			//// 传感器给定
			//float angle = (28 + 90) / CV_PI;
			//const float height = 100.0f;	// 飞行高度
			//cv::Point2d center(frame.cols / 2, frame.rows / 2);
			//cv::Point resPt(curRect.x + curRect.width / 2, curRect.y + curRect.height / 2);
			//cv::Point2d ptInFrame;
			//// 用图片大小来归一化
			//ptInFrame.x = (resPt.x - center.x) / frame.cols;
			//ptInFrame.y = (resPt.y - center.y) / frame.rows;
			//cv::Point2d ptInMap;
			

			//std::cout << "END " << curRect << std::endl;
			cv::rectangle(frame, curRect, GREEN);
			cv::rectangle(frame, searchROIRect, PINK);

			if (KFInited)
				cv::rectangle(frame, cv::Rect(KFPt.x, KFPt.y, curRect.width, curRect.height), YELLOW);
		}
		cv::imshow(frameWinName, frame);
		char ch = cv::waitKey(10);
		if (toupper(ch) == 'Q')
			break;
		else if (toupper(ch) == 'P')
			cv::waitKey(0);

		//writer << frame;
	}
	//writer.release();

	return 0;
}