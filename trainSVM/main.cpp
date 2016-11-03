#include <opencv2/opencv.hpp>
#include <iostream>
#include "common.h"
#include "liblinear\linear.h"
#include <cassert>
std::string modelFilename;
#define Malloc(type, n) (type *)malloc((n) * sizeof(type))
static void print_null(const char *) {}

//void test(const cv::Mat &model);
void getData(CStr txtFilename, vecM &X, int total);
void trainStage(CStr posFilename, CStr negFilename, int posTotal, int negTotal, double C, double eps);
cv::Mat trainSVM(const vecM &pX, const vecM &nX, int sT, double C, double bias = -1, double eps = 0.01, int maxTrainNum = 100000);
cv::Mat trainSVM(const cv::Mat &X1f, const vecI &Y, int sT, double C, double bias, double eps);
void saveModel(CStr modelFilename, const cv::Mat &model)
{
	cv::FileStorage fs(modelFilename, cv::FileStorage::WRITE);
	fs << "model" << model;
	fs.release();
}


void makeFeature(const cv::Mat &mat, feature_node *x_space)
{
	//x_space = Malloc(feature_node, (prob.n + 1) * prob.l);
	const int DIM_FEA = mat.cols;
	//x_space = Malloc(feature_node, DIM_FEA + 1);	//bias > 0
	int i = 0;
	const float* xData = mat.ptr<float>(i);
	for (i = 0; i < DIM_FEA; ++i) {
		x_space[i].index = i + 1;
		x_space[i].value = xData[i];
	}
	// bias > 0
	x_space[i].index = DIM_FEA + 1;
	x_space[i++].value = 1;
	x_space[i].index = -1;
}

void testStage(CStr posFilename, CStr negFilename, int posTotal, int negTotal)
{
	vecM pX, nX;
	std::cout << "Start to get positive test data" << std::endl;
	TEST_TIME(getData(posFilename, pX, posTotal));
	std::cout << "Finish getting positive test data" << std::endl;

	std::cout << "Start to get negative test data" << std::endl;
	TEST_TIME(getData(negFilename, nX, negTotal));
	std::cout << "Finish getting negative test data" << std::endl;


	std::cout << "Start to read model" << std::endl;
	struct model *svmModel = load_model(modelFilename.c_str());
	std::cout << "Finish reading model" << std::endl;

	const int DIM_FEA = pX[0].cols;
	feature_node *x_space = Malloc(feature_node, DIM_FEA + 1 + 1);
	int testPosTotal = pX.size(), testNegTotal = nX.size();
	int testPosPass = 0, testNegPass = 0;
	for (int i = 0; i < pX.size(); ++i) {
		makeFeature(pX[i], x_space);
		double label = predict(svmModel, x_space);
		std::cout << label << std::endl;
		if (label >= 0)
			++testPosPass;
	}
	for (int i = 0; i < nX.size(); ++i) {
		makeFeature(nX[i], x_space);
		double label = predict(svmModel, x_space);
		std::cout << label << std::endl;
		if (label < 0)
			++testNegPass;
	}

	printf("%8d/%8d = %2.2f\n", testPosPass, testPosTotal, testPosPass * 1.0 / testPosTotal);
	printf("%8d/%8d = %2.2f\n", testNegPass, testNegTotal, testNegPass * 1.0 / testNegTotal);

	free(x_space);
	free_and_destroy_model(&svmModel);
}

void getData(CStr txtFilename, vecM &X, int total)
{
	X.clear();
	X.resize(total);
	std::ifstream fin(txtFilename);
	MY_ASSERT(fin.is_open());
	std::string line;
	int i = 0;

	std::cout << "Total: " << total << std::endl;
	while (std::getline(fin, line)) {
		if (line.find(".jpg") == std::string::npos)
			continue;
		cv::Mat img = cv::imread(line);
		cv::resize(img, img, cv::Size(200, 200));

		//SHOW_WAIT(img);
		img = img.reshape(1, 1);	// 0表示通道数不变，行数变为1
		cv::Mat temp;
		img.convertTo(temp, CV_32F);
		//X.push_back(img);
		temp.copyTo(X[i]);
		++i;
		if (i % 5000 == 0)
			std::cout << "Having read: " << i << std::endl;
		if (i >= total)
			break;
	}
}
void trainStage(CStr posFilename, CStr negFilename, int posTotal, int negTotal, double C, double eps)
{
	// 正负样本的特征
	vecM pX, nX;
	std::cout << "Start to get positive data" << std::endl;
	TEST_TIME(getData(posFilename, pX, posTotal));
	std::cout << "Start to get negtive data" << std::endl;
	TEST_TIME(getData(negFilename, nX, negTotal));

	cv::Mat model = trainSVM(pX, nX, L2R_L2LOSS_SVC, C, 1, eps);
	std::cout << "Start to write model" << std::endl;
	//cv::imwrite("model.Mat", model);
	saveModel("model.xml", model);
	//model = model.colRange(0, model.cols - 1).reshape(1, 64);
}

cv::Mat trainSVM(const vecM &pX, const vecM &nX, int sT, double C, double bias, double eps, int maxTrainNum)
{
	int pTotal = static_cast<int>(pX.size());
	int nTotal = static_cast<int>(nX.size());
	int featureDim = pX[0].cols;
	int sampelTotal = pTotal + nTotal;

	cv::Mat X1f(sampelTotal, featureDim, CV_32F);
	
	vecI Y(sampelTotal);
	std::cout << "Start to reshape data" << std::endl;
	for (int i = 0; i < pTotal; ++i) {
		pX[i].copyTo(X1f.row(i));
		Y[i] = 1;
	}
	
	for (int i = 0; i < nTotal; ++i) {
		nX[i].copyTo(X1f.row(i + pTotal));
		Y[i + pTotal] = -1;
	}
	std::cout << "Finish reshape data" << std::endl;
	return trainSVM(X1f, Y, sT, C, bias, eps);
}

cv::Mat trainSVM(const cv::Mat &X1f, const vecI &Y, int sT, double C, double bias, double eps)
{
	// Set SVM parameters
	parameter param;

	{
		param.solver_type = sT; // L2R_L2LOSS_SVC_DUAL;
		param.C = C;
		param.eps = eps; // see setting below
		param.p = 0.1;
		param.nr_weight = 0;
		param.weight_label = NULL;
		param.weight = NULL;
		set_print_string_function(print_null);
		CV_Assert(X1f.rows == Y.size() && X1f.type() == CV_32F);
	}

	// Initialize a problem
	feature_node *x_space = NULL;
	problem prob;
	
	{
		prob.l = X1f.rows;	// 训练样本数
		prob.bias = bias;	// 偏置项
		prob.y = Malloc(double, prob.l);	// 样本的标签，即多少个样本
		prob.x = Malloc(feature_node*, prob.l);	// 样本的特征
		const int DIM_FEA = X1f.cols;
		prob.n = DIM_FEA + (bias >= 0 ? 1 : 0);	// 特征维度
		x_space = Malloc(feature_node, (prob.n + 1) * prob.l);
		int j = 0;
		for (int i = 0; i < prob.l; i++) {
			prob.y[i] = Y[i];
			prob.x[i] = &x_space[j];
			const float* xData = X1f.ptr<float>(i);
			for (int k = 0; k < DIM_FEA; k++) {
				x_space[j].index = k + 1;
				x_space[j++].value = xData[k];
			}
			if (bias >= 0) {
				x_space[j].index = prob.n;
				x_space[j++].value = bias;
			}
			x_space[j++].index = -1;
		}
		CV_Assert(j == (prob.n + 1) * prob.l);
	}

	// Training SVM for current problem
	const char*  error_msg = check_parameter(&prob, &param);
	if (error_msg) {
		fprintf(stderr, "ERROR: %s\n", error_msg);
		exit(1);
	}
	std::cout << "Start train..." << std::endl;
	model *svmModel = NULL;
	TEST_TIME(svmModel= train(&prob, &param));
	save_model(modelFilename.c_str(), svmModel);
	std::cout << "Finish train" << std::endl;
	cv::Mat wMat(1, prob.n, CV_64F, svmModel->w);
	wMat.convertTo(wMat, CV_32F);
	free_and_destroy_model(&svmModel);
	destroy_param(&param);
	free(prob.y);
	free(prob.x);
	free(x_space);
	std::cout << "Finish free resouces" << std::endl;
	return wMat;
}

static void help_test(void)
{
	std::cout << "Test SVM\n"
		"test_svm.exe model.dat test_pos.txt 3333 test_neg.txt 4444\n"
		"1. model.dat SVM保存的模型文件名\n"
		"2. test_pos.txt 正样本的文件，每一行是一个jpg的文件\n"
		"3. 3333 用于测试的正样本的个数\n"
		"4. test_neg.txt\n"
		"5. 4444 用于测试的负样本的个数\n"
		<< std::endl;
}

static void test(int argc, char *argv[])
{
	if (argc != 6) {
		help_test();
		exit(-1);
	}
	modelFilename = argv[1];
	int testPosTotal = atoi(argv[3]);
	int testNegTotal = atoi(argv[5]);
	testStage(argv[2], argv[4], testPosTotal, testNegTotal);
}

static void help_train_and_test(void)
{
	std::cout << "Train and test SVM\n"
		"train_svm.exe modeld.dat train_pos.txt 1111 train_neg.txt 2222 test_pos.txt 3333 test_neg.txt 4444 10 0.01\n"
		"1. model.dat 保存与训练的模型文件名\n"
		"2. train_pos.txt 正样本的文件，每一行是一个jpg的文件\n"
		"3. 1111 用于训练的正样本的个数\n"
		"4. train_neg.txt\n"
		"5. 2222 用于训练的负样本的个数\n"
		"6. test_pos.txt\n"
		"7. 3333 用于测试的正样本的个数\n"
		"8. test_neg.txt\n"
		"9. 4444 用于测试的负样本的个数\n"
		"10. 10 C的值，一般取1\n"
		"11. 0.01 EPS的值，一般取0.1\n"
		<< std::endl;
}

static void train_and_test(int argc, char *argv[])
{
	if (argc != 12) {
		help_train_and_test();
		exit(-1);
	}
	modelFilename = argv[1];
	int trainPosTotal = atoi(argv[3]);
	int trainNegTotal = atoi(argv[5]);
	int testPosTotal = atoi(argv[7]);
	int testNegTotal = atoi(argv[9]);
	double C = atof(argv[10]);
	double eps = atof(argv[11]);
	trainStage(argv[2], argv[4], trainPosTotal, trainNegTotal, C, eps);
	testStage(argv[6], argv[8], testPosTotal, testNegTotal);
}

int main(int argc, char *argv[])
{
	train_and_test(argc, argv);
	//test(argc, argv);
	return 0;
}