#include <cstdio>
#include <iostream>
#include <opencv2\opencv.hpp>
#include "utils_opencv.h"

static int test_cnt = 0;
static int test_pass = 0;

const double EPS = 1e-7;

#define EXPECT_EQ_BASE(equality, expect, actual, format)\
	do {\
		++test_cnt;\
		if (equality)\
			++test_pass;\
		else {\
			fprintf(stderr, "%s:%d expect: " format " actual: " format "\n", __FILE__, __LINE__, expect, actual);\
		}\
	} while (0)

#define EXPECT_EQ_INT(expect, actual) EXPECT_EQ_BASE((expect) == (actual), expect, actual, "%d")
#define EXPECT_EQ_DOUBLE(expect, actual) EXPECT_EQ_BASE(fabs((expect) - (actual)) < EPS, expect, actual, "%.7g")
//#define EXPECT_EQ_DOUBLE(expect, actual) EXPECT_EQ_BASE((expect) == (actual), expect, actual, "%.17g")

void test_type()
{

}

void test_show()
{

}

void test_draw()
{
	cv::Mat img = cv::imread("test.jpg");
	if (!img.data) {
		std::cerr << "Please add test.jpg in the solution." << std::endl;
		exit(-1);
	}
	DRAW_CROSS(img, cv::Point(10, 10), RED, 3);
	SHOW_WAIT(img);
}

void test_dist()
{
	EXPECT_EQ_DOUBLE(5.0, dist(cv::Point(3, 0), cv::Point(0, 4)));
	EXPECT_EQ_DOUBLE(sqrt(0.5), dist(cv::Point(1, 1), cv::Point(3, 0), cv::Point(0, 3)));
}

void test_angle()
{
	EXPECT_EQ_DOUBLE(45.0 / 180.0 * CV_PI, calcAngle(cv::Point(2, 0), cv::Point(1, 1)));
}

int main()
{
	test_type();
	test_show();
	test_draw();
	test_dist();
	test_angle();
	printf("%d/%d (%3.2f%%) passed\n", test_pass, test_cnt, test_pass * 100.0 / test_cnt);
	return 0;
}