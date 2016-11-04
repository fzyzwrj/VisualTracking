#pragma once
#ifndef COMMON_H__
#define COMMON_H__

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <assert.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <stack>
#include <queue>
#include <set>
#include <functional>
#include <algorithm>
#include <iomanip>
#include <iterator>

#include <direct.h>
#include <windows.h>

//#ifdef WIN32
//#define MY_EXPORT __declspec(dllexport)
//#else
//#define MY_EXPORT
//#endif

#define MY_ASSERT(expr)	(void)((expr) || (My_assert(#expr, __FUNCTION__, __FILE__, __LINE__), 0))

inline void My_assert(const char *expr, const char *function, const char *file, int line)
{
	printf("Assertion failed. expr: %s, func: %s, file: %s, line: %d\n", expr, function, file, line);
	abort();
}

//#ifdef WIN32
#define TIME(func)\
	do {\
	    LARGE_INTEGER time_begin;\
	    LARGE_INTEGER time_end;\
	    LARGE_INTEGER f;\
	    QueryPerformanceFrequency(&f);\
		double dqFreq = (double)f.QuadPart;\
		QueryPerformanceCounter(&time_begin);\
		func;\
		QueryPerformanceCounter(&time_end);\
		double time_used = (time_end.QuadPart - time_begin.QuadPart) * 1000.0 / dqFreq;\
		printf("TIME: %s, %6.3lfms\n", #func, time_used);\
	} while (0)
//#else
//#include <sys/time.h>
//#define TIME(func)\
//	do {\
//		double time_used = 0.0f;\
//		struct timeval time_begin;\
//		struct timeval time_end;\
//		gettimeofday(&time_begin, NULL);\
//		func;\
//		gettimeofday(&time_end, NULL);\
//		time_used = (time_end.tv_sec-time_begin.tv_sec)*1000000+(time_end.tv_usec-time_begin.tv_usec);\
//		printf("TIME: %s, %6.3lfms\n", #func, time_used);\
//	} while (0)
//#endif

// 常用别名
typedef std::vector<int> vecI;
typedef std::vector<float> vecF;
typedef std::vector<double> vecD;
typedef std::vector<std::string> vecS;
typedef const std::string CStr;

#endif /* COMMON_H__ */
