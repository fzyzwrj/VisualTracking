#include <cstdio>
#include <iostream>
#include "utils.h"

void test_assert()
{
	MY_ASSERT(1);
	//MY_ASSERT(1 - 1);
	//MY_ASSERT(false);
}

void test_time()
{
	TIME(Sleep(1000));
}

void test_type()
{
	vecI i;
	vecF f;
	vecD d;
	vecS s;
	CStr cstr("Hello, world");
	i.push_back(1);
	f.push_back(1.1f);
	d.push_back(1.2);
	s.push_back("Hello, world");
}

int main()
{
	test_assert();
	test_time();
	test_type();
	std::cout << "All work successfully." << std::endl;
	return 0;
}