// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include<stdio.h>
#include<gtest/gtest.h>
#include "./LidarAlgorithm/LASProfile.h"


int main(int argc ,char* argv[])
{
 	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}

