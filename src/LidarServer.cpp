// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include <vector>
#include <map>
#include <string>
#include <exception>
#include <fstream>
#include "LidarBase/LASReceive.h"

int main(int argc, char **argv)
{
	LASDatasetReceiveHttp lasReceiveHttps;
	printf("%s\n",lasReceiveHttps.ReceiveLASDataset("http://192.168.0.105/fileshare/Lib20180808.zip","./data/").c_str());
	return 0;
}
