// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include <vector>
#include <map>
#include <string>
#include <exception>
#include <fstream>
#include <iterator>
#include "httplib.h"
#include "LidarUtil/FileHelper.h"
#include "LidarBase/LASReceive.h"
#include "LidarResearch/LASFormatTransform.h"

int main(int argc, char **argv)
{
	LASTransToPotree lasFormatTrans;
	lasFormatTrans.LASTransToPotree_Trans("../data/color.las");

    using namespace httplib;
    Server svr;
    svr.Get("/info", [](const Request& req, Response& res) {
		ifstream incpu("/proc/cpuinfo");
		istreambuf_iterator<char> cpu_begin(incpu);
		istreambuf_iterator<char> cpu_end;
		string cpu_str(cpu_begin, cpu_end);

		ifstream inmem("/proc/cpuinfo");
		istreambuf_iterator<char> mem_begin(inmem);
		istreambuf_iterator<char> mem_end;
		string mem_str(mem_begin, mem_end);

		string info = cpu_str+"\n"+mem_str;

        res.set_content(info, "text/plain");
    });

	svr.Get("/datalist", [](const Request& req, Response& res){
		string dirNameList="";
		vector<string> namelist;
		FileHelper::listDirNames("./www/pointclouds/",namelist);
		for(int i=0;i<namelist.size();++i)
			dirNameList+=namelist[i]+";";
		res.set_content(dirNameList, "text/plain");
		res.status=200;
	});

	svr.set_base_dir("./www");

    svr.listen("localhost", 1234);
}
