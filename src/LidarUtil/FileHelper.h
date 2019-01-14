#pragma once
#include <string>
#include <vector>
using namespace std;
class FileHelper
{
public:
	FileHelper();
	~FileHelper();

	/*
		遍历目录中所有文件（包括目录和子目录）
		@param dir：输入目录以 \\结束
	*/
	static void listFilesIncludeSubDir(const char * dir);  //遍历目录中的所有文件(包括子目录)

	/*
		遍历目录下的所有文件(不包括子目录)
		@param dir：输入文件目录 以\\结束
		@param fileNameList：将文件目录保存到数据中
	*/
	static void listFiles(const char * dir, vector<string>& fileNameList);  //遍历目录下的所有文件(不包括子目录)	
	/*
		遍历目录下的所有文件(不包括子目录)
		@param dir：输入文件目录 以\\结束
		@param files：将文件目录保存到数据中-文件全部路径
		@param ext：后缀名.xxx
	*/
	static void listFiles(string cate_dir, vector<string> &files, string ext);//获取文件夹下所有特定后缀的目录
	
	/*
		遍历目录下的所有文件(不包括子目录)
		@param dir：输入文件目录 以\\结束
		@param files：将文件目录保存到数据中-文件名
		@param ext：后缀名.xxx
	*/
	static void listNames(string cate_dir, vector<string> &files, string ext);//获取文件名
};

