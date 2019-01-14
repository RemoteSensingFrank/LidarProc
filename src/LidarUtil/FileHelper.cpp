#include "FileHelper.h"

#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <string.h>
#ifdef LINUX
	#include<sys/types.h>
	#include<dirent.h>
	#include<unistd.h>
#endif


FileHelper::FileHelper()
{
}


FileHelper::~FileHelper()
{
}

void FileHelper::listFiles(const char * dir, vector<string>& fileNameList)
{
#ifdef WIN32
	char dirNew[200];
	strcpy_s(dirNew, dir);
	strcat_s(dirNew, "\\*.*");    // 在目录后面加上"\\*.*"进行第一次搜索

	intptr_t handle;
	_finddata_t findData;

	handle = _findfirst(dirNew, &findData);    // 查找目录中的第一个文件
	if (handle == -1)
	{
		cout << "Failed to find first file!\n";
		return;
	}

	do
	{
		if (!(findData.attrib & _A_SUBDIR))
		{
			fileNameList.push_back(findData.name);
		}
	} while (_findnext(handle, &findData) == 0);    // 查找目录中的下一个文件

	cout << "GetFileList Done!\n";
	_findclose(handle);    // 关闭搜索句柄
#endif

#ifdef LINUX
	DIR *dirDir;
	struct dirent *fptr;

	if((dirDir=opendir(dir))==NULL)
	{
		printf("failed to open directory");
		exit(-1);
	}
	while((fptr=readdir(dirDir))!=NULL)
	{
		if(strcmp(fptr->d_name,".")||strcmp(fptr->d_name,".."))
			continue;
		else if(fptr->d_type==8){
			fileNameList.push_back(fptr->d_name);
		}
		else{
			continue;
		}
	}
	closedir(dirDir);
	printf("GetFileList Done!\n");
#endif
}

void FileHelper::listFilesIncludeSubDir(const char * dir) 
{
#ifdef WIN32
	char dirNew[200];
	strcpy_s(dirNew, dir);
	strcat_s(dirNew, "\\*.*");    // 在目录后面加上"\\*.*"进行第一次搜索

	intptr_t handle;
	_finddata_t findData;

	handle = _findfirst(dirNew, &findData);
	if (handle == -1)        // 检查是否成功
		return;

	do
	{
		if (findData.attrib & _A_SUBDIR)
		{
			if (strcmp(findData.name, ".") == 0 || strcmp(findData.name, "..") == 0)
				continue;

			
			cout << findData.name << "\t<dir>\n";

			// 在目录后面加上"\\"和搜索到的目录名进行下一次搜索
			strcpy_s(dirNew, dir);
			strcat_s(dirNew, "\\");
			strcat_s(dirNew, findData.name);

			listFilesIncludeSubDir(dirNew);
		}
		else
			cout << findData.name << "\t" << findData.size << " bytes.\n";
	} while (_findnext(handle, &findData) == 0);

	_findclose(handle);    // 关闭搜索句柄
#endif

#ifdef LINUX
	DIR *tdir;
	struct dirent *ptr;
	char base[1000];

	if ((tdir = opendir(dir)) == NULL)
	{
		perror("Open dir error...");
		exit(1);
	}

	while ((ptr = readdir(tdir)) != NULL)
	{
		if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)    ///current dir OR parrent dir
			continue;
		else if (ptr->d_type == 8)    ///file
		{
			string name = string(ptr->d_name);
			cout<<name<<endl;
		}
		else if (ptr->d_type == 10)    ///link file
			continue;
		else if (ptr->d_type == 4)    ///dir
		{
			listFilesIncludeSubDir(ptr->d_name);
		}
	}
	closedir(tdir);
#endif

}

void FileHelper::listFiles(string cate_dir, vector<string> &files, string ext)
{
#ifdef WIN32
	_finddata_t file;
	intptr_t lf;
	if ((lf = _findfirst((cate_dir+"*"+ext).c_str(), &file)) == -1) {
		printf("not found!");
	}
	else {
		files.push_back(cate_dir + file.name);
		while (_findnext(lf, &file) == 0) {
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;
			
			files.push_back(cate_dir+file.name);
		}
	}
	_findclose(lf);
#endif

#ifdef LINUX
	DIR *dir;
	struct dirent *ptr;
	char base[1000];

	if ((dir = opendir(cate_dir.c_str())) == NULL)
	{
		perror("Open dir error...");
		exit(1);
	}

	while ((ptr = readdir(dir)) != NULL)
	{
		if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)    ///current dir OR parrent dir
			continue;
		else if (ptr->d_type == 8)    ///file
		{
			string name = string(ptr->d_name);
			string::size_type pos = name.rfind('.');
			string ext = name.substr(pos == string::npos ? name.length() : pos + 1);

			if (!strcmp("ext", ext.c_str()))
				files.push_back(string(cate_dir) + name);

		}
		else if (ptr->d_type == 10)    ///link file
									   //printf("d_name:%s/%s\n",basePath,ptr->d_name);
			continue;
		else if (ptr->d_type == 4)    ///dir
		{
			files.push_back(string(ptr->d_name));
		}
	}
	closedir(dir);
#endif
	std::sort(files.begin(), files.end());
}


void FileHelper::listNames(string cate_dir, vector<string> &files, string ext)
{
#ifdef WIN32
	_finddata_t file;
	intptr_t lf;
	if ((lf = _findfirst((cate_dir + "*" + ext).c_str(), &file)) == -1) {
		printf("not found!");
	}
	else {
		files.push_back(file.name);
		while (_findnext(lf, &file) == 0) {
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;

			files.push_back(file.name);
		}
	}
	_findclose(lf);
#endif

#ifdef LINUX
	DIR *dir;
	struct dirent *ptr;
	char base[1000];

	if ((dir = opendir(cate_dir.c_str())) == NULL)
	{
		perror("Open dir error...");
		exit(1);
	}

	while ((ptr = readdir(dir)) != NULL)
	{
		if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)    ///current dir OR parrent dir
			continue;
		else if (ptr->d_type == 8)    ///file
		{
			string name = string(ptr->d_name);
			string::size_type pos = name.rfind('.');
			string ext = name.substr(pos == string::npos ? name.length() : pos + 1);

			if (!strcmp("ext", ext.c_str()))
				files.push_back(name);

		}
		else if (ptr->d_type == 10)    ///link file
									   //printf("d_name:%s/%s\n",basePath,ptr->d_name);
			continue;
		else if (ptr->d_type == 4)    ///dir
		{
			files.push_back(string(ptr->d_name));
		}
	}
	closedir(dir);
#endif
	std::sort(files.begin(), files.end());
}
