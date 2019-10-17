#pragma once
//
// Created by wuwei on 17-12-26.
//

#ifndef LASLIB_LASREADER_H
#define LASLIB_LASREADER_H


#include <stdio.h>
#include "LASHeader.h"
#include "LASPoint.h"


class ILASDataset;

/*读写lidar文件基类*/
class LASReader
{
public:
	LASReader() { m_isDatasetOpen = false; m_lasFile = nullptr; }
	~LASReader() {
		m_isDatasetOpen = false;
		if (m_lasFile != nullptr)
			fclose(m_lasFile);
	}

	//1.打开las文件
	virtual long LidarReader_Open(const char* pathLidar, ILASDataset* dataset) = 0;
	//2.读取las文件
	virtual long LidarReader_Read(bool inMemory, int skip, ILASDataset* dataset) = 0;
	//3.将las文件写入
	virtual long LidarReader_Write(const char* pathLidar, ILASDataset* dataset) = 0;
	//4.将某一类数据写入las文件中
	virtual long LidarReader_Write(const char* pathLidar, ILASDataset* dataset, eLASClassification classType) = 0;
	//4.设置进度条
	//virtual void LidarReader_SetProgress(GDALProcessBase* progress) { m_Progress = progress; }

public:
	//以上为las文件数据集
	FILE *m_lasFile;
	bool m_isDatasetOpen;
	//GDALProcessBase		*m_Progress;
};



//version 1.2
//author: Frank.Wu
class LidarMemReader : public  LASReader
{
public:
	//1.打开las文件
	/*
		打开数据不读取(仅仅读取头文件)
		@param pathLidar:输入数据路径
		@param dataset:读取的数据集
	*/
	long LidarReader_Open(const char* pathLidar, _OUT_ ILASDataset* dataset);

	//2.读取las文件
	/*
		读取数据集(将数据集读取到文件中)
		@param inMemory：是否读取到内存中（可以只读取索引）
		@param skip：选择读取点的采样频率
		@param dataset：数据集
	*/
	long LidarReader_Read(bool inMemory, int skip, _OUT_ ILASDataset* dataset);

	//3.将las文件写入
	/*
		将LAS数据集写入文件中
		@param pathLidar:文件路径
		@param dataset：待写入的数据集
	*/
	long LidarReader_Write(const char* pathLidar, ILASDataset* dataset);

	//4.将某一类las文件写入，写入文件中为LAS格式
	/*
		将某一类的数据集写入文件中
		@param pathLidar:文件路径
		@param dataset：待写入的数据集
		@param classType：待写入的文件的类型
	*/
	long LidarReader_Write(const char* pathLidar, ILASDataset* dataset, eLASClassification classType);
	
	//5.las文件以同一个颜色写入
	/*
		将所有点云数据以某一个颜色写入文件中
		@param pathLidar:文件路径
		@param dataset：待写入的数据集
		@param classType：颜色信息
	*/
	long LidarReader_WriteWithColor(const char* pathLidar, ILASDataset* dataset, LASColorExt color);
	
	//6.将LAS文件合并读取
	/*
		读取多个las文件信息到一个dataset中
		@param lasFiles：文件集合
		@param dataset：读取到的数据集
	*/
	long LidarReader_ReadMerge(std::vector<string> lasFiles, _OUT_ ILASDataset* dataset);

	/**
	* @brief  7.根据分类情况导出数据，这里的导出是导出为txt的点云格式
	* @note   
	* @param  pathLidar: 
	* @param  dataset: 
	* @param  classType: 
	* @retval 
	*/
	long LidarReader_Export(const char* pathLidar, ILASDataset* dataset, int classType);

protected:
	//颗粒度更小，使得代码复用性更高
	/*
	*	@param fs:文件指针
	*	@param dataset：数据集
	*	@param widthPre：网格宽度
	*	@param heightPre：网格高度
	*	@param widthNum：网格行数
	*	@param heightNum：网格列数
	*	@param skip：读取文件采样数
	*	@param numPtsRect：每一个网格中点的个数
	*/
	long LidarReader_RectNumbers(FILE* fs, ILASDataset* dataset,double widthPre,double heightPre,
		                         int widthNum,int heightNum, int skip,_OUT_ int *numPtsRect);

	/*
	*   读取每个Rect中的点云数据读取到一个指针中
	*	@param fs:文件指针
	*	@param dataset：数据集
	*	@param widthPre：网格宽度
	*	@param heightPre：网格高度
	*	@param widthNum：网格行数
	*	@param heightNum：网格列数
	*	@param skip：读取文件采样数
	*	@param numPtsRect：每一个网格中点的个数
	*/
	long LidarReader_RectPoints(FILE* fs, ILASDataset* dataset, double widthPre, double heightPre,
								 int widthNum, int heightNum,bool inMemory, int skip, int &totallasPnts, _OUT_ int *pointsRect);


	/*
	*	读取每个Rect中的点云数据读取到vector中
	*	@param fs:文件指针
	*	@param dataset：数据集
	*	@param widthPre：网格宽度
	*	@param heightPre：网格高度
	*	@param widthNum：网格行数
	*	@param heightNum：网格列数
	*	@param skip：读取文件采样数
	*/
	long LidarReader_RectPoints(FILE* fs, ILASDataset* dataset, double widthPre, double heightPre,
		int widthNum, int heightNum, bool inMemory, int skip, int &totallasPnts);
};

/*
	分块读取的功能：
	create by Frank.Wu
*/
class LidarPatchReader : public LidarMemReader
{
public:
	/**
	* 读取LAS头
	* @param lf
	* @param lasHeader
	* @return
	*/
	long LidarReader_ReadHeader(FILE* lf, LASHeader &lasHeader);

	/**
	* 写LAS头
	* @param lf
	* @param lasHeader
	* @return
	*/
	long LidarReader_WriteHeader(FILE* lf, LASHeader lasHeader);

	/**
	* 读取一个Patch
	* @param lf
	* @param lasHeader
	* @param points
	* @param number
	* @return patch的范围
	*/
	Rect2D LidarReader_ReadPatch(FILE* lf, LASHeader lasHeader, LASPoint* points, int &number);

	/**
	* 写一个Patch
	* @param lf
	* @param lasHeader
	* @param points
	* @param number
	* @return patch的范围
	*/
	long LidarReader_WritePatch(FILE* lf, LASHeader lasHeader, LASPoint* points, int number);
};

class LidarReaderTxt :public LidarMemReader {
public:
	/**
	* careate LAS header
	* @param lf
	* @param lasHeader
	* @return
	*/
	long LidarReader_Open(const char* pathLidar, ILASDataset* dataset);

	/*
		Read las from txt file
	*/
	long LidarReader_Read(bool inMemory,eTxtLASType type, ILASDataset* dataset);

};


#endif //LASLIB_LASREADER_H000
