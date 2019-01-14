#pragma once
//
// Created by wuwei on 18-1-24.
// email：wuwei_cug@163.com

#ifndef LASGUI_LASDANGERPOINTS_H
#define LASGUI_LASDANGERPOINTS_H

#include <vector>

#include "../LidarGeometry/Geometry.h"
#include "../LidarGeometry/GeometryFlann.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASReader.h"
using namespace GeometryLas;

//pre define complex flann type
typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;


/**
* 获取线路危险点信息
*/
class LASDangerPoints {


public:
	/**
	* 根据距离获取线路危险点信息
	* @param distance
	* @return
	*/
	long LASDangerPoints_Detect(float distance, ILASDataset* datasetLine, ILASDataset* datasetVegterain);

	/**
	* danger level according distance
	* @param distance
	* @param dangerSectionNumber
	* @param datasetLine
	* @param datasetVegterain
	* @return
	*/
	long LASDangerPoints_Detect(float* distance, int dangerSectionNumber, ILASDataset* datasetLine, ILASDataset* datasetVegterain);

private:
	/**
	* 将危险点区分出来写入文件中
	* @param datasetVegterain
	* @param pathSplit
	* @return
	* deleted by Frank.Wu 通过直接通过写函数将危险点写入
	long LADDangerPoints_SplitDanger(ILASDataset* datasetVegterain, const char* pathSplit);
	*/

	/**
	* 检测某一个点范围
	* @param distance
	* @param pnt
	* @param datasetVegterian
	* @return
	*/
	long LASDangerPoints_PerPoint(float distance, const Point3D* pnt, ILASDataset* datasetVegterian);

	/**
	* danger level according distance pre point
	* @param distance
	* @param dangerSectionNumber
	* @param pnt
	* @param datasetVegterian
	* @return
	*/
	long LASDangerPoints_PerPoint(float* distance, int dangerSectionNumber, const Point3D* pnt, ILASDataset* datasetVegterian);

protected:
	/**
	* 获取危险点可能所在的范围
	* @param pnt
	* @param distance
	* @param datasetVegterain
	* @param rectIds
	* @return
	*/
	long LASDangerPoints_Range(const Point3D *pnt, float distance, ILASDataset* datasetVegterain, std::vector<int> &rectIds);
};

/*
	获取线路危险点信息，通过ANN算法进行加速实现，相比于简单分块
	采用kdtree算法能够极大的提高处理效率
	***由于Adaptor以及点云索引的构建因此在处理过程中需要较大内存***
	***一般来说需要超过点云大小两到三倍内存才能够进行处理否则可能产生内存不足错误***
	Created by Frank.Wu 2018-07-19
*/
class LASDangerPointsFlann
{
public:
	/**
	* 根据距离获取线路危险点信息
	* @param distance
	* @return
	*/
	long LASDangerPoints_Detect(float distance, ILASDataset* datasetLine, ILASDataset* datasetVegterain);

	/**
	* danger level according distance
	* @param distance
	* @param dangerSectionNumber
	* @param datasetLine
	* @param datasetVegterain
	* @return
	*/
	long LASDangerPoints_Detect(float* distance, int dangerSectionNumber, ILASDataset* datasetLine, ILASDataset* datasetVegterain);

private:
	/**
	* 检测某一个点范围
	* @param distance
	* @param pnt
	* @param treeVege
	* @return
	*/
	long LASDangerPoints_PerPoint(float distance, const Point3D* pnt, kd_tree &treeVege, ILASDataset* datasetVegterain);

	/**
	* danger level according distance pre point
	* @param distance
	* @param dangerSectionNumber
	* @param pnt
	* @param datasetVegterian
	* @return
	*/
	long LASDangerPoints_PerPoint(float* distance, int dangerSectionNumber, const Point3D* pnt, kd_tree &treeVege, ILASDataset* datasetVegterain);
};

/*
* detect falling trees
* */
class LASFallingTreesDangerPoints : public LASDangerPoints
{
public:
	/**
	* get height at the placce
	* @param dataImg: dem data
	* @param xsize :xsize of the dem image
	* @param ysize :ysize of the dem image
	* @param adfGeoTrans :geo transform of the dem image
	* @param gx :geo position x
	* @param gy :geo posision y
	* @return height
	*/
	float LASDangerPoints_Elevation(float* dataImg, int xsize, int ysize, double* adfGeoTrans, float gx, float gy);

	/**
	*
	* @param distance
	* @param dangerSectionNumber
	* @param pathDEM
	* @param datasetLine
	* @param datasetVegterain
	* @return error code
	*/
	long LASDangerPoints_FallingTree(float* distance, int dangerSectionNumber, const char* pathDEM, ILASDataset* datasetLine, ILASDataset* datasetVegterain);

private:
	/**
	*
	* @param distance
	* @param dangerSectionNumber
	* @param demData
	* @param xsize
	* @param ysize
	* @param pnt
	* @param datasetVegterian
	* @return
	*/
	long LASDangerPoints_FllingTree_PrePoint(float* distance, int dangerSectionNumber, float *demData, int xsize, int ysize,
		double *adfGeotrans, const Point3D* pnt, ILASDataset* datasetVegterian);
};


/*
	merge danger point datset to discrete points
	using dbscan algorithm to process
	get danger points and set classify discret distance
*/
class LASDangerPointsMergeArrgegate
{
public:
	/*
		extract danger points from lastadaset
		@param:asDataset:las dataset;
		@param:dangerPnts:danger points extract from dataset
	*/
	long LASDangerExtract(ILASDataset* lasDataset, Point3Ds &dangerPnts);

	/*
		extract correspond line point with the danger points
		@param:lineDataset line point set;
		@param:dangerPnts danger points;
		@linePnts:correspond line points
	*/
	long LASDangerExtractLinePoints(ILASDataset *lineDataset, Point3Ds dangerPnts, Point3Ds &linePnts);
	
	/*
		aggregate points set as a distance using knn
		@param:dangerPnts danger point set;
		@param:type aggregate type(directly using nature number)
		@param:knnRange range distance
	*/
	long LASDangerAggregate(Point3Ds dangerPnts,int *type,float knnRange);

	/*
		merge one type to one point and find the correspond line points
		@param dangerPnts:danger points
		@param type:class types
		@param typeNumbers:type numbers
		@pntDiscrete:point with the correspond line pts
		@correspond:correspond danger points and line point 
	*/
	long LASDangerMerge(Point3Ds dangerPnts, int *type, int typeNumbers, Point3Ds linePnts, int *correspondPairs);

	//for test
	long LASDangerTestFlann();
};

#endif //LASGUI_LASDANGERPOINTS_H
