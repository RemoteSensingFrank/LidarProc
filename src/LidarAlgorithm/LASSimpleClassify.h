#pragma once
//
// Created by wuwei on 18-1-14.
//

#ifndef LASGUI_LASSIMPLECLASSIFY_H
#define LASGUI_LASSIMPLECLASSIFY_H

#include <vector>
#include "../LidarBase/LASPoint.h"
//#include <gdal_alg.h>
#ifndef ColorClassInfo
typedef struct ColorClassInfo
{
	int red;
	int green;
	int blue;
	eLASClassification classType;
}ColoInfo;
#endif

/*
	simple classify using the base information of the las dataset
	author:wuwei
	version:1.0.0.0
*/
class LASSimpleClassify {
public:
	/**
	* classify the point cloud by number of echo
	* @param dataset
	* @return
	*/
	long LASVegetationByEcho(ILASDataset* dataset);

	/**
	* classify data by elevation
	* @param dataset :lidar dataset
	* @param elevation :input elevation
	* @param direct :above or below the elevation(true above false below)
	* @param eclass :input the type
	* @return
	*/
	long LASClassifyByElevation(ILASDataset* dataset, float elevation, bool direct, eLASClassification eclass);


	/**
	* classufy data by intensity
	* @param dataset
	* @param elevation
	* @param direct
	* @param eclass
	* @return
	*/
	long LASClassifyByIntensity(ILASDataset* dataset, float intensity, bool direct, eLASClassification eclass);

	/**
	* classify data by RGB color
	* @param dataset :lidar dataset(in & out)
	* @param colorInfo : classify color info (in)
	* @return
	*/
	long LASClassifyByColor(ILASDataset* dataset, std::vector<ColoInfo> colorInfo);
};

/**
    classify las dataset with limited memory
 	author:wuwei
	version:1.0.0.0
*/
class LASClassifyMemLimited : public LASSimpleClassify
{
public:
	/**
	* export the dataset by each types
	* @param pathLas
	* @param type
	* @param pathExport
	* @return
	*/
	long LASExportClassifiedPoints(const char* pathLas, eLASClassification type, const char* pathExport);
};

/*
used for electric patrol fast
classify the tower, line vegetation and ground
the algorithm is used for engineering so the convient and
the speed is the factor considered first in this case
we try to use the simplest algorithm to do the work and the
accuracy will inevitably decrease
*/

struct LASIndexDis {
	LASIndex indx;
	double distance;
};
typedef std::vector<LASIndexDis> LASIndexDisList;

/*
	extract electric line information and classify the object auto fast
	author:wuwei
	version:1.0.0.0
*/
class classifyElectricPatrolFast {
public:
	/*
		extract tower points set from las data
		the algorithm decscribed as follows:
		1. get the tower point range and find all the points in the range;
		2. for the tower points is higher than the vegetation points use precentage to extract point higher than 40%
		3. some vegetation points will mixed inevitable so we use dbscan algorthm to remove vegetation points
		ILASDataset* dataset:las dataset
		Point2D towerPnt:tower position
		double range:tower range
		LASColorExt color: the tower color
	*/
	long ElectricPatrolFast_Tower(ILASDataset* dataset, Point2D towerPnt, double range, LASColorExt color);

	long ElectricPatrolFast_Tower(ILASDataset* dataset, Point2Ds towerPnt, double range, LASColorExt color);
	/*
		extract line point set from las data
		the algorithm decscribed as follows:
		1. get the tower point range and find all the points in the range and the the base height;
		2. caculate the plane and the points above the plane will be considerd as line point(if not classified)
		3. use region grow up algoritm to get all the points
		ILASDataset* dataset:las dataset
		Point2D *towerPnt:tower position
		double range:tower range
		double height:height threshold
		LASColorExt color: the tower color
	*/
	long ElectricPatrolFast_Lines(ILASDataset* dataset, Point2Ds towerPnt, double range, double height, LASColorExt color);


	/*
		extract ground points 
		the algorithm decscribed as follows:
		1.get local minmal point as the ground point
		2.remove the points that make the slope steep
		3.construct the TIN using the points
		4.if the points near the TIN classify the points as the ground point
		ILASDataset* dataset:las dataset
		LASColorExt color: the tower color
	*/
	long ElectricPatrolFast_Ground(ILASDataset* dataset, double rectRange, double disThres, double angle, LASColorExt color);

	/*
		extract vegetation points
		the algorithm decscribed as follows:
		1.get local minmal point as the ground point
		2.remove the points that make the slope steep
		3.construct the TIN using the points
		4.if the points near the TIN classify the points as the ground point
		ILASDataset* dataset:las dataset
		LASColorExt color: the tower color
	*/
	long ElectricPatrolFast_Vegetation(ILASDataset* dataset, double rectRange, double disThres, LASColorExt color);

	/**
	 * classified the vegetation points using the calssified point
	 * direct classified the vegetation if the point is not classified to Towers,Lines and Ground
	 */
	long ElectricPatrolFast_VegetationLast(ILASDataset* dataset,LASColorExt color);

private:
	/*
		get local min points and remove the noise points
	*/
	long ElectricPatrolFast_LocalMinNonMax(ILASDataset* dataset, std::vector<LASIndex> pntIdxs,double rectRange, eLASClassification cls, Point3Ds &localMin);

	/*
		classify the ground pnt by dis
	*/
	long ElectricPatrolFast_GroundDis(ILASDataset* dataset, double rectRange, double disThres, std::vector<LASIndex> &pntIdxs);

	/*
		classify the ground pnt by angle
		created by: Frank.Wu
		version:v1.0.0.0
		changed by: Frank.Wu
		version:v1.0.1.0 the last version calcualte the angle with the local min point
						 but actually the angle of the point with the plane should
						 be calculated 
	*/
	long ElectricPatrolFast_Angle(ILASDataset* dataset, double rectRange, double angle, std::vector<LASIndex> &pntIdxs);
	/*
		construct triangle using the local min points
	*/
	//GDALTriangulation* ElectricPatrolFast_Triangle(Point3Ds localMinPts);

	/*
		tower segment algorithm
		ILASDataset* dataset:las dataset
		LASIndexDisList idxDisLists:las data index
		LASColorExt color: the tower color
		double distance:scan distance
	*/
	long ElectrixPatrolFast_Seg(ILASDataset* dataset, LASIndexDisList idxDisLists, LASColorExt color, double distance);


	/*
		seed grow up algorithm
		ILASDataset* dataset:las dataset
		Point3Ds seedPoints:seed points
		double range:seed distance
		LASColorExt color:color
	*/
	long ElectrixPatrolFast_Seed(ILASDataset* dataset, Point3Ds seedPoints, double range, LASColorExt color);
};


#endif //LASGUI_LASSIMPLECLASSIFY_H

