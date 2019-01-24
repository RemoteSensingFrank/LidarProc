#pragma once

#include"../LidarAlgorithm/LASSimpleClassify.h"
#include<string>
using namespace std;

//method in my arcticle to extract tower points from points set
#ifdef _ARTICLE__USE_
/**
classify tower points from las dataset
author:wuwei
version:1.0.0.0
**/
class LASClassifyTower :public LASSimpleClassify
{
public:
	/*
	* set position of the tower
	* param x : coordinate x
	* param y : coordinate y
	* @return error code
	*/
	long LASSetTowerLocation(double x, double y);

	//TODO: Get tower position from las dataset
	/*
	* Get tower position from las dataset
	* @param dataset :las point dataset
	*/
	//long LASGetTowerLocation(ILASDataset *dataset);

		/*
	* get tower position 
	* @param pathLocate :path
	*/
	long LASGetTowerLocation(const char* pathLocate);

	/*
	get tower location by pre define
	*/
	long LASGetTowerLocation();


	/**
	* set tower range and in the range set the classify to elcTowerRange
	* @param dataset :lidar dataset(in & out)
	* @param range : classify color info (in)
	* @param rectTowers: rect towers
	* @return error code
	*/
	long LASTowerRect(ILASDataset* dataset, double range, vector<Rect2D> &rectTowers);

	/**
	* classify tower roughly(according to the average height) classify the point to elcTower
	* @param dataset :lidar dataset(in & out)
	* @param rectTowers : range tower
	* @param heightThreshod: height thresthod
	* @return  error code
	*/
	long LASTowerRough(ILASDataset* dataset, vector<Rect2D> rectTowers, float heightThreshold = 10);


	/**
	* calculate the plane of the ground in the tower range
	* @param dataset :lidar dataset(in & out)
	* @param rectTowers: rect towers
	* @param abc : params
	* @return  error code
	*/
	long LASTowerPlaneFit(ILASDataset* dataset, vector<Rect2D> &rectTowers, double *abc);

	/*
	* classify tower according to the plane and 
	  classify the tower point according to the 
	  relative height classify the point to elcTower
	* param dataset:las point dataset
	* param rectTowers:tower rect
	* param param:parameters of the plane
	* param heightThreshod:relatively height thresthold
	* @return  error code
	*/
	long LASTowerRoughPlane(ILASDataset* dataset, vector<Rect2D> rectTowers, double* param, float heightThreshold = 10);


	//这里这个refine的逻辑是：
	//如果一点周围一定范围内同类点数目超过一定的数目则认为这个点的分类是正确的，
	//如果这个点周围一定范围内同类点数目小于一定的数目则认为这个分类是噪声需要被剔除，
	//但是在使用过程中发现并没有起到作用
	/**
	* classify tower refine
	* @param dataset :lidar dataset(in & out)
	* @param range : range tower
	* @param cubeDis : range cube
	* @param cubePoints: point thresthod
	* @return
	*/
	long LASTowerRefine(ILASDataset* dataset, double range, float cubeDis = 0.5, float cubePoints = 20);

	/**
	* classify tower refine for classified points
	* @param dataset :lidar dataset(in & out)
	* @param range : range tower
	* @param cubeDis : range cube
	* @param cubePoints: point thresthod
	* @return
	*/
	long LASTowerRefineClassified(ILASDataset* dataset, double range, float cubeDis = 0.5, float cubePoints = 20);

	//------------------------------通过以上操作杆塔上半部分的点被分为elcTowerUp----------------------------------
	/**
	* export the point classified elcTowerRange as  block
	* @param dataset :dataset(in & out)
	* @param rectTowers : range tower
	* @param cubeDis : range cube
	* @param stepDis: setp distance
	* constant directory "F:\\LAS\\test\\"
	* @return
	*/
	vector<string> LASTowerRectExport(ILASDataset* dataset, const char* pathDir, vector<Rect2D> rectTowers, float cubeDis, float stepDis);


	/**
	* calculate each block and if above the threshold output the block
	* @param pathBlockDir:block file directory
	* @param threshold : range tower
	* @param pathMerge : merge points file
	* @return error code
	*/
	long LASTowerEigenClassify(const char* pathBlockDir, double threshold, const char* pathMerge);

	/**
	* Filter the vegetation points and left tower points
	* param datasetMerge: merge dataset of the points
	* param pathSeedTower
	*/
	long LASTowerSegDBScan(ILASDataset *datasetMerge, double segDis, const char* pathSeedTower);

	/*
		根据种子点对杆塔进行扩充
	*/
	long LASTowerSeed(ILASDataset* dataset, double hDis, double vDis, Point3Ds seedPoints, const char* pathOut);
	long LASTowerSeed(ILASDataset* dataset, double hDis, double vDis, const char* seedPath, const char* pathOut);
	//------------------------到此应该提取了杆塔的塔基坐标了，然后再进行修饰--------------------------
	/**
	* merge the towerup and tower bottom to construct a whole tower
	* param datasetUp:points of tower up
	* param datasetDown:points of tower down
	* param pathOut:output points path 
	**/
	long LASTowerTotalMerge(ILASDataset *datasetUp, ILASDataset *datasetDown, const char* pathOut);
	
	/*
	get tower localtion test
	*/
	long LASTowerRoughTest();

private:
	/*
	* calculate the Eigen of each block point
	* param strFile:block point file
	* param eigen:eigen values
	*/
	long calculateEigen(string strFile, vector<double> &eigen);

	
	//tower position
	vector<Point2D> m_towerLocate;
};
#endif



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
	long ElectricPatrolFast_Lines(ILASDataset* dataset, Point2D* towerPnt, double range, double height, LASColorExt color);

private:
	/*
		tower segment algorithm
		ILASDataset* dataset:las dataset
		LASIndexDisList idxDisLists:las data index
		LASColorExt color: the tower color
		double distance:scan distance
	*/
	long ElectrixPatrolFast_Seg(ILASDataset* dataset, LASIndexDisList idxDisLists, LASColorExt color,double distance);


	/*
		seed grow up algorithm
		ILASDataset* dataset:las dataset
		Point3Ds seedPoints:seed points
		double range:seed distance
		LASColorExt color:color
	*/
	long ElectrixPatrolFast_Seed(ILASDataset* dataset, Point3Ds seedPoints, double range, LASColorExt color);
};
