/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2020-09-07 15:40:45
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-09-07 16:19:31
 */
#pragma once
//
// Created by wuwei on 18-1-5.
//

#ifndef LASGUI_LASMODEL_H
#define LASGUI_LASMODEL_H

#include <vector>
#include <stdio.h>
#include <memory>
#include "../LidarGeometry/Geometry.h"

class ILASDataset;

using namespace GeometryLas;

//点密度和模型的定义的定义
struct DenseSt {
	Point3I point;
	int dense;
};

/**
 * (x-x0)/m=(y-y0)/n=(z-z0)/p
 * k1=m/p
 * k2=n/p
 * b1=x0-m/p*z0
 * b2=y0-u/p*z0
 */
struct LineModelSt{
	double k1;
	double k2;
	double b1;
	double b2;
};

#define  LASDense std::vector<DenseSt>

class ILASDataset;

/**
* 定义由LAS数据集构建的模型
*/
class LASModel
{
public:
	/**
	* 根据点云数据构建点密度模型
	* @param dataset
	* @param cubeRange
	*/
	virtual void LASModel_PointsDense(ILASDataset* dataset, float cubeRange);

	/**
	* 根据点密度模型构建模型
	*/
	void LASModel_Build(ILASDataset* dataset, float cubeRange, const char* pathExport);

private:
	LASDense m_model;
};

/**
 * 	三维点云的分段线性拟合，
 *  TODO:
 * 	通过前端交互获取的点提取一定缓冲区范围内的点云数据；
 *  根据点云数据构建三维分段线性拟合模拟实现分段线性拟合；
 *  最后导出KML模型结构实现杆塔模拟人机交互拟合构建；
 *  参考文献：薛丽红. 三维空间点中基于最小二乘法的分段直线拟合方法[J]. 齐齐哈尔大学学报(自然科学版), 2015(04):87-88+92.
 **/
class LASMutiLineModel
{
public:
	/**
	 * @name: LASMutiLineModel_PointBuffer
	 * @msg: get buffer point set according to the input point 
	 * @param Point3Ds mutiLinePoints: input muti line 3D points
	 * 	    Point3Ds &pointSet：buffer point set
	 * 		double bufferSize：buffer size
	 * @return error code
	 */
	long LASMutiLineModel_PointBuffer(Point3Ds mutiLinePoints,Point3Ds &pointSet,double bufferSize);

	/**
	 * @name: LASMutiLineModel_Fit
	 * @msg: get the fitted model
	 * @param Point3Ds pointSet	: point set to be fitted
	 * 		 vector<LineModelSt> &lineModelSet ：muti line model
	 * @return error code
	 */
	long LASMutiLineModel_Fit(Point3Ds pointSet,vector<LineModelSt> &lineModelSet);

private:
	/**
	 * @name: LASMutiLineModel_FitEpoch
	 * @msg: fit for a line
	 * @param Point3Ds pointSet	: point set to be fitted,
	 * 		  vector<int> &labeledSet:if lebaeld in the epoch
	 * 		  LineModelSt &lineModel：one line model
	 * @return {type} 
	 */	
	long LASMutiLineModel_FitEpoch(Point3Ds pointSet,vector<int> &labeledSet,LineModelSt &lineModel);
};

#endif //LASGUI_LASMODEL_H
