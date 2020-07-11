/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-11-18 21:31:08
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-07-11 11:32:38
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
 *	激光点云不变性特征提取 
 */
class LASInvarianceFeatureExtract
{
public:
}


#endif //LASGUI_LASMODEL_H
