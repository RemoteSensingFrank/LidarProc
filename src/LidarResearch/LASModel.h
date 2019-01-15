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



#endif //LASGUI_LASMODEL_H
