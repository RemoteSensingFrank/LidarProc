#pragma once
//
// Created by Frank.Wu on 19-10-17.
// 点云数据骨架提取算法：
// 
// 

#ifndef _POINTFILTER_H_
#define _POINTFILTER_H_

#include "Geometry.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASReader.h"
using namespace GeometryLas;

namespace LasAlgorithm 
{
    
    /**
    * @brief  extract skeleton from point cloud 直接根据重心收缩方法提取点云骨架
    * @note   
    * @retval None
    */
    class PointCloudShrinkSkeleton
    {
        /**
        * @brief  通过质心收缩算法提取骨架点,判断是否包含质心点，如果包含质心点则不处理这个点簇
        * @note   
        * @param  clusterIdx: 聚类簇的点的id
        * @param  clusterNum: 聚类簇的点云数目
        * @retval 
        */
        int PointCloudShrinkSkeleton_Centroid(int* clusterIdx,int clusterNum);

        /**
        * @brief  迭代收缩算法
        * @note   
        * @param  pointSet: 点云数据集
        * @param  nearPointNum: 查询临近点数目
        * @param  iteratorNum: 收缩算法迭代次数
        * @retval 
        */
        long  PointCloudShrinkSkeleton_Shrink(Point3Ds pointSet,int nearPointNum,int iteratorNum);
    };
}