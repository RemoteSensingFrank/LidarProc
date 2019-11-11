#pragma once
//
// Created by Frank.Wu on 19-10-17.
// 点云数据骨架提取算法：
// 1. 简单的骨架提取算法，在不考虑噪声点的情况下提取骨架的方法：
// 首先根据某一点获取其N近邻个点，在获取这些点的基础上计算N近邻点
// 的中心点（通过重心法计算中心点），然后迭代，找出所有点
// 
// 

#ifndef _POINT_SKELETON_H_
#define _POINT_SKELETON_H_

#include "../LidarGeometry/Geometry.h"
#include "../LidarBase/LASPoint.h"
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
    public:
        /**
        * @brief  通过质心收缩算法提取骨架点,判断是否包含质心点，如果包含质心点则不处理这个点簇
        * @note   
        * @param  clusterIdx: 聚类簇的点的id
        * @param  clusterNum: 聚类簇的点云数目
        * @retval 
        */
        int PointCloudShrinkSkeleton_Centroid(Point3Ds pointSet ,size_t* clusterIdx,int clusterNum);

        /**
        * @brief  迭代收缩算法运行一次之后的结果
        * @note   
        * @param  pointSet: 点云数据集
        * @param  nearPointNum: 查询临近点数目
        * @retval 返回运行一次之后的骨架点点云数据集
        **/
        Point3Ds PointCloudShrinkSkeleton_Once(Point3Ds pointSet,int nearPointNum);
        
        
        /**
        * @brief  迭代收缩算法
        * @note   
        * @param  pointSet: 点云数据集
        * @param  nearPointNum: 查询临近点数目
        * @param  iteratorNum: 收缩算法迭代次数
        * @retval 返回得到的骨架点点集 
        */
        Point3Ds  PointCloudShrinkSkeleton_Shrink(Point3Ds pointSet,int nearPointNum,int iteratorNum);
        
        Point3Ds  PointCloudShrinkSkeleton_Shrink(ILASDataset* lasDataset,int nearPointNum,int iteratorNum);
    };
}

#endif