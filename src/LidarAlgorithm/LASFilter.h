#pragma once
//
// Created by Frank.Wu on 19-10-17.
// 点云滤波算法，针对杆塔数据的噪声进行滤除，获取
// 较为准确的杆塔部分点云数据，主要思路有两个
// 1:根据点云数据的密度滤除噪声，这个是主流思路，但是要考虑到密度的计算方式；
// 2:根据RGB特点进行滤除，这个针对特殊的应用的方法，不具备通用性，但是实现简单易操作；

#ifndef _POINTFILTER_H_
#define _POINTFILTER_H_

#include "Geometry.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASReader.h"
using namespace GeometryLas;

namespace LasAlgorithm 
{
	/*
        point cloud filter algorithm get DEM data
        using interpolation method 
	*/
	class PointCloudFilterDEM
	{
	public:
		/*
            get dem image data according to the imput las dataset
		*/
		long PointCloudFilter_Point2DEM(ILASDataset *lasDataset, float resolution, const char* pathChr, int filterTimes = 0);

		/*
            P.S. when the problem of search all the data and no range;
            the data tree should be constructed once more
		*/
		long PointCloudFilter_Point2DEMFlann(ILASDataset *lasDataset, float resolution, const char* pathChr);

	private:
        /**
        * @brief  dem filter core
        * @note   
        * @param  *dataDEM: dem data
        * @param  xsize: image xsize
        * @param  ysize: image ysize
        * @retval 
        */
		long PointCloudFilter_DEMFilter(float *dataDEM, int xsize, int ysize);
	};

    /**
    * @brief  remove noise using the method of filter
    * @note   
    * @retval None
    */
    class PointCloudFilterNoise
    {
    public:
        /**
        * @brief  离群点去除，通过密度滤波的方法剔除
        * @note   
        * @param  *lasDataset: 输入点云数据集,将噪声点标记为已删除
        * @param  ptNumerThreshod: 点云数量阈值
        * @param  rangeThreshod: 点云范围阈值
        * @retval 
        */
        long PointCloudFilter_Outlier(ILASDataset *lasDataset,int ptNumerThreshod,double rangeThreshod);

        /**
        * @brief  滤除对应RGB颜色的点
        * @note   
        * @param  *lasDataset: 点云数据集
        * @param  r: red
        * @param  g: green
        * @param  b: blue
        * @retval 
        */
        long PointCloudFilter_RGBOutlier(ILASDataset *lasDataset,int r,int g,int b);
    };


}
#endif