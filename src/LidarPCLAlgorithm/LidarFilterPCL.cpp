/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-11-18 21:31:07
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-06-03 11:09:57
 */ 
#include "LidarFilterPCL.h"

#ifdef _USE_PCL_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
long LidarFilterPCL::LidarFilterPCL_StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input,int minNum,double thresDis,const char* pathOut)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);                   //设置待滤波的点云
	sor.setMeanK(minNum);                       //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(thresDis);           //设置判断是否为离群点的阀值
	sor.filter(*cloud_filtered);                //存储
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud(cloud_filtered);          //设置待滤波的点云
	sor.setMeanK(minNum/2);                       //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(thresDis);           //设置判断是否为离群点的阀值
	sor.filter(*cloud_filtered2);               //存储

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(pathOut, *cloud_filtered2, false);
}

long LidarFilterPCL::LidarFilterPCL_VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr input,double gridsizeX,double gridsizeY,double gridsizeZ,const char* pathOut)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(input);
	sor.setLeafSize(gridsizeX, gridsizeY, gridsizeZ);
	sor.filter(*cloud_filtered);
    pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(pathOut, *cloud_filtered, false);
}

#endif