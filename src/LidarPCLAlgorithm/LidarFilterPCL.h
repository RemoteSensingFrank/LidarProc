#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 * @brief  基于PCL库的点云滤波算法，如果没有PCL库则无法运行
 * @note   
 * @retval None
 */

class LidarFilterPCL
{
public:
    /**
    * @brief  统计方法去除离群点
    * @note   
    * @param  input: 输入点云数据
    * @param  minNum: 最小点云数
    * @param  thresDis: 距离阈值
    * @param  pathOut: 输出点云路径
    * @retval 
    */
    long LidarFilterPCL_StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input,int minNum,double thresDis,const char* pathOut);

    /**
    * @brief  采用VoxelGrid方法进行滤波，提取骨架点
    * @note   
    * @param  input: 输入点云
    * @param  gridsizeX: Grid 的X的尺寸
    * @param  gridsizeY: Grid 的Y的尺寸
    * @param  gridsizeZ: Grid 的Z的尺寸
    * @param  pathOut: 文件输出路径
    * @retval 
    */
    long LidarFilterPCL_VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr input,double gridsizeX,double gridsizeY,double gridsizeZ,const char* pathOut);
};