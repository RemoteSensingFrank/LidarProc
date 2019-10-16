#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
};