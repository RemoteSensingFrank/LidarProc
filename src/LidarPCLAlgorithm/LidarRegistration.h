/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-11-18 21:31:07
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2019-12-26 15:12:55
 */
#ifdef _USE_PCL_

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>

/**
 * @brief  使用PCL库实现ICP点云配准
 * @note   
 * @retval None
 */
class LidarRegistration
{
public:
    /**
    * @brief  通过PCL的ICP方法对点云进行配准
    * @note   
    * @param  ref_cloud: 参考点云
    * @param  input_cloud: 待配准点云
    * @param  pathRegistration: 配准后点云路径
    * @retval 返回处理结果
    */
    long LidarRegistration_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                          const char* pathRegistration);
    /**
    * @brief  通过PCL的FPFH特征对点云进行配准
    * @note   
    * @param  ref_cloud: 参考点云
    * @param  input_cloud: 待配准点云
    * @param  pathRegistration: 配准后点云路径
    * @retval 返回处理结果
    */
    long LidarRegistration_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                          const char* pathRegistration);

    /**
    * @brief  计算FPFH特征
    * @note   
    * @param  input_cloud: 输入点云数据
    * @param  tree: kd树
    * @retval 
    */
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                                    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

};

#endif