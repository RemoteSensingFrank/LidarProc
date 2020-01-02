/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-11-18 21:31:07
 * @LastEditors  : Frank.Wu
 * @LastEditTime : 2019-12-29 09:23:14
 */
#ifdef _USE_PCL_
#include "LidarRegistration.h"
#include <iostream>
#include <pcl/registration/icp.h>           //ICP类相关头文件
#include <pcl/registration/icp_nl.h>        //非线性ICP 相关头文件
#include <pcl/registration/transforms.h>      //变换矩阵类头文件


#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除

using namespace std;
long LidarRegistration::LidarRegistration_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                          const char* pathRegistration)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;   //创建IterativeClosestPoint的对象
    icp.setInputCloud(input_cloud);                                 //cloud_in设置为点云的源点
    icp.setInputTarget(ref_cloud);                                  //cloud_out设置为与cloud_in对应的匹配目标
    icp.setMaxCorrespondenceDistance(5);
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (1);
    pcl::PointCloud<pcl::PointXYZ> registration;                    //存储经过配准变换点云后的点云
    icp.align(registration);                                        //打印配准相关输入信息
    std::cout << "has converged:" << icp.hasConverged() <<" score: " <<icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    //配准点云保存到文件中
    pcl::io::savePCDFileASCII (pathRegistration, registration); //将点云保存到PCD文件中
    return 0;
}

long LidarRegistration::LidarRegistration_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                          const char* pathRegistration)
{
    clock_t start,end,time;
    start  = clock();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh =  compute_fpfh_feature(input_cloud,tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh =  compute_fpfh_feature(ref_cloud,tree);

    //对齐(占用了大部分运行时间)
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(input_cloud);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(ref_cloud);
    sac_ia.setTargetFeatures(target_fpfh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr align (new pcl::PointCloud<pcl::PointXYZ>());
    //sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
    sac_ia.setCorrespondenceRandomness(6); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
    sac_ia.align(*align); 
    end = clock();
    pcl::io::savePCDFile (pathRegistration, *align);
    printf("calculate time is: %lf\n",float (end-start)/CLOCKS_PER_SEC);
}

/**
 * TODO:
 * 在计算FPFH特征的过程中一直提示内存溢出，另外计算的特征向量的值一直不太对，需要进一步检查
 **/
pcl::PointCloud<pcl::FPFHSignature33>::Ptr LidarRegistration::compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    //法向量
    pcl::PointCloud<pcl::Normal>::Ptr point_normal (new  pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> est_normal;
    est_normal.setInputCloud(input_cloud);
    est_normal.setSearchMethod(tree);
    //est_normal.setKSearch(5);
    est_normal.setRadiusSearch(5);
    est_normal.compute(*point_normal);
    //std::cout<<point_normal->points[1000]<<std::endl;
    //fpfh 估计
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh (new pcl::PointCloud<pcl::FPFHSignature33>());
    //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
    pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
    est_fpfh.setNumberOfThreads(4); //指定4核计算
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
    est_fpfh.setInputCloud(input_cloud);
    est_fpfh.setInputNormals(point_normal);
    est_fpfh.setSearchMethod(tree);
    // est_fpfh.setKSearch(10);
    est_fpfh.setRadiusSearch(8);
    est_fpfh.compute(*fpfh);
    return fpfh;
}

#endif