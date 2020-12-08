/*
 * @Descripttion: 点云数据骨架提取算法：
 * 1. 简单的骨架提取算法，在不考虑噪声点的情况下提取骨架的方法：
 *  首先根据某一点获取其N近邻个点，在获取这些点的基础上计算N近邻点
 *  的中心点（通过重心法计算中心点），然后迭代，找出所有点，完成
 *  简单骨架提取算法，但是效率比较低-先进行效率分析
 * 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 19-10-17. 13:11:49
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-11-25 10:55:43
 */
#pragma once

#ifndef _POINT_SKELETON_H_
#define _POINT_SKELETON_H_

#include "../LidarGeometry/Geometry.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASReader.h"
#include "Eigen/Eigen"

using namespace GeometryLas;
using namespace Eigen;

namespace LasAlgorithm 
{
    
    /**
    * @brief  extract skeleton from point cloud 直接根据重心收缩方法提取点云骨架
    * @note   
    * @retval None
    */
    class PointCloudShrinkSkeleton
    {
    private:
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
        
    public:
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

    /**
    * @brief  extract line skeleton from point cloud robost 
    * 从点集中提取出具有线性特征的点集
    * @note   
    * @retval None
    */
    class PointCloudLineSkeleton
    {
    protected:
        /**
         * @name: 判断点簇中哪些点集存在线性特征
         * @msg:  具体的判断方法为，找到任意两个点的直线，
         *        连接成直线判断点集拟合优度，选取最好的
         *        拟合优度的直线，如果最好的拟合优度直线大于某一个阈值，
         *        则说明具有线性特征（效率比较低，对于离群点的处理问题）
         * 
         * @param Point3Ds pointCluster：点簇的点集
         * @param double threshold：拟合优度阈值
         * @param MatrixXd &param：拟合直线的参数
         * @return: 
         */
        bool PointCloudLineSkeleton_LineExtractRaw(Point3Ds pointCluster,double threshold,MatrixXd &param);
    private:
        /**
         *  计算拟合残差
         **/
        double PointCloudLineSkeleton_LineResidual(Point3Ds pointCluster,MatrixXd lineParam);

        /**
         * @name: 对每个点进行解析，解析一轮，不知道是否要考虑多轮的解析，
         *        如果考虑多轮的解析则重新调用一次就好了
         * @param Point3Ds pointSet：全部数据点集
         * @param int nearPointNum：近邻点数目
         * @param double lineResidual：线性拟合残差
         * @return: 
         */   
         Point3Ds PointCloudLineSkeleton_Once(Point3Ds pointSet,int nearPointNum,double lineResidual);

    public:
        /**
         * @name: PointCloudLineSkeleton_Extract
         * @msg: 解析出具有线性特征的骨架点
         * @param Point3Ds pointSet:    输入点集
         * @param int nearPointNum:     临近点个数
         * @param double lineResidual:  线性残差阈值
         * @return: 
         */
        virtual Point3Ds PointCloudLineSkeleton_Extract(Point3Ds pointSet,int nearPointNum,double lineResidual);

        virtual Point3Ds PointCloudLineSkeleton_Extract(ILASDataset* lasDataset,int nearPointNum,double lineResidual);

        /**
        *  测试直线拟合函数是否正确
        **/ 
        void PointCloudShrinkSkeleton_LineTest();        
    };

    class PointCloudLineRefineSkeleton:public PointCloudLineSkeleton
    {
    protected:
        /**
         * @name: 重新定义点云线性特征
         * @msg: 首先判断点集是否具有线性特征，在具有线性特征的基础上
         *       获取数据点的主方向，然后根据数据点主方向重新对阈值进行定义
         *       判断数据点与阈值的关系，得到精化后的点云数据
         * @param Point3Ds pointCluster：点簇的点集
         * @param double threshold：拟合优度阈值
         * @return: 返回具有明显线性特征的点集
         */        
        vector<int> PointCloudLineSkeleton_LineRefine(Point3D ptCnt,Point3Ds pointCluster,double threshold);

    private:
        
        /**
         * @name: 对每个点进行解析，解析一轮，不知道是否要考虑多轮的解析，
         *        如果考虑多轮的解析则重新调用一次就好了
         * @param Point3Ds pointSet：全部数据点集
         * @param int nearPointNum：近邻点数目
         * @param double lineResidual：线性拟合残差
         * @return: 
         */   
         Point3Ds PointCloudLineSkeleton_Once(Point3Ds pointSet,int nearPointNum,double lineResidual);

    public:
        /**
         * @name: PointCloudLineSkeleton_Extract
         * @msg: 解析出具有线性特征的骨架点
         * @param Point3Ds pointSet:    输入点集
         * @param int nearPointNum:     临近点个数
         * @param double lineResidual:  线性残差阈值
         * @return: 
         */
        virtual Point3Ds PointCloudLineSkeleton_Extract(Point3Ds pointSet,int nearPointNum,double lineResidual);

        virtual Point3Ds PointCloudLineSkeleton_Extract(ILASDataset* lasDataset,int nearPointNum,double lineResidual);
    };
    
    /**
    * @brief  extract line refine interactive
    * 从点集中提取出具有线性特征的点集
    * @note   此方式为实验性方法，主要目的为专利与论文，主要参考文献为：
    * 参考文献：薛丽红. 三维空间点中基于最小二乘法的分段直线拟合方法[J]. 齐齐哈尔大学学报(自然科学版), 2015(04):87-88+92.
    * @retval None
    */
    class PointCloudLineInteractive
    {
    public:
        virtual long PointCloudLineInteractive_ModelRefine(ILASDataset *dataset,vector<Point3Ds> &featureLines)=0;
    
        /**
         * @name: PointCloudLineInteractive_GetPointsRange
         * @msg:  以线建立缓冲区，查找在缓冲区内所有点云数据
         * @param {type} 
         * @return: 
         */
        long PointCloudLineInteractive_GetPointsRange(ILASDataset *dataset,double range,Point3Ds points,Point3Ds &innerPoints);

        /**
         * @name: PointCloudLineInteractive_FindNearestLinePoitns
         * @msg: 在缓冲区点集中找到在距离所有直线中距离当前直线最近的点的点集
         * @param Point3Ds innerPoints:所有近邻点
         *                 Point3Ds &nearestPoints:距离给定线最近的点
         *                 Point3Ds mutiLines:多段线
         *                 int idx:给定直线从1开始，以idx-1以及idx两个点构成直线
         * @return {*}
         */                
        long PointCloudLineInteractive_FindNearestLinePoitns(Point3Ds innerPoints,Point3Ds &nearestPoints,Point3Ds mutiLines,int idx);
    };

    /**
     * @name: PointCloudLineInteractiveSimple
     * @msg: 在实际处理过程中对空间多段线的全局最优的约束求解实际上存在较大困难；
     *       如果在无法证明是全局最优选点的情况下难以进行很好的直线拟合算法因此
     *       对整体算法进行了简化，简化的主要过程为将多段线转换为简单直线进行拟合
     *       然后对简单直线进行合并
     * @param {*}
     * @return {*}
     */
    class PointCloudLineInteractiveSimple:public PointCloudLineInteractive
    {
    public:
        virtual long PointCloudLineInteractive_ModelRefine(ILASDataset *dataset,vector<Point3Ds> &featureLines);

        /**
         * @name: PointCloudLineInteractive_LineCheck
         * @msg:  判断两个点是否包含直线
         * @param {type} 
         * @return: true means maybe contain line else means don't contain
         */        
        bool PointCloudLineInteractive_LineCheck(ILASDataset *dataset,double range,double density, Point3Ds points);

        
        /**
         * @name: PointCloudLineInteractive_LineGet
         * @msg: 获取直线
         * @param {*}
         * @return {*}
         */
        long PointCloudLineInteractive_LineGet(ILASDataset *dataset,double range,double density, Point3Ds points,vector<Point3Ds> &lines);


        /**
         * @name: PointCloudLineInteractive_LineFitOnce
         * @msg: 单条直线拟合，与PointCloudLineInteractive_FindNearestLinePoitns配合使用，idx输入保持一致
         * @param {*}
         * @return {*}
         */
        long PointCloudLineInteractive_LineFitOnce(Point3Ds nearestPoints,Point3Ds &mutiLines,int idx);


        /**
         * @name: PointCloudLineInteractive_LineMerge
         * @msg: 对所有的点和线进行调整和合并，将两个近邻点合并为一个点，将点与线近邻的情况点连接到线上
         * @param {*}
         * @return {*}vector<
         */
        long PointCloudLineInteractive_LineMerge(vector<Point3Ds> &mutiSimpleLines,double disThreshold=0.3);


        /**
         * @name: PointCloudLineInteractive_Trans2Simple
         * @msg: trans mutilines to simple lines
         * @param {*}
         * @return {*}
         */
        long PointCloudLineInteractive_Trans2Simple(vector<Point3Ds> mutiComplexLines,vector<Point3Ds> &mutiSimpleLines);
 
        long PointCloudLineInteractive_Trans2Point(vector<Point3Ds> mutiComplexLines,Point3Ds &points);
    };
}

#endif