/*
 * @Descripttion: 数据模拟
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-12-26 15:12:36
 * @LastEditors  : Frank.Wu
 * @LastEditTime : 2019-12-28 09:29:04
 */
#include "Eigen/Eigen"
#include "../LidarBase/LASPoint.h"
using namespace Eigen;
/**
 * @name: 点云配准算法的数据模拟
 * @msg: 
 * @param {type} 
 * @return: 
 */
class LidarRegistrationUtil
{
public:
    /**
     * @name: 
     * @msg: 
     * @param {type} 
     * @return: 
     */    
    LidarRegistrationUtil(double vx,double vy,double vz,double vrx,double vry,double vrz);

    LidarRegistrationUtil(const LidarRegistrationUtil &util);

    LidarRegistrationUtil();
    
    /**
     * @name: 配准模拟，根据旋转角和平移向量
     * @msg: 
     * @param {type} 
     * const char* pathSrc：输入数据集
     * const char* pathTrans：模拟转换后数据集
     * @return: 
     */
    long LidarRegistration_Simulation(const char* pathSrc,const char* pathTrans);

private:
    /**
     * @name: 根据旋转角度计算旋转矩阵
     * @msg: 
     * @return: 
     */
    long LidarRegistration_RotCalculate(bool deg);

    /**
     * @name: 转换一个点
     * @msg: 
     * @param Point3D &pt 三维点
     * @param Point3D &centerPt 中心点
     * @return: 
     */
    long LidarRegistration_TransPer(Point3D &pt,Point3D centerPt);

    double mx;
    double my;
    double mz;
    double rx;
    double ry;
    double rz;
    MatrixXd rotMat;
};