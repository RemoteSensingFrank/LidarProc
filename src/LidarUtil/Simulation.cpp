/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-12-26 15:39:14
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-07-01 10:49:17
 */

#include "Simulation.h"
#include "../LidarBase/LASReader.h"

#define deg2rad(x) ((x)*M_PI/180.)
#define rad2deg(x) ((x)*180./M_PI)

long LidarRegistrationUtil::LidarRegistration_RotCalculate(bool deg)
{
    if(deg)
    {
        rx=deg2rad(rx);
        ry=deg2rad(ry);
        rz=deg2rad(rz);
    }
    rotMat = MatrixXd::Identity(3,3);
    rotMat(0,0) = cos(ry)*cos(rz);
    rotMat(0,1) = sin(rx)*sin(ry)*cos(rz) - cos(rx)*sin(rz);
    rotMat(0,2) = cos(rx)*sin(ry)*cos(rz) + sin(rx)*sin(rz);
    rotMat(1,0) = cos(ry)*sin(rz);
    rotMat(1,1) = sin(rx)*sin(ry)*sin(rz) + cos(rx)*cos(rz);
    rotMat(1,2) = cos(rx)*sin(ry)*sin(rz) - sin(rx)*cos(rz);
    rotMat(2,0) = -sin(ry);
    rotMat(2,1) = sin(rx)*cos(ry);
    rotMat(2,2) = cos(rx)*cos(ry);

    return 0;
}

long LidarRegistrationUtil::LidarRegistration_TransPer(Point3D &pt,Point3D centerPt)
{
    MatrixXd ptMat(3,1);
    ptMat(0,0) = pt.x-centerPt.x;
    ptMat(1,0) = pt.y-centerPt.y;
    ptMat(2,0) = pt.z-centerPt.z;

    //rotation
    MatrixXd transMat=rotMat*ptMat;
    
    pt.x = transMat(0,0)+centerPt.x-mx;
    pt.y = transMat(1,0)+centerPt.y-my;
    pt.z = transMat(2,0)+centerPt.z-mz;

    return 0;
}

long LidarRegistrationUtil::LidarRegistration_Simulation(const char* pathSrc,const char* pathTrans)
{
    ILASDataset *lasdst1 = new ILASDataset();
    LASReader *reader1 = new LidarMemReader();
    reader1->LidarReader_Open(pathSrc,lasdst1);
    reader1->LidarReader_Read(true,1,lasdst1);
    const LASHeader &header = lasdst1->m_lasHeader;
    Point3D centerPt((header.max_x+header.min_x)/2,(header.max_y+header.min_y)/2,(header.max_z+header.min_z)/2);
    for (int i = 0; i < lasdst1->m_totalReadLasNumber; ++i)
    {
        const LASIndex &idx = lasdst1->m_LASPointID[i];
        Point3D &pt = lasdst1->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d;
        LidarRegistration_TransPer(pt,centerPt);
    }

    reader1->LidarReader_Write(pathTrans,lasdst1);
    delete lasdst1;
    delete reader1;
}

LidarRegistrationUtil::LidarRegistrationUtil(double vx,double vy,double vz,double vrx,double vry,double vrz)
{
    mx = vx;
    my = vy;
    mz = vz;
    
    rx = vrx;
    ry = vry;
    rz = vrz;
    
    LidarRegistration_RotCalculate(true);
}

LidarRegistrationUtil::LidarRegistrationUtil(const LidarRegistrationUtil &util)
{
    mx = util.mx;
    my = util.my;
    mz = util.mz;
    
    rx = util.rx;
    ry = util.ry;
    rz = util.rz;
    
    LidarRegistration_RotCalculate(true);
}

LidarRegistrationUtil::LidarRegistrationUtil()
{
    mx = 0;
    my = 0;
    mz = 0;
    
    rx = 0;
    ry = 0;
    rz = 0;
    
    LidarRegistration_RotCalculate(true);
}