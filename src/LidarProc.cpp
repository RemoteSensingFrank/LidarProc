// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include<stdio.h>
//#include<gtest/gtest.h>
#include "./LidarBase/LASReader.h"
#include "./LidarAlgorithm/LASProfile.h"
#include "./LidarAlgorithm/LASFilter.h"
#include "./LidarResearch/LASFormatTransform.h"
#include "./LidarPCLAlgorithm/LidarFeaturePoints.h"
#include "./LidarPCLAlgorithm/LidarRegistration.h"
#include "./LidarPCLAlgorithm/LidarFilterPCL.h"
#include "./LidarAlgorithm/LASSimpleClassify.h"
#include "./LidarAlgorithm/LASDangerPoints.h"

void PorfileGenerateSample()
{
    LASProfile profile;
    ProfileDecorate decorateParam;
    decorateParam.lbType = LABEL_SQUARE;

    Point2D pntTower[2];
    pntTower[0].x = /* 204742.7500 */182716.690002;
    pntTower[0].y = /* 2477649.12 */2481004.000000;
	
    pntTower[1].x = /* 204781.66 */183023.250000;
    pntTower[1].y = /* 2477626.81 */2480775.869999;

 	//根据大小计算需要的分辨率，控制图像大小
	double xscale = fabs(pntTower[0].x-pntTower[1].x)/600;
	double yscale = fabs(pntTower[0].y -pntTower[1].y)/800;
	double scale = min(xscale,yscale);
	double hspan = max(fabs(pntTower[0].x-pntTower[1].x)/10,fabs(pntTower[0].y -pntTower[1].y)/10);

    decorateParam.hspan_dis = hspan;
    decorateParam.vspan_dis = 10;
    decorateParam.labelPnts.push_back(Point3D(182766.440002,2480992.250000,66.100002));

    profile.LASProfile_Verticle("../data/default/color.las",pntTower,20,scale,"../data/profile/v.jpg",&decorateParam);
    profile.LASProfile_Horizontal("../data/default/color.las",pntTower,20,scale,"../data/profile/h.jpg",&decorateParam);
    profile.LASProfile_Front("../data/default/color.las",pntTower,20,scale,"../data/profile/f.jpg",&decorateParam);

}

void PCLSample()
{
#ifdef _USE_PCL_
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudI(new pcl::PointCloud<pcl::PointXYZ>);
    // ILASDataset *lasdst1 = new ILASDataset();
    // LASReader *reader4 = new LidarMemReader();
    // reader4->LidarReader_Open("../data/default/more.las",lasdst1);
    // reader4->LidarReader_Read(true,1,lasdst1);

    // LasAlgorithm::PointCloudFilterNoise lidarfilter;
    // lidarfilter.PointCloudFilter_Outlier(lasdst1,30,0.07);
    // reader4->LidarReader_Write("../data/default/morenoise.las",lasdst1,elcDeletedPoint);

  
    // LASTransToPCL transPCL;
    // transPCL.LASTransToPCL_Trans(lasdst1,pclPointCloudI);
    // LidarFilterPCL filterPcl;
    // filterPcl.LidarFilterPCL_VoxelGrid(pclPointCloudI,0.5,0.5,0.5,"../data/default/more.pcd");

    // delete lasdst1;
    // delete reader4;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudI(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudO(new pcl::PointCloud<pcl::PointXYZ>);
    // ILASDataset *lasdst1 = new ILASDataset();
    // LASReader *reader4 = new LidarMemReader();
    // reader4->LidarReader_Open("../data/default/ref.las",lasdst1);
    // reader4->LidarReader_Read(true,1,lasdst1);

    // ILASDataset *lasdst2 = new ILASDataset();
    // LASReader *reader5 = new LidarMemReader();
    // reader5->LidarReader_Open("../data/default/trans.las",lasdst2);
    // reader5->LidarReader_Read(true,1,lasdst2);

    // LASTransToPCL transPCL;
    // transPCL.LASTransToPCL_Trans(lasdst1,pclPointCloudI);
    // transPCL.LASTransToPCL_Trans(lasdst2,pclPointCloudO);

    // LidarRegistration lidarReg;
    // lidarReg.LidarRegistration_FPFH(pclPointCloudI,pclPointCloudO,"../data/default/reg.pcd");
   
    // //lidarReg.LidarRegistration_ICP(pclPointCloudI,pclPointCloudO,"../data/default/reg.pcd");
    
    // LidarFeaturePoints lidarFeatures;
    // pcl::PointCloud<int> siftPointIdx;
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    // lidarFeatures.LidarFeature_Sift(pclPointCloudI,siftPointIdx,fpfhs);
   
    // //pcl::PointCloud<int> narfIndex;
    // //pcl::PointCloud<pcl::Narf36> narfDesc;
    // //lidarFeatures.LidarFeature_NARF(pclPointCloudI,narfIndex,narfDesc);

    // delete lasdst1;
    // delete reader4;
    // delete lasdst2;
    // delete reader5;
 
 #endif
}

void ClassifySample()
{
    ILASDataset *lasdst1 = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    reader4->LidarReader_Open("../data/default/segment2.las",lasdst1);
    reader4->LidarReader_Read(true,1,lasdst1);
    
    classifyElectricPatrolFast classFast;
    Point2Ds points;
    LASColorExt colorTower;
    colorTower.Red=colorTower.Blue=colorTower.Green=255;
    
    points.push_back(Point2D(182221.410003662109,2481208.119995117188));
    points.push_back(Point2D(182704.270019531250,2481000.869995117188));
    classFast.ElectricPatrolFast_Tower(lasdst1,points,18,colorTower);
    
    LASColorExt lineTower;
    lineTower.Red=lineTower.Green=0;lineTower.Blue=255;
    long err=classFast.ElectricPatrolFast_Lines(lasdst1,points,18,7,lineTower);
    //reader4->LidarReader_Write("../data/default/classifiedLine.las",lasdst1,elcLine);

    LASColorExt groundTower;
    groundTower.Red=groundTower.Green=255;groundTower.Blue=0;
    classFast.ElectricPatrolFast_Ground(lasdst1,5,2,20,groundTower);
    
    
    LASColorExt vegetationTower;
    vegetationTower.Red=vegetationTower.Blue=0;vegetationTower.Green=255;
    //classFast.ElectricPatrolFast_Vegetation(lasdst1,5,20,vegetationTower);
    classFast.ElectricPatrolFast_VegetationLast(lasdst1,vegetationTower);
    //reader4->LidarReader_Write("../data/default/classifiedVege.las",lasdst1,elcVegetation);

    LASDangerPointsFlann dangerDetect;
    dangerDetect.LASDangerPoints_Detect(22,lasdst1);
    //reader4->LidarReader_Write("../data/default/classifiedDanger.las",lasdst1,elcDanger);

    reader4->LidarReader_Write("../data/default/classified.las",lasdst1);


    delete lasdst1;
    delete reader4;
}

int main(int argc ,char* argv[])
{
    ClassifySample();
    return 0;
}

