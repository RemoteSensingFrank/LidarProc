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
#include "./LidarAlgorithm/LASSkeleton.h"
#include "./LidarAlgorithm/LASDangerPoints.h"
#include "./LidarUtil/Simulation.h"
#include "./LidarResearch/LASModel.h"

void PorfileGenerateSample()
{
    LASProfile profile;
    ProfileDecorate decorateParam;
    decorateParam.lbType = LABEL_SQUARE;

    //pcShrinkRobost.PointCloudShrinkSkeleton_LineTest();
    // LASProfile profile;
    // ProfileDecorate decorateParam;
    // decorateParam.lbType = LABEL_SQUARE;

    Point2D pntTower[2];
    pntTower[0].x = /* 204742.7500 */182716.690002;
    pntTower[0].y = /* 2477649.12 */2481004.000000;
	
    pntTower[1].x = /* 204781.66 */183023.250000;
    pntTower[1].y = /* 2477626.81 */2480775.869999;

    // LidarRegistrationUtil simuUtil(0.5,0.3,0.2,15,5,10);
    // simuUtil.LidarRegistration_Simulation("../data/default/more.las","../data/default/moreSimulate.las");

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
    reader4->LidarReader_Open("../data/default/mergecolor.las",lasdst1);
    reader4->LidarReader_Read(true,1,lasdst1);
    
    classifyElectricPatrolFast classFast;
    // Point2Ds points;
    // LASColorExt colorTower;
    // colorTower.Red=colorTower.Blue=colorTower.Green=255;
    
    // points.push_back(Point2D(182221.410003662109,2481208.119995117188));
    // points.push_back(Point2D(182704.270019531250,2481000.869995117188));
    // classFast.ElectricPatrolFast_Tower(lasdst1,points,18,colorTower);
    
    // LASColorExt lineTower;
    // lineTower.Red=lineTower.Green=0;lineTower.Blue=255;
    // long err=classFast.ElectricPatrolFast_Lines(lasdst1,points,18,7,lineTower);
    //reader4->LidarReader_Write("../data/default/classifiedLine.las",lasdst1,elcLine);

    LASColorExt groundTower;
    groundTower.Red=groundTower.Green=255;groundTower.Blue=0;
    classFast.ElectricPatrolFast_Ground(lasdst1,8,0.5,15,groundTower);
    reader4->LidarReader_Write("../data/default/classifiedGround.las",lasdst1,elcGround);
    
    // LASColorExt vegetationTower;
    // vegetationTower.Red=vegetationTower.Blue=0;vegetationTower.Green=255;
    // //classFast.ElectricPatrolFast_Vegetation(lasdst1,5,20,vegetationTower);
    // classFast.ElectricPatrolFast_VegetationLast(lasdst1,vegetationTower);
    // //reader4->LidarReader_Write("../data/default/classifiedVege.las",lasdst1,elcVegetation);

    // LASDangerPointsFlann dangerDetect;
    // dangerDetect.LASDangerPoints_Detect(22,lasdst1);
    // //reader4->LidarReader_Write("../data/default/classifiedDanger.las",lasdst1,elcDanger);

    // reader4->LidarReader_Write("../data/default/classified.las",lasdst1);


    delete lasdst1;
    delete reader4;
}

void SimpleLineExtract()
{
    LasAlgorithm::PointCloudLineRefineSkeleton ptLineSk;
    ILASDataset *lasdst1 = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    reader4->LidarReader_Open("../data/default/segment1.las",lasdst1);
    reader4->LidarReader_Read(true,1,lasdst1);
    Point3Ds pts=ptLineSk.PointCloudLineSkeleton_Extract(lasdst1,50,0.15);
    FILE *fs = fopen("../data/line.txt","w+");
    //遍历点云数据输出
    for(int i=0;i<pts.size();++i)
    {
        fprintf(fs,"%lf,%lf,%lf\n",pts[i].x,pts[i].y,pts[i].z);
    }
    fclose(fs);
    fs=nullptr;
    delete lasdst1;
    delete reader4;
}

int main(int argc ,char* argv[])
{
    // LidarRegistrationUtil lidarRegUtil(100,0,0,30,15,30);
    // lidarRegUtil.LidarRegistration_Simulation("/local/data/more.las","/local/data/moreSimulate2.las");

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudI(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudO(new pcl::PointCloud<pcl::PointXYZ>);

    ILASDataset *lasdst1 = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    reader4->LidarReader_Open("../data/cube_filtered.las",lasdst1);
    reader4->LidarReader_Read(true,1,lasdst1);

    ILASDataset *lasdst2 = new ILASDataset();
    LASReader *reader5 = new LidarMemReader();
    reader5->LidarReader_Open("../data/cube_filtered_t.las",lasdst2);
    reader5->LidarReader_Read(true,1,lasdst2);

    //点云局部分析
    // LASInvarianceFeatureExtract lasInvariance;
    // int pointIndex=1;
    // Point3Ds pnts1=lasInvariance.LASInvariancePointsPart(lasdst1,500,10);
    // Point3Ds pnts2=lasInvariance.LASInvariancePointsPart(lasdst2,500,10);

    // double disHistro1[20],disHistro2[20];
    // lasInvariance.LASInvariancePointsLASDisHistroCal(pnts1,disHistro1);
    // lasInvariance.LASInvariancePointsLASDisHistroCal(pnts2,disHistro2);
    // lasInvariance.LASInvariancePointsLASAngleHistroCal(pnts1,disHistro1);
    // lasInvariance.LASInvariancePointsLASAngleHistroCal(pnts2,disHistro2);
    
    LASTransToPCL transPCL;
    transPCL.LASTransToPCL_Trans(lasdst1,pclPointCloudI);
    // transPCL.LASTransToPCL_Trans(lasdst2,pclPointCloudO);

    LidarFeaturePoints lidarFeatures;
    pcl::PointCloud<int> siftPointIdx1;
    pcl::PointCloud<int> siftPointIdx2;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33>());
    lidarFeatures.LidarFeature_Sift(pclPointCloudI,siftPointIdx1,fpfhs1);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33>());
    lidarFeatures.LidarFeature_Sift(pclPointCloudO,siftPointIdx2,fpfhs2);

    //std::string path="../data/"+to_string(int(0))+".txt";
    // FILE* fs = fopen("../data/0.txt","w+");
    // fclose(fs);
    // LidarFeatureRegistration lidarReg;
    // std::vector<MATCHHISTRODIS> matches;
    
    // LASHeader header=lasdst1->m_lasHeader;
    // GeometryLas::Point3D centerPt((header.max_x+header.min_x)/2,(header.max_y+header.min_y)/2,(header.max_z+header.min_z)/2);
    
    // int type=1;
    //lidarReg.LidarRegistration_SiftFPFHMatch(fpfhs1,fpfhs2,matches);
    //lidarReg.LidarRegistration_Match(siftPointIdx1,siftPointIdx2,pclPointCloudI,pclPointCloudO,100,matches);
    //lidarReg.LidarRegistration_RANSC(pclPointCloudI,pclPointCloudO,siftPointIdx1,siftPointIdx2,0,matches);
    //lidarReg.LidarRegistration_OutputTest(pclPointCloudI,pclPointCloudO,siftPointIdx1,siftPointIdx2,0,matches);
    //double rot[]={30,0,0,100,0,0};
    //lidarReg.LidarRegistration_Check(pclPointCloudI,pclPointCloudO,siftPointIdx1,siftPointIdx2,type,matches,centerPt,rot);
    
    delete lasdst1;
    delete reader4;
    delete lasdst2;
    delete reader5;

    return 0;
}

