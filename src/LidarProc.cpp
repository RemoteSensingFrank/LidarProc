// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include<stdio.h>
//#include<gtest/gtest.h>
#include "./LidarBase/LASReader.h"
#include "./LidarAlgorithm/LASProfile.h"
#include "./LidarResearch/LASFormatTransform.h"
#include "./LidarPCLAlgorithm/LidarFeaturePoints.h"
#include "./LidarPCLAlgorithm/LidarRegistration.h"
#include "./LidarPCLAlgorithm/LidarFilterPCL.h"
int main(int argc ,char* argv[])
{

    LASProfile profile;
    ProfileDecorate decorateParam;
    decorateParam.lbType = LABEL_SQUARE;

    Point2D pntTower[2];
    pntTower[0].x = /* 204742.7500 */182716.690002;
    pntTower[0].y = /* 2477649.12 */2481004.000000;
	
    pntTower[1].x = /* 204781.66 */183023.250000;
    pntTower[1].y = /* 2477626.81 */2480775.869999;

/* 	//根据大小计算需要的分辨率，控制图像大小
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
 */
#ifdef _USE_PCL_
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudI(new pcl::PointCloud<pcl::PointXYZ>);
    ILASDataset *lasdst1 = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    reader4->LidarReader_Open("../data/default/more.las",lasdst1);
    reader4->LidarReader_Read(true,1,lasdst1);
    LASTransToPCL transPCL;
    transPCL.LASTransToPCL_Trans(lasdst1,pclPointCloudI);
    LidarFilterPCL filterPcl;
    filterPcl.LidarFilterPCL_VoxelGrid(pclPointCloudI,0.5,0.5,0.5,"../data/default/more.pcd");

    delete lasdst1;
    delete reader4;

/*    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudI(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudO(new pcl::PointCloud<pcl::PointXYZ>);
    ILASDataset *lasdst1 = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    reader4->LidarReader_Open("../data/default/ref.las",lasdst1);
    reader4->LidarReader_Read(true,1,lasdst1);

    ILASDataset *lasdst2 = new ILASDataset();
    LASReader *reader5 = new LidarMemReader();
    reader5->LidarReader_Open("../data/default/trans.las",lasdst2);
    reader5->LidarReader_Read(true,1,lasdst2);

    LASTransToPCL transPCL;
    transPCL.LASTransToPCL_Trans(lasdst1,pclPointCloudI);
    transPCL.LASTransToPCL_Trans(lasdst2,pclPointCloudO);

    LidarRegistration lidarReg;
    lidarReg.LidarRegistration_FPFH(pclPointCloudI,pclPointCloudO,"../data/default/reg.pcd");
*/    
    //lidarReg.LidarRegistration_ICP(pclPointCloudI,pclPointCloudO,"../data/default/reg.pcd");
    
/*     LidarFeaturePoints lidarFeatures;
    pcl::PointCloud<int> siftPointIdx;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    lidarFeatures.LidarFeature_Sift(pclPointCloudI,siftPointIdx,fpfhs);
  */   
    //pcl::PointCloud<int> narfIndex;
    //pcl::PointCloud<pcl::Narf36> narfDesc;
    //lidarFeatures.LidarFeature_NARF(pclPointCloudI,narfIndex,narfDesc);

/*  delete lasdst1;
    delete reader4;
    delete lasdst2;
    delete reader5;
 */
 #endif


/*  testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();  */
}

