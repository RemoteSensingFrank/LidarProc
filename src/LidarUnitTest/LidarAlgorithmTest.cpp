#include "../LidarBase/LASHeader.h"
#include "../LidarBase/LASReader.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarAlgorithm/LASSimpleClassify.h"
#include "../LidarAlgorithm/LASProfile.h"
#include "../LidarAlgorithm/PointProcAlgorithm.h"
#include "../LidarAlgorithm/LASSkeleton.h"

#include <stdio.h>
#include <vector>
#include <string>
#include <gtest/gtest.h>

using namespace std;
using namespace LasAlgorithm;

//define secret id and key
// string secretId ="AKID3tY1HMEqFRQFqDLL4ZyAC4mGVWLwCdor";
// string secretKey="1pQnwra6MOEoCMGuLVS9ZlhRv86nLemf";

class LidarBaseTest : public testing::Test
{
protected:
	static void SetUpTestCase()
	{
		printf("file helper function class test start\n");
		//首先检查网络是否通畅
	}

	static void TearDownTestCase()
	{
		printf("file helper function class test end\n");
        
	}

    /**
    * check network if connected network then download test data
    * else do not run the unit test
    **/
    static bool CheckNetwork()
    {
        
    }

    virtual void SetUp() {}
    virtual void TearDown() {}

    static string m_UrlTestFilePath;
};


TEST(LASSIMPLECLASSIFY,LASSIMPLECLASSIFYFastTestCase)
{
    Point2Ds pntTower(2);

    pntTower[0].x = 182232.660003662109;
    pntTower[0].y = 2481201.869995117188;
    
    pntTower[1].x = 182707.520019531250;
    pntTower[1].y = 2481000.750000000000;
    
    ILASDataset *dataset   = new ILASDataset();
    LidarMemReader *reader = new LidarMemReader();

    reader->LidarReader_Open("../data/default/segment2.las",dataset);
    reader->LidarReader_Read(true,1,dataset); 
    LASColorExt color;
    color.Red   = 255;
    color.Green = 255;
    color.Blue  = 0;
    classifyElectricPatrolFast classifyFast;
    //EXPECT_EQ(-1,classifyFast.ElectricPatrolFast_Tower(nullptr,pntTower[0],13,color));
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Tower(dataset,pntTower[0],13,color));
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Tower(dataset,pntTower[1],13,color));

    color.Red   = 0;
    color.Green = 0;
    color.Blue  = 255;
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Lines(dataset,pntTower,13,5,color));

    color.Red   = 218;
    color.Green = 165;
    color.Blue  = 32;
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Ground(dataset,5,0.5,10,color));

    LASColorExt vegetationTower;
    vegetationTower.Red=vegetationTower.Blue=0;vegetationTower.Green=255;
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_VegetationLast(dataset,vegetationTower));

    reader->LidarReader_Write("../data/default/classifiedTest.las",dataset);

    delete dataset;
    delete reader;
}


TEST(LASPROFILE,LASPROFILEProfileTestCase)
{
#ifdef _USE_OPENCV_
    LASProfile profile;
    ProfileDecorate decorateParam;
    decorateParam.hspan_dis = 10;
    decorateParam.vspan_dis = 10;
    decorateParam.lbType = LABEL_CIRCLE;

    Point2D pntTower[2];
    pntTower[0].x = 204742.7500;
    pntTower[0].y = 2477649.12;
    
    pntTower[1].x = 204781.66;
    pntTower[1].y = 2477626.81;
    profile.LASProfile_Verticle("../data/default/colorLasFile.las",pntTower,20,0.5,"../data/profile/v.jpg",&decorateParam);
    profile.LASProfile_Horizontal("../data/default/colorLasFile.las",pntTower,20,0.5,"../data/profile/h.jpg",&decorateParam);
    profile.LASProfile_Front("../data/default/colorLasFile.las",pntTower,20,0.5,"../data/profile/f.jpg",&decorateParam);
#endif  
    EXPECT_EQ(0,0);
}


TEST(LASSKELETONShrink,LASSKELETONShrinkTestCase)
{
    ILASDataset *dataset   = new ILASDataset();
    LidarMemReader *reader = new LidarMemReader();

    reader->LidarReader_Open("../data/default/more.las",dataset);
    reader->LidarReader_Read(true,1,dataset); 
    Point3Ds points;
    PointCloudShrinkSkeleton pcSkeleton;
    points=pcSkeleton.PointCloudShrinkSkeleton_Shrink(dataset,5,5);
    
    FILE *fs = fopen("../data/testSkeleton.txt","w+");
    //遍历点云数据输出
    for(int i=0;i<points.size();++i)
    {
        fprintf(fs,"%lf,%lf,%lf\n",points[i].x,points[i].y,points[i].z);
    }
    fclose(fs);
    fs=nullptr;
    
    EXPECT_GT(points.size(),0);
}


TEST(LASKMEANS,LASKMEANSTestCase)
{
    ILASDataset *dataset   = new ILASDataset();
    LidarMemReader *reader = new LidarMemReader();

    reader->LidarReader_Open("../data/default/more.las",dataset);
    reader->LidarReader_Read(true,1,dataset); 
    int *iTypes = new int[dataset->m_totalReadLasNumber];
    memset(iTypes,0,sizeof(int)*dataset->m_totalReadLasNumber);
    PointCloudSegmentWithKMeans pointSegKMeans;
    long kmeans = pointSegKMeans.PointCloudSegment_KMeans(dataset,100,iTypes,0.5);
    EXPECT_EQ(kmeans,0);


    FILE *fs = fopen("../data/testSegmentKMeans.txt","w+");
    //遍历点云数据输出
    for(int i=0;i<dataset->m_totalReadLasNumber;++i)
    {
        const LASIndex &ind = dataset->m_LASPointID[i];
        const Point3D &pt = dataset->m_lasRectangles[ind.rectangle_idx].m_lasPoints[ind.point_idx_inRect].m_vec3d;
        //if(iTypes[i]==0){
            fprintf(fs,"%lf,%lf,%lf,%d\n",pt.x,pt.y,pt.z,iTypes[i]);
        //}
    }
    fclose(fs);
    fs=nullptr;
    delete[]iTypes;iTypes=nullptr;

    //Point3Ds points;
    //PointCloudShrinkSkeleton pcSkeleton;
    //points=pcSkeleton.PointCloudShrinkSkeleton_Shrink(dataset,5,3);
    
    // FILE *fs = fopen("../data/test.txt","w+");
    // //遍历点云数据输出
    // for(int i=0;i<points.size();++i)
    // {
    //     fprintf(fs,"%lf,%lf,%lf\n",points[i].x,points[i].y,points[i].z);
    // }
    // fclose(fs);
    // fs=nullptr;
    
    // EXPECT_GT(points.size(),0);
}


TEST(LASSKELETONLineInteractive,LASSKELETONLineInteractiveTestCase)
{
    ILASDataset *dataset   = new ILASDataset();
    LidarMemReader *reader = new LidarMemReader();

    reader->LidarReader_Open("../data/default/more.las",dataset);
    reader->LidarReader_Read(true,1,dataset); 
    Point3Ds mutiLines;
    mutiLines.push_back(Point3D(529486.689014739939,2371539.049981002696,-16.912999782562));
    mutiLines.push_back(Point3D(529487.870014495798,2371540.794981841929,-4.618999633789));
    mutiLines.push_back(Point3D(529504.262012786814,2371543.854981307872,-4.507999572754));
    mutiLines.push_back(Point3D(529506.700012512156,2371542.475981597789,-17.439999732971));
    
    PointCloudLineInteractive lineInteractive;
    Point3Ds innerPoints;
    long err=lineInteractive.PointCloudLineInteractive_GetPointsRange(dataset,0.5,mutiLines,innerPoints);
    EXPECT_EQ(err,0);
    Point3Ds nearestPoints;

    int idx = 1;
    err = lineInteractive.PointCloudLineInteractive_FindNearestLinePoitns(innerPoints,nearestPoints,mutiLines,idx);
    EXPECT_EQ(err,0);

    printf("pt1 before: %lf,%lf,%lf\n",mutiLines[idx-1].x,mutiLines[idx-1].y,mutiLines[idx-1].z);
    printf("pt2 before: %lf,%lf,%lf\n",mutiLines[idx].x,mutiLines[idx].y,mutiLines[idx].z);
    err = lineInteractive.PointCloudLineInteractive_LineFitOnce(nearestPoints,mutiLines,idx);
    EXPECT_EQ(err,0);
    printf("pt1 after: %lf,%lf,%lf\n",mutiLines[idx-1].x,mutiLines[idx-1].y,mutiLines[idx-1].z);
    printf("pt2 after: %lf,%lf,%lf\n",mutiLines[idx].x,mutiLines[idx].y,mutiLines[idx].z);
    printf("\n");

    idx=2;
    err = lineInteractive.PointCloudLineInteractive_FindNearestLinePoitns(innerPoints,nearestPoints,mutiLines,idx);
    EXPECT_EQ(err,0);

    printf("pt1 before: %lf,%lf,%lf\n",mutiLines[idx-1].x,mutiLines[idx-1].y,mutiLines[idx-1].z);
    printf("pt2 before: %lf,%lf,%lf\n",mutiLines[idx].x,mutiLines[idx].y,mutiLines[idx].z);
    err = lineInteractive.PointCloudLineInteractive_LineFitOnce(nearestPoints,mutiLines,idx);
    EXPECT_EQ(err,0);
    printf("pt1 after: %lf,%lf,%lf\n",mutiLines[idx-1].x,mutiLines[idx-1].y,mutiLines[idx-1].z);
    printf("pt2 after: %lf,%lf,%lf\n",mutiLines[idx].x,mutiLines[idx].y,mutiLines[idx].z);
    printf("\n");

    idx=1;
    err = lineInteractive.PointCloudLineInteractive_FindNearestLinePoitns(innerPoints,nearestPoints,mutiLines,idx);
    EXPECT_EQ(err,0);

    printf("pt1 before: %lf,%lf,%lf\n",mutiLines[idx-1].x,mutiLines[idx-1].y,mutiLines[idx-1].z);
    printf("pt2 before: %lf,%lf,%lf\n",mutiLines[idx].x,mutiLines[idx].y,mutiLines[idx].z);
    err = lineInteractive.PointCloudLineInteractive_LineFitOnce(nearestPoints,mutiLines,idx);
    EXPECT_EQ(err,0);
    printf("pt1 after: %lf,%lf,%lf\n",mutiLines[idx-1].x,mutiLines[idx-1].y,mutiLines[idx-1].z);
    printf("pt2 after: %lf,%lf,%lf\n",mutiLines[idx].x,mutiLines[idx].y,mutiLines[idx].z);
    printf("\n");
    
    FILE *fs = fopen("../data/test/innerline.txt","w+");
    //遍历点云数据输出
    for(int i=0;i<innerPoints.size();++i)
    {
        fprintf(fs,"%lf,%lf,%lf\n",innerPoints[i].x,innerPoints[i].y,innerPoints[i].z);
    }
    fclose(fs);

    FILE *fs2 = fopen("../data/test/nearestline.txt","w+");
    //遍历点云数据输出
    for(int i=0;i<nearestPoints.size();++i)
    {
        fprintf(fs2,"%lf,%lf,%lf,%d,%d,%d\n",nearestPoints[i].x,nearestPoints[i].y,nearestPoints[i].z,255,255,0);
    }
    fclose(fs2);
    
    fs =nullptr;
    fs2=nullptr;
    
}