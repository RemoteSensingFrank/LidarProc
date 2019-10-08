#include "../LidarBase/LASHeader.h"
#include "../LidarBase/LASReader.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarAlgorithm/LASSimpleClassify.h"
#include "../LidarAlgorithm/LASProfile.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <gtest/gtest.h>
using namespace std;

//define secret id and key
string secretId ="AKID3tY1HMEqFRQFqDLL4ZyAC4mGVWLwCdor";
string secretKey="1pQnwra6MOEoCMGuLVS9ZlhRv86nLemf";

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


TEST(LASSIMPLECLASSIFY,ClassifyElectricPatrolFastTestCase)
{
    Point2D pntTower[2];
    pntTower[0].x = 201136.7999877;
    pntTower[0].y = 2488348.81000137;
    
    pntTower[1].x = 201485.930057;
    pntTower[1].y = 2488217.08000183;
    
    ILASDataset *dataset   = new ILASDataset();
    LidarMemReader *reader = new LidarMemReader();

    reader->LidarReader_Open("../data/default/classify.las",dataset);
    reader->LidarReader_Read(true,1,dataset); 
    LASColorExt color;
    color.Red   = 255;
    color.Green = 255;
    color.Blue  = 0;
    classifyElectricPatrolFast classifyFast;
    //EXPECT_EQ(-1,classifyFast.ElectricPatrolFast_Tower(nullptr,pntTower[0],13,color));
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Tower(dataset,pntTower[0],13,color));
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Tower(dataset,pntTower[1],13,color));
    reader->LidarReader_Write("../data/default/Tower.las",dataset,elcTowerUp);

    color.Red   = 0;
    color.Green = 0;
    color.Blue  = 255;
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Lines(dataset,pntTower,13,5,color));
    reader->LidarReader_Write("../data/default/Line.las",dataset,elcLine);

    color.Red   = 218;
    color.Green = 165;
    color.Blue  = 32;
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Ground(dataset,5,0.5,10,color));
    reader->LidarReader_Write("../data/default/Ground.las",dataset,elcGround);
    reader->LidarReader_Write("../data/default/classify_tower_line.las",dataset);
}

TEST(LASPROFILE,LASPROFILEProfileTestCase)
{
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
    EXPECT_EQ(0,0);
}