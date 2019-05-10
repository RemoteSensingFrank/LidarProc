#include "../LidarBase/LASHeader.h"
#include "../LidarBase/LASReader.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarAlgorithm/LASSimpleClassify.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <gtest/gtest.h>
using namespace std;

TEST(LASSIMPLECLASSIFY,ClassifyElectricPatrolFastTestCase)
{
    Point2D pntTower[2];
    pntTower[0].x = 201136.7999877;
    pntTower[0].y = 2488348.81000137;
    
    pntTower[1].x = 201485.930057;
    pntTower[1].y = 2488217.08000183;
    
    ILASDataset *dataset   = new ILASDataset();
    LidarMemReader *reader = new LidarMemReader();

    reader->LidarReader_Open("../data/classify.las",dataset);
    reader->LidarReader_Read(true,1,dataset); 
    LASColorExt color;
    color.Red   = 255;
    color.Green = 255;
    color.Blue  = 0;
    classifyElectricPatrolFast classifyFast;
    //EXPECT_EQ(-1,classifyFast.ElectricPatrolFast_Tower(nullptr,pntTower[0],13,color));
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Tower(dataset,pntTower[0],13,color));
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Tower(dataset,pntTower[1],13,color));
    reader->LidarReader_Write("../data/Tower.las",dataset,elcTowerUp);

    color.Red   = 0;
    color.Green = 0;
    color.Blue  = 255;
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Lines(dataset,pntTower,13,5,color));
    reader->LidarReader_Write("../data/Line.las",dataset,elcLine);

    color.Red   = 218;
    color.Green = 165;
    color.Blue  = 32;
    EXPECT_EQ(0,classifyFast.ElectricPatrolFast_Ground(dataset,5,0.5,10,color));
    reader->LidarReader_Write("../data/Ground.las",dataset,elcGround);

    reader->LidarReader_Write("../data/classify_tower_line.las",dataset);
}