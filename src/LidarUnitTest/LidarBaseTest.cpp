#include "../LidarBase/LASHeader.h"
#include "../LidarBase/LASReader.h"
#include "../LidarBase/LASPoint.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <gtest/gtest.h>
using namespace std;

TEST(LASHEADER,LasHeaderTestCase)
{
    LASHeader lasHd;
    EXPECT_EQ(0, lasHd.number_of_point_records);
    LASHeader *ptrLasHd = new LASHeader();
    EXPECT_EQ(0, lasHd.number_of_point_records);
}

TEST(LASREADER, LasPointReaderCase1)
{
    ILASDataset *lasdst = new ILASDataset();
    LASReader *reader1 = new LidarMemReader();
    EXPECT_EQ(-1,reader1->LidarReader_Open("",lasdst));
    EXPECT_EQ(-1,reader1->LidarReader_Read(true,1,lasdst));
    EXPECT_EQ(-1,reader1->LidarReader_Write("",lasdst));
    EXPECT_EQ(-1,reader1->LidarReader_Write("/home/frank/test.las",lasdst));
    delete lasdst;lasdst=nullptr;
}

TEST(LASREADER, LasPointReaderCase2)
{
    ILASDataset *lasdst = new ILASDataset();
    LidarMemReader *reader2 = new LidarMemReader();
    EXPECT_EQ(0,reader2->LidarReader_Open("../data/CEDD_Building.las",lasdst));
    EXPECT_EQ(0,reader2->LidarReader_Read(true,1,lasdst));
    EXPECT_EQ(-1,reader2->LidarReader_Write("",lasdst));
    EXPECT_EQ(0,reader2->LidarReader_Write("../data/test.las",lasdst));
    EXPECT_EQ(0,reader2->LidarReader_Write("../data/testclassify.las",lasdst,elcCreated));
    LASColorExt colorExt;
    colorExt.Red=255;
    colorExt.Green=0;
    colorExt.Blue=0;
    EXPECT_EQ(0,reader2->LidarReader_WriteWithColor("../data/testclassifycolor.las",lasdst,colorExt));
    EXPECT_EQ(0,reader2->LidarReader_Export("../data/test.txt",lasdst,0));
    delete lasdst;lasdst=nullptr;
}

TEST(LASREADER, LasPointReaderCase3)
{
    ILASDataset *lasdst = new ILASDataset();
    LASReader *reader3 = new LidarMemReader();
    EXPECT_EQ(-1,reader3->LidarReader_Open("",lasdst));
    EXPECT_EQ(-1,reader3->LidarReader_Read(false,1,lasdst));
    delete lasdst;lasdst=nullptr;
}

TEST(LASREADER, LasPointReaderCase4)
{
    ILASDataset *lasdst = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    EXPECT_EQ(0,reader4->LidarReader_Open("../data/CEDD_Building.las",lasdst));
    EXPECT_EQ(0,reader4->LidarReader_Read(false,1,lasdst));
    delete lasdst;lasdst=nullptr;
}