#include "../LidarBase/LASHeader.h"
#include "../LidarBase/LASReader.h"
#include "../LidarBase/LASPoint.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <gtest/gtest.h>
using namespace std;

TEST(LASHEADER LasHeaderTestCase)
{
    LASHeader lasHd;
    EXPECT_EQ(0 lasHd.number_of_point_records);
    LASHeader *ptrLasHd = new LASHeader();
    EXPECT_EQ(0 lasHd.number_of_point_records);
}

TEST(LASREADER LasPointREADERCase)
{
    LASPoint lasPoint;
    LASReader *reader = new LidarMemReader();
    ILASDataset *lasdst = new ILASDataset();

    EXPECT_EQ(-1,reader->LidarReader_Open("",lasdst));
    EXPECT_EQ(-1,reader->LidarReader_Read(true,1,lasdst));
    EXPECT_EQ(-1,reader->LidarReader_Write("",lasdst));
    EXPECT_EQ(-1,reader->LidarReader_Write("../../data/testwrite.las",lasdst));

    EXPECT_EQ(0,reader->LidarReader_Open("",lasdst));
    EXPECT_EQ(0,reader->LidarReader_Read(true,1,lasdst));
    EXPECT_EQ(-1,reader->LidarReader_Write("",lasdst));
    EXPECT_EQ(0,reader->LidarReader_Write("../../data/testwrite.las",lasdst));


}
