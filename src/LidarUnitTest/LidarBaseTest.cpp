#include "../LidarBase/LASHeader.h"
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

