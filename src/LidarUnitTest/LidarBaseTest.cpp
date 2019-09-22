#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <random>
#include <gtest/gtest.h>

#include "../LidarBase/LASHeader.h"
#include "../LidarBase/LASReader.h"
#include "../LidarBase/LASPoint.h"

using namespace std;

/**
 * @brief  测试代码-针对所有基础数据进行测试
 *         根据单元测试运行条件，单元测试应
 *         独立与系统能够自动运行并检测系统正确性
 *         因此应该独立于数据单独运行
 * @note   
 * @retval None
 * @atuhor Frank.W.W
 * @date: 2019-09-21
 */
class LidarBaseTest : public testing::Test
{
protected:
	static void SetUpTestCase()
	{
		printf("Lidar Base Operation test startup\n");
        printf("Create test file\n");

        //if the test directory does not exist
        if(access("./test/", 0) == -1){
		    mkdir("./test/",S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH);	
        }

        //create test dataset
        LASHeader lasHd;
        lasHd.number_of_point_records = 500;
        lasHd.point_data_record_length = 34;
	    lasHd.point_data_format = 3;
	    lasHd.version_major = 1;
	    lasHd.version_minor = 2;
        lasHd.offset_to_point_data=227;
        lasHd.x_scale_factor=lasHd.y_scale_factor=lasHd.z_scale_factor=0.01;
        lasHd.x_offset=lasHd.y_offset=lasHd.z_offset=0;
        lasHd.max_x=lasHd.max_y=lasHd.max_z=1;
        lasHd.min_x=lasHd.min_y=lasHd.min_z=0;

        FILE* fLasOut = fopen(m_baseTestFilePath.c_str(), "wb");
        if (fLasOut == nullptr)
        {
            printf("create las file failed!\n");
            return ;
        }
        lasHd.WriteHeader(fLasOut);

        //中间有问题 不知道怎么搞 变长字段还没有进行处理
        int sizeBuff = lasHd.offset_to_point_data - sizeof(LASHeader);
        if (sizeBuff != 0)
        {
            char* buffer = new char[sizeBuff];
            memset(buffer, 0, sizeof(char) * sizeBuff);
            fwrite(buffer, 1, sizeBuff, fLasOut);
            delete[]buffer; buffer = NULL;
        }
        
        //产生随机数引擎C++11 标准
        default_random_engine e; 
        uniform_int_distribution<unsigned> u(0, 100); 
        //写数据
        for (int k = 0; k<500; ++k)
        {
            //printf("\r%d", k);
            LASPoint pnt;
            pnt.m_vec3d.x = u(e);
            pnt.m_vec3d.y = u(e);
            pnt.m_vec3d.z = u(e);
            int x = pnt.m_vec3d.x;
            int y = pnt.m_vec3d.y;
            int z = pnt.m_vec3d.z;

            fwrite(&x, sizeof(int), 1, fLasOut);
            fwrite(&y, sizeof(int), 1, fLasOut);
            fwrite(&z, sizeof(int), 1, fLasOut);

            fwrite(&pnt.m_intensity, sizeof(unsigned short) , 1, fLasOut);
            fwrite(&pnt.m_rnseByte,  sizeof(unsigned char)  , 1, fLasOut);
            fwrite(&pnt.m_classify,  sizeof(unsigned char)  , 1, fLasOut);
            fwrite(&pnt.m_scanAngle, sizeof(unsigned char)  , 1, fLasOut);
            fwrite(&pnt.m_userdata,  sizeof(unsigned char)  , 1, fLasOut);
            fwrite(&pnt.m_flightID,  sizeof(unsigned short) , 1, fLasOut);
            fwrite(&pnt.m_gpsTime,   sizeof(double)         , 1, fLasOut);
            fwrite(&pnt.m_colorExt,  sizeof(LASColorExt)    , 1, fLasOut);
        }
        fclose(fLasOut);
	}

	static void TearDownTestCase()
	{
		printf("file helper function class test end\n");
        remove(m_baseTestFilePath.c_str());
	}

    virtual void SetUp() {}
    virtual void TearDown() {}

    static string m_baseTestFilePath;
};

string LidarBaseTest::m_baseTestFilePath="./test/basetestfile.las";



TEST_F(LidarBaseTest,LasHeaderTestCase0)
{
    LASHeader lasHd;
    EXPECT_EQ(0, lasHd.number_of_point_records);
    LASHeader *ptrLasHd = new LASHeader();
    EXPECT_EQ(0, lasHd.number_of_point_records);
}

TEST_F(LidarBaseTest,LasHeaderTestCase1)
{
    ILASDataset *lasdst = new ILASDataset();
    LASReader *reader1 = new LidarMemReader();
    EXPECT_EQ(-1,reader1->LidarReader_Open("",lasdst));
    EXPECT_EQ(-1,reader1->LidarReader_Read(true,1,lasdst));
    EXPECT_EQ(-1,reader1->LidarReader_Write("",lasdst));
    EXPECT_EQ(-1,reader1->LidarReader_Write("/home/frank/test.las",lasdst));
    delete lasdst;lasdst=nullptr;
}


TEST_F(LidarBaseTest,LasHeaderTestCase2)
{
    ILASDataset *lasdst = new ILASDataset();
    LidarMemReader *reader2 = new LidarMemReader();
    EXPECT_EQ(0,reader2->LidarReader_Open(LidarBaseTest::m_baseTestFilePath.c_str(),lasdst));
    EXPECT_EQ(0,reader2->LidarReader_Read(true,1,lasdst));
    EXPECT_EQ(-1,reader2->LidarReader_Write("",lasdst));
    EXPECT_EQ(0,reader2->LidarReader_Write("./test/write1.las",lasdst));
    EXPECT_EQ(0,reader2->LidarReader_Write("./test/write2.las",lasdst,elcCreated));
    LASColorExt colorExt;
    colorExt.Red=255;
    colorExt.Green=0;
    colorExt.Blue=0;
    EXPECT_EQ(0,reader2->LidarReader_WriteWithColor("./test/write3.las",lasdst,colorExt));
    EXPECT_EQ(0,reader2->LidarReader_Export("./test/write4.txt",lasdst,0));
    delete lasdst;lasdst=nullptr;
}


TEST_F(LidarBaseTest,LasHeaderTestCase3)
{
    ILASDataset *lasdst = new ILASDataset();
    LASReader *reader3 = new LidarMemReader();
    EXPECT_EQ(-1,reader3->LidarReader_Open("",lasdst));
    EXPECT_EQ(-1,reader3->LidarReader_Read(false,1,lasdst));
    delete lasdst;lasdst=nullptr;
}

TEST_F(LidarBaseTest,LasHeaderTestCase4)
{
    ILASDataset *lasdst = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    EXPECT_EQ(0,reader4->LidarReader_Open(LidarBaseTest::m_baseTestFilePath.c_str(),lasdst));
    EXPECT_EQ(0,reader4->LidarReader_Read(false,1,lasdst));
    delete lasdst;lasdst=nullptr;
}