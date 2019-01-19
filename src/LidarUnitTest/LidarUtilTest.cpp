#include "../LidarUtil/FileHelper.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <gtest/gtest.h>
using namespace std;

/*
Testing class for file helper class
*/
class FileHelperTest : public testing::Test
{
protected:
	virtual void SetUp()
	{
		printf("file helper funcion class test startup\n");
	}

	virtual void TearDown()
	{
		printf("file helper function class test end\n");
	}
};

TEST_F(FileHelperTest,PrintFilesIncludeSubDirs)
{
	EXPECT_EQ(0 ,FileHelper::listFilesIncludeSubDir("../src/LidarBase/"));	
	EXPECT_EQ(-2 ,FileHelper::listFilesIncludeSubDir("../LidarFault/"));
}

TEST_F(FileHelperTest,ListFiles)
{
	vector<string> filenameLists;
	EXPECT_EQ(0 ,FileHelper::listFiles("../src/LidarBase/",filenameLists));
	EXPECT_EQ(7 ,filenameLists.size());
	EXPECT_EQ(-2 ,FileHelper::listFiles("../LidarFault/",filenameLists));
	EXPECT_EQ(7 ,filenameLists.size());
}

TEST_F(FileHelperTest,ListFilesExt)
{
	vector<string> filenameLists;
	EXPECT_EQ(0 ,FileHelper::listFiles("../src/LidarBase/",filenameLists,"h"));
	EXPECT_EQ(3 ,filenameLists.size());
	EXPECT_EQ(-2 ,FileHelper::listFiles("../LidarFault/",filenameLists,"h"));
	EXPECT_EQ(3 ,filenameLists.size());
}
