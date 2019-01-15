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
	EXPECT_EQ(0 ,FileHelper::listFilesIncludeSubDir("/home/frank/Downloads/"));	
	EXPECT_EQ(-2 ,FileHelper::listFilesIncludeSubDir("/home/frank/Download"));
}

TEST_F(FileHelperTest,ListFiles)
{
	vector<string> filenameLists;
	EXPECT_EQ(0 ,FileHelper::listFiles("/home/frank/Downloads/",filenameLists));
	EXPECT_EQ(8 ,filenameLists.size());
	EXPECT_EQ(-2 ,FileHelper::listFiles("/home/frank/Dwonload/",filenameLists));
	EXPECT_EQ(8 ,filenameLists.size());
}

TEST_F(FileHelperTest,ListFilesExt)
{
	vector<string> filenameLists;
	EXPECT_EQ(0 ,FileHelper::listFiles("/home/frank/Downloads/",filenameLists,".slpk"));
	EXPECT_EQ(2 ,filenameLists.size());
	EXPECT_EQ(-2 ,FileHelper::listFiles("/home/frank/Dwonload/",filenameLists,".slpk"));
	EXPECT_EQ(2 ,filenameLists.size());

}
