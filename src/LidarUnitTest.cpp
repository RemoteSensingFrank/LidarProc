/**
 * @brief  运行单元测试，在代码编译完成后直接运行单元测试对代码进行测试
 * @note   
 * @param  argc: 
 * @param  argv[]: 
 * @retval 
 */
#include<stdio.h>
#include<gtest/gtest.h>
int main(int argc ,char* argv[])
{
    testing::InitGoogleTest(&argc,argv);
    testing::GTEST_FLAG(filter) = "LidarBaseTest*:FileHelperTest*:LASSKELETON*";
	return RUN_ALL_TESTS();  
}

