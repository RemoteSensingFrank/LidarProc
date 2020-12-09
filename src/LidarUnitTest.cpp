/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-12-02 00:15:10
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-11-25 14:59:36
 */
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
    testing::GTEST_FLAG(filter) = "LASSKELETONLineInteractive*";
	return RUN_ALL_TESTS();  
}

