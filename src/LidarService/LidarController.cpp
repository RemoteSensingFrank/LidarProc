/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-12-02 00:15:10
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2019-12-14 12:13:57
 */
#include "LidarController.h"

using namespace rapidjson;
using namespace httplib;


void LidarService::LidarService_Run()
{
	for(int i=0;i<controllerList.size();++i){
		controllerList[i]->LidarController_Run();
	}
	
	int port=1234;
	printf("localhost:%d\n",port);

	//the max parallel connection count
	set_keep_alive_max_count(500);
	set_base_dir("./www");
    listen("localhost", port);
}

void LidarService::LidarService_Register(LidarController* controller)
{
	controllerList.push_back(controller);
}