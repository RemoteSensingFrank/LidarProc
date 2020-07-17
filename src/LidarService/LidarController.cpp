/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-12-02 00:15:10
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-07-11 09:54:04
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
	printf("http://106.52.133.33/:%d\n",port);

	//the max parallel connection count
	set_keep_alive_max_count(500);
	set_base_dir("./www");
    listen("http://106.52.133.33/", port);
}

void LidarService::LidarService_Register(LidarController* controller)
{
	controllerList.push_back(controller);
}