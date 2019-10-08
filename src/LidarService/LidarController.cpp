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