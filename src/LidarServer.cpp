// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include"./LidarService/LidarController.h"

int main(int argc, char **argv)
{
    LidarService *lidarService = new LidarService();
	LidarControllerInfo *lidarControllerInfo = new LidarControllerInfo(lidarService);
	LidarControllerExhibit *lidarControllerExhibit = new LidarControllerExhibit(lidarService);
	LidarControllerDatalist *lidarControllerDatalist = new LidarControllerDatalist(lidarService);
	LidarControllerDatadelete *lidarControllerDatadelete = new LidarControllerDatadelete(lidarService);
	LidarControllerExhibitdelete *lidarControllerExhibitdelete = new LidarControllerExhibitdelete(lidarService);
	LidarControllerDatatrans *lidarControllerDatatrans = new LidarControllerDatatrans(lidarService);
	LidarControllerDataclasstype *lidarControllerDataclasstype = new LidarControllerDataclasstype(lidarService);
	LidarControllerDoc *lidarControllerDoc = new LidarControllerDoc(lidarService);
	lidarService->LidarService_Register(lidarControllerInfo);
	lidarService->LidarService_Register(lidarControllerExhibit);
	lidarService->LidarService_Register(lidarControllerDatalist);
	lidarService->LidarService_Register(lidarControllerDatadelete);
	lidarService->LidarService_Register(lidarControllerExhibitdelete);
	lidarService->LidarService_Register(lidarControllerDatatrans);
	lidarService->LidarService_Register(lidarControllerDataclasstype);
	lidarService->LidarService_Register(lidarControllerDoc);
	lidarService->LidarService_Run();
}
