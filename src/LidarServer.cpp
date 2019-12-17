/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-12-04 15:10:02
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2019-12-17 14:53:11
 */
// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include"./LidarService/LidarController.h"
// #include <iostream>
// #include <fstream>
// using namespace httplib;
// using namespace std;

// const char* html = R"(
// <form id="formElem">
//   Name: <input type="text" name="name" value="John">
//   File: <input type="file" name="file" accept="image/*">
//   <input type="submit">
// </form>
// <script>
//   formElem.onsubmit = async (e) => {
//     e.preventDefault();
//     let res = await fetch('/post', {
//       method: 'POST',
//       body: new FormData(formElem)
//     });
//     console.log(await res.text());
//   };
// </script>
// )";

// int main(void) {
//   Server svr;

//   svr.Get("/", [](const Request & /*req*/, Response &res) {
//     res.set_content(html, "text/html");
//   });

//   svr.Post("/post", [](const Request & req, Response &res) {
//     auto name = req.get_file_value("name");
//     auto file = req.get_file_value("file");

//     cout << "name: " << req.body.substr(name.offset, name.length) << endl;
//     cout << "file: " << file.offset << ":" << file.length << ":" << file.filename << endl;

//     ofstream ofs(file.filename, ios::binary);
//     ofs << req.body.substr(file.offset, file.length);

//     res.set_content("done", "text/plain");
//   });

//   cout << "start..." << endl;
//   svr.listen("localhost", 1234);
// }

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
	LidarControllerClassfication *lidarControllerClass = new LidarControllerClassfication(lidarService);
	LidarControllerUpload *lidarControllerUploads=new LidarControllerUpload(lidarService);
	LidarControllerUploadFinished *lidarControllerUploadFinish=new LidarControllerUploadFinished(lidarService);

	lidarService->LidarService_Register(lidarControllerInfo);
	lidarService->LidarService_Register(lidarControllerExhibit);
	lidarService->LidarService_Register(lidarControllerDatalist);
	lidarService->LidarService_Register(lidarControllerDatadelete);
	lidarService->LidarService_Register(lidarControllerExhibitdelete);
	lidarService->LidarService_Register(lidarControllerDatatrans);
	lidarService->LidarService_Register(lidarControllerDataclasstype);
	lidarService->LidarService_Register(lidarControllerDoc);
	lidarService->LidarService_Register(lidarControllerUploads);
	lidarService->LidarService_Register(lidarControllerUploadFinish);
	lidarService->LidarService_Register(lidarControllerClass);
	
	lidarService->LidarService_Run();
}
