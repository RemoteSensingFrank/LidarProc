#include <stdio.h>
#include <vector>
#include <map>
#include <string>
#include <exception>
#include <iterator>

//httplib from https://github.com/yhirose/cpp-httplib.git
#include "httplib.h"


#include "../LidarUtil/FileHelper.h"
//json parase code from https://github.com/Nomango/jsonxx.git
#include "../LidarUtil/json.hpp"

#include "../LidarBase/LASReceive.h"
#include "../LidarBase/LASReader.h"
#include "../LidarResearch/LASFormatTransform.h"
#include "../LidarAlgorithm/LASSimpleClassify.h"
#include "../LidarAlgorithm/LASDangerPoints.h"
#include "../LidarAlgorithm/LASSkeleton.h"


using namespace std;
using namespace httplib;
using namespace jsonxx;

class LidarService;

/**
* 字符串分割
* @param const string& s 带分割的字符串
* @param vector<string>& result	分割结果
* @param const string seperator	分割字符
*/
void static SplitString(const string& s, vector<string>& result, const string seperator)
{
	typedef string::size_type string_size;
	string_size i = 0;

	while (i != s.size()) {
		//找到字符串中首个不等于分隔符的字母；
		int flag = 0;
		while (i != s.size() && flag == 0) {
			flag = 1;
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[i] == seperator[x]) {
					++i;
					flag = 0;
					break;
				}
		}

		//找到又一个分隔符，将两个分隔符之间的字符串取出；
		flag = 0;
		string_size j = i;
		while (j != s.size() && flag == 0) {
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[j] == seperator[x]) {
					flag = 1;
					break;
				}
			if (flag == 0)
				++j;
		}
		if (i != j) {
			result.push_back(s.substr(i, j - i));
			i = j;
		}
	}
}



class LidarController
{
public:
    LidarController(LidarService *tService){
        service = tService;
    }

    virtual void LidarController_Run()=0;

protected:
    LidarService* service;
};


/**
 * @brief  注册并运行服务
 * @note   
 * @retval None
 */
class LidarService : public Server
{
public:  
    /**
    * @brief  运行服务
    * @note   
    * @retval None
    */
    void LidarService_Run();


    /**
    * @brief  将controller进行注册
    * @note   
    * @param  controller: 
    * @retval None
    */
    void LidarService_Register(LidarController* controller);

private:  
     vector<LidarController*> controllerList;
};


/** API都是继承自Controller 然后在Service中进行注册，完成注册后运行
    通过注册的方式能够动态的添加controller 而不需要将所有的controller
    写到同一个函数中，能够方便进行扩展
 * @brief  
 * @note   
 * @retval None
 */

/**
 * @brief  系统信息展示接口
 * @note   
 * @retval None
 */
class LidarControllerInfo:public LidarController
{
public:
    LidarControllerInfo(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get("/info", [](const Request& req, Response& res) {
            json infoJson;
            ifstream incpu("/proc/cpuinfo");
            istreambuf_iterator<char> cpu_begin(incpu);
            istreambuf_iterator<char> cpu_end;
            string cpu_str(cpu_begin, cpu_end);

            ifstream inmem("/proc/cpuinfo");
            istreambuf_iterator<char> mem_begin(inmem);
            istreambuf_iterator<char> mem_end;
            string mem_str(mem_begin, mem_end);

            infoJson["cpuinfo"]=cpu_str;
            infoJson["memInfo"]=mem_str;

            res.set_content(infoJson.dump(), "application/json");
        });        
    }    
};

/**
 * @brief  展示所有能够查看的点云数据接口
 * @note   
 * @retval None
 */
class LidarControllerExhibit:public LidarController
{
public:
    LidarControllerExhibit(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get("/exhibitlist", [](const Request& req, Response& res){
            vector<string> namelist;
            FileHelper::listDirNames("./www/pointclouds/",namelist);
            json listFileJson;
            listFileJson["filecount"]=namelist.size();
            listFileJson["fileList"]=json::array({});
            for(int i=0;i<namelist.size();++i)
            {
                listFileJson["fileList"].push_back({{"filename",namelist[i]}});
            }
                
            res.set_content(listFileJson.dump(), "application/json");
            res.status=200;
	    });        
    }    
};

/**
 * @brief  列出所有可处理数据接口
 * @note   
 * @retval None
 */
class LidarControllerDatalist:public LidarController
{
public:
    LidarControllerDatalist(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get("/datalist", [](const Request& req, Response& res){
            string jstr="";
            vector<string> dirs;
            FileHelper::listDirNames("../data/",dirs);
            json dirsJson;
            dirsJson["dircount"]=dirs.size();
            dirsJson["dirobjects"]=json::array({});

            for(int i=0;i<dirs.size();++i)
            {
                std::vector<std::string> filelist;
                FileHelper::listNames(string("../data/"+dirs[i]+"/"),filelist,"las");
                json dirJson;
                dirJson["filecount"]=filelist.size();
                dirJson["dirname"] = dirs[i];
                dirJson["fileobjects"]=json::array({});

                for(int j=0;j<filelist.size();++j)
                {
                    dirJson["fileobjects"].push_back({{"name",filelist[j]}});
                }
                dirsJson["dirobjects"].push_back({{"dir",dirJson}});
            }
            res.set_content(dirsJson.dump(), "application/json");
            res.status=200;
        });        
    }    
};


/**
 * @brief  删除处理数据
 * @note   
 * @retval None
 */
class LidarControllerDatadelete:public LidarController
{
public:
    LidarControllerDatadelete(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get(R"(/datadelete/(.*?))",[](const Request& req, Response& res){
            string path="../data/"+string(req.matches[1]);
            remove(path.c_str());
            json delJson=json::object({"delete","success"});
            res.set_content(delJson.dump(), "application/json");
            res.status=200;
        });       
    }    
};

/**
 * @brief  删除展示数据
 * @note   
 * @retval None
 */
class LidarControllerExhibitdelete:public LidarController
{
public:
    LidarControllerExhibitdelete(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get(R"(/exhibitdelete/(.*?))",[](const Request& req, Response& res){
            string path="rm -rf ./www/pointclouds/"+string(req.matches[1])+"/";
            system(path.c_str());
            json delJson=json::object({"delete","success"});
            res.set_content(delJson.dump(), "application/json");
            res.status=200;
        });      
    }    
};


/**
 * @brief  LAS点云数据数据转换为展示数据
 * @note   
 * @retval None
 */
class LidarControllerDatatrans:public LidarController
{
public:
    LidarControllerDatatrans(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get(R"(/datatrans/(.*?))",[](const Request& req, Response& res){
            string path="../data/"+string(req.matches[1]);
            LASTransToPotree lasTrans;
            lasTrans.LASTransToPotree_Trans(path.c_str());
            json lasPathJson=json::object({"path",path});
            res.set_content(lasPathJson.dump(), "application/json");
            res.status=200;
        });    
    }    
};


/**
 * @brief  获取点云分类类型
 * @note   
 * @retval None
 */
class LidarControllerDataclasstype:public LidarController
{
public:
    LidarControllerDataclasstype(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get("/dataclasstype",[](const Request& req, Response& res){
            FILE *fs = fopen("./config/classtype.conf","r+");
            char line[2048];
            json typeJson=json::array({});
            while(fgets (line, 2048, fs)!=NULL)
            {
                vector<string> lineItems;
                SplitString(string(line),lineItems,",");
                typeJson.push_back({{lineItems[0],lineItems[1]}});
            }
            fclose(fs);
            res.set_content(typeJson.dump(), "application/json");
            res.status=200;
        });   
    }    
};

/**
 * @brief  文件分块上传接口的文件传输部分
 * @note   
 * @retval None
 */
class LidarControllerUpload:public LidarController
{
public:
    LidarControllerUpload(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Post("/upload",[](const Request& req, Response& res){
            auto name = req.get_file_value("name");
            string nameStr = req.body.substr(name.offset, name.length);
            auto task = req.get_file_value("task_id"); 
            string taskStr = req.body.substr(task.offset, task.length);   
            auto size = req.get_file_value("size"); 
            string sizeStr = req.body.substr(size.offset, size.length);     
            auto file = req.get_file_value("file");

            int chunk=0;
            string chunkStr="0";
            if(req.has_file("chunk"))
            {
                auto chunks = req.get_file_value("chunk");
                chunkStr = req.body.substr(chunks.offset, chunks.length);  
                chunk = atoi(chunkStr.c_str());
            }
            string fileStr = req.body.substr(file.offset, file.length);
            FILE *f;
            
            //创建文件夹
            string createdir="mkdir ../data/tmp/"+taskStr;
            system(createdir.c_str());

            string path="../data/tmp/"+taskStr+"/"+taskStr+"_"+chunkStr;
            f=fopen(path.c_str(),"wb+");
            fwrite(fileStr.c_str(),1,file.length,f);
            fclose(f);
            res.set_content("upload chunk success", "text/plain;charset=utf-8");
            res.status=200;
        });  
    }   
};

//完成文件上传（必须从开始到完成一整个流程才算结束）
class LidarControllerUploadFinished:public LidarController
{
public:    
    LidarControllerUploadFinished(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Post("/upload-finish",[](const Request& req, Response& res){
            vector<string> splitStr;
            string params = req.body.c_str();
            SplitString(params,splitStr,",");
            FILE *f;
            string path="../data/tmp/"+splitStr[1];
            f=fopen(path.c_str(),"wb+");
            int i=0;
            bool fileExist=false;
            do{
                FILE *fi = fopen(string("../data/tmp/"+splitStr[0]+"/"+splitStr[0]+"_"+to_string(i)).c_str(),"rb");
                if(fi!=nullptr)
                {
                    fileExist=true;
                    fseek(fi,0,SEEK_END);
                    int n=ftell(fi);
                    fseek(fi,0,SEEK_SET);
                    char* data = new char[n];
                    fread(data,n,sizeof(char),fi);
                    fwrite(data,n,sizeof(char),f);
                    fflush(f);
                    delete[]data;data=nullptr;
                    fclose(fi);fi=nullptr;
                    ++i;
                }else{
                    fileExist=false;
                    break;
                }
            }while(true);
            fclose(f);f=nullptr;
            string cmd="rm -rf ../data/tmp/"+splitStr[0];
            system(cmd.c_str());
            res.set_content("upload success", "text/plain;charset=utf-8");
            res.status=200;
        });  
    }   
};

/**
 * @name: 点云简单分类算法
 * @msg: 
 * @param {type} 
 * @return: 
 */
class LidarControllerClassfication:public LidarController
{
public:
    LidarControllerClassfication(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Post(R"(/classification)",[](const Request& req, Response& res){
            string params = req.body.c_str();
            json paramsJson;
            paramsJson=json::parse(params);
            std::vector<json> pnts = paramsJson["points"];
            double towerRange = paramsJson["towerRange"];
            double lineHeight = paramsJson["lineHeight"];
            double groundNetSize = paramsJson["groundNetSize"];
            double groundNetAngle = paramsJson["groundNetAngle"];
            double vegeDistance = paramsJson["vegeDistance"];
            string classifiedName = paramsJson["classifiedName"];
            string dirFileName = paramsJson["dirFileName"];
            string srcFileName = paramsJson["srcFileName"];
            ILASDataset *lasdst1 = new ILASDataset();
            LASReader *reader4 = new LidarMemReader();
            string pathsrc=string("../data/")+dirFileName+string("/")+srcFileName;
            reader4->LidarReader_Open(pathsrc.c_str(),lasdst1);
            reader4->LidarReader_Read(true,1,lasdst1);
            printf("1");
            classifyElectricPatrolFast classFast;
            Point2Ds points;
            LASColorExt colorTower;
            colorTower.Red=colorTower.Blue=colorTower.Green=255;
            
            points.push_back(Point2D(pnts[0]["x"],pnts[0]["y"]));
            points.push_back(Point2D(pnts[1]["x"],pnts[1]["y"]));
            classFast.ElectricPatrolFast_Tower(lasdst1,points,towerRange,colorTower);
            printf("2");
            LASColorExt lineTower;
            lineTower.Red=lineTower.Green=0;lineTower.Blue=255;
            long err=classFast.ElectricPatrolFast_Lines(lasdst1,points,towerRange,lineHeight,lineTower);
            printf("3");
            LASColorExt groundTower;
            groundTower.Red=groundTower.Green=255;groundTower.Blue=0;
            classFast.ElectricPatrolFast_Ground(lasdst1,groundNetSize,groundNetAngle,vegeDistance,groundTower);
            printf("4");
            LASColorExt vegetationTower;
            vegetationTower.Red=vegetationTower.Blue=0;vegetationTower.Green=255;
            classFast.ElectricPatrolFast_VegetationLast(lasdst1,vegetationTower);
            printf("5");

            string classifiedpath="../data/default/"+classifiedName;
            reader4->LidarReader_Write(classifiedpath.c_str(),lasdst1);

            delete lasdst1;
            delete reader4;

            json procJson=json::object({"process","classified"});
            res.set_content(procJson.dump(), "application/json");
            res.status=200;
        });      
    }  
};


/**
 * @name:  模型精化
 * @msg: 
 * @param {type} 
 * @return: 
 */

/**
 * @name: LidarControllerLineModelRefineProc
 * @msg: 
 * @param {*}
 * @return {*}
 */
static json LidarControllerLineModelRefineProc(json featureJson)
{
    std::vector<json> features=featureJson["features"];

    vector<Point3Ds> line3ds;
    for(int i=0;i<features.size();++i)
    {
        Point3Ds line;
        std::vector<json> coordinates = features[i]["geometry"]["coordinates"];
        for(int j=0;j<coordinates.size();++j)
        {
            Point3D pnt3d;
            int k=0;
            for (auto& tmp : coordinates[j]) 
            {
                switch (k)
                {
                case 0:
                    if(tmp.is_integer()){
                        int tmpInt=0;
                        tmpInt=tmp;
                        pnt3d.x = (double)tmpInt;
                    }else{
                        pnt3d.x = (double)tmp;
                    }
                    k++;
                    break;
                case 1:
                    if(tmp.is_integer()){
                        int tmpInt=0;
                        tmpInt=tmp;
                        pnt3d.y = (double)tmpInt;
                    }else{
                        pnt3d.y = (double)tmp;
                    }
                    k++;
                    break;
                case 2:
                    if(tmp.is_integer()){
                        int tmpInt=0;
                        tmpInt=tmp;
                        pnt3d.z = (double)tmpInt;
                    }else{
                        pnt3d.z = (double)tmp;
                    }
                    k++;
                    break;
                default:
                    break;
                }
                
            }
            line.push_back(pnt3d);
        }
        line3ds.push_back(line);
    }
    //for test
    ILASDataset *lasdst1 = new ILASDataset();
    LASReader *reader4 = new LidarMemReader();
    reader4->LidarReader_Open("../data/default/more.las",lasdst1);
    reader4->LidarReader_Read(true,1,lasdst1);
    LasAlgorithm::PointCloudLineInteractiveSimple lineInteractive;

    lineInteractive.PointCloudLineInteractive_ModelRefine(lasdst1,line3ds);

    json featureLines;
    featureLines["featurelines"]=json::array({});
    for(int i=0;i<line3ds.size();++i)
    {
        json lineJson;
        lineJson=json::array({});
        for(int j=0;j<line3ds[i].size();++j)
        {
            json pointJson;
            pointJson["x"]=line3ds[i][j].x;
            pointJson["y"]=line3ds[i][j].y;
            pointJson["z"]=line3ds[i][j].z;
            lineJson.push_back({{"point",pointJson}});
        }
        featureLines["featurelines"].push_back({{"line",lineJson}});
    }
    return featureLines;
}

class LidarControllerLineModelRefine:public LidarController
{
public:
    LidarControllerLineModelRefine(LidarService* tService):LidarController(tService){}


    virtual void LidarController_Run()
    {
        service->Post(R"(/line_model_refine)",[](const Request& req, Response& res){

            string params = req.body.c_str();
            json paramsJson;
            paramsJson=json::parse(params);
            json featureResult = LidarControllerLineModelRefineProc(paramsJson);
            res.set_content(featureResult.dump(), "application/json");
            res.status=200;
        });      
    }  
};



/**
 * @brief  查看API文档-手动维护
 * @note   
 * @retval None
 */
class LidarControllerDoc:public LidarController
{
public:
    LidarControllerDoc(LidarService* tService):LidarController(tService){}

    virtual void LidarController_Run()
    {
        service->Get("/apidoc",[](const Request& req, Response& res){

            json apiListInfo=json::array({});

            json apiInfoJson;
            apiInfoJson["api"]="http://localhost:1234/info";
            apiInfoJson["params"]="null";
            apiInfoJson["paramType"]="";
            apiInfoJson["requestType"]="GET";
            apiInfoJson["Desc"]="查看系统信息";
            apiListInfo.push_back({{"apiInfo",apiInfoJson}});

            json apiExhibitJson;
            apiExhibitJson["api"]="http://localhost:1234/exhibitlist";
            apiExhibitJson["params"]="null";
            apiExhibitJson["paramType"]="";
            apiExhibitJson["requestType"]="GET";
            apiExhibitJson["Desc"]="列出所有可展示数据文件夹";
            apiListInfo.push_back({{"exhibitInfo",apiExhibitJson}});
            
            json apiDataJson;
            apiDataJson["api"]="http://localhost:1234/datalist";
            apiDataJson["params"]="null";
            apiDataJson["paramType"]="";
            apiDataJson["requestType"]="GET";
            apiDataJson["Desc"]="列出所有可处理数据";
            apiListInfo.push_back({{"dataInfo",apiDataJson}});
            
            json datadeleteJson;
            datadeleteJson["api"]="http://localhost:1234/datadelete";
            datadeleteJson["params"]="待删除数据文件名（根据列出数据文件获取";
            datadeleteJson["paramType"]="url参数";
            datadeleteJson["requestType"]="GET";
            datadeleteJson["Desc"]="删除处理数据";
            apiListInfo.push_back({{"datadelete",datadeleteJson}});            
            

            string apiExhibitdelete="Api:http://localhost:1234/exhibitdelete\nParams:待删除数据文件夹（根据列出数据文件获取）\nParamType:url参数\nRequestType:GET\nDesc:删除展示数据\n\n";
            json exhibitdeleteJson;
            exhibitdeleteJson["api"]="http://localhost:1234/exhibitdelete";
            exhibitdeleteJson["params"]="待删除数据文件名（根据列出数据文件获取";
            exhibitdeleteJson["paramType"]="url参数";
            exhibitdeleteJson["requestType"]="GET";
            exhibitdeleteJson["Desc"]="删除展示数据";
            apiListInfo.push_back({{"deleteexhibit",exhibitdeleteJson}});            
            
            json dataTransJson;
            dataTransJson["api"]="http://localhost:1234/datatrans";
            dataTransJson["params"]="待转换数据文件名";
            dataTransJson["paramType"]="url参数";
            dataTransJson["requestType"]="GET";
            dataTransJson["Desc"]="将处理数据转换为展示数据";
            apiListInfo.push_back({{"dataTrans",dataTransJson}});                
            
            
            string apiDataclasstype="Api:http://localhost:1234/dataclasstype\nParams:null\nParamType:\nRequestType:GET\nDesc:将处理数据转换为展示数据\n\n";
            json classTypeJson;
            classTypeJson["api"]="http://localhost:1234/dataclasstype";
            classTypeJson["params"]="null";
            classTypeJson["paramType"]="";
            classTypeJson["requestType"]="GET";
            classTypeJson["Desc"]="获取分类展示类型";
            apiListInfo.push_back({{"classType",classTypeJson}});              
            
            res.set_content(apiListInfo.dump(), "application/json;charset=utf-8");
            res.status=200;
        });  
    }   
};
