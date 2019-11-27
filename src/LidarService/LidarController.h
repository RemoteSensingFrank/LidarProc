#include <stdio.h>
#include <vector>
#include <map>
#include <string>
#include <exception>
#include <fstream>
#include <iterator>

#include "httplib.h"
#include "../LidarUtil/FileHelper.h"
#include "../LidarBase/LASReceive.h"
#include "../LidarBase/LASReader.h"
#include "../LidarResearch/LASFormatTransform.h"

using namespace std;
using namespace httplib;


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
            ifstream incpu("/proc/cpuinfo");
            istreambuf_iterator<char> cpu_begin(incpu);
            istreambuf_iterator<char> cpu_end;
            string cpu_str(cpu_begin, cpu_end);

            ifstream inmem("/proc/cpuinfo");
            istreambuf_iterator<char> mem_begin(inmem);
            istreambuf_iterator<char> mem_end;
            string mem_str(mem_begin, mem_end);

            string info = cpu_str+"\n"+mem_str;

            res.set_content(info, "text/plain");
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
            string dirNameList="";
            vector<string> namelist;
            FileHelper::listDirNames("./www/pointclouds/",namelist);
            for(int i=0;i<namelist.size();++i)
                dirNameList+=namelist[i]+";";
            res.set_content(dirNameList, "text/plain");
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
            for(int i=0;i<dirs.size();++i)
            {
                jstr+=","+dirs[i]+";";
                vector<string> filelist;
                FileHelper::listNames(string("../data/"+dirs[i]+"/"),filelist,"las");
                for(int j=0;j<filelist.size();++j)
                {
                    jstr+=filelist[j]+";";
                }
            }
            res.set_content(jstr, "text/plain");
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
            res.set_content("deleted", "text/plain");
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
            res.set_content("deleted", "text/plain");
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
            res.set_content("transed", "text/plain");
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
            string jstr="";
            while(fgets (line, 2048, fs)!=NULL)
            {
                vector<string> lineItems;
                SplitString(string(line),lineItems,",");
                jstr+=lineItems[0]+","+lineItems[1]+";";
            }
            fclose(fs);
            res.set_content(jstr, "text/plain");
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
            string pageInfo="";
            string apiDoc = "";
            string apiInfo="Api:http://localhost:1234/info\nParams:null\nParamType:null\nRequestType:GET\nDesc:查看系统信息\n\n";
            pageInfo+=apiInfo;
            string apiExhibitlist="Api:http://localhost:1234/exhibitlist\nParams:null\nParamType:null\nRequestType:GET\nDesc:列出所有可展示数据文件夹\n\n";
            pageInfo+=apiExhibitlist;
            string apiDatalist="Api:http://localhost:1234/datalist\nParams:null\nParamType:null\nRequestType:GET\nDesc:列出所有可处理数据\n\n";
            pageInfo+=apiDatalist;
            string apiDatadelete="Api:http://localhost:1234/datadelete\nParams:待删除数据文件名（根据列出数据文件获取）\nParamType:url参数\nRequestType:GET\nDesc:删除处理数据\n\n";
            pageInfo+=apiDatadelete;
            string apiExhibitdelete="Api:http://localhost:1234/exhibitdelete\nParams:待删除数据文件夹（根据列出数据文件获取）\nParamType:url参数\nRequestType:GET\nDesc:删除展示数据\n\n";
            pageInfo+=apiExhibitdelete;
            string apiDatatrans="Api:http://localhost:1234/datatrans\nParams:待转换数据文件名\nParamType:url参数\nRequestType:GET\nDesc:将处理数据转换为展示数据\n\n";
            pageInfo+=apiDatatrans;
            string apiDataclasstype="Api:http://localhost:1234/dataclasstype\nParams:null\nParamType:\nRequestType:GET\nDesc:将处理数据转换为展示数据\n\n";
            pageInfo+=apiDataclasstype;
            string doc=pageInfo;
            res.set_content(doc, "text/plain;charset=utf-8");
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
            MultipartFile name = req.get_file_value("name");
            string nameStr = req.body.substr(name.offset, name.length);
            MultipartFile task = req.get_file_value("task_id"); 
            string taskStr = req.body.substr(task.offset, task.length);   
            MultipartFile size = req.get_file_value("size"); 
            string sizeStr = req.body.substr(size.offset, size.length);     
            MultipartFile file = req.get_file_value("file");
            string fileStr = req.body.substr(file.offset, file.length);

            printf("%s\n",req.body.c_str());
            // printf("%s\n",nameStr.c_str());
            // printf("%s\n",taskStr.c_str());
            // printf("%s\n",sizeStr.c_str());

            // FILE *f;
            // string path="../data/default/"+nameStr;
            // f=fopen(path.c_str(),"wb+");
            // fwrite(fileStr.c_str(),1,name.length,f);
            // fclose(f);
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

        });  
    }   
};

