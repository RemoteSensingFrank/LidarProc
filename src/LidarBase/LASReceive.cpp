#include "LASReceive.h"
//
// Created by wuwei on 17-12-25.
//

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include "LidarUtil/FileHelper.h"
#pragma warning(disable:4996)

string LASDatasetReceiveHttp::ReceiveLASDataset(string remote_path,string localpath)
{
    string cmd_construct="wget -P "+localpath+" "+remote_path;
    system(cmd_construct.c_str());

    return localpath+FileHelper::getFileName(remote_path);
}

string LASDatasetReceiveHttp::ReceiveLASDataset(string remote_path)
{
    string cmd_construct="wget -P "+m_localpath+" "+remote_path;
    system(cmd_construct.c_str());
    return m_localpath+FileHelper::getFileName(remote_path);
}