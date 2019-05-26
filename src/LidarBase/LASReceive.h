#pragma once
//
// Created by wuwei on 17-12-25.
// this is a very point file for las point process
// if the las dataset is in the remote server and
// shared with internet protocol we must first copy the
// dataset into the local machine and then process
// in local machine using the wget command
//
#ifndef LIDARPROC_LASRECEIVE_H
#define LIDARPROC_LASRECEIVE_H

#include<string>
using namespace std;
/**
 * @brief  virtual class for interface
 * @note   
 * @retval None
 */
class LASDatasetReceive{
public:
    /**
    * @brief  recevice dataset into the user path
    * @note   
    * @param  remote_path: remote dataset path
    * @param  localpath: user local path
    * @retval loacal dataset path
    */
    virtual string ReceiveLASDataset(string remote_path,string localpath)=0;

    /**
    * @brief  recevice dataset into daefaule path
    * @note   
    * @param  remote_path: remote dataset path
    * @retval loacal dataset path
    */
    virtual string ReceiveLASDataset(string remote_path)=0;

    /**
    * @brief  set local path
    * @note   
    * @param  localpath: 
    * @retval None
    */
    virtual void SetLocalPath(string localpath){
        m_localpath = localpath;
    };

protected:
    /**
    * @brief  localpath
    * @note   
    * @retval None
    */
    string m_localpath;
};

/**
 * @brief  receive dataset with ftp portocol
 * @note   
 * @retval None
 */
class LASDatasetReceiveFtp: public LASDatasetReceive{
public:
    string ReceiveLASDataset(string remote_path,string localpath);

    string ReceiveLASDataset(string remote_path);
};

/**
 * @brief  receive dataset with http portocol
 * @note   
 * @retval None
 */
class LASDatasetReceiveHttp:public LASDatasetReceive{
public:
    string ReceiveLASDataset(string remote_path,string localpath);

    string ReceiveLASDataset(string remote_path);
};

#endif 