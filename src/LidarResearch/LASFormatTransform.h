//
// Created by wuwei on 18-3-13.
// Convert LAS Dataset into Potree format
// Then Copy the Converted data into 
// build directory and trans to exhibit
//

#ifndef LIDARPROC_LASFORMATTRANSFORM_H
#define LIDARPROC_LASFORMATTRANSFORM_H
#include "../LidarBase/LASPoint.h"

//#ifdef _USE_POTREE_

#include <string>
#ifdef _USE_PCL_
    #include <pcl/io/pcd_io.h>
    #include <pcl/segmentation/extract_clusters.h>
#endif
#include "experimental/filesystem"
#include "PotreeConverter.h"

namespace fs = std::experimental::filesystem;

using Potree::PotreeConverter;
using Potree::StoreOption;
using Potree::ConversionQuality;
using std::exception;


/**
 * @brief  convert las to potree format
 *         using potree lib rely on liblas and laszip
 *         so pls install liblas and laszip first 
 * @note   
 * @retval None
 */
class LASTransToPotree{
public:
    /**
    * @brief trans las data to the user trans directory  
    * @note   
    * @param  lasPath:  path of the las points data
    * @param  transdir: path of the transformed data directory
    * @retval success 0 failed other values
    */
    long LASTransToPotree_Trans(const char* lasPath,const char* transdir);

    /**
    * @brief trans las data to the server directory  
    * @note   
    * @param  lasPath: 
    * @retval 
    */
    long LASTransToPotree_Trans(const char* lasPath);

};
//#endif _USE_POTREE_
//if the pcl lib is included

#ifdef _USE_PCL_

class LASTransToPCL{
public:
    /**
    * @brief trans las data to the format of PCL  
    * @note   
    * @param :ILASDataset* dataset: 数据集
    * @param :pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud : PCL 点云数据集
    */
    long LASTransToPCL_Trans(ILASDataset* dataset, pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud);

    long LASTransToPCL_Trans(ILASDataset* dataset, pcl::PointCloud<pcl::PointXYZ> &pclPointCloud);
};
#endif

#endif //LIDARPROC_LASFORMATTRANSFORM_H
