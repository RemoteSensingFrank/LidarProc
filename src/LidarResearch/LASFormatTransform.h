//
// Created by wuwei on 18-3-13.
// Convert LAS Dataset into Potree format
// Then Copy the Converted data into 
// build directory and trans to exhibit
//

#ifndef LIDARPROC_LASFORMATTRANSFORM_H
#define LIDARPROC_LASFORMATTRANSFORM_H

#include <string>
#include <AABB.h>
#include <PotreeConverter.h>
#include <experimental/filesystem>

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

#endif //LIDARPROC_LASFORMATTRANSFORM_H
