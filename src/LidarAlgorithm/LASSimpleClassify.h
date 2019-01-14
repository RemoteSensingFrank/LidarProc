#pragma once
//
// Created by wuwei on 18-1-14.
//

#ifndef LASGUI_LASSIMPLECLASSIFY_H
#define LASGUI_LASSIMPLECLASSIFY_H

#include <vector>
#include "../LidarBase/LASPoint.h"

#ifndef ColorClassInfo
typedef struct ColorClassInfo
{
	int red;
	int green;
	int blue;
	eLASClassification classType;
}ColoInfo;
#endif

/*
	simple classify using the base information of the las dataset
	author:wuwei
	version:1.0.0.0
*/
class LASSimpleClassify {
public:
	/**
	* classify the point cloud by number of echo
	* @param dataset
	* @return
	*/
	long LASVegetationByEcho(ILASDataset* dataset);

	/**
	* classify data by elevation
	* @param dataset :lidar dataset
	* @param elevation :input elevation
	* @param direct :above or below the elevation(true above false below)
	* @param eclass :input the type
	* @return
	*/
	long LASClassifyByElevation(ILASDataset* dataset, float elevation, bool direct, eLASClassification eclass);


	/**
	* classufy data by intensity
	* @param dataset
	* @param elevation
	* @param direct
	* @param eclass
	* @return
	*/
	long LASClassifyByIntensity(ILASDataset* dataset, float intensity, bool direct, eLASClassification eclass);

	/**
	* classify data by RGB color
	* @param dataset :lidar dataset(in & out)
	* @param colorInfo : classify color info (in)
	* @return
	*/
	long LASClassifyByColor(ILASDataset* dataset, std::vector<ColoInfo> colorInfo);
};

/**
    classify las dataset with limited memory
 	author:wuwei
	version:1.0.0.0
*/
class LASClassifyMemLimited : public LASSimpleClassify
{
public:
	/**
	* export the dataset by each types
	* @param pathLas
	* @param type
	* @param pathExport
	* @return
	*/
	long LASExportClassifiedPoints(const char* pathLas, eLASClassification type, const char* pathExport);


};

#endif //LASGUI_LASSIMPLECLASSIFY_H

