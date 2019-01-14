#pragma once
//
// Created by Frank.Wu on 18-07-10.
//

#ifndef _POINTPROCALG_H_
#define _POINTPROCALG_H_

#include "Geometry.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASReader.h"
using namespace GeometryLas;
/*
point cloud segment algorithm class
*/
class PointCloudSegment
{
public:
	/*
		using dbscan algorithm to segment point cloud
		@param pointSet:input point set
		@param type:output type of each point
		@param knnRange:param of distance
		@return segment number
	*/
	long PointCloudSegment_DBScan(Point3Ds pointSet, int *type, float knnRange);
};

#endif // !_POINTPROCALG_H_


