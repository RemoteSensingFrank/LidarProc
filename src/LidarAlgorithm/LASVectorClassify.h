#pragma once
#ifndef _LAS_VECTORCLASSIFY_H_
#define _LAS_VECTORCLASSIFY_H_

#include <map>
#include "LASSimpleClassify.h"
#include "../LidarBase/LASPoint.h"
#include "ogrsf_frmts.h"

class LASVectorClassify : public LASSimpleClassify
{
public:
	/**
	* Rect2D transform GDAL OGREnvelope
	* @param rect
	* @return
	*/
	OGREnvelope LASRectToEnvelope(Rect2D rect);

	/**
	* OGREnvelope transform Rect2D
	* @param m_Envelope
	* @return
	*/
	Rect2D LASEnvelopToRect(OGREnvelope *m_Envelope);

	/**
	* name tpye mapping
	* vector<string> nameKey
	* vector<eLASClassification>typeKey
	*/
	bool  LASSetMaps(vector<string> nameKey, vector<eLASClassification>typeKey);

	/**
	* Las classify by shp data
	* @param pathLas
	* @param pShpData
	* @param pathOutLas
	* @return
	*/
	virtual void LASClassifyByVector(const char* pathLas, map<string,string> pShpPath, const char* pathOutLas)=0;


	bool TestGeos();

protected:
	/*
	* trans polygon to points
	* Point2Ds points
	 * todo:
	*/
	virtual void VectorPointsToPolygon(Point2Ds points, OGRPolygon* poPolygon);

	virtual void VectorPolygonToPoints(OGRPolygon* poPolygon, Point2Ds &points);

	std::map<string, int> name_type_maps;
};


class LASShpClassify : public LASVectorClassify
{
public:

	/**
	* Las classify by shp data
	* @param pathLas
	* @param pShpPath
	* @param pathOutLas
	* @return
	*/
	void LASClassifyByVector(const char* pathLas, map<string,string> pShpPath, const char* pathOutLas);

	/** 将一档中多个las数据文件读入,分类出该档内特定一类的数据
	* Las classify by shp data
	* @param lasPaths
	* @param pShpPath
	* @param lasOutPaths
	* @param classifyType
	* @param algorithmType 0:根据winding number算法,1:根据GDAL算法
	* @return
	*/
	void LASClassifyBySingleTypeVector(std::vector <string> lasPaths, string lasOutPaths, string pShpPath, const char classifyType, int algorithmType = 0);
};

#endif
