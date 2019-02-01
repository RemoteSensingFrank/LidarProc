#pragma once
#ifndef _LAS_VECTORCLASSIFY_H_
#define _LAS_VECTORCLASSIFY_H_

#include <map>
#include <vector>
#include "ogrsf_frmts.h"
#include "LASSimpleClassify.h"
#include "../LidarBase/LASPoint.h"
#include "../Utility/FileHelper.h"
#include "../LidarAlgorithm/WorkingConditionSimulation.h"

using namespace std;
using namespace LasAlgorithm;
//class WireModel;

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

    #pragma region 导线点云处理
	/**
	* Las Point To Vector Line,
	* @param pathLas
	* @param pShpData
	* @param pathOutLas
	* @return
	*/
	void LidarToVector(vector<Point3D> pntCloud,int * pointTypes, vector<Point3D> lowerTowerPoints);

	/**
	* Las Point To Vector Line,
	* @param pathLas
	* @param pShpData
	* @param pathOutLas
	* @return 
	*/
	vector <LasAlgorithm::TensionSegment> LidarToVector(const char * pathFileConfig, string towerSpanName);

	/**
	* Vector line to Points
	* @param pathLas
	* @param pShpData
	* @param pathOutLas
	* @return
	*/
	void VectorToLidar();

	/**
	* 已知该段点云真实导线数，按点的数量多少清除多余真实导线数的点云
	* @param pointClouds 导线点云
	* @param realLineCount 真实导线数
	* @return 清理后的点云
	*/
	vector <Point3Ds> ClearSmallPoints(vector <Point3Ds> &pointClouds, long realLineCount);

	/**
	* 分割直线塔相连的点云
	* @param pointClouds 导线点云
	* @param tensionSeg 耐张段模型
	* @param turnNumber 回路数
	* @return 分割后的点云
	*/
	vector <Point3Ds> SegmentTangentTowerPoints(vector <Point3Ds> &pointClouds, vector <TensionSegment> &tensionSeg, long turnNumber);

	/**
	* 获取一相导线耐张段所有线,计算代表档距使用
	* @param tensionSeg 耐张段模型
	* @return
	*/
	void GetWholeLineSegment(vector <TensionSegment> &tensionSeg);

	/**
	* 计算剖面的导线抛物线模型
	* @param lineModel 线模型
	* @param linePoints 导线点云
	* @return
	*/
	void GetSectionLineModel(WireModel &lineModel,Point3Ds linePoints);

	/**
	* 二维坐标点转换成尺度因子S
	* @param point2D 需要转换的二维点
	* @param OFootPoint 原点O到直线的垂足点
	* @param lineAngle 直线斜率夹角
	* @return 尺度因子S
	*/
	double Point2DToScaleFactor(Point2D point2D,Point2D OFootPoint,double lineAngle);

	/**
	* 尺度因子S转换成二维坐标点
	* @param scaleFactor 需要转换的尺度因子S
	* @param OFootPoint 原点O到直线的垂足点
	* @param lineAngle 直线斜率夹角
	* @param lineFactors 直线方程因子
	* @return 二维坐标点
	*/
	Point2D ScaleFactorToPoint2D(double scaleFactor, Point2D OFootPoint, double lineAngle, vector <double> lineFactors);

	/**
	* 二维坐标点转换成剖面点的X坐标
	* @param point2D 需要转换的二维点
	* @param OPoint 剖面坐标系原点
	* @return 剖面点的X坐标
	*/
	double Point2DToSectionX(Point2D point2D, Point2D OPoint);

	/**
	* 剖面点的X坐标转换成二维坐标点
	* @param sectoinX 剖面点的X坐标
	* @param OPoint 剖面坐标系原点
	* @param lineAngle 直线斜率夹角
	* @return 二维坐标点
	*/
	Point2D SectionXToPoint2D(double sectoinX, Point2D OPoint, double lineAngle);

	/**
	* read towerPoints from txt
	* @param pathTowerPoints
	* @param pair <Point2D,int>,type:0直线塔,1:耐张塔
	* @return 是否有直线塔
	*/
	bool ReadTowerPointsByTxt(const char* pathTowerPoints,vector <pair <Point2D,int>> &towerPoints, vector <TensionSegment> &tensionSeg, int &towerSpanNum);

	/**
	* 根据导线模型和采样间隔生成导线点云
	* @param lineModel 线模型
	* @param sampleSpacing 导线点采样间隔
	* @param isNeedSplit 是否需要分裂，0：输出中心线，1：输出分裂导线
	* @param pointOutType  输出类型：0:左右风偏点都输出,1:只输出左风偏点,2:只输出右风偏点
	* @return 导线点云
	*/
	Point3Ds GetPointsByLineModel(WireModel lineModel,double sampleSpacing, int isNeedSplit, int pointOutType = 0);

	/**
	* 根据导线模型和采样间隔生成导线点云
	* @param tensionSeg 耐张段模型
	* @param lineIndex  导线模型下标
	* @param sampleSpacing 导线点采样间隔
	* @param isNeedSplit 是否需要分裂，0：输出中心线，1：输出分裂导线
	* @return 导线点云
	*/
	Point3Ds GetPointsByWindLineModel(TensionSegment tensionSeg, size_t lineIndex, double sampleSpacing, int isNeedSplit);

	/**
	* 根据导线模型计算导线最大弧垂
	* @param lineModel 线模型
	* @return 导线最大弧垂
	*/
	double GetWireSagByLineModel(WireModel lineModel);

	/**
	* 输出点云到Txt
	* @param filename  文件名
	* @param filePath  文件路径
	* @param pntClouds 点云点
	* @return
	*/
	static void OutPoint3DsToTxt(string filename,string filePath,vector<Point3Ds> pntClouds);

	/**
	* 从点云中找到离已知点最近的点
	* @param knownPoints  已知点数据集
	* @param pointCloud 点云点
	* @return 最近的点集
	*/
	static Point3Ds FindNearestPoint(Point3Ds knownPoints, Point3Ds pointCloud);

	/**
	* 输出已知点和点云点位误差
	* @param knownPoints  已知点数据集
	* @param lidarPoints 点云点
	* @return 
	*/
	static void PrintPointsError(Point3Ds knownPoints, Point3Ds lidarPoints);

	void ErrorAnalysis(vector <Point3Ds> vecPoints);

	/**
	* 输出线模型到矢量文件SHP
	* @param filename  文件名
	* @param filePath  文件路径
	* @param lineModel 线模型
	* @param sampleSpacing 导线点采样间隔
	* @return
	*/
	//void OutLineModelToShp(string fileName, string filePath, WireModel lineModel, double sampleSpacing);

    #pragma endregion

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
