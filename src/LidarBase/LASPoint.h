#pragma once
//
// Created by wuwei on 17-12-25.
//

#ifndef LASLIB_LASPOINT_H
#define LASLIB_LASPOINT_H
#include <string>
#include <functional>
#include <stdio.h>

#include "LASHeader.h"
#include "../LidarGeometry/Geometry.h"
#include "../LidarGeometry/GeometryFlann.h"
#include "../LidarGeometry/RTree.hpp"

using namespace std;
using namespace GeometryLas;
//struct LASPoint;
//typedef std::function<void(LASPoint*)> callback_operation_points_Ptr;

static unsigned char GetReturnNumber(unsigned char temp) { return (temp & 0x07) ; /*0x00000111*/ }
static unsigned char GetNumberOfReturns(unsigned char temp) { return (temp & 0x38) >> 3; /*00111000*/ }

//待测试
static bool GetScanDirectionFlag(unsigned char temp) { return ((temp & 0x02) >> 1) == 1 ? true : false; /*00000010*/ }
static bool GetEdgeOfFlightLine(unsigned char temp) { return temp & 0x01;	/*00000001*/ }

#ifndef _OUT_
#define _OUT_
#endif 

#ifndef _IN_
#define _IN_
#endif

#ifndef _INOUT_
#define _INOUT_
#endif

#ifndef COLORREF
#define COLORREF int
#endif

#ifndef RGB
#define RGB(r,g,b)          ((COLORREF)(((unsigned char)(r)|((unsigned int)((unsigned char)(g))<<8))|(((unsigned int)(unsigned char)(b))<<16)))
#endif

#ifndef ExRGB
#define ExRGB(color,extColor)							\
{														\
	extColor.Red = (color << 24) >> 24 & 0x000000ff;	\
	extColor.Green = (color << 16) >> 24 & 0x000000ff;	\
	extColor.Blue = (color << 8) >> 24 & 0x000000ff;	\
}
#endif

/*Las1.2颜色扩展*/
#pragma pack(1)
struct LASColorExt
{
	unsigned short Red;
	unsigned short Green;
	unsigned short Blue;
};
#pragma pack()

//las点文件
#pragma pack(1)/*字节对齐*/
class LASPoint
{
public:
	/*
	* 读写
	*/
	void Write(FILE *fs, const LASHeader& info) const;
	void Read(FILE *fs, const LASHeader& info);

	/**
	* 内存中解析出单个的点云数据
	* @param data
	* @param info
	*/
	void ExtractFromBuffer(const unsigned char* data, const LASHeader& info);
	void ExportToBuffer(unsigned char* data, const LASHeader& info) const;

	/**
	* @brief  extract the number of returns
	* @note   
	* @retval 
	*/
	int  ExtractNumberOfReturns();

	/**
	* @brief  extract the return number
	* @note   
	* @retval None
	*/
	int  ExtractReturnNumber();

public:
	Point3D			m_vec3d;
	unsigned short  m_intensity;
	unsigned char   m_rnseByte;
	char			m_classify;
	char			m_scanAngle;
	unsigned char	m_userdata;
	unsigned short  m_flightID;
	double			m_gpsTime;
	LASColorExt		m_colorExt;
};
#pragma pack()


/*
	adaptor it's hard to adjust the 
*/
struct PointCloudBlockAdaptor
{
	std::vector<LASPoint> lasPoints;

	//reload operator [] to keep the code with same format
	inline void push_back(LASPoint pnt) { lasPoints.push_back(pnt); }

	//reload operator [] to keep the code with same format
	inline LASPoint &operator[](int i) { return lasPoints[i]; }

	//clear data
	inline void clear() { lasPoints.clear(); }

	// must return the number of data points
	//(for adatopr the function must be relized)
	inline size_t kdtree_get_point_count() const { return lasPoints.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline double kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim == 0) return lasPoints[idx].m_vec3d.x;
		else if (dim == 1) return lasPoints[idx].m_vec3d.y;
		else return lasPoints[idx].m_vec3d.z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

typedef RTree<int, double, 2, double, 4>  LASBlockTree;
typedef PointCloudBlockAdaptor PCBlockAdaptor;
typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCBlockAdaptor>, PCBlockAdaptor, 3> kd_tree_block;

/*激光回波信号*/
enum  eLASEcho
{
	eLidarEchoOnly = 0,
	eLidarEchoFirst = 1,
	eLidarEchoMidian = 2,
	eLidarEchoLast = 3
};

enum  eTxtLASType
{
	LASXYZ,
	LASXYZRGB
};

/*点云的类别*/
#pragma pack(1)
enum  eLASClassification
{
	elcCreated			 = 0,	// 创建的，没有被分类的
	elcUnclassified		 = 1,	// 无类别的，或无法识别类别的点
	elcGround			 = 2,	// 地面点
	elcLowVegetation	 = 3,	// 矮的植被
	elcMediumVegetation  = 4,	// 中等高度的植被
	elcHighVegetation	 = 5,	// 高的植被
	elcBuilding			 = 6,	// 建筑物
	elcLowPoint			 = 7,	// 低于地表的点（噪音）
	elcModelKeyPoint	 = 8,	// 控制点
	elcWater			 = 9,	// 水
	elcOverlapPoint		 = 12,	// 航带重叠点

	elcDanger			 = 13,	//
	elcDangerLevel1		 = 14,	// 
	elcDangerLevel2		 = 15,
	elcDangerLevel3		 = 16,
	//...ext
	elcDangerEnd		 = 23,

	elcTowerRange	 	 = 24,		// 
	elcTowerUp			 = 25,
	elcTowerDown		 = 26,
	elcDriveWay			 = 27,      //公路

	elcFallingTree		 = 30,		//树木倒伏
	elcFallingTreeLevel1 = 31,
	elcFallingTreeLevel2 = 32,
	elcFallingTreeLevel3 = 33,
	elcFallingTreeEnd	 = 34,
	elcLine				 = 35,		//电力线
	elcVegetation		 = 36,
	elcDeletedPoint		 = -1	// 已删除的点
};
#pragma pack()

static eLASClassification GetLidarClassification(unsigned char clsType)
{
	return (eLASClassification)clsType;
}


#pragma pack(1)/*字节对齐*/
struct LASIndex
{
	int rectangle_idx;
	int point_idx_inRect;
};
#pragma pack()



/*
* 点云文件块
* version 1.2
* author: Frank.Wu
* 分块索引后添加KD树索引
*/
class LASRectBlock {
public:
	LASRectBlock() { m_lasPoints.clear(); m_lasPoints_numbers = 0; m_block_tree = nullptr; }
	~LASRectBlock() {
		if (!m_lasPoints.lasPoints.empty())
			m_lasPoints.clear();
		if (m_block_tree != nullptr)
			delete m_block_tree;
		m_block_tree = nullptr;
		m_lasPoints_numbers = 0;
	}

	/*****************************************************************************
	* @brief : 分配内存,判断是否将点分配进入内存中，或者只存索引
	* @author : W.W.Frank
	* @date : 2015/11/29 20:02
	* @version : version 1.0
	* @inparam :
	* @outparam :
	*****************************************************************************/
	void LASRect_AllocateMemory(int lasPoints, bool inMemory, Rect2D rect);


	/*****************************************************************************
	* @brief : 对每个块建立kd树索引
	* @author : W.W.Frank
	* @date : 2018.07.24
	* @version : version 1.1
	* @inparam :
	* @outparam :
	*****************************************************************************/
	void LASRectBuildTree();


public:
	Point3D			 m_rectCenter;
	Rect2D			 m_Rectangle;
	long long		 m_lasPoints_numbers;

	//construct the kd tree for each point
	kd_tree_block*	 m_block_tree;
	PCBlockAdaptor	 m_lasPoints;
};

/**
* 点云数据集,点云数据集分为两个部分，首先对点云数据集以KD树进行分块，其中每一块文件为LASRectBlock
* 获取点云数据将点云数据分配到LASRectBlock中，每一个Block构建kd树，构建双重索引
*/
class ILASDataset {

public:
	ILASDataset();
	~ILASDataset();

	//构建R树的过程
	long LASDataset_BuildTree();
	//分配内存，是否在内存中分配，或者只是读取index
	void LASDataset_AllocateMemory(int lasRects);
	//对数据进行整理
	void LASDataset_Trim(bool inMemory);

	//遍历函数没想太清楚
	//bool LASDataset_Iterator(callback_operation_points_Ptr ptrFun);
	bool LASDataset_FixHeader();

	//找到匹配的矩形的id，根据id获取在哪个矩形中
	bool LASDataset_Search(int rectID, Rect2D  searchRect, vector<int> &rects);
	bool LASDataset_Search(int rectID, Point3D searchPnt, vector<int> &rects);

	//根据顺次次序获取三维点
	bool LASDataset_Search(int pointID, Point3D &searchPnt);

public:
	LASHeader			m_lasHeader;
	LASVariableRecord*	m_lasvarHeader;
	LASVariable_header_geo_keys  m_lasgeokeyHeader;
	LASVariable_header_key_entry m_lasgeoentryHeader;

	double              m_xrange[2], m_yrange[2], m_zrange[2];
	Point3D			    m_SetCenter;
	LASRectBlock       *m_lasRectangles;
	LASBlockTree		m_lasBlockTree;
	int					m_numRectangles;
	int                 m_totalReadLasNumber;
	LASIndex           *m_LASPointID; //全局点在局部矩形中的编号
};


/*
to meet the require of flann
using adaptor mode
*/
template<typename PCDrived>
struct PointCloudAdaptor
{
	//constructor
	PointCloudAdaptor(const PCDrived &obj_) : obj(obj_) { }

	const PCDrived &obj; //!using reference to save mem and spped up

						 //get datasource using inline to speed up
	inline const PCDrived& derived() const { return obj; }

	// must return the number of data points
	//(for adatopr the function must be relized)
	inline size_t kdtree_get_point_count() const { return derived().size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline double kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim == 0) return derived()[idx].x;
		else if (dim == 1) return derived()[idx].y;
		else return derived()[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

template<typename PCDrived>
struct PointCloud2DAdaptor
{
	//constructor
	PointCloud2DAdaptor(const PCDrived &obj_) : obj(obj_) { }

	const PCDrived &obj; //!using reference to save mem and spped up

						 //get datasource using inline to speed up
	inline const PCDrived& derived() const { return obj; }

	// must return the number of data points
	//(for adatopr the function must be relized)
	inline size_t kdtree_get_point_count() const { return derived().size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline double kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim == 0) return derived()[idx].x;
		else  return derived()[idx].y;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};


#endif //LASLIB_LASPOINT_H
