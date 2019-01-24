#pragma once
#ifndef _GEOMETRY_ALGORITHMS_H_
#define _GEOMETRY_ALGORITHMS_H_

#include "Geometry.h"
using namespace GeometryLas;

class  GeometryRelation
{
public:
	static int IsPointOnPolyline(double x, double y, const double* polyline, int doubleArrayCnt,
		double tolerance, bool isClosed, int offset = sizeof(Point3D) / sizeof(double));
	/*
	* 判断点是否在线段上（空间关系：line contains point）
	*/
	static bool IsPointOnLine(double x, double y, double x0, double y0, double x1, double y1,
		double tolerance);
	/*
	* 判断点是否在直线上（空间关系：line contains point）
	*/
	static bool IsPointOnBeeline(double x, double y, double x0, double y0, double x1, double y1,
		double tolerance);
	/*
	* 判断点是否在多边形里，包括在边界上（空间关系：region contains point）
	* 顺时针计算每个边
	* @param allowonisin : 是否认为在边界上也算在边界内，为true，则tolerance有效，否则无效
	*/
	static bool IsPointInPolygon(double x, double y, const double* polygon, double doubleArrayCnt,
		bool allowonisin, double tolerance, int offset = sizeof(Point3D) / sizeof(double));
	/*
	* 判断点是否在多边形里
	*/
	static bool IsPointInPolygon(double x, double y, const double* polygon,
		int doubleArrayCnt, int offset = sizeof(Point3D) / sizeof(double));
	static bool IsPointInPolygon(double x, double y, const Point2Ds& polygon);
	/*
	* used for IsPointInPolygon follows
	*/
	static double IsLeft(double startx, double starty, double endx, double endy, double x, double y);
	/*
	* 点是否在矩形边界上（空间关系：lines contains point）
	*/
	static bool IsPointOnRect(double x, double y, double minx, double miny,
		double maxx, double maxy, double tolerance);
	/*
	* 判断折线、多边形是否在矩形里，包括顶点在矩形边界上（空间关系：region contains region, region contains lines）
	*/
	static bool IsPolylineInRect(const double* polyline, int doubleArrayCnt, double minx,
		double miny, double maxx, double maxy, int offset = sizeof(Point3D) / sizeof(double));
	/*
	* 判断线段与矩形边界是否相交，包括边界点（空间关系：line crosses line）
	*/
	static bool IsLineIntersectRect(double x0, double y0, double x1, double y1, double xmin,
		double ymin, double xmax, double ymax);
	/*
	* 判断矩形边界与折线是否相交，包括边界点（空间关系：line crosses line）
	*/
	static bool IsRectIntersectPolyline(double xmin, double ymin, double xmax,
		double ymax, const double* polyline, int doubleArrayCnt, bool isClosed,
		int offset = sizeof(Point3D) / sizeof(double));
	static bool IsPointInRect(int x, int y, int rect_x1, int rect_y1, int rect_x2, int rect_y2);
	/*
	* 判断正矩形与斜矩形是否相交
	*/
	static bool IsRectIntersectRect(const Rect2D& rect1, const Rect2D& rect2);
	static bool IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts);
	static bool IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts,const Rect2D& slantingRect);

	/*
	* 求两个向量的夹角
	*/
	static double VectorAngle(const Point3D vec1,const Point3D vec2);

};

/*距离计算函数*/
class  DistanceComputation
{
public:
	/*
	* 点到直线的距离的平方
	*/
	static double SquarePointToBeeline(double x, double y, double x0, double y0, double x1, double y1);
	/*
	* 计算点集中所有点间的距离之和，欧氏距离
	*/
	static double Distance(Point2Ds& pts, bool isclosed);

	/*
	* 计算两点间的距离
	*/
	static double Distance(Point3D pt1, Point3D pt2);
};

/*点线相交函数*/
class  PointJointComputation
{
public:
	/*
	* 点(x, y)到直线(bln0x, bln0y)(bln1x, bln1y)的垂足占线段的比例
	*/
	static void PointJointBeeline(double x, double y, double bln0x, double bln0y, double bln1x,
		double bln1y, double& ratio);
	/*
	* 点(x, y)到直线(bln0x, bln0y)(bln1x, bln1y)的垂足
	*/
	static Point2D PointJointBeeline(double x, double y, double bln0x, double bln0y, double bln1x,
		double bln1y);
};

/*投影坐标转换*/
class  PointProjection
{
public:
	/*经纬度坐标转换到投影坐标*/
	static void PointProjectLatLngToUTM(double lat, double lng, int nZone, double &x, double &y);
};

#endif