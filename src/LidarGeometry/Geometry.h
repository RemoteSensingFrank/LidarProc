#pragma once
/*****************************************************************************
* @brief : 几何形状的定义
* @author : W.W.Frank
* @date : 2015/11/30 8:21
* @version : version 1.0
* @inparam :
* @outparam :
*****************************************************************************/
#ifndef _GEOMETRY_OBJECTIONS_H_
#define _GEOMETRY_OBJECTIONS_H_

#ifdef LIBLAS
#define LIBLAS extern "C" _declspec(dllimport)
#else
#define LIBLAS extern "C" _declspec(dllexport)
#endif
#endif

#include <math.h>
#include <algorithm>
#include <vector>

using namespace std;

/*****************************************************************************
* @brief : 点位置的定义
* @author : W.W.Frank
* @date : 2015/11/29 14:20
* @version : version 1.0
*****************************************************************************/
//废弃
//struct  vector3D
//{
//    vector3D(double x,double y,double z){this->x=x;this->y=y;this->z=z;}
//    vector3D()					{ x = y = z = 0; }
//    vector3D(const vector3D &p)	{ x = p.x; y = p.y; z = p.z; }
//
//    double x;
//    double y;
//    double z;
//
//    bool operator==(vector3D point) const;
//    bool operator!=(vector3D point) const;
//
//    vector3D& operator=(vector3D v);
//
//    void operator+=(vector3D v);
//    void operator-=(vector3D v);
//
//    vector3D operator+(vector3D v) const;
//    vector3D operator-(vector3D v) const;
//    vector3D operator*(double scalar) const;
//    vector3D operator/(double scalar) const;
//    vector3D operator*(vector3D v)  const;
//
//};
//inline bool vector3D::operator==(vector3D v) const
//{
//    return v.x == x && v.y == y && v.z == z;
//}
//inline bool vector3D::operator!=(vector3D v) const
//{
//    return v.x != x || v.y != y || v.z != z;
//}
//inline vector3D& vector3D::operator=(vector3D v)
//{
//    x = v.x; y = v.y; z = v.z; return *this;
//}
//inline void vector3D::operator+=(vector3D v)
//{
//    x += v.x;  y += v.y; z += v.z;
//}
//inline void vector3D::operator-=(vector3D v)
//{
//    x -= v.x;  y -= v.y; z -= v.z;
//}
//inline vector3D vector3D::operator-(vector3D v) const
//{
//    return vector3D(x - v.x,y - v.y,z - v.z);
//}
//inline vector3D vector3D::operator+(vector3D v) const
//{
//    return vector3D(x + v.x,y + v.y,z + v.z);
//}
//inline vector3D vector3D::operator*(double scalar) const
//{
//    return vector3D(x * scalar,y * scalar,z * scalar);
//}
//inline vector3D vector3D::operator/(double scalar) const
//{
//    return vector3D(x / scalar,y / scalar,z/scalar);
//}

namespace GeometryLas {
	/*****************************************************************************
	* @brief : 点集的定义
	* @author : W.W.Frank
	* @date : 2015/11/30 9:28
	* @version : version 1.0
	* @inparam :
	* @outparam :
	*****************************************************************************/
#ifndef _POINT_H_
#define _POINT_H_

	template<class T> class Point2_T;
#define RCPoint2_T const Point2_T<T>&

	typedef Point2_T<double> Point2D;
	typedef vector<Point2D> Point2Ds;
	typedef const Point2D*	PCPoint2D;
	typedef const Point2D&  RCPoint2D;

	typedef Point2_T<int> Point2I;
	typedef vector<Point2I> Point2Is;
	typedef const Point2I*	PCPoint2I;
	typedef const Point2I&  RCPoint2I;

	template<class T>
	class Point2_T
	{
	public:
		static int SizeofPoint2;
		T	x, y;

		Point2_T() { }
		Point2_T(RCPoint2_T p) { x = p.x; y = p.y; }
		Point2_T(T _x, T _y) : x(_x), y(_y) { }
	public:
		/*
		*	operator =
		*/
		Point2_T<T>& operator=(RCPoint2_T src);

		void Assign(T _x, T _y) { x = _x; y = _y; }
		void Assign(RCPoint2_T src) { *this = src; }

		void Offset(T dx, T dy) { x += dx;			y += dy; }
		void Offset(RCPoint2_T point) { x += point.x;	y += point.y; }

		void Rotate(RCPoint2_T org, T cosa, T sina);
		void Scale(RCPoint2_T org, T s);

		// void mirror(const Point2D& p0, const Point2D& p1);
		bool operator==(RCPoint2_T point) const { return x == point.x && y == point.y; }
		bool operator!=(RCPoint2_T point) const { return x != point.x || y != point.y; }

		void operator+=(RCPoint2_T point) { x += point.x;  y += point.y; }
		void operator-=(RCPoint2_T point) { x -= point.x;  y -= point.y; }

		Point2_T<T> operator+(RCPoint2_T arg) const;
		Point2_T<T> operator-(RCPoint2_T arg) const;

		Point2_T<T> operator*(const double scalar) const;
		Point2_T<T> operator/(const double scalar) const;

		void Negative() { x = -x;	y = -y; }
		bool ApproxEqual(RCPoint2_T src, T tolerance) const;
		double Distance(RCPoint2_T pt) const;
		double DistanceSquare(RCPoint2_T v) const;
	};

	template<class T>
	inline double Point2_T<T>::DistanceSquare(RCPoint2_T v) const
	{
		Point2_T<T> p = *this;
		p -= v;
		return p.x * p.x + p.y * p.y;
	}

	template<class T>
	int Point2_T<T>::SizeofPoint2 = sizeof(Point2_T<T>) / sizeof(T);

	template<class T>
	Point2_T<T>& Point2_T<T>::operator=(RCPoint2_T src)
	{
		x = src.x;
		y = src.y;
		return *this;
	}

	template<class T>
	Point2_T<T> Point2_T<T>::operator+(RCPoint2_T arg) const
	{
		Point2D rtv;
		rtv.x = x + arg.x;
		rtv.y = y + arg.y;
		return rtv;
	}

	template<class T>
	Point2_T<T> Point2_T<T>::operator-(RCPoint2_T arg) const
	{
		Point2D rtv;
		rtv.x = x - arg.x;
		rtv.y = y - arg.y;
		return rtv;
	}

	template<class T>
	Point2_T<T> Point2_T<T>::operator*(const double scalar) const
	{
		return Point2D(x * scalar, y * scalar);
	}

	template<class T>
	Point2_T<T> Point2_T<T>::operator/(const double scalar) const
	{
		return Point2D(x / scalar, y / scalar);
	}

	template<class T>
	bool Point2_T<T>::ApproxEqual(RCPoint2_T src, T tolerance) const
	{
		assert(tolerance> 0);
		return sqrt((x - src.x)*(x - src.x)) + sqrt((y - src.y)*(y - src.y)) <= sqrt(tolerance);
	}

	template<class T>
	void Point2_T<T>::Rotate(RCPoint2_T co, T cosa, T sina)
	{
		T temp = (x -= co.x);
		y -= co.y;
		x = temp * cosa - y * sina + co.x;
		y = temp * sina + y * cosa + co.y;
	}

	template<class T>
	void Point2_T<T>::Scale(RCPoint2_T org, T s)
	{
		x = org.x + s * (x - org.x);
		y = org.y + s * (y - org.y);
	}

	template<class T>
	double Point2_T<T>::Distance(const Point2_T<T>& pt) const
	{
		Point2_T<T> p = *this;
		p -= pt;
		return ::sqrt(double(p.x * p.x + p.y * p.y));
	}


	//三维点
	template<class T> class Point3_T;
#define RCPoint3_T const Point3_T<T>&

	typedef Point3_T<double> Point3D;
	typedef vector<Point3D> Point3Ds;
	typedef const Point3D*	PCPoint3D;
	typedef const Point3D&  RCPoint3D;

	typedef Point3_T<int> Point3I;
	typedef vector<Point3I> Point3Is;
	typedef const Point3I*	PCPoint3I;
	typedef const Point3I&  RCPoint3I;

	template<class T>
	class Point3_T
	{
	public:
		static int SizeofPoint3;
		T	x, y, z;

		Point3_T() { }
		Point3_T(RCPoint3_T p) { x = p.x; y = p.y; z = p.z; }
		Point3_T(T _x, T _y, T _z) : x(_x), y(_y), z(_z) { }
	public:
		/*
		*	operator =
		*/
		Point3_T<T>& operator=(RCPoint3_T src);

		void Assign(T _x, T _y, T _z) { x = _x; y = _y; z = _z; }
		void Assign(RCPoint3_T src) { *this = src; }

		void Offset(T dx, T dy, T dz) { x += dx;			y += dy;		z += dz; }
		void Offset(RCPoint3_T point) { x += point.x;	y += point.y;		z += point.z; }

		//void Rotate(RCPoint3_T org, T cosa, T sina,T);
		void Scale(RCPoint3_T org, T s);

		// void mirror(const Point2D& p0, const Point2D& p1);
		bool operator==(RCPoint3_T point) const { return x == point.x && y == point.y&&z == point.z; }
		bool operator!=(RCPoint3_T point) const { return x != point.x || y != point.y || z != point.x; }
		bool operator< (RCPoint3_T point) const { return x < point.x; }

		void operator+=(RCPoint3_T point) { x += point.x;  y += point.y; z += point.z; }
		void operator-=(RCPoint3_T point) { x -= point.x;  y -= point.y;	z -= point.z; }

		Point3_T<T> operator+(RCPoint3_T arg) const;
		Point3_T<T> operator-(RCPoint3_T arg) const;

		Point3_T<T> operator*(const double scalar) const;
		Point3_T<T> operator/(const double scalar) const;

		void Negative() { x = -x;	y = -y;		z = -z; }
		bool ApproxEqual(RCPoint3_T src, T tolerance) const;
		double Distance(RCPoint3_T pt) const;
		double DistanceXY(RCPoint3_T pt) const;
		double DistanceSquare(RCPoint3_T v) const;
	};

	template<class T>
	inline double Point3_T<T>::DistanceSquare(RCPoint3_T v) const
	{
		Point3_T<T> p = *this;
		p -= v;
		return p.x * p.x + p.y * p.y + p.z*p.z;
	}

	template<class T>
	int Point3_T<T>::SizeofPoint3 = sizeof(Point3_T<T>) / sizeof(T);

	template<class T>
	Point3_T<T>& Point3_T<T>::operator=(RCPoint3_T src)
	{
		x = src.x;
		y = src.y;
		z = src.z;
		return *this;
	}

	template<class T>
	Point3_T<T> Point3_T<T>::operator+(RCPoint3_T arg) const
	{
		Point3_T rtv;
		rtv.x = x + arg.x;
		rtv.y = y + arg.y;
		rtv.z = y + arg.z;
		return rtv;
	}

	template<class T>
	Point3_T<T> Point3_T<T>::operator-(RCPoint3_T arg) const
	{
		Point3_T rtv;
		rtv.x = x - arg.x;
		rtv.y = y - arg.y;
		rtv.z = z - arg.z;
		return rtv;
	}

	template<class T>
	Point3_T<T> Point3_T<T>::operator*(const double scalar) const
	{
		return Point3_T(x * scalar, y * scalar, z*scalar);
	}

	template<class T>
	Point3_T<T> Point3_T<T>::operator/(const double scalar) const
	{
		return Point3_T(x / scalar, y / scalar, z / scalar);
	}

	template<class T>
	bool Point3_T<T>::ApproxEqual(RCPoint3_T src, T tolerance) const
	{
		assert(tolerance> 0);
		return sqrt((x - src.x)*(x - src.x)) + sqrt((y - src.y)*(y - src.y)) + sqrt((z - src.z)*(z - src.z)) <= sqrt(tolerance);
	}

	template<class T>
	void Point3_T<T>::Scale(RCPoint3_T org, T s)
	{
		x = org.x + s * (x - org.x);
		y = org.y + s * (y - org.y);
		z = org.z + s * (z - org.z);
	}

	template<class T>
	double Point3_T<T>::Distance(const Point3_T<T>& pt) const
	{
		Point3_T<T> p = *this;
		p -= pt;
		return ::sqrt(double(p.x * p.x + p.y * p.y + p.z*p.z));
	}
	template<class T>
	double Point3_T<T>::DistanceXY(const Point3_T<T>& pt) const
	{
		Point3_T<T> p = *this;
		p -= pt;
		return ::sqrt(double(p.x * p.x + p.y * p.y));
	}
#endif // _POINT_H_

	/*****************************************************************************
	* @brief : RECT矩形的定义
	* @author : W.W.Frank
	* @date : 2015/11/30 8:21
	* @version : version 1.0
	* @inparam :
	* @outparam :
	*****************************************************************************/
#ifndef _RECT_H_
#define _RECT_H_
	template<class T> class Rect2_T;
#define RCRect2_T const Rect2_T<T>&

	typedef Rect2_T<double> Rect2D;
	typedef const Rect2D* PCRect2D;
	typedef const Rect2D& RCRect2D;

	typedef Rect2_T<int> Rect2I;
	typedef const Rect2I* PCRect2I;
	typedef const Rect2I& RCRect2I;

	enum eRectPointRelation
	{
		RF_In = 0x00,	// 在矩形内
		RF_OutLeft = 0x01,	// 在矩形左边
		RF_OutRight = 0x02,	// 在矩形右边
		RF_OutTop = 0x04,	// 在矩形上边
		RF_OutBottom = 0x08,	// 在矩形下边
		RF_OnLeft = 0x10,	// 在矩形左边界上，包括延长线
		RF_OnRight = 0x20, // 在矩形右边界上
		RF_OnTop = 0x40, // 在矩形上边界上
		RF_OnBottom = 0x80, // 在矩形下边界上
	};

	template<class T>
	class Rect2_T
	{
	public:
		T	minx, miny, maxx, maxy;
		Rect2_T() { SetNull(); }
		Rect2_T(T mx, T my, T Mx, T My) : minx(mx), miny(my), maxx(Mx), maxy(My) { }

	public:
		Rect2_T<T>& operator=(RCRect2_T src);

		void SetNull() { maxx = maxy = -(minx = miny = 2147483647); }
		bool IsNull() const { return maxx < minx || maxy < miny; }

		void	Set(T l, T t, T r, T b) { minx = l;  miny = t; maxx = r;  maxy = b; }
		void	Set(RCRect2_T rhs) { *this = rhs; }

		T		Width() const { return maxx - minx; }
		T		Height() const { return maxy - miny; }

		/*
		* Expand
		*/
		void Expand(T dx, T dy);
		void Expand(RCRect2_T r) { minx -= r.minx;	maxx += r.maxx; miny -= r.miny; maxy += r.maxy; }
		void Expand(T l, T t, T r, T b) { minx -= l; miny -= t;	maxx += r; maxy += b; }
		/*
		* scale
		*/
		void Scale(double sx, double sy);
		/*
		* offset
		*/
		void Offset(T dx, T dy) { minx += dx; maxx += dx; miny += dy; maxy += dy; }

		bool IsEmpty() const { return minx >= maxx || miny >= maxy; }

		bool IsIntersect(RCRect2_T r) const { return !(r.minx>maxx || r.maxx<minx || r.miny>maxy || r.maxy<miny); }
		bool IsDisjoint(RCRect2_T r) const { return r.minx>maxx || r.maxx<minx || r.miny>maxy || r.maxy<miny; }
		bool IsInclude(RCRect2_T r) const { return (r.minx >= minx && r.maxx <= maxx && r.miny >= miny && r.maxy <= maxy); }
		bool IsInclude(T x, T y) const { return x >= minx && x <= maxx && y >= miny && y <= maxy; }

		// point in rect
		int	PointIn(T x, T y) const { return PointInRect(x, y, *this, 0); }
		int	PointIn(T x, T y, double tole) const { return PointInRect(x, y, *this, tole); }

		// operator
		bool Equal(RCRect2_T r)		   const { return operator == (r); }
		bool operator == (RCRect2_T r) const { return minx == r.minx && maxx == r.maxx && miny == r.miny && maxy == r.maxy; }
		bool operator != (RCRect2_T r) const { return minx != r.minx || maxx != r.maxx || miny != r.miny || maxy != r.maxy; }
		void operator &= (RCRect2_T r) { Intersect(*this, r); }

		void operator |= (RCRect2_T r) { UnionRect(*this, r); }

		// merge a point, inflate rect if necessary
		void Merge(T x, T y);
		void Normalize();

		void Union(RCRect2_T r);
		bool Intersect(RCRect2_T r);

		void UnionRect(RCRect2_T r1, RCRect2_T r2);
		bool Intersect(RCRect2_T r1, RCRect2_T r2);
	};
	template<class T>
	void Rect2_T<T>::Merge(T x, T y)
	{
		if (x < minx)	minx = x;
		if (x > maxx)	maxx = x;
		if (y < miny)	miny = y;
		if (y > maxy)	maxy = y;
	}

	template<class T>
	Rect2_T<T>&  Rect2_T<T>::operator=(RCRect2_T src)
	{
		minx = src.minx;
		miny = src.miny;
		maxx = src.maxx;
		maxy = src.maxy;
		return *this;
	}

	template<class T>
	void Rect2_T<T>::Expand(T dx, T dy)
	{
		minx -= dx;
		maxx += dx;
		miny -= dy;
		maxy += dy;
	}

	template<class T>
	void Rect2_T<T>::Normalize()
	{
		if (minx > maxx)  swap(minx, maxx);
		if (miny > maxy)  swap(miny, maxy);
	}

	template<class T>
	bool Rect2_T<T>::Intersect(RCRect2_T r)
	{
		minx = max(minx, r.minx);
		miny = max(miny, r.miny);
		maxx = min(maxx, r.maxx);
		maxy = min(maxy, r.maxy);
		return maxx >= minx && maxy >= miny;
	}

	template<class T>
	bool Rect2_T<T>::Intersect(RCRect2_T r1, RCRect2_T r2)
	{
		minx = max(r1.minx, r2.minx);
		miny = max(r1.miny, r2.miny);
		maxx = min(r1.maxx, r2.maxx);
		maxy = min(r1.maxy, r2.maxy);
		return maxx >= minx && maxy >= miny;
	}

	template<class T>
	void Rect2_T<T>::Union(RCRect2_T r)
	{
		minx = min(minx, r.minx);
		miny = min(miny, r.miny);
		maxx = max(maxx, r.maxx);
		maxy = max(maxy, r.maxy);
	}

	template<class T>
	void Rect2_T<T>::UnionRect(RCRect2_T r1, RCRect2_T r2)
	{
		minx = min(r1.minx, r2.minx);
		miny = min(r1.miny, r2.miny);
		maxx = max(r1.maxx, r2.maxx);
		maxy = max(r1.maxy, r2.maxy);
	}
	/********************************************************************
	*  用于点与矩形关系的标记
	********************************************************************/
	//enum eRectPointRelation
	//{
	//    RF_In		= 0x00,	// 在矩形内
	//    RF_OutLeft	= 0x01,	// 在矩形左边
	//    RF_OutRight	= 0x02,	// 在矩形右边
	//    RF_OutTop	= 0x04,	// 在矩形上边
	//    RF_OutBottom= 0x08,	// 在矩形下边
	//    RF_OnLeft	= 0x10,	// 在矩形左边界上，包括延长线
	//    RF_OnRight	= 0x20, // 在矩形右边界上
	//    RF_OnTop	= 0x40, // 在矩形上边界上
	//    RF_OnBottom	= 0x80, // 在矩形下边界上
	//};

	template<class T>
	int PointInRect(T x, T y, RCRect2_T r, double tolerance)
	{
		int b = RF_In;

		double dis1 = abs(x - r.minx);
		double dis2 = abs(x - r.maxx);
		if (r.minx == r.maxx && dis1 < tolerance && dis2 < tolerance)
			return b;

		if (dis1 < dis2 && dis1 < tolerance)		 // 在左边界上
			b |= RF_OnLeft;
		else if (dis1 > dis2 && dis2 < tolerance) // 在右边界上
			b |= RF_OnRight;
		else if (x < r.minx)						 // 在左边
			b |= RF_OutLeft;
		else if (x > r.maxx)					 // 在右边
			b |= RF_OutRight;

		dis1 = abs(y - r.miny);
		dis2 = abs(y - r.maxy);
		if (r.miny == r.maxy && dis1 < tolerance && dis2 < tolerance)
			return b;

		if (dis1 < dis2 && dis1 < tolerance)		 // 在上边界上
			b |= RF_OnTop;
		else if (dis1 > dis2 && dis2 < tolerance) // 在下边界上
			b |= RF_OnBottom;
		else if (y > r.maxy)					 // 在上边
			b |= RF_OutBottom;
		else if (y < r.miny)						 // 在下边
			b |= RF_OutTop;

		return b;
	}

	/********************************************************************
	*  利用RectPointRelation标记来帮助判断点与矩形的关系
	********************************************************************/
	class RectPointRelationJudge
	{
	public:
		/*
		* 是否在左边界上，包括顶点和延长线
		*/
		static bool IsOnLeftEdge(int rel) { return (rel & RF_OnLeft) == RF_OnLeft; }
		/*
		* 是否在右边界上，包括顶点和延长线
		*/
		static bool IsOnRightEdge(int rel) { return (rel & RF_OnRight) == RF_OnRight; }
		/*
		* 是否在上边界上，包括顶点和延长线
		*/
		static bool IsOnTopEdge(int rel) { return (rel & RF_OnTop) == RF_OnTop; }
		/*
		* 是否在下边界上，包括顶点和延长线
		*/
		static bool IsOnBottomEdge(int rel) { return (rel & RF_OnBottom) == RF_OnBottom; }
		/*
		* 是否在矩形内，不包括边界
		*/
		static bool IsInRect(int rel) { return rel == RF_In; }
		/*
		* 是否在矩形外
		*/
		static bool IsOutRect(int rel) { return (rel & 0x0F) != 0; }
		/*
		* 是否在矩形边界上，不包括边界延长线，不包括顶点
		*/
		static bool IsOnEdge(int rel) { return !IsInRect(rel) && !IsOutRect(rel); }
		/*
		* 是否在矩形边界上，不包括边界延长线，不包括顶点
		* !!! 调用者自己确保不在顶点上
		* @param return : 不在边界上返回0，左边界返回1，右边界返回2，上边界返回3，下边界返回4
		*/
		static int OnEdgeNo(int rel)
		{
			if (!IsInRect(rel) && !IsOutRect(rel))
			{
				if (rel == RF_OnLeft)
					return 1;
				else if (rel == RF_OnRight)
					return 2;
				else if (rel == RF_OnTop)
					return 3;
				else if (rel == RF_OnBottom)
					return 4;
			}
			return 0;
		}
		/*
		* 点所在的矩形顶点序号
		* @param return : 没在定点上为0，左上角顶点为1，右上角顶点为2，右下角顶点为3，左下角顶点为4
		*/
		static int OnVertexNo(int rel)
		{
			if ((rel & 0xF0) != 0) // 至少在边界延长线上
			{
				if ((rel & RF_OnLeft) == RF_OnLeft)
				{
					if ((rel & RF_OnTop) == RF_OnTop)
						return 1;
					else if ((rel & RF_OnBottom) == RF_OnBottom)
						return 4;
				}
				else if ((rel & RF_OnRight) == RF_OnRight)
				{
					if ((rel & RF_OnTop) == RF_OnTop)
						return 2;
					else if ((rel & RF_OnBottom) == RF_OnBottom)
						return 3;
				}
			}
			return 0;
		}
	};
#endif

	/*****************************************************************************
	* @brief : 斜矩形的定义
	* @author : W.W.Frank
	* @date : 2015/11/30 9:27
	* @version : version 1.0
	* @inparam :
	* @outparam :
	*****************************************************************************/
#ifndef _TILT_RECTANGLE_H_
#define _TILT_RECTANGLE_H_
	class TileRect
	{
	public:
		Point2Ds rect_tile_pts;
	};
#endif

	/*****************************************************************************
	* @brief : AABB最小包围盒的定义
	* @author : W.W.Frank
	* @date : 2015/11/30 16:37
	* @version : version 1.0
	* @inparam :
	* @outparam :
	*****************************************************************************/
#ifndef _AABB_H_
#define _AABB_H_
	class AABB
	{
	public:
		/*
		* xyz轴坐标最小点
		*/
		Point3D PointMin;
		/*
		* xyz轴坐标最大点
		*/
		Point3D PointMax;

		double MinX() const { return PointMin.x; }
		double MaxX() const { return PointMax.x; }
		double MinY() const { return PointMin.y; }
		double MaxY() const { return PointMax.y; }
		double MinZ() const { return PointMin.z; }
		double MaxZ() const { return PointMax.z; }

		double WidthX() const { return PointMax.x - PointMin.x; }
		double WidthY() const { return PointMax.y - PointMin.y; }
		double WidthZ() const { return PointMax.z - PointMin.z; }

		AABB() { SetNull(); }
		AABB(const Point3D& min, const Point3D& max) { Assign(min, max); }
		AABB(const AABB& src) { PointMin = src.PointMin; PointMax = src.PointMax; }
		AABB(double mx, double my, double mz, double Mx, double My, double Mz) { Assign(mx, my, mz, Mx, My, Mz); }
		AABB(const Rect2D& r) { Assign(r.minx, r.miny, 0, r.maxx, r.maxy, 0); }

		void SetNull() { PointMax.x = PointMax.y = PointMax.z = -(PointMin.x = PointMin.y = PointMin.z = 1e30); }
		// 	bool IsNull() const	{ return PointMax.x < PointMin.x || PointMax.y < PointMin.y || PointMax.z < PointMin.z; }
		// 	bool IsNull2() const	{ return PointMax.x <= PointMin.x || PointMax.y <= PointMin.y || PointMax.z <= PointMin.z; }

		void Assign(const AABB& aabb) { Assign(aabb.PointMin, aabb.PointMax); }
		void Assign(const Rect2D& r) { Assign(r.minx, r.miny, 0, r.maxx, r.maxy, 0); }
		void Assign(const Point3D& min, const Point3D& max) { PointMin = min; PointMax = max; }
		void Assign(double mx, double my, double mz, double Mx, double My, double Mz) { PointMin.x = mx; PointMin.y = my; PointMin.z = mz; PointMax.x = Mx; PointMax.y = My; PointMax.z = Mz; }
		/*
		* 根据传入的点集计算外包
		*/
		void Assign(const Point2Ds& pts);
		/*
		* scale
		*/
		void Scale(const Point3D& pt, double sx, double sy, double sz);
		void Scale(double sx, double sy, double sz);
		/*
		* offset
		*/
		void Offset(double dx, double dy, double dz);
		void Offset(const Point3D& p) { Offset(p.x, p.y, p.z); }

		Rect2D ToRect2D() const { return Rect2D(PointMin.x, PointMin.y, PointMax.x, PointMax.y); }
		void GetRect2D(Rect2D& r) const { r.Set(PointMin.x, PointMin.y, PointMax.x, PointMax.y); }

		void GetCenter(Point3D& pt) const { pt = PointMax + PointMin; pt = pt * 0.5; }
		void GetSize(Point3D& size) const { size = PointMax - PointMin; }
		/*
		* add a point, inflate rect if necessary
		*/
		void Merge(double x, double y, double z)
		{
			if (x < PointMin.x)	PointMin.x = x;
			if (x > PointMax.x)	PointMax.x = x;
			if (y < PointMin.y)	PointMin.y = y;
			if (y > PointMax.y)	PointMax.y = y;
			if (z < PointMin.z)	PointMin.z = z;
			if (z > PointMax.z)	PointMax.z = z;
		}
		void Merge(const Point3D& v) { Merge(v.x, v.y, v.z); }
		void Merge(const AABB& aabb) { Merge(aabb.PointMin); Merge(aabb.PointMax); }

		void Union(const AABB& rhs);
		/*
		* 返回8个顶点数组，顶点排列顺序如图
		<pre>
		   1-----2
		  /|    /|
		 / |   / |
		5-----4  |
		|  0--|--3
		| /   | /
		|/    |/
		6-----7
		</pre>
		*/
		void GetCorners(Point3D  _Corners[8]) const;
		//新添加，选择是忽略高程 cz 090512***********************************
		bool IsIntersect2D(const AABB& b2) const;
		bool IsInBox2D(const Point3D& pt, bool includeBound = true) const;
		//新添加，选择是忽略高程 cz 090512*****************************************

		bool IsIntersect(const AABB& b2, bool ignorez = false) const;
		AABB Intersection(const AABB& b2) const;
		/*
		* 计算包围盒体积
		*/
		inline double Volume(void) const
		{
			Point3D diff = PointMax - PointMin;
			return diff.x * diff.y * diff.z;
		}
		/*
		* 判断点是否在包围盒内
		*/
		bool IsInBox(const Point3D& pt, bool includeBound = true, bool ignoreZ = false) const
		{
			if(includeBound)
			{
				if(PointMin.x <= pt.x && pt.x <= PointMax.x){
					if(PointMin.y <= pt.y && pt.y <= PointMax.y){
						if(ignoreZ){
							return true;
						}
						else{
							if(PointMin.z <= pt.z && pt.z <= PointMax.z){
								return true;
							}
						}
					}
				}
			}
			else{
				if(PointMin.x < pt.x && pt.x < PointMax.x){
					if(PointMin.y < pt.y && pt.y < PointMax.y){
						if(ignoreZ){
							return true;
						}
						else{
							if(PointMin.z < pt.z && pt.z < PointMax.z){
								return true;
							}
						}
					}
				}
			}
			return false;
		}
		bool IsInBox(const Point3D& pt, double tolerance, bool ignoreZ = false) const
		{
			if(pt.x-PointMin.x >=-tolerance  && PointMax.x-pt.x >=-tolerance ){
				if(pt.y-PointMin.y >=-tolerance  && PointMax.y-pt.y >=-tolerance){
					if(ignoreZ){
						return true;
					}
					else{
						if(pt.z-PointMin.z >=-tolerance  && PointMax.z-pt.z >=-tolerance){
							return true;
						}
					}
				}
			}
			return false;
		}


		AABB& operator=(const AABB& box) { PointMin = box.PointMin; PointMax = box.PointMax; return *this; }
		bool operator==(const AABB& box) const {
			return (PointMin.x == box.PointMin.x) && (PointMin.y == box.PointMin.y) && (PointMin.z == box.PointMin.z) &&
				(PointMax.x == box.PointMax.x) && (PointMax.y == box.PointMax.y) && (PointMax.z == box.PointMax.z);
		}
		bool operator!=(const AABB& box) const {
			return (PointMin.x == box.PointMin.x) && (PointMin.y == box.PointMin.y) && (PointMin.z == box.PointMin.z) &&
				(PointMax.x == box.PointMax.x) && (PointMax.y == box.PointMax.y) && (PointMax.z == box.PointMax.z);
		}
	};
#endif
}

