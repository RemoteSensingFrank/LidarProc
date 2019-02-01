#pragma once
#ifndef _GEOMETRY_ALGORITHMS_H_
#define _GEOMETRY_ALGORITHMS_H_

#include "Geometry.h"
namespace GeometryLas {
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
		static bool IsPointInRect(double x, double y, double rect_x1, double rect_y1, double rect_x2, double rect_y2);
		/*
		* 判断正矩形与斜矩形是否相交
		*/
		static bool IsRectIntersectRect(const Rect2D& rect1, const Rect2D& rect2);
		static bool IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts);
		static bool IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts,
			const Rect2D& slantingRect);

		/*
		* 求两个向量的夹角
		*/
		static double VectorAngle(const Point3D vec1, const Point3D vec2);
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

		/*
		* 计算点到面之间的距离
		*/
		static double Distance(Point3D pt1, Point3D pl1, Point3D pl2, Point3D pl3);

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

	/*几何计算*/
	class  GeometryComputation
	{
	public:
		/**
		* 最小二乘曲线拟合（二维XY）
		* @param point  相对坐标系平面坐标点集
		* @param maxPower  多项式最大阶数（幂）
		* @return 多项式系数，从低到高（阶数从0开始）
		*/
		//template <typename T, typename T1>
		//static vector <double> LeastSquare(T point, T1 maxPower);
		template <typename T, typename T1>
		static vector <double> LeastSquare(T point, T1 maxPower)
		{
			vector <double> factors;
			int rows = point.size();
			MatrixXd A(rows, maxPower + 1);
			VectorXd b(rows);
			for (int i = 0; i < rows; i++)
			{
				for (int j = 0; j < maxPower + 1; j++)
				{
					A(i, j) = pow(point[i].x, j);
				}
				b(i) = point[i].y;
			}
			VectorXd d = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
			for (size_t k = 0; k < d.rows(); k++)
			{
				factors.push_back(d(k));
			}
			return factors;
		}


		/**
		* 抗差整体最小二乘曲线拟合（二维XY）(去除粗差点的影响)
		* @param points    相对坐标系平面坐标点集(返回结果中会去除粗差点)
		* @param residual  输入为迭代终止阈值，返回为平均残差
		* @param maxPower  多项式最大阶数（幂）
		* @return 多项式系数，从低到高（阶数从0开始）
		*/
		//template <typename T, typename T1, typename T2>
		//static vector <double> RobustLeastSquare(vector <T> &points, T1 &residual, T2 maxPower);
		template <typename T, typename T1, typename T2>
		static vector <double> RobustLeastSquare(vector <T> &points, T1 &residual, T2 maxPower)
		{
			int iteration = 0;
			vector <double> factors;
			VectorXd F;

			vector <T> tmpPoints;
			double residualNorm = residual + 1;
			double thelta0 = 0;
			while (residualNorm > residual)
			{
				int rows = points.size();
				MatrixXd A(rows, maxPower + 1);
				VectorXd L(rows);
				VectorXd EL(rows);
				for (int i = 0; i < rows; i++)
				{
					for (int j = 0; j < maxPower + 1; j++)
					{
						A(i, j) = pow(points[i].x, j);
					}
					L(i) = points[i].y;
				}
				F = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(L);
				for (size_t i = 0; i < rows; i++)
				{
					Point2D knownPoint(points[i].x, points[i].y);
					EL(i) = GetPointToLineDistance(F, knownPoint);
				}
				thelta0 = EL.norm() / sqrt(EL.rows());
				residualNorm = thelta0;

				for (size_t i = 0; i < EL.rows(); i++)
				{
					if (abs(EL(i)) < 3 * thelta0)
					{
						tmpPoints.push_back(points[i]);
					}
				}
				if (tmpPoints.size() == points.size())
				{
					vector <T>().swap(tmpPoints);
					break;
				}
				vector <T>().swap(points);
				points = tmpPoints;
				vector <T>().swap(tmpPoints);
				iteration++;
			}

			residual = residualNorm;
			for (size_t k = 0; k < F.rows(); k++)
			{
				factors.push_back(F(k));
			}
			return factors;
		}

		/**
		* 投影到XOY平面
		* @param pntCloud  三维点云
		* @return 二维的XOY坐标系平面坐标点集
		*/
		static vector <Point2D> ProjectToXOY(vector<Point3D> pntCloud);

		/**
		* 投影到XOY平面
		* @param pntCloud  三维点云
		* @param Opoint    建立坐标系的坐标原点
		* @return 断面坐标系平面坐标点集
		*/
		static vector <Point2D> ProjectToSection(vector<Point3D> pntCloud, Point3D Opoint);//暂时没用到

																						   /**
																						   * 已知直线的斜率方程，求解点到直线的垂足
																						   * @param lineFactors 直线方程参数，从低阶到高阶
																						   * @param knownPoint  已知点
																						   * @return
																						   */
		static Point2D GetFootOfPerpendicular(vector <double> lineFactors, Point2D knownPoint);

		/**
		* 已知直线的斜率方程，求解点到直线的垂距
		* @param lineFactors 直线方程参数，从低阶到高阶
		* @param knownPoint  已知点
		* @return
		*/
		//template <typename T, typename T1>
		//static double GetPointToLineDistance(T lineFactors, T1 knownPoint);
		template <typename T, typename T1>
		static double GetPointToLineDistance(T lineFactors, T1 knownPoint)
		{
			double dis = 0;
			if (lineFactors[1] == 0)
			{
				assert(-1);
			}
			dis = abs((lineFactors[1] * knownPoint.x - knownPoint.y + lineFactors[0]) / sqrt(pow(lineFactors[1], 2) + 1));
			return dis;
		}

		/**
		* 已知多项式方程和X求Y
		* @param lineFactors 多项式方程，从低阶到高阶
		* @param x
		* @return
		*/
		static double GetYByLineFactors(vector <double> lineFactors, double x);

		/**
		* 已知多项式方程、已知x坐标和XY偏移坐标求Y坐标
		* @param x  已知x坐标
		* @param lineFactors 多项式方程，从低阶到高阶
		* @param offsetPoint 偏移坐标
		* @return
		*/
		static Point2D GetXYByFactorsOffset(double x, vector <double> lineFactors, Point2D offsetPoint);//暂时没用到
	};

#endif
}
