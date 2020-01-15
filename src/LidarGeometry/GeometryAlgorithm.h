#pragma once
#ifndef _GEOMETRY_ALGORITHMS_H_
#define _GEOMETRY_ALGORITHMS_H_

#include "Eigen/Eigen"
#include "Geometry.h"

using namespace Eigen;

namespace GeometryLas {
	class  GeometryRelation
	{
	public:
		static int IsPointOnPolyline(double x, double y, const double* polyline, int doubleArrayCnt,
			double tolerance, bool isClosed, int offset = sizeof(Point3D) / sizeof(double));
		/*
		* �жϵ��Ƿ����߶��ϣ��ռ��ϵ��line contains point��
		*/
		static bool IsPointOnLine(double x, double y, double x0, double y0, double x1, double y1,
			double tolerance);
		/*
		* �жϵ��Ƿ���ֱ���ϣ��ռ��ϵ��line contains point��
		*/
		static bool IsPointOnBeeline(double x, double y, double x0, double y0, double x1, double y1,
			double tolerance);
		/*
		* �жϵ��Ƿ��ڶ����������ڱ߽��ϣ��ռ��ϵ��region contains point��
		* ˳ʱ�����ÿ����
		* @param allowonisin : �Ƿ���Ϊ�ڱ߽���Ҳ���ڱ߽��ڣ�Ϊtrue����tolerance��Ч��������Ч
		*/
		static bool IsPointInPolygon(double x, double y, const double* polygon, double doubleArrayCnt,
			bool allowonisin, double tolerance, int offset = sizeof(Point3D) / sizeof(double));
		/*
		* �жϵ��Ƿ��ڶ������
		*/
		static bool IsPointInPolygon(double x, double y, const double* polygon,
			int doubleArrayCnt, int offset = sizeof(Point3D) / sizeof(double));
		static bool IsPointInPolygon(double x, double y, const Point2Ds& polygon);
		/*
		* used for IsPointInPolygon follows
		*/
		static double IsLeft(double startx, double starty, double endx, double endy, double x, double y);
		/*
		* ���Ƿ��ھ��α߽��ϣ��ռ��ϵ��lines contains point��
		*/
		static bool IsPointOnRect(double x, double y, double minx, double miny,
			double maxx, double maxy, double tolerance);
		/*
		* �ж����ߡ�������Ƿ��ھ�������������ھ��α߽��ϣ��ռ��ϵ��region contains region, region contains lines��
		*/
		static bool IsPolylineInRect(const double* polyline, int doubleArrayCnt, double minx,
			double miny, double maxx, double maxy, int offset = sizeof(Point3D) / sizeof(double));
		/*
		* �ж��߶�����α߽��Ƿ��ཻ�������߽�㣨�ռ��ϵ��line crosses line��
		*/
		static bool IsLineIntersectRect(double x0, double y0, double x1, double y1, double xmin,
			double ymin, double xmax, double ymax);
		/*
		* �жϾ��α߽��������Ƿ��ཻ�������߽�㣨�ռ��ϵ��line crosses line��
		*/
		static bool IsRectIntersectPolyline(double xmin, double ymin, double xmax,
			double ymax, const double* polyline, int doubleArrayCnt, bool isClosed,
			int offset = sizeof(Point3D) / sizeof(double));
		static bool IsPointInRect(int x, int y, int rect_x1, int rect_y1, int rect_x2, int rect_y2);
		static bool IsPointInRect(double x, double y, double rect_x1, double rect_y1, double rect_x2, double rect_y2);
		/*
		* �ж���������б�����Ƿ��ཻ
		*/
		static bool IsRectIntersectRect(const Rect2D& rect1, const Rect2D& rect2);
		static bool IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts);
		static bool IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts,
			const Rect2D& slantingRect);

		/*
		* �����������ļн�
		*/
		static double VectorAngle(const Point3D vec1, const Point3D vec2);

		/** 
		 * @name: 归一化 
		 * @msg: 输入向量归一化，规则化
		 * @param const Point3D vec1:输入向量
		 * 		  Point3D &vec2:归一化后的向量
		 * @return: 
		 */
		static void Normalize(const Point3D vec1,Point3D &vec2);
	};

	/*������㺯��*/
	class  DistanceComputation
	{
	public:
		/*
		* �㵽ֱ�ߵľ����ƽ��
		*/
		static double SquarePointToBeeline(double x, double y, double x0, double y0, double x1, double y1);
		/*
		* ����㼯�����е��ľ���֮�ͣ�ŷ�Ͼ���
		*/
		static double Distance(Point2Ds& pts, bool isclosed);

		/*
		* ���������ľ���
		*/
		static double Distance(Point3D pt1, Point3D pt2);

		/*
		* ����㵽��֮��ľ���
		*/
		static double Distance(Point3D pt1, Point3D pl1, Point3D pl2, Point3D pl3);

	};

	/*�����ཻ����*/
	class  PointJointComputation
	{
	public:
		/*
		* ��(x, y)��ֱ��(bln0x, bln0y)(bln1x, bln1y)�Ĵ���ռ�߶εı���
		*/
		static void PointJointBeeline(double x, double y, double bln0x, double bln0y, double bln1x,
			double bln1y, double& ratio);
		/*
		* ��(x, y)��ֱ��(bln0x, bln0y)(bln1x, bln1y)�Ĵ���
		*/
		static Point2D PointJointBeeline(double x, double y, double bln0x, double bln0y, double bln1x,
			double bln1y);
	};

	/*ͶӰ����ת��*/
	class  PointProjection
	{
	public:
		/*��γ������ת����ͶӰ����*/
		static void PointProjectLatLngToUTM(double lat, double lng, int nZone, double &x, double &y);
	};

	/*���μ���*/
	class  GeometryComputation
	{
	public:
		/**
		* ��С����������ϣ���άXY��
		* @param point  �������ϵƽ������㼯
		* @param maxPower  ����ʽ���������ݣ�
		* @return ����ʽϵ�����ӵ͵��ߣ�������0��ʼ��
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
		* ����������С����������ϣ���άXY��(ȥ���ֲ���Ӱ��)
		* @param points    �������ϵƽ������㼯(���ؽ���л�ȥ���ֲ��)
		* @param residual  ����Ϊ������ֹ��ֵ������Ϊƽ���в�
		* @param maxPower  ����ʽ���������ݣ�
		* @return ����ʽϵ�����ӵ͵��ߣ�������0��ʼ��
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
		* ͶӰ��XOYƽ��
		* @param pntCloud  ��ά����
		* @return ��ά��XOY����ϵƽ������㼯
		*/
		static vector <Point2D> ProjectToXOY(vector<Point3D> pntCloud);

		/**
		* ͶӰ��XOYƽ��
		* @param pntCloud  ��ά����
		* @param Opoint    ��������ϵ������ԭ��
		* @return ��������ϵƽ������㼯
		*/
		static vector <Point2D> ProjectToSection(vector<Point3D> pntCloud, Point3D Opoint);//��ʱû�õ�

		/**
		 * ��ֱ֪�ߵ�б�ʷ��̣����㵽ֱ�ߵĴ���
		 * @param lineFactors ֱ�߷��̲������ӵͽ׵��߽�
		 * @param knownPoint  ��֪��
		 * @return
		 */
		static Point2D GetFootOfPerpendicular(vector <double> lineFactors, Point2D knownPoint);

		/**
		* ��ֱ֪�ߵ�б�ʷ��̣����㵽ֱ�ߵĴ���
		* @param lineFactors ֱ�߷��̲������ӵͽ׵��߽�
		* @param knownPoint  ��֪��
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
		* ��֪����ʽ���̺�X��Y
		* @param lineFactors ����ʽ���̣��ӵͽ׵��߽�
		* @param x
		* @return
		*/
		static double GetYByLineFactors(vector <double> lineFactors, double x);

		/**
		* ��֪����ʽ���̡���֪x�����XYƫ��������Y����
		* @param x  ��֪x����
		* @param lineFactors ����ʽ���̣��ӵͽ׵��߽�
		* @param offsetPoint ƫ������
		* @return
		*/
		static Point2D GetXYByFactorsOffset(double x, vector <double> lineFactors, Point2D offsetPoint);//��ʱû�õ�
	};

#endif
}
