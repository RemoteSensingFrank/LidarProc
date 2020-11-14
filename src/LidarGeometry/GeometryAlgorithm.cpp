#include <assert.h>
#include <stdio.h>

#include "../LidarGeometry/GeometryAlgorithm.h"
#include "../LidarGeometry/GeometryAlgorithm.h"

#include "../LidarGeometry/tsmToUTM.h"

#pragma warning(disable:4996)
namespace GeometryLas {
	static bool Is_Equal_Eps(double v1, double v2, double epsilon)
	{
		return abs(v1 - v2) <= epsilon;
	}

#ifndef EPSILON
	const static double EPSILON = 1.0e-8;
#endif // !EPSILON

#ifndef EPSILONF
	const static float  EPSILONF = 1.0e-6F;
#endif // !EPSILONF


	int GeometryRelation::IsPointOnPolyline(double x, double y, const double* polyline, int doubleArrayCnt,
		double tolerance, bool isClosed, int offset)
	{
		int ptsCnt = doubleArrayCnt / offset;
		assert(ptsCnt >= 2);

		int cnt = ptsCnt - 1;
		int index, index2;
		for (int k = 0; k < cnt; k++)
		{
			index = k * offset;
			index2 = index + offset;
			if (IsPointOnLine(x, y, polyline[index], polyline[index + 1], polyline[index2],
				polyline[index2 + 1], tolerance))
			{
				return k;
			}
		}
		if (isClosed)
		{
			index = cnt * offset;
			if (IsPointOnLine(x, y, polyline[index], polyline[index + 1], polyline[0], polyline[1], tolerance))
			{
				return cnt;
			}
		}
		return -1;
	}
	bool GeometryRelation::IsPointOnLine(double x, double y, double x0, double y0, double x1, double y1, double tolerance)
	{
		assert(tolerance >= 0);

		Rect2D rect;
		rect.minx = min(x0, x1);
		rect.maxx = max(x0, x1);
		rect.miny = min(y0, y1);
		rect.maxy = max(y0, y1);

		rect.Normalize();
		rect.minx -= tolerance;
		rect.maxx += tolerance;
		rect.miny -= tolerance;
		rect.maxy += tolerance;
		if (rect.IsInclude(x, y))
		{
			return IsPointOnBeeline(x, y, x0, y0, x1, y1, tolerance);
		}
		return false;
	}
	bool GeometryRelation::IsPointOnBeeline(double x, double y, double x0, double y0, double x1, double y1,
		double tolerance)
	{
		assert(tolerance >= 0);

		double dissqr = DistanceComputation::SquarePointToBeeline(x, y, x0, y0, x1, y1);
		return dissqr < tolerance * tolerance;
	}
	bool GeometryRelation::IsPointInPolygon(double x, double y, const double* polygon, double doubleArrayCnt,
		bool allowonisin, double tolerance, int offset)
	{
		int ptsCnt = int(doubleArrayCnt / offset);
		assert(ptsCnt >= 3);

		bool isin = false;
		if (allowonisin)
		{
			int index = IsPointOnPolyline(x, y, polygon, (int)doubleArrayCnt, tolerance, true, offset);
			isin = index != -1;
		}
		if (!isin)
		{
			isin = IsPointInPolygon(x, y, polygon, (int)doubleArrayCnt, offset);
		}
		return isin;
	}
	bool GeometryRelation::IsPointInPolygon(double x, double y, const Point2Ds& polygon)
	{
		return GeometryRelation::IsPointInPolygon(x, y, (const double*)&(polygon[0].x), polygon.size() * 2, Point2D::SizeofPoint2);
	}
	bool GeometryRelation::IsPointInPolygon(double x, double y, const double* polygon,
		int doubleArrayCnt, int offset)
	{
		int ptsCnt = doubleArrayCnt / offset;
		assert(ptsCnt >= 3);

		int wn = 0;    // the winding number counter
		double startx, starty, nextx, nexty;
		for (int i = 0; i < ptsCnt - 1; i++)
		{
			int index = i * offset;
			startx = polygon[index];
			starty = polygon[index + 1];
			nextx = polygon[index + offset];
			nexty = polygon[index + offset + 1];
			if (starty <= y) // start y <= pt->y
			{
				if (nexty > y)      // an upward crossing
					if (IsLeft(startx, starty, nextx, nexty, x, y) > 0)  // P left of edge
						++wn;            // have a valid up intersect
			}
			else// start y > P.y (no test needed)
			{
				if (nexty <= y)     // a downward crossing
					if (IsLeft(startx, starty, nextx, nexty, x, y) < 0)  // P right of edge
						--wn;            // have a valid down intersect
			}
		}
		int index = (ptsCnt - 1) * offset;
		startx = polygon[index];
		starty = polygon[index + 1];
		nextx = polygon[0];
		nexty = polygon[1];
		if (starty <= y) // start y <= pt->y
		{
			if (nexty > y)      // an upward crossing
				if (IsLeft(startx, starty, nextx, nexty, x, y) > 0)  // P left of edge
					++wn;            // have a valid up intersect
		}
		else
		{                       // start y > P.y (no test needed)
			if (nexty <= y)     // a downward crossing
				if (IsLeft(startx, starty, nextx, nexty, x, y) < 0)  // P right of edge
					--wn;            // have a valid down intersect
		}

		if (wn == 0)
			return false;
		return true;
	}
	double GeometryRelation::IsLeft(double startx, double starty, double endx, double endy,
		double x, double y)
	{
		return ((endx - startx) * (y - starty) - (x - startx) * (endy - starty));
	}
	bool GeometryRelation::IsPointOnRect(double x, double y, double xmin, double ymin,
		double xmax, double ymax, double tolerance)
	{
		bool b = (x >= xmin - tolerance && x <= xmax + tolerance
			&& (Is_Equal_Eps(y, ymin, tolerance) || Is_Equal_Eps(y, ymax, tolerance)))
			|| (y >= ymin - tolerance && y <= ymax + tolerance
				&& (Is_Equal_Eps(x, xmin, tolerance) || Is_Equal_Eps(x, xmax, tolerance)));
		return b;
	}
	bool GeometryRelation::IsPolylineInRect(const double* polyline, int doubleArrayCnt, double minx,
		double miny, double maxx, double maxy, int offset)
	{
		Rect2D user(minx, miny, maxx, maxy);
		user.Normalize();

		int ptsCnt = doubleArrayCnt / offset;
		for (int i = 0; i < ptsCnt; i++)
		{
			int index = i * offset;
			double x = polyline[index];
			double y = polyline[index + 1];
			if (!user.IsInclude(x, y))
			{
				return false;
			}
		}
		return true;
	}
	bool GeometryRelation::IsLineIntersectRect(double x0, double y0, double x1, double y1, double xmin,
		double ymin, double xmax, double ymax)
	{
		double deltax = x1 - x0;
		double deltay = y1 - y0;

		if (deltax == 0 && deltay == 0)
		{
			if (GeometryRelation::IsPointOnRect(x0, y0, xmin, ymin, xmax, ymax, EPSILON))
			{
				return true;
			}
		}
		else
		{
			if (deltax != 0)
			{
				//????
				double u = (xmin - x0) / deltax;
				if (u >= 0 && u <= 1)//???xmin???line??
				{
					double y = y0 + u * deltay;
					if (y >= ymin && y <= ymax)
					{
						return true;
					}
				}

				//????
				u = (xmax - x0) / deltax;
				if (u >= 0 && u <= 1)//???xmax???line??
				{
					double y = y0 + u * deltay;
					if (y >= ymin && y <= ymax)
					{
						return true;
					}
				}
			}
			if (deltay != 0)
			{
				//???
				double u = (ymin - y0) / deltay;
				if (u >= 0 && u <= 1)//???ymin???line??
				{
					double x = x0 + u * deltax;
					if (x >= xmin && x <= xmax)
					{
						return true;
					}
				}

				//????
				u = (ymax - y0) / deltay;
				if (u >= 0 && u <= 1)//???ymax???line??
				{
					double x = x0 + u * deltax;
					if (x >= xmin && x <= xmax)
					{
						return true;
					}
				}
			}
		}
		return false;
	}
	bool GeometryRelation::IsRectIntersectPolyline(double xmin, double ymin, double xmax,
		double ymax, const double* polyline, int doubleArrayCnt, bool isClosed, int offset)
	{
		int ptsCnt = doubleArrayCnt / offset;
		int index = -1, index2 = -1;
		for (int k = 1; k < ptsCnt; k++)
		{
			index2 = k * offset;
			index = index2 - offset;
			double x1 = polyline[index];
			double y1 = polyline[index + 1];
			double x2 = polyline[index2];
			double y2 = polyline[index2 + 1];
			if (IsLineIntersectRect(x1, y1, x2, y2, xmin, ymin, xmax, ymax))
			{
				return true;
			}
		}

		if (isClosed)
		{
			index = (ptsCnt - 1) * offset;
			double x1 = polyline[index];
			double y1 = polyline[index + 1];
			double x2 = polyline[0];
			double y2 = polyline[1];
			if (IsLineIntersectRect(x1, y1, x2, y2, xmin, ymin, xmax, ymax))
			{
				return true;
			}
		}

		return false;
	}
	bool GeometryRelation::IsPointInRect(int x, int y, int rect_x1, int rect_y1, int rect_x2, int rect_y2)
	{
		if (((x < rect_x1 && x > rect_x2) || (x > rect_x1 && x < rect_x2)) &&
			((y < rect_y1 && y > rect_y2) || (y > rect_y1 && y < rect_y2)))
			return true;
		return false;
	}
	bool GeometryRelation::IsPointInRect(double x, double y, double rect_x1, double rect_y1, double rect_x2, double rect_y2)
	{
		if (((x < rect_x1 && x > rect_x2) || (x > rect_x1 && x < rect_x2)) &&
			((y < rect_y1 && y > rect_y2) || (y > rect_y1 && y < rect_y2)))
			return true;
		return false;
	}
	bool GeometryRelation::IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts)
	{
		assert(pts.size() == 4);
		Rect2D bound;       // ?????????

		size_t ptId = 0;
		size_t ptCnt = pts.size();
		for (ptId = 0; ptId < ptCnt; ptId++)
		{
			bound.Merge(pts[ptId].x, pts[ptId].y);
		}

		return GeometryRelation::IsRectIntersectSlantingRect(rect, pts, bound);
	}
	bool GeometryRelation::IsRectIntersectSlantingRect(const Rect2D& rect, const Point2Ds& pts,
		const Rect2D& slantingRect)
	{
		if (!rect.IsIntersect(slantingRect))
		{
			return false;
		}

		size_t lineId = 0;
		size_t startPtId, endPtId, crossPtId;
		for (lineId = 0; lineId < 4; lineId++)
		{
			startPtId = lineId;
			endPtId = (startPtId + 1) % 4;
			crossPtId = (endPtId + 1) % 4;

			double crossDir = GeometryRelation::IsLeft(pts[startPtId].x, pts[startPtId].y,
				pts[endPtId].x, pts[endPtId].y, pts[crossPtId].x, pts[crossPtId].y);

			double curDir0 = GeometryRelation::IsLeft(pts[startPtId].x, pts[startPtId].y,
				pts[endPtId].x, pts[endPtId].y, rect.minx, rect.miny);
			if (crossDir > EPSILON && curDir0 > EPSILON ||
				crossDir < -EPSILON && curDir0 < -EPSILON)
			{
				continue;
			}

			double curDir1 = GeometryRelation::IsLeft(pts[startPtId].x, pts[startPtId].y,
				pts[endPtId].x, pts[endPtId].y, rect.minx, rect.maxy);
			if (crossDir > EPSILON && curDir1 > EPSILON ||
				crossDir < -EPSILON && curDir1 < -EPSILON)
			{
				continue;
			}

			double curDir2 = GeometryRelation::IsLeft(pts[startPtId].x, pts[startPtId].y,
				pts[endPtId].x, pts[endPtId].y, rect.maxx, rect.miny);
			if (crossDir > EPSILON && curDir2 > EPSILON ||
				crossDir < -EPSILON && curDir2 < -EPSILON)
			{
				continue;
			}

			double curDir3 = GeometryRelation::IsLeft(pts[startPtId].x, pts[startPtId].y,
				pts[endPtId].x, pts[endPtId].y, rect.maxx, rect.maxy);
			if (crossDir > EPSILON && curDir3 > EPSILON ||
				crossDir < -EPSILON && curDir3 < -EPSILON)
			{
				continue;
			}

			return false;
		}

		return true;
	}
	bool GeometryRelation::IsRectIntersectRect(const Rect2D& rect1, const Rect2D& rect2)
	{
		//�жϾ����ཻ�Ĵ����д���
		//if (IsPointInRect(rect1.minx, rect1.miny, rect2.minx, rect2.miny, rect2.maxx, rect2.maxy))
		//{
		//	return true;
		//}
		//else if (IsPointInRect(rect1.minx, rect1.maxy, rect2.minx, rect2.miny, rect2.maxx, rect2.maxy))
		//{
		//	return true;
		//}
		//else if (IsPointInRect(rect1.maxx, rect1.miny, rect2.minx, rect2.miny, rect2.maxx, rect2.maxy))
		//{
		//	return true;
		//}
		//else if (IsPointInRect(rect1.maxx, rect1.maxy, rect2.minx, rect2.miny, rect2.maxx, rect2.maxy))
		//{
		//	return true;
		//}
		//else if (IsPointInRect(rect2.minx, rect2.miny, rect1.minx, rect1.miny, rect1.maxx, rect1.maxy))
		//{
		//	return true;
		//}
		//else if (IsPointInRect(rect2.minx, rect2.maxy, rect1.minx, rect1.miny, rect1.maxx, rect1.maxy))
		//{
		//	return true;
		//}
		//else if (IsPointInRect(rect2.maxx, rect2.miny, rect1.minx, rect1.miny, rect1.maxx, rect1.maxy))
		//{
		//	return true;
		//}
		//else if (IsPointInRect(rect2.maxx, rect2.maxy, rect1.minx, rect1.miny, rect1.maxx, rect1.maxy))
		//{
		//	return true;
		//}
		//else
		//	return false;

		double  minx = max(rect1.minx, rect2.minx);
		double 	miny = max(rect1.miny, rect2.miny);
		double 	maxx = min(rect1.maxx, rect2.maxx);
		double 	maxy = min(rect1.maxy, rect2.maxy);

		if (minx > maxx || miny > maxy)
		{
			//#ifdef _DEBUG
			//		FILE* ofs = fopen("E:\\rect.txt", "a+");
			//		fprintf(ofs, "%lf,%lf,%lf,%lf;%lf,%lf,%lf,%lf;\n", rect1.minx, rect1.miny, rect1.maxx, rect1.maxy,
			//			rect2.minx, rect2.miny, rect2.maxx, rect2.maxy);
			//		fclose(ofs);
			//#endif // DEBUG
			return false;
		}
		else
			return true;

	}

	double GeometryRelation::VectorAngle(const Point3D vec1, const Point3D vec2)
	{
		double aDotb[3] = { vec1.x*vec2.x,vec1.y*vec2.y ,vec1.z*vec2.z };
		double aMo = sqrt(vec1.x*vec1.x + vec1.y*vec1.y + vec1.z*vec1.z);
		double bMo = sqrt(vec2.x*vec2.x + vec2.y*vec2.y + vec2.z*vec2.z);
		double abMo = sqrt(aDotb[0] * aDotb[0] + aDotb[1] * aDotb[1] + aDotb[2] * aDotb[2]);

		return acos(abMo / aMo / bMo);
	}

	void GeometryRelation::Normalize(const Point3D vec1,Point3D &vec2)
	{
		double dlen = sqrt(vec1.x*vec1.x+vec1.y*vec1.y +vec1.z*vec1.z);
		vec2.x = vec1.x/dlen;
		vec2.y = vec1.y/dlen;
		vec2.z = vec1.z/dlen;
	}

	Point3D GeometryRelation::ProjectionPoint(const Point3D pnt1,Point3Ds line)
	{
		Point3D ac,ab;
		ac.x=pnt1.x-line[0].x; ac.y=pnt1.y-line[0].y; ac.z=pnt1.z-line[0].z;
		ab.x=line[1].x-line[0].x;ab.y=line[1].y-line[0].y;ab.z=line[1].z-line[0].z;
		double acDotab = ac.x*ab.x+ac.y*ab.y+ac.z*ab.z;
		double dis2_ab = ab.x*ab.x+ab.y*ab.y+ab.z*ab.z;
		double d22 = acDotab*acDotab/dis2_ab;
		double ratio = d22/dis2_ab;


		if(acDotab>0)
		{
			return Point3D(line[0].x+ab.x*ratio,line[0].y+ab.y*ratio,line[0].z+ab.z*ratio);
		}else{
			ratio=ratio*(-1);
			return Point3D(line[0].x+ab.x*ratio,line[0].y+ab.y*ratio,line[0].z+ab.z*ratio);
		}
	}

	/*****************************************************************************
	* @brief : ???????
	* @author : W.W.Frank
	* @date : 2015/11/30 14:53
	* @version : version 1.0
	* @inparam :
	* @outparam :
	*****************************************************************************/
	double DistanceComputation::SquarePointToBeeline(double x, double y, double x0, double y0,
		double x1, double y1)
	{
		Point2D pt = PointJointComputation::PointJointBeeline(x, y, x0, y0, x1, y1);
		double xx = pt.x - x;
		double yy = pt.y - y;
		return xx * xx + yy * yy;
	}
	double DistanceComputation::Distance(Point2Ds& pts, bool isclosed)
	{
		double dis = 0.0;
		for (size_t i = 0; i < pts.size() - 1; i++)
		{
			dis += pts[i].Distance(pts[i + 1]);
		}
		if (isclosed)
		{
			dis += pts[pts.size() - 1].Distance(pts[0]);
		}
		return dis;
	}

	double DistanceComputation::Distance(Point3D pt1, Point3D pt2)
	{
		return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) +
			(pt1.y - pt2.y)*(pt1.y - pt2.y) +
			(pt1.z - pt2.z)*(pt1.z - pt2.z));
	}

	double DistanceComputation::Distance(Point3D pt, Point3D pl1, Point3D pl2,bool segment/*=true*/)
	{
		// reference: http://wuweiblog.com/2020/11/02/%E7%82%B9%E5%88%B0%E4%B8%89%E7%BB%B4%E7%A9%BA%E9%97%B4%E7%9B%B4%E7%BA%BF%E8%B7%9D%E7%A6%BB%E8%AE%A1%E7%AE%97%E4%BB%A5%E5%90%91%E9%87%8F%E6%96%B9%E5%BC%8F%E8%AE%A1%E7%AE%97/
		Point3D ac,ab;
		ac.x=pt.x-pl1.x; ac.y=pt.y-pl1.y; ac.z=pt.z-pl1.z;
		ab.x=pl2.x-pl1.x;ab.y=pl2.y-pl1.y;ab.z=pl2.z-pl1.z;

		double acDotab = ac.x*ab.x+ac.y*ab.y+ac.z*ab.z;
		double dis2_ac = ac.x*ac.x+ac.y*ac.y+ac.z*ac.z;
		double dis2_ab = ab.x*ab.x+ab.y*ab.y+ab.z*ab.z;

		double d12 = dis2_ac;
		double d22 = acDotab*acDotab/dis2_ab;

		// //just for check the rightness of the computation
		// if(acDotab/sqrt(dis2_ac)/sqrt(dis2_ab)>1)
		// {
		// 	printf("%lf, %lf,%lf\n",acDotab,sqrt(dis2_ac),sqrt(dis2_ab));
		// }

		//if compute the segment
		if(segment)
		{
			if(acDotab/sqrt(dis2_ac)/sqrt(dis2_ab)<0)
			{
				return Distance(pt,pl1);
			}else if(acDotab/sqrt(dis2_ab)>sqrt(dis2_ab)){
				return Distance(pt,pl2);
			}else{
				return sqrt(d12-d22);
			}
		}
		else{
			return sqrt(d12-d22);
		}
	}

	double DistanceComputation::Distance(Point3D pt1, Point3D pl1, Point3D pl2, Point3D pl3)
	{
		double a = (pl2.y - pl1.y)*(pl3.z - pl1.z) - (pl2.z - pl1.z)*(pl3.y - pl1.y);
		double b = (pl2.z - pl1.z)*(pl3.x - pl1.x) - (pl2.x - pl1.x)*(pl3.z - pl1.z);
		double c = (pl2.x - pl1.x)*(pl3.y - pl1.y) - (pl2.y - pl1.y)*(pl3.x - pl1.x);
		double d = 0 - (a * pl1.x + b*pl1.y + c*pl1.z);

		return fabs(a * pt1.x + b*pt1.y + c*pt1.z + d) / sqrt(a * a + b * b + c * c);
	}

	/*****************************************************************************
	* @brief : ?????????
	* @author : W.W.Frank
	* @date : 2015/11/30 14:55
	* @version : version 1.0
	* @inparam :
	* @outparam :
	*****************************************************************************/
	void PointJointComputation::PointJointBeeline(double x, double y, double bln0x, double bln0y, double bln1x,
		double bln1y, double& ratio)
	{
		double x0 = bln0x;
		double y0 = bln0y;
		double x1 = bln1x;
		double y1 = bln1y;
		double deltax = x1 - x0;
		double deltay = y1 - y0;
		double xp = x;
		double yp = y;

		if (deltay == 0 && deltax == 0)
			ratio = 0;
		else
			ratio = ((xp - x0) * deltax + (yp - y0) * deltay) / (deltax * deltax + deltay * deltay);
	}

	Point2D PointJointComputation::PointJointBeeline(double x, double y, double bln0x, double bln0y, double bln1x,
		double bln1y)
	{
		double u;
		PointJointBeeline(x, y, bln0x, bln0y, bln1x, bln1y, u);
		double x0 = bln0x;
		double y0 = bln0y;
		double x1 = bln1x;
		double y1 = bln1y;
		double deltax = x1 - x0;
		double deltay = y1 - y0;

		return Point2D(x0 + u * deltax, y0 + u * deltay);
	}

	void PointProjection::PointProjectLatLngToUTM(double lat, double lng, int nZone, double &x, double &y)
	{
		tsmLatLongToUTM(lat, lng, &nZone, &x, &y);
	}

	vector <Point2D> GeometryComputation::ProjectToXOY(vector<Point3D> pntCloud)
	{
		vector <Point2D> point2D;
		for (size_t i = 0; i < pntCloud.size(); i++)
		{
			point2D.push_back(Point2D(pntCloud[i].x, pntCloud[i].y));
		}
		return point2D;
	}

	vector <Point2D> GeometryComputation::ProjectToSection(vector<Point3D> pntCloud, Point3D Opoint)
	{
		vector <Point2D> point2D;
		for (int i = 0; i < pntCloud.size(); i++)
		{
			double x = sqrt(pow((pntCloud[i].x - Opoint.x), 2) + pow((pntCloud[i].y - Opoint.y), 2));
			double y = pntCloud[i].z - Opoint.z;
			point2D.push_back(Point2D(pntCloud[i].x, pntCloud[i].y));
		}
		return point2D;
	}

	Point2D GeometryComputation::GetFootOfPerpendicular(vector <double> lineFactors, Point2D knownPoint)
	{
		if (lineFactors[1] == 0)
		{
			assert(-1);
		}
		Point2D resPoint;
		//resPoint.x = (lineFactors[1] * knownPoint.x + knownPoint.y - lineFactors[0]) / (2 * lineFactors[1]);
		//resPoint.y = (lineFactors[1] * knownPoint.x + knownPoint.y + lineFactors[0]) / 2.0;
		resPoint.x = (knownPoint.x + lineFactors[1] * knownPoint.y - lineFactors[1] * lineFactors[0]) / (pow(lineFactors[1], 2.0) + 1.0);
		resPoint.y = (lineFactors[1] * knownPoint.x + pow(lineFactors[1], 2.0) * knownPoint.y + lineFactors[0]) / (pow(lineFactors[1], 2.0) + 1.0);
		return resPoint;
	}

	double GeometryComputation::GetYByLineFactors(vector <double> lineFactors, double x)
	{
		double result = 0;
		for (int i = 0; i < lineFactors.size(); i++)
		{
			result += pow(x, i) * lineFactors[i];
		}
		return result;
	}

	Point2D GeometryComputation::GetXYByFactorsOffset(double x, vector <double> lineFactors, Point2D offsetPoint)
	{
		Point2D resPoint;
		double y = 0;
		for (int i = 0; i < lineFactors.size(); i++)
		{
			y += pow(x, i) * lineFactors[i];
		}
		resPoint.x = x + offsetPoint.x;
		resPoint.y = y + offsetPoint.y;
		return resPoint;
	}
}