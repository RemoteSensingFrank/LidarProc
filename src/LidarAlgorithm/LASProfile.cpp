
#include "LASProfile.h"

#include<iostream>
#include<cmath>
#include<gdal_priv.h>
#include"../LidarGeometry/GeometryAlgorithm.h"
#include"../LidarBase/LASReader.h"

#pragma  comment(lib, "gdal_i.lib")

#ifdef _USE_OPENCV_
	#include<opencv2/imgproc.hpp>  
	#include<opencv2/highgui.hpp>  
	#include<opencv2/core.hpp>
	#ifdef win32
		#ifdef _DEBUG
			#pragma  comment(lib, "opencv_world400d.lib")
		#else
			#pragma  comment(lib, "opencv_world400.lib")
		#endif // DEBUG
	#endif
#endif

#define TOWER_ORDER

#ifdef max
#define max a>b?a:b
#endif


#ifdef min
#define min a>b?b:a
#endif

const int bordersize1  	 = 80;
const int bordersize2 	 = 130;
const int bordersize3    = 10;

void  LASProfile::LASProfile_GetPCARotMat(Point2D pntTowers[2], Eigen::MatrixXd &rotMat)
{
	double theta = 0;
	if (pntTowers[1].x != pntTowers[0].x)
	{
		theta = atan2(pntTowers[1].y - pntTowers[0].y, pntTowers[1].x - pntTowers[0].x);
	}
	rotMat = Eigen::MatrixXd::Zero(2, 2);
	rotMat(0, 0) = cos(theta);
	rotMat(0, 1) = -sin(theta);
	rotMat(1, 0) = sin(theta);
	rotMat(1, 1) = cos(theta);
	//rotMat = rotMat.transpose();
}

void LASProfile::LASProfile_GetPointsRange(Point2D pntTowers[2], Eigen::MatrixXd rotMat,Rect2D &rect, float range)
{
	Eigen::MatrixXd p1(1, 2), p2(1, 2);
	p1(0, 0) = pntTowers[0].x; p1(0, 1) = pntTowers[0].y;
	p2(0, 0) = pntTowers[1].x; p2(0, 1) = pntTowers[1].y;
	Eigen::MatrixXd rp1 = p1 * rotMat;
	Eigen::MatrixXd rp2 = p2 * rotMat;

	rect.minx = min(rp1(0, 0), rp2(0, 0)) - range;
	rect.miny = min(rp1(0, 1), rp2(0, 1)) - range;
	rect.maxx = max(rp1(0, 0), rp2(0, 0)) + range;
	rect.maxy = max(rp1(0, 1), rp2(0, 1)) + range;
}

#ifdef _USE_OPENCV_
void LASProfile::LASProfile_ImageFillHorizontal(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, cv::Mat &img, ProfileDecorate *decorateParams,bool order)
{
	double xmax = -9999999, xmin = 9999999, ymax = -9999999, ymin = 9999999;
	//get intersect rect
	std::vector<int> rectIds;
	//��ȡԭʼ�ĸ��ǵ�
	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx,miny,maxx,maxy);


	dataset->LASDataset_Search(0, sRect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];

			Eigen::MatrixXd pnt(1, 2);
			pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;

			if (GeometryRelation::IsPointInRect(rotPnt(0, 0), rotPnt(0, 1),
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				xmax = max(xmax, rotPnt(0, 0));
				xmin = min(xmin, rotPnt(0, 0));
				ymax = max(ymax, rotPnt(0, 1));
				ymin = min(ymin, rotPnt(0, 1));
			}
		}
	}


	int xsize = (xmax - xmin) / resolution + 1;
	int ysize = (ymax - ymin) / resolution + 1;

	unsigned char* imageData[3];
	for (int i = 0; i < 3; ++i)
	{
		imageData[i] = new unsigned char[xsize*ysize];
		memset(imageData[i], 255, sizeof(unsigned char)*xsize*ysize);
	}

	//point in which pixel
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;

			if (GeometryRelation::IsPointInRect(rotPnt(0, 0), rotPnt(0, 1),
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				double x=rotPnt(0, 0);
				double y=rotPnt(0, 1);
				int pixelx = (x - xmin) / resolution;
				int pixely = (y - ymin) / resolution;
				if (order)
					pixelx = xsize - pixelx-1;

				imageData[0][pixely*xsize + pixelx] = lasPnt.m_colorExt.Blue;
				imageData[1][pixely*xsize + pixelx] = lasPnt.m_colorExt.Green;
				imageData[2][pixely*xsize + pixelx] = lasPnt.m_colorExt.Red;
			}
		}
	}

	std::vector<cv::Mat> vec_mat;
	for (int i = 0; i < 3; ++i)
	{
		cv::Mat tmpMat = cv::Mat(ysize, xsize, CV_8UC1, imageData[i]);
		vec_mat.push_back(tmpMat);
	}

	img.create(ysize, xsize, CV_8UC3);
	cv::merge(vec_mat, img);
	vec_mat.clear();
	if (decorateParams != nullptr)
	{	
		cv::Mat axisImg;
		decorateParams->range_rect = Rect2D(xmin, ymin, xmax, ymax);
		decorateParams->ProfileDecorate_AxisHorizontal(img, axisImg);
		decorateParams->ProfileDecorate_TowerHeightSpan(axisImg);
		decorateParams->ProfileDecorate_TowerLabelPos(axisImg);
		img = axisImg;
	}
	for (int i = 0; i < 3; ++i)
	{
		delete[]imageData[i];
		imageData[i] = nullptr;
	}
}

void LASProfile::LASProfile_ImageFillVertical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, cv::Mat &img, ProfileDecorate *decorateParams, bool order)
{
	//get image size of verticle
	double xmax = -9999999, xmin = 9999999, ymax = -9999999, ymin = 9999999, zmax = -9999999, zmin = 9999999;;
	//get intersect rect and image size
	std::vector<int> rectIds;

	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);

	dataset->LASDataset_Search(0, sRect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);
			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{

				xmin = min(xmin, x);
				xmax = max(xmax, x);
				ymin = min(ymin, y);
				ymax = max(ymax, y);
				zmin = min(zmin, lasPnt.m_vec3d.z);
				zmax = max(zmax, lasPnt.m_vec3d.z);
			}
		}
	}
	zmax += bordersize3;
	
	int xsize = (xmax - xmin) / resolution + 1;
	int ysize = (zmax - zmin) / resolution + 1;
	unsigned char* imageData[3];
	for (int i = 0; i < 3; ++i)
	{
		imageData[i] = new unsigned char[xsize*ysize];
		memset(imageData[i], 255, sizeof(unsigned char)*xsize*ysize);
	}

	//fill image
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];

			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);

			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				int pixelx = (x - xmin) / resolution;
				int pixely = (lasPnt.m_vec3d.z - zmin) / resolution;
				if (order) {
					pixelx = xsize - pixelx - 1;
				}

				imageData[0][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Blue;
				imageData[1][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Green;
				imageData[2][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Red;
			}
		}
	}

	std::vector<cv::Mat> vec_mat;
	for (int i = 0; i < 3; ++i)
	{
		cv::Mat tmpMat = cv::Mat(ysize, xsize, CV_8UC1, imageData[i]);
		vec_mat.push_back(tmpMat);
	}

	img.create(ysize, xsize, CV_8UC3);
	cv::merge(vec_mat, img);
	vec_mat.clear();

	if (decorateParams != nullptr)
	{
		cv::Mat axisImg;
		decorateParams->range_rect = Rect2D(xmin, zmin, xmax, zmax);
		decorateParams->ProfileDecorate_AxisVertical(img, axisImg);
		decorateParams->ProfileDecorate_TowerHeightSpan(axisImg);
		decorateParams->ProfileDecorate_PointsLabel(axisImg);
		img = axisImg;
	}

	for (int i = 0; i < 3; ++i)
	{
		delete[]imageData[i];
		imageData[i] = nullptr;
	}
}

void LASProfile::LASProfile_ImageFillFront(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, cv::Mat &img, ProfileDecorate *decorateParams, bool order)
{
	//get image size of verticle
	double xmax = -9999999, xmin = 9999999, ymax = -9999999, ymin = 9999999, zmax = -9999999, zmin = 9999999;;
	//get intersect rect and image size
	std::vector<int> rectIds;

	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);

	dataset->LASDataset_Search(0, sRect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);
			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				xmin = min(xmin, x);
				xmax = max(xmax, x);
				ymin = min(ymin, y);
				ymax = max(ymax, y);
				zmin = min(zmin, lasPnt.m_vec3d.z);
				zmax = max(zmax, lasPnt.m_vec3d.z);
			}
		}
	}
	zmax += bordersize3;
	int xsize = (xmax - xmin) / resolution + 1;
	int ysize = (zmax - zmin) / resolution + 1;
	unsigned char* imageData[3];
	for (int i = 0; i < 3; ++i)
	{
		imageData[i] = new unsigned char[xsize*ysize];
		memset(imageData[i], 255, sizeof(unsigned char)*xsize*ysize);
	}


	//fill image
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);
			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				double x = rotPnt(0, 0);
				double y = rotPnt(0, 1);

				int pixelx = (x - xmin) / resolution;
				int pixely = (lasPnt.m_vec3d.z - zmin) / resolution;
				if (order) {
					pixelx = xsize - pixelx - 1;
				}

				imageData[0][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Blue;
				imageData[1][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Green;
				imageData[2][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Red;
			}
		}
	}

	std::vector<cv::Mat> vec_mat;
	for (int i = 0; i < 3; ++i)
	{
		cv::Mat tmpMat = cv::Mat(ysize, xsize, CV_8UC1, imageData[i]);
		vec_mat.push_back(tmpMat);
	}

	img.create(ysize, xsize, CV_8UC3);
	cv::merge(vec_mat, img);
	vec_mat.clear();

	if (decorateParams != nullptr)
	{
		decorateParams->range_rect = Rect2D(xmin, zmin, xmax, zmax);
		//TODO:
		//cv::Mat axisImg;
		//img = axisImg;
	}

	for (int i = 0; i < 3; ++i)
	{
		delete[]imageData[i];
		imageData[i] = nullptr;
	}
}

#else
void LASProfile::LASProfile_ImageFillHorizontal(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, unsigned char* ucImageData,double xmin, double ymin, double xmax, double ymax, 
	int xsize, int ysize, bool order/* = false*/)
{
	//get intersect rect
	std::vector<int> rectIds;
	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);
	dataset->LASDataset_Search(0, sRect, rectIds);
	memset(ucImageData, 255, sizeof(unsigned char)*xsize*ysize*3);

	//point in which pixel
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;

			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);

			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				int pixelx = (x - xmin) / resolution;
				int pixely = (y - ymin) / resolution;
				if (order)
					pixelx = xsize - pixelx - 1;
				ucImageData[pixely*xsize + pixelx] = lasPnt.m_colorExt.Red;
				ucImageData[xsize*ysize+pixely*xsize + pixelx] = lasPnt.m_colorExt.Green;
				ucImageData[2*xsize*ysize+pixely*xsize + pixelx] = lasPnt.m_colorExt.Blue;
			}
		}
	}
}

void LASProfile::LASProfile_ImageFillVertical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, unsigned char* ucImageData, double xmin, double ymin, double xmax, double ymax,
	double zmin, double zmax, int xsize, int ysize, bool order/* = false*/)
{
	//get intersect rect and image size
	std::vector<int> rectIds;

	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);
	dataset->LASDataset_Search(0, sRect, rectIds);
	memset(ucImageData, 255, sizeof(unsigned char)*xsize*ysize*3);

	//fill image
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);

			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				int pixelx = (x - xmin) / resolution;
				int pixely = (lasPnt.m_vec3d.z - zmin) / resolution;
				if (order) {
					pixelx = xsize - pixelx - 1;
				}

				ucImageData[(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Red;
				ucImageData[xsize*ysize+(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Green;
				ucImageData[2*xsize*ysize+(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Blue;
			}
		}
	}
}

void LASProfile::LASProfile_ImageFillFront(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, unsigned char* ucImageData, double xmin, double ymin, double xmax, double ymax,
	double zmin, double zmax, int xsize, int ysize, bool order/* = false*/)
{
	//get intersect rect and image size
	std::vector<int> rectIds;

	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);
	dataset->LASDataset_Search(0, sRect, rectIds);
	memset(ucImageData, 255, sizeof(unsigned char)*xsize*ysize * 3);

	//fill image
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);

			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				int pixelx = (x - xmin) / resolution;
				int pixely = (lasPnt.m_vec3d.z - zmin) / resolution;
				if (order) {
					pixelx = xsize - pixelx - 1;
				}

				ucImageData[(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Red;
				ucImageData[xsize*ysize + (ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Green;
				ucImageData[2 * xsize*ysize + (ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Blue;
			}
		}
	}
}

#endif

void LASProfile::LASProfile_Horizontal(const char* strLasDataset, Point2D pntTowers[2], float fRange,
	float fResolution, const char* strOutImg, ProfileDecorate *decorateParams)
{
	Rect2D rect;
	Eigen::MatrixXd rotMat(2,2);
	LASProfile_GetPCARotMat(pntTowers, rotMat);
	LASProfile_GetPointsRange(pntTowers, rotMat, rect, fRange);

	ILASDataset *dataset = new ILASDataset();
	LASReader *reader = new LidarMemReader();
	reader->LidarReader_Open(strLasDataset, dataset);
	reader->LidarReader_Read(true, 1, dataset);

	//order?
	bool order = false;
	//if want to keep order of small left and big right please define TOWER_ORDER
#ifdef TOWER_ORDER
	Eigen::MatrixXd tp1(1, 2), tp2(1, 2);
	tp1(0, 0) = pntTowers[0].x; tp1(0, 1) = pntTowers[0].y;
	tp2(0, 0) = pntTowers[1].x; tp2(0, 1) = pntTowers[1].y;
	Eigen::MatrixXd trp1 = tp1 * rotMat;
	Eigen::MatrixXd trp2 = tp2 * rotMat;
	order = trp2(0, 0)<trp1(0, 0) ? true : false;
#endif // TOWER_ORDER

#ifdef _USE_OPENCV_
	cv::Mat img;
	if (decorateParams != nullptr)
	{
		decorateParams->resolution = fResolution;
		decorateParams->rotMat = rotMat;
		decorateParams->towerOrder = order;
		memcpy(decorateParams->towerPnt, pntTowers, 2 * sizeof(Point2D));
	}
	LASProfile_ImageFillHorizontal(dataset, rect, rotMat, fResolution, img, decorateParams, order);
	cv::imwrite(strOutImg, img);
	img.release();
#else // GDAL
	double xmax = -9999999, xmin = 9999999, ymax = -9999999, ymin = 9999999;
	//get intersect rect
	std::vector<int> rectIds;
	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);
	dataset->LASDataset_Search(0, sRect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt(1, 2);
			pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			if (GeometryRelation::IsPointInRect(rotPnt(0,0), rotPnt(0,1),
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				xmax = max(xmax, rotPnt(0, 0));
				xmin = min(xmin, rotPnt(0, 0));
				ymax = max(ymax, rotPnt(0, 1));
				ymin = min(ymin, rotPnt(0, 1));
			}
		}
	}

	int xsize = (xmax - xmin) / fResolution + 1;
	int ysize = (ymax - ymin) / fResolution + 1;
	GDALAllRegister();
	unsigned char *ucData = nullptr;
	try {
		ucData = new unsigned char[xsize*ysize * 3];
		LASProfile_ImageFillHorizontal(dataset, rect, rotMat,fResolution, ucData,xmin,ymin,xmax,ymax, xsize, ysize, order);
		GDALDatasetH datasetOut = GDALCreate(GDALGetDriverByName("BMP"), strOutImg, xsize, ysize, 3, GDT_Byte, nullptr);
		for (int i = 0; i < 3; ++i)
			GDALRasterIO(GDALGetRasterBand(datasetOut, i + 1), GF_Write, 0, 0, xsize, ysize, ucData + i * xsize*ysize, xsize, ysize, GDT_Byte, 0, 0);
		GDALClose(datasetOut);
	}
	catch (std::bad_alloc e) {
		cerr << "allocate memory failed!" << endl;
	}
	delete[]ucData;
#endif //USE_GDAL

	delete dataset;
	delete reader;
}

void LASProfile::LASProfile_Verticle(const char* strLasDataset, Point2D pntTowers[2], float fRange,
	float fResolution, const char* strOutImg, ProfileDecorate *decorateParams)
{
	Rect2D pointRange;
	Eigen::MatrixXd rotMat;
	LASProfile_GetPCARotMat(pntTowers, rotMat);
	LASProfile_GetPointsRange(pntTowers, rotMat, pointRange,fRange);
	//get las dataset
	ILASDataset *dataset = new ILASDataset();
	LASReader *reader = new LidarMemReader();
	reader->LidarReader_Open(strLasDataset, dataset);
	reader->LidarReader_Read(true, 1, dataset);

	bool order = false;
	//if want to keep order of small left and big right please define TOWER_ORDER
#ifdef TOWER_ORDER
	Eigen::MatrixXd tp1(1, 2), tp2(1, 2);
	tp1(0, 0) = pntTowers[0].x; tp1(0, 1) = pntTowers[0].y;
	tp2(0, 0) = pntTowers[1].x; tp2(0, 1) = pntTowers[1].y;
	Eigen::MatrixXd trp1 = tp1 * rotMat;
	Eigen::MatrixXd trp2 = tp2 * rotMat;
	order = trp2(0, 0)<trp1(0, 0) ? true : false;
#endif // TOWER_ORDER

#ifdef _USE_OPENCV_
	cv::Mat img;
	if (decorateParams != nullptr)
	{
		decorateParams->resolution = fResolution;
		decorateParams->rotMat = rotMat;
		decorateParams->towerOrder = order;
		memcpy(decorateParams->towerPnt, pntTowers, 2 * sizeof(Point2D));
	}
	LASProfile_ImageFillVertical(dataset, pointRange, rotMat, fResolution, img, decorateParams, order);
	cv::imwrite(strOutImg, img);
	img.release();
#else // _USE_OPENCV_
	//get image size of verticle
	double xmax = -9999999, xmin = 9999999, ymax = -9999999, ymin = 9999999, zmax = -9999999, zmin = 9999999;;
	//get intersect rect and image size
	std::vector<int> rectIds;
	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = pointRange.minx; p1(0, 1) = pointRange.miny;
	p2(0, 0) = pointRange.maxx; p2(0, 1) = pointRange.miny;
	p3(0, 0) = pointRange.minx; p3(0, 1) = pointRange.maxy;
	p4(0, 0) = pointRange.maxx; p4(0, 1) = pointRange.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);
	dataset->LASDataset_Search(0, sRect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);
			if (GeometryRelation::IsPointInRect(x,y,
				pointRange.minx, pointRange.miny, pointRange.maxx, pointRange.maxy))
			{

				xmin = min(xmin, x);
				xmax = max(xmax, x);
				ymin = min(ymin, y);
				ymax = max(ymax, y);
				zmin = min(zmin, lasPnt.m_vec3d.z);
				zmax = max(zmax, lasPnt.m_vec3d.z);
			}
		}
	}
	GDALAllRegister();
	int xsize = (xmax - xmin) / fResolution + 1;
	int ysize = (zmax - zmin) / fResolution + 1;
	unsigned char *ucData = nullptr;
	try {
		ucData = new unsigned char[xsize*ysize * 3];
		LASProfile_ImageFillVertical(dataset, pointRange, rotMat, fResolution, ucData, xmin, ymin, xmax, ymax,zmin,zmax, xsize, ysize, order);
		GDALDatasetH datasetOut = GDALCreate(GDALGetDriverByName("BMP"), strOutImg, xsize, ysize, 3, GDT_Byte, nullptr);
		for (int i = 0; i < 3; ++i)
			GDALRasterIO(GDALGetRasterBand(datasetOut, i + 1), GF_Write, 0, 0, xsize, ysize, ucData + i * xsize*ysize, xsize, ysize, GDT_Byte, 0, 0);
		GDALClose(datasetOut);
	}
	catch (std::bad_alloc e) {
		cerr << "allocate memory failed!" << endl;
	}
	delete[]ucData;

#endif

	delete dataset;
	delete reader;
}

void LASProfile::LASProfile_Front(const char* strLasDataset, Point2D pntTowers[2], float fRange,
	float fResolution, const char* strOutImg, ProfileDecorate *decorateParams)
{
	Eigen::MatrixXd rotMat(2, 2);
	Eigen::MatrixXd rotMatVertical(2, 2);
	rotMatVertical(0, 0) = 0; rotMatVertical(0, 1) = -1;
	rotMatVertical(1, 0) = 1; rotMatVertical(1, 1) = 0;
	LASProfile_GetPCARotMat(pntTowers, rotMat);
	rotMat = rotMat * rotMatVertical;

	Rect2D rect;
	LASProfile_GetPointsRange(pntTowers, rotMat, rect, fRange);

	ILASDataset *dataset = new ILASDataset();
	LASReader *reader = new LidarMemReader();
	reader->LidarReader_Open(strLasDataset, dataset);
	reader->LidarReader_Read(true, 1, dataset);

	//order?
	bool order = false;
	//if want to keep order of small left and big right please define TOWER_ORDER
#ifdef TOWER_ORDER
	Eigen::MatrixXd tp1(1, 2), tp2(1, 2);
	tp1(0, 0) = pntTowers[0].x; tp1(0, 1) = pntTowers[0].y;
	tp2(0, 0) = pntTowers[1].x; tp2(0, 1) = pntTowers[1].y;
	Eigen::MatrixXd trp1 = tp1 * rotMat;
	Eigen::MatrixXd trp2 = tp2 * rotMat;
	order = trp2(0, 1)<trp1(0, 1) ? true : false;
#endif // TOWER_ORDER

#ifdef _USE_OPENCV_
	cv::Mat img;
	if (decorateParams != nullptr)
	{
		decorateParams->resolution = fResolution;
		decorateParams->rotMat = rotMat;
	}
	LASProfile_ImageFillFront(dataset, rect, rotMat, fResolution, img, decorateParams, order);
	cv::imwrite(strOutImg, img);
	img.release();
#else // _USE_OPENCV_
	//get image size of verticle
	double xmax = -9999999, xmin = 9999999, ymax = -9999999, ymin = 9999999, zmax = -9999999, zmin = 9999999;;
	//get intersect rect and image size
	std::vector<int> rectIds;
	Eigen::MatrixXd p1(1, 2), p2(1, 2), p3(1, 2), p4(1, 2);
	p1(0, 0) = rect.minx; p1(0, 1) = rect.miny;
	p2(0, 0) = rect.maxx; p2(0, 1) = rect.miny;
	p3(0, 0) = rect.minx; p3(0, 1) = rect.maxy;
	p4(0, 0) = rect.maxx; p4(0, 1) = rect.maxy;
	Eigen::MatrixXd rp1 = p1 * rotMat.transpose();
	Eigen::MatrixXd rp2 = p2 * rotMat.transpose();
	Eigen::MatrixXd rp3 = p3 * rotMat.transpose();
	Eigen::MatrixXd rp4 = p4 * rotMat.transpose();
	double minx = min(min(rp1(0, 0), rp2(0, 0)), min(rp3(0, 0), rp4(0, 0)));
	double maxx = max(max(rp1(0, 0), rp2(0, 0)), max(rp3(0, 0), rp4(0, 0)));

	double miny = min(min(rp1(0, 1), rp2(0, 1)), min(rp3(0, 1), rp4(0, 1)));
	double maxy = max(max(rp1(0, 1), rp2(0, 1)), max(rp3(0, 1), rp4(0, 1)));

	Rect2D sRect(minx, miny, maxx, maxy);
	dataset->LASDataset_Search(0, sRect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
			pnt(0, 0) = lasPnt.m_vec3d.x;
			pnt(0, 1) = lasPnt.m_vec3d.y;
			Eigen::MatrixXd rotPnt = pnt * rotMat;
			double x = rotPnt(0, 0);
			double y = rotPnt(0, 1);
			
			if (GeometryRelation::IsPointInRect(x, y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				xmin = min(xmin, x);
				xmax = max(xmax, x);
				ymin = min(ymin, y);
				ymax = max(ymax, y);
				zmin = min(zmin, lasPnt.m_vec3d.z);
				zmax = max(zmax, lasPnt.m_vec3d.z);
			}
		}
	}
	GDALAllRegister();
	int xsize = (xmax - xmin) / fResolution + 1;
	int ysize = (zmax - zmin) / fResolution + 1;
	unsigned char *ucData = nullptr;
	try {
		ucData = new unsigned char[xsize*ysize * 3];
		LASProfile_ImageFillFront(dataset, rect, rotMat, fResolution, ucData, xmin, ymin, xmax, ymax, zmin, zmax, xsize, ysize, order);
		GDALDatasetH datasetOut = GDALCreate(GDALGetDriverByName("BMP"), strOutImg, xsize, ysize, 3, GDT_Byte, nullptr);
		for (int i = 0; i < 3; ++i)
			GDALRasterIO(GDALGetRasterBand(datasetOut, i + 1), GF_Write, 0, 0, xsize, ysize, ucData + i * xsize*ysize, xsize, ysize, GDT_Byte, 0, 0);
		GDALClose(datasetOut);
	}
	catch (std::bad_alloc e) {
		cerr << "allocate memory failed!" << endl;
	}
	delete[]ucData;

#endif

	delete dataset;
	delete reader;
}


#ifdef _USE_OPENCV_

void ProfileDecorate::ProfileDecorate_AxisVertical(cv::Mat srcImg, cv::Mat &axisImg)
{
	cv::Scalar bValue(255, 255, 255);
	cv::copyMakeBorder(srcImg, axisImg, bordersize1, bordersize2, bordersize1, bordersize3, cv::BORDER_CONSTANT, bValue);
	cv::Size imgsizeSrc(srcImg.cols,srcImg.rows);

#ifdef _DEBUG
	cv::imwrite("border.jpg", axisImg);
#endif
	
	//calculate resolution
	double resolutionx = fabs(range_rect.minx - range_rect.maxx) / double(imgsizeSrc.width);
	double resolutiony = fabs(range_rect.miny - range_rect.maxy) / double(imgsizeSrc.height);

	double widthpix = hspan_dis / resolutionx;
	double heighpix = vspan_dis / resolutiony;
		
	cv::line(axisImg,cv::Point(70,imgsizeSrc.height+170), cv::Point(70+imgsizeSrc.width+bordersize3, imgsizeSrc.height + 170),cv::Scalar(0,0,0));//x axis
	cv::line(axisImg, cv::Point(70, imgsizeSrc.height + 170), cv::Point(70,20), cv::Scalar(0, 0, 0));//y axis

	int font_face = cv::FONT_HERSHEY_COMPLEX;
	double font_scale = 0.5;
	int thickness = 1;
	int baseline;

	for (int i = 0; i *widthpix<(imgsizeSrc.width+bordersize3); ++i)
	{
		string txt = cv::format("%d", int(i*hspan_dis));
		cv::line(axisImg, cv::Point(bordersize1 + i*widthpix, imgsizeSrc.height + 170), cv::Point(bordersize1 + i*widthpix, imgsizeSrc.height+ 165), cv::Scalar(0, 0, 0));
		txt= cv::format("%d", int(i*this->hspan_dis));
		cv::Size text_size = cv::getTextSize(txt, font_face, font_scale, thickness, &baseline);

		//���ı�����л���
		cv::Point origin;
		origin.x = bordersize1 + i*widthpix - text_size.width / 2;
		origin.y = imgsizeSrc.height + 170 + 5 + text_size.height;
		cv::putText(axisImg, txt, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness);
	}

	for (int i = 0; i*heighpix < (imgsizeSrc.height + 50); ++i)
	{
		cv::line(axisImg, cv::Point(70, imgsizeSrc.height+bordersize1-i*heighpix), cv::Point(75, imgsizeSrc.height + bordersize1 - i*heighpix), cv::Scalar(0, 0, 0));

		string txt = cv::format("%d", int(i*this->vspan_dis));
		cv::Size text_size = cv::getTextSize(txt, font_face, font_scale, thickness, &baseline);

		//���ı�����л���
		cv::Point origin;
		origin.x = 50 - text_size.width / 2;
		origin.y = imgsizeSrc.height + bordersize1 - i*heighpix + text_size.height/2;
		cv::putText(axisImg, txt, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness);
	}

#ifdef _DEBUG
	cv::imwrite("borderAxis.jpg", axisImg);
#endif
}

void ProfileDecorate::ProfileDecorate_AxisHorizontal(cv::Mat srcImg, cv::Mat &axisImg)
{
	cv::Scalar bValue(255, 255, 255);
	cv::copyMakeBorder(srcImg, axisImg, bordersize1, bordersize2, bordersize1, bordersize3, cv::BORDER_CONSTANT, bValue);
	cv::Size imgsizeSrc(srcImg.cols, srcImg.rows);

#ifdef _DEBUG
	cv::imwrite("border.jpg", axisImg);
#endif

	//calculate resolution
	double resolutionx = fabs(range_rect.minx - range_rect.maxx) / double(imgsizeSrc.width);
	double resolutiony = fabs(range_rect.miny - range_rect.maxy) / double(imgsizeSrc.height);

	double widthpix = hspan_dis / resolutionx;
	double heighpix = vspan_dis / resolutiony;

	cv::line(axisImg, cv::Point(70, imgsizeSrc.height + 180), cv::Point(70 + imgsizeSrc.width + bordersize3, imgsizeSrc.height + 180), cv::Scalar(0, 0, 0));//x axis
	cv::line(axisImg, cv::Point(70, imgsizeSrc.height + 180), cv::Point(70, 20), cv::Scalar(0, 0, 0));//y axis

	int font_face = cv::FONT_HERSHEY_COMPLEX;
	double font_scale = 0.5;
	int thickness = 1;
	int baseline;

	for (int i = 0; i *widthpix<(imgsizeSrc.width + bordersize3); ++i)
	{
		string txt = cv::format("%d", int(i*hspan_dis));
		cv::line(axisImg, cv::Point(bordersize1 + i*widthpix, imgsizeSrc.height + 180), cv::Point(bordersize1 + i*widthpix, imgsizeSrc.height + 175), cv::Scalar(0, 0, 0));
		txt = cv::format("%d", int(i*this->hspan_dis));
		cv::Size text_size = cv::getTextSize(txt, font_face, font_scale, thickness, &baseline);

		cv::Point origin;
		origin.x = bordersize1 + i*widthpix - text_size.width / 2;
		origin.y = imgsizeSrc.height + 190 + 5 + text_size.height;
		cv::putText(axisImg, txt, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness);
	}

	for (int i = 0; i*heighpix < (imgsizeSrc.height + 50); ++i)
	{
		cv::line(axisImg, cv::Point(70, imgsizeSrc.height + bordersize2 - i*heighpix), cv::Point(75, imgsizeSrc.height + bordersize2 - i*heighpix), cv::Scalar(0, 0, 0));

		string txt = cv::format("%d", int(i*this->vspan_dis));
		cv::Size text_size = cv::getTextSize(txt, font_face, font_scale, thickness, &baseline);

		cv::Point origin;
		origin.x = 50 - text_size.width / 2;
		origin.y = imgsizeSrc.height + bordersize2 - i*heighpix + text_size.height / 2;
		cv::putText(axisImg, txt, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness);
	}

#ifdef _DEBUG
	cv::imwrite("borderAxis.jpg", axisImg);
#endif
}

void ProfileDecorate::ProfileDecorate_TowerHeightSpan(cv::Mat &axisImg)
{
	Eigen::MatrixXd p1(1, 2), p2(1, 2);
	p1(0, 0) = towerPnt[0].x; p1(0, 1) = towerPnt[0].y;
	p2(0, 0) = towerPnt[1].x; p2(0, 1) = towerPnt[1].y;
	Eigen::MatrixXd rp1 = p1 * rotMat;
	Eigen::MatrixXd rp2 = p2 * rotMat;
	double span = fabs(rp2(0, 0) - rp1(0, 0));

	int font_face = cv::FONT_HERSHEY_COMPLEX;
	double font_scale = 0.5;
	int thickness = 1;
	int baseline;
	string txt = cv::format("span distance=%0.4lf meters", span);
	cv::Size text_size = cv::getTextSize(txt, font_face, font_scale, thickness, &baseline);

	cv::Point origin;
	int xleft  = (min(rp2(0, 0), rp1(0, 0)) - range_rect.minx) / resolution+bordersize1;
	int xright = (max(rp2(0, 0), rp1(0, 0)) - range_rect.minx) / resolution+bordersize1;

	origin.x = (xleft+xright)/2  - text_size.width / 2;
	origin.y = axisImg.rows - bordersize1 + text_size.height / 2;
	cv::putText(axisImg, txt, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness);

	cv::line(axisImg, cv::Point(xleft, axisImg.rows - 70), cv::Point(xleft, axisImg.rows - 90), cv::Scalar(0, 0, 0));
	cv::line(axisImg, cv::Point(xright, axisImg.rows - 70), cv::Point(xright, axisImg.rows - 90), cv::Scalar(0, 0, 0));

	//虚线
	int per = (origin.x - xleft) / 30;
	for (int i = 0; i <= 30; i+=2)
	{
		cv::line(axisImg, cv::Point(xleft+i*per, axisImg.rows - bordersize1), cv::Point(xleft+(i+1)*per, axisImg.rows - bordersize1), cv::Scalar(0, 0, 0));
	}
	int count = (xright - (origin.x + text_size.width))/per;
	for (int i = 0; i <= count; i += 2)
	{
		cv::line(axisImg, cv::Point(origin.x + text_size.width+i*per, axisImg.rows - bordersize1), cv::Point(origin.x + text_size.width + (i+1)*per, axisImg.rows - bordersize1), cv::Scalar(0, 0, 0));
	}
}

void ProfileDecorate::ProfileDecorate_TowerLabelPos(cv::Mat &axisImg)
{
 	Eigen::MatrixXd p1(1, 2), p2(1, 2);
	p1(0, 0) = towerPnt[0].x; p1(0, 1) = towerPnt[0].y;
	p2(0, 0) = towerPnt[1].x; p2(0, 1) = towerPnt[1].y;
	Eigen::MatrixXd rp1 = p1 * rotMat;
	Eigen::MatrixXd rp2 = p2 * rotMat;

	int x1 = (rp1(0,0)-range_rect.minx)/resolution;
	int x2 = (rp2(0,0)-range_rect.minx)/resolution;

	int y1 = (rp1(0,1)-range_rect.miny)/resolution;
	int y2 = (rp2(0,1)-range_rect.miny)/resolution;

	switch(lbType){
		case LABEL_CIRCLE:
			{
				cv::Point c1(x1+bordersize1,y1+bordersize1);
				cv::Point c2(x2+bordersize1,y2+bordersize1);
				cv::circle(axisImg,c1,2/resolution,cv::Scalar(0, 0, 255),2);
				cv::circle(axisImg,c2,2/resolution,cv::Scalar(0, 0, 255),2);
			}
			break;

		case LABEL_SQUARE:
			{
				cv::Point ltpt1(x1+bordersize1-2/resolution,y1+bordersize1-2/resolution);
				cv::Point rbpt1(x1+bordersize1+2/resolution,y1+bordersize1+2/resolution);
				cv::rectangle(axisImg, ltpt1, rbpt1, cv::Scalar(0,0,255),2,8,0);

				cv::Point ltpt2(x2+bordersize1-2/resolution,y2+bordersize1-2/resolution);
				cv::Point rbpt2(x2+bordersize1+2/resolution,y2+bordersize1+2/resolution);
				cv::rectangle(axisImg, ltpt2, rbpt2, cv::Scalar(0,0,255),2,8,0);	
			}		
			break;
	}
}

void ProfileDecorate::ProfileDecorate_PointsLabel(cv::Mat &axisImg)
{
	//对每一个杆塔点计算其位置
	for(int i=0;i<labelPnts.size();++i){
		Eigen::MatrixXd p1(1, 2), p2(1, 2);
		p1(0, 0) = labelPnts[0].x; p1(0, 1) = labelPnts[0].y;
		Eigen::MatrixXd rp1 = p1 * rotMat;
		int xsize = (range_rect.maxx - range_rect.minx) / resolution + 1;
		int ysize = (range_rect.maxy - range_rect.miny) / resolution + 1;

		int x = (rp1(0,0)-range_rect.minx)/resolution;
		int y = (labelPnts[0].z - range_rect.miny)/resolution;
		y = ysize - y - 1;

		switch(lbType){
			case LABEL_CIRCLE:
				{
					cv::Point c1(x+bordersize1,y+bordersize1);
					cv::circle(axisImg,c1,2/resolution,cv::Scalar(0, 0, 255),2);
				}
				break;
			case LABEL_SQUARE:
				{
					cv::Point ltpt1(x+bordersize1-2/resolution,y+bordersize1-2/resolution);
					cv::Point rbpt1(x+bordersize1+2/resolution,y+bordersize1+2/resolution);
					cv::rectangle(axisImg, ltpt1, rbpt1, cv::Scalar(0,0,255),2,8,0);
				}		
				break;
		}
	}
}

#endif