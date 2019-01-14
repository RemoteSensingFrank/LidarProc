#pragma once

#include "../LidarBase/LASPoint.h"
#include "../LidarGeometry/Geometry.h"
#include "Eigen/Dense"

#ifdef _USE_OPENCV_
#include <opencv/cv.h>

//draw label with opencv
enum LabelType {
	Circle_Only,
	Circle_with_Text,
	Star_Only,
	Star_with_Text
};

//keyPnts 第一个点为中心点，在绘制的时候需要注意
class DrawLabel
{
public:
	virtual void DrawLabel_CenterVertical(Point3D pntLabel, Eigen::MatrixXd rotMat, float fResolution,int xisze,int ysize,
		double dMin[3], double dMax[3],bool order);
	virtual void DrawLabel_CenterHorizontal(Point3D pntLabel, Eigen::MatrixXd rotMat, float fResolution, int xisze, int ysize,
		double dMin[3], double dMax[3], bool order);
	
	//写中心点信息
	void DrawLabel_Center(Point3D pntLabel, Eigen::MatrixXd rotMat, float fResolution,
		int xisze, int ysize, double dMin[3], double dMax[3], bool order);

	virtual void DrawLabel_Construct_Label() = 0;

	virtual void DrawLabel_Draw(cv::Mat &img);
public:
	vector<cv::Point> keyPnts;
};

class DrawLabelVerticalCircle_Only: public DrawLabel
{
public:
	virtual void DrawLabel_Construct_Label();

	virtual void DrawLabel_Draw(cv::Mat &img);
};

#endif

class LASProfile
{
public:
	/*
		get the direction of the section
		input  : tower point of the section
		output : the rotation matrix
	*/
	void LASProfile_GetPCARotMat(Point2D pntTowers[2], Eigen::MatrixXd &rotMat);

	/*
		use to get profile
	*/
	void LASProfile_Verticle(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, const char* strOutImg);

	void LASProfile_Horizontal(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, const char* strOutImg);

	void LASProfile_Front(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, const char* strOutImg);

#ifdef _USE_OPENCV_
	/*
		generate picture for each label point?
	*/
	void LASProfile_VerticleLabel(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, 
		LabelType enumLabelType,vector<Point2D> vecLabelPnts, string strImgDir);

	void LASProfile_HorizontalLabel(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, 
		LabelType enumLabelType, vector<Point2D> vecLabelPnts, string strImgDir);
#endif

private:

	/*
		get points range
	*/
	void LASProfile_GetPointsRange(Point2D pntTowers[2], Rect2D &rect, float range);
	
#ifdef _USE_OPENCV_
	/*
		get image size according to points range using opencv
	*/
	void LASProfile_ImageFillHorizontal(ILASDataset* dataset,Rect2D rect, Eigen::MatrixXd rotMat,
											float resolution, cv::Mat &img,bool order=false);

	void LASProfile_ImageFillVartical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
											float resolution, cv::Mat &img, bool order = false);

	void LASProfile_ImageFillFront(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
		float resolution, cv::Mat &img, bool order = false);

#else
	/*
	get image size according to points range using opencv
	*/
	void LASProfile_ImageFillHorizontal(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
		float resolution, unsigned char* ucImageData,double xmin,double ymin,double xmax,double ymax,
		int xsize, int ysize, bool order = false);

	void LASProfile_ImageFillVartical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
		float resolution, unsigned char* ucImageData, double xmin, double ymin, double xmax, double ymax, 
		double zmin,double zmax,int xsize,int ysize, bool order = false);

	void LASProfile_ImageFillFront(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
		float resolution, unsigned char* ucImageData, double xmin, double ymin, double xmax, double ymax,
		double zmin, double zmax, int xsize, int ysize, bool order = false);
#endif
};
