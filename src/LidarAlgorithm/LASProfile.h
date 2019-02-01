#pragma once

#include"../LidarBase/LASPoint.h"
#include"../LidarAlgorithm/Geometry.h"
#include <Eigen/Dense>
#include <string>
#ifdef _USE_OPENCV_

	#include<opencv2/opencv.hpp>
#endif
class ProfileDecorate;

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
		use to get profile vertical;horizontal;front
	*/
	void LASProfile_Verticle(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, const char* strOutImg, ProfileDecorate *decorateParams = nullptr);

	void LASProfile_Horizontal(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, const char* strOutImg, ProfileDecorate *decorateParams = nullptr);

	void LASProfile_Front(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution, const char* strOutImg,ProfileDecorate *decorateParams= nullptr);

private:

	/*
		get points range
	*/
	void LASProfile_GetPointsRange(Point2D pntTowers[2], Eigen::MatrixXd rotMat,Rect2D &rect, float range);
	
#ifdef _USE_OPENCV_

	/*
		get image size according to points range using opencv
	*/
	void LASProfile_ImageFillHorizontal(ILASDataset* dataset,Rect2D rect, Eigen::MatrixXd rotMat,
											float resolution, cv::Mat &img, ProfileDecorate *decorateParams=nullptr,bool order=false);

	void LASProfile_ImageFillVertical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
											float resolution, cv::Mat &img, ProfileDecorate *decorateParams= nullptr, bool order = false);

	void LASProfile_ImageFillFront(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
											float resolution, cv::Mat &img, ProfileDecorate *decorateParams= nullptr, bool order = false);
#else
	/*
	get image size according to points range using opencv
	*/
	void LASProfile_ImageFillHorizontal(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
		float resolution, unsigned char* ucImageData,double xmin,double ymin,double xmax,double ymax,
		int xsize, int ysize, bool order = false);

	void LASProfile_ImageFillVertical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
		float resolution, unsigned char* ucImageData, double xmin, double ymin, double xmax, double ymax, 
		double zmin,double zmax,int xsize,int ysize, bool order = false);

	void LASProfile_ImageFillFront(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
		float resolution, unsigned char* ucImageData, double xmin, double ymin, double xmax, double ymax,
		double zmin, double zmax, int xsize, int ysize, bool order = false);
#endif
};

#ifdef _USE_OPENCV_

enum ProfileDecorateLabelType {
	LABEL_CIRCLE,
	LABEL_SQUARE,
	LABEL_STAR,
	LABEL_IMG
};

class ProfileDecorate 
{
	//define friend class
	friend class LASProfile;
public:

	ProfileDecorate() {
		towerHeight[0] = towerHeight[1] = 0;
		towerName[0] = towerName[1] = "";
	}

private:
	/**
		add Axis for the vertical profile if opencv is used
		cv::Mat srcImg:input image
		cv::Mat &axisImg:decorated image
	*/
	void ProfileDecorate_AxisVertical(cv::Mat srcImg,cv::Mat &axisImg);

	/*
		add Axis for the vertical profile if opencv is used
	*/
	void ProfileDecorate_AxisHorizontal(cv::Mat srcImg, cv::Mat &axisImg);



	/*
		add tower height and span in the picture	
	*/
	void ProfileDecorate_TowerHeightSpan(cv::Mat &axisImg);

	/*
		add tower label and tower position(coordinate)
	*/
	void ProfileDecorate_TowerLabelPos(cv::Mat &axisImg);


private:
	int    vspan_num;		//v axis span number
	int    hspan_num;		//h axis span number
	Eigen::MatrixXd	rotMat;	//
	Rect2D	range_rect;		//	
	double	resolution;		//
	bool   towerOrder;	    //
	Point2D towerPnt[2];
public:
	double vspan_dis;		//required
	double hspan_dis;		//required
	std::vector<Point3D>	 labelPnts;	//label point coordinate in world coordination
	ProfileDecorateLabelType lbType;	//label type
	std::string				 lbUrl;		//if label type used LABEL_IMG url must be set
	double towerHeight[2];				//
	std::string towerName[2];			//
};




#endif

