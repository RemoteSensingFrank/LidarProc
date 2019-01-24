#include "LASProfile.h"

#include<iostream>
#include<math.h>
#include"gdal_priv.h"

#include"../LidarGeometry/GeometryAlgorithm.h"
#include"../LidarBase/LASReader.h"


#ifdef _USE_OPENCV_
	#include<opencv2/imgproc/imgproc.hpp>  
	#include<opencv2/highgui/highgui.hpp>  
#endif

#define TOWER_ORDER

/*
获取PCA变换的旋转矩阵
*/
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
}

void LASProfile::LASProfile_GetPointsRange(Point2D pntTowers[2], Rect2D &rect, float range)
{
	rect.minx = min(pntTowers[0].x, pntTowers[1].x) - range;
	rect.miny = min(pntTowers[0].y, pntTowers[1].y) - range;
	rect.maxx = max(pntTowers[0].x, pntTowers[1].x) + range;
	rect.maxy = max(pntTowers[0].y, pntTowers[1].y) + range;
}

#ifdef _USE_OPENCV_
void LASProfile::LASProfile_ImageFillHorizontal(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, cv::Mat &img,bool order)
{
	double xmax = _MIN_LIMIT_, xmin = _MAX_LIMIT_, ymax = _MIN_LIMIT_, ymin = _MAX_LIMIT_;
	//get intersect rect
	std::vector<int> rectIds;
	dataset->LASDataset_Search(0, rect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt(1, 2);
				pnt= Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;

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
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1,2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;

				double x=rotPnt(0, 0);
				double y=rotPnt(0, 1);
				int pixelx = (x - xmin) / resolution;
				int pixely = (y - ymin) / resolution;
				if (order)
					pixelx = xsize - pixelx-1;

				imageData[0][pixely*xsize + pixelx] = lasPnt.m_colorExt.Red;
				imageData[1][pixely*xsize + pixelx] = lasPnt.m_colorExt.Green;
				imageData[2][pixely*xsize + pixelx] = lasPnt.m_colorExt.Blue;
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

	for (int i = 0; i < 3; ++i)
	{
		delete[]imageData[i];
		imageData[i] = nullptr;
	}
}

void LASProfile::LASProfile_ImageFillVartical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, cv::Mat &img, bool order)
{
	//get image size of verticle
	double xmax = _MIN_LIMIT_, xmin = _MAX_LIMIT_, ymax = _MIN_LIMIT_, ymin = _MAX_LIMIT_, zmax = _MIN_LIMIT_, zmin = _MAX_LIMIT_;;
	//get intersect rect and image size
	std::vector<int> rectIds;
	dataset->LASDataset_Search(0, rect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;
				double x = rotPnt(0, 0);
				double y = rotPnt(0, 1);
				xmin = min(xmin, x);
				xmax = max(xmax, x);
				ymin = min(ymin, y);
				ymax = max(ymax, y);
				zmin = min(zmin, lasPnt.m_vec3d.z);
				zmax = max(zmax, lasPnt.m_vec3d.z);
			}
		}
	}

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
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;
				double x = rotPnt(0, 0);
				double y = rotPnt(0, 1);

				int pixelx = (x - xmin) / resolution;
				int pixely = (lasPnt.m_vec3d.z - zmin) / resolution;
				if (order) {
					pixelx = xsize - pixelx - 1;
				}

				imageData[0][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Red;
				imageData[1][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Green;
				imageData[2][(ysize - pixely - 1)*xsize + pixelx] = lasPnt.m_colorExt.Blue;
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
	for (int i = 0; i < 3; ++i)
	{
		delete[]imageData[i];
		imageData[i] = nullptr;
	}
}


void LASProfile::LASProfile_VerticleLabel(const char* strLasDataset, Point2D pntTowers[2], float fRange, float fResolution,
	LabelType enumLabelType, vector<Point2D> vecLabelPnts, string strImgDir)
{
	Eigen::MatrixXd rotMat(2, 2);
	LASProfile_GetPCARotMat(pntTowers, rotMat);

	Rect2D rect;
	LASProfile_GetPointsRange(pntTowers, rect, fRange);

	ILASDataset *dataset = new ILASDataset();
	LASReader *reader = new LidarMemReader();
	reader->LidarReader_Open(strLasDataset, dataset);
	reader->LidarReader_Read(true, 1, dataset);

	//order?
	bool order = false;
	//if want to keep order of small left and big right please define TOWER_ORDER
#ifdef TOWER_ORDER
	Eigen::MatrixXd p1(1, 2), p2(1, 2);
	p1(0, 0) = pntTowers[0].x; p1(0, 1) = pntTowers[0].y;
	p2(0, 0) = pntTowers[1].x; p2(0, 1) = pntTowers[1].y;
	Eigen::MatrixXd rp1 = p1 * rotMat;
	Eigen::MatrixXd rp2 = p2 * rotMat;
	order = rp2(0, 0)<rp1(0, 0) ? true : false;
#endif // TOWER_ORDER

	cv::Mat img;
	LASProfile_ImageFillVartical(dataset, rect, rotMat, fResolution, img, order);
	DrawLabel *draw=nullptr;



	draw->DrawLabel_Construct_Label();


}

#else
void LASProfile::LASProfile_ImageFillHorizontal(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, unsigned char* ucImageData,double xmin, double ymin, double xmax, double ymax, 
	int xsize, int ysize, bool order/* = false*/)
{
	//get intersect rect
	std::vector<int> rectIds;
	dataset->LASDataset_Search(0, rect, rectIds);
	memset(ucImageData, 255, sizeof(unsigned char)*xsize*ysize*3);

	//point in which pixel
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;

				double x = rotPnt(0, 0);
				double y = rotPnt(0, 1);
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

void LASProfile::LASProfile_ImageFillVartical(ILASDataset* dataset, Rect2D rect, Eigen::MatrixXd rotMat,
	float resolution, unsigned char* ucImageData, double xmin, double ymin, double xmax, double ymax,
	double zmin, double zmax, int xsize, int ysize, bool order/* = false*/)
{
	
	//get intersect rect and image size
	std::vector<int> rectIds;
	dataset->LASDataset_Search(0, rect, rectIds);
	memset(ucImageData, 255, sizeof(unsigned char)*xsize*ysize*3);

	//fill image
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;
				double x = rotPnt(0, 0);
				double y = rotPnt(0, 1);

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
	dataset->LASDataset_Search(0, rect, rectIds);
	memset(ucImageData, 255, sizeof(unsigned char)*xsize*ysize * 3);

	//fill image
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;
				double x = rotPnt(0, 0);
				double y = rotPnt(0, 1);

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
	float fResolution, const char* strOutImg)
{
	Eigen::MatrixXd rotMat(2,2);
	LASProfile_GetPCARotMat(pntTowers, rotMat);

	Rect2D rect;
	LASProfile_GetPointsRange(pntTowers, rect, fRange);

	ILASDataset *dataset = new ILASDataset();
	LASReader *reader = new LidarMemReader();
	reader->LidarReader_Open(strLasDataset, dataset);
	reader->LidarReader_Read(true, 1, dataset);

	//order?
	bool order = false;
	//if want to keep order of small left and big right please define TOWER_ORDER
#ifdef TOWER_ORDER
	Eigen::MatrixXd p1(1, 2), p2(1, 2);
	p1(0, 0) = pntTowers[0].x; p1(0, 1) = pntTowers[0].y;
	p2(0, 0) = pntTowers[1].x; p2(0, 1) = pntTowers[1].y;
	Eigen::MatrixXd rp1 = p1 * rotMat;
	Eigen::MatrixXd rp2 = p2 * rotMat;
	order = rp2(0, 0)<rp1(0, 0) ? true : false;
#endif // TOWER_ORDER

#ifdef _USE_OPENCV_
	cv::Mat img;
	LASProfile_ImageFillHorizontal(dataset, rect, rotMat, fResolution, img, order);
	cv::imwrite(strOutImg, img);
	img.release();
#else // _USE_OPENCV_
	double xmax = _MIN_LIMIT_, xmin = _MAX_LIMIT_, ymax = _MIN_LIMIT_, ymin = _MAX_LIMIT_;
	//get intersect rect
	std::vector<int> rectIds;
	dataset->LASDataset_Search(0, rect, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				rect.minx, rect.miny, rect.maxx, rect.maxy))
			{
				Eigen::MatrixXd pnt(1, 2);
				pnt = Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;

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
	float fResolution, const char* strOutImg)
{
	Rect2D pointRange;
	Eigen::MatrixXd rotMat;
	LASProfile_GetPCARotMat(pntTowers, rotMat);
	LASProfile_GetPointsRange(pntTowers, pointRange,fRange);

	//get las dataset
	ILASDataset *dataset = new ILASDataset();
	LASReader *reader = new LidarMemReader();
	reader->LidarReader_Open(strLasDataset, dataset);
	reader->LidarReader_Read(true, 1, dataset);

	bool order = false;
	//if want to keep order of small left and big right please define TOWER_ORDER
#ifdef TOWER_ORDER
	Eigen::MatrixXd p1(1, 2), p2(1, 2);
	p1(0, 0) = pntTowers[0].x; p1(0, 1) = pntTowers[0].y;
	p2(0, 0) = pntTowers[1].x; p2(0, 1) = pntTowers[1].y;
	Eigen::MatrixXd rp1 = p1 * rotMat;
	Eigen::MatrixXd rp2 = p2 * rotMat;
	order = rp2(0, 0)<rp1(0, 0) ? true : false;
#endif // TOWER_ORDER

#ifdef _USE_OPENCV_
	cv::Mat img;
	LASProfile_ImageFillVartical(dataset, pointRange, rotMat, fResolution, img, order);
	cv::imwrite(strOutImg, img);
	img.release();
#else // _USE_OPENCV_
	//get image size of verticle
	double xmax = _MIN_LIMIT_, xmin = _MAX_LIMIT_, ymax = _MIN_LIMIT_, ymin = _MAX_LIMIT_, zmax = _MIN_LIMIT_, zmin = _MAX_LIMIT_;;
	//get intersect rect and image size
	std::vector<int> rectIds;
	dataset->LASDataset_Search(0, pointRange, rectIds);
	for (int i = 0; i < rectIds.size(); ++i)
	{
		LASRectBlock &lasBlock = dataset->m_lasRectangles[rectIds[i]];
		for (int j = 0; j < lasBlock.m_lasPoints_numbers; ++j)
		{
			LASPoint &lasPnt = lasBlock.m_lasPoints[j];
			if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,
				pointRange.minx, pointRange.miny, pointRange.maxx, pointRange.maxy))
			{
				Eigen::MatrixXd pnt = Eigen::MatrixXd::Zero(1, 2);
				pnt(0, 0) = lasPnt.m_vec3d.x;
				pnt(0, 1) = lasPnt.m_vec3d.y;
				Eigen::MatrixXd rotPnt = pnt * rotMat;
				double x = rotPnt(0, 0);
				double y = rotPnt(0, 1);
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
		LASProfile_ImageFillVartical(dataset, pointRange, rotMat, fResolution, ucData, xmin, ymin, xmax, ymax,zmin,zmax, xsize, ysize, order);
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
void DrawLabel::DrawLabel_Center(Point3D pntLabel, Eigen::MatrixXd rotMat, float fResolution, 
								int xisze, int ysize,double dMin[3], double dMax[3], bool order)
{
	Eigen::MatrixXd matrixMat(1, 2);
	matrixMat(0, 0) = pntLabel.x;
	matrixMat(0, 1) = pntLabel.y;
	Eigen::MatrixXd matrixRotMat = matrixMat * rotMat;

	int pixelx = (matrixMat(0, 0) - dMin[0]) / fResolution;
	int pixely = (pntLabel.z - dMin[2]) / fResolution;
	cv::Point cvpntCenter(pixelx, pixely);
	keyPnts.push_back(cvpntCenter);



}

void DrawLabel::DrawLabel_Draw(cv::Mat &img) 
{
	int pnts[1];
	cv::Point *root_points[1];
	root_points[0] = new cv::Point[keyPnts.size()-1];
	int iter = 0;
	std::for_each(keyPnts.begin()+1, keyPnts.end(), [&](cv::Point pt) { root_points[0][iter] = pt; iter++; });
	const cv::Point* cntPt[1] = { root_points[0] };

	pnts[0] = keyPnts.size();
	cv::fillPoly(img, cntPt, pnts, 1, cv::Scalar(0, 0, 255), cv::LINE_8);

	delete[]root_points[0];
}


void DrawLabelVerticalCircle_Only::DrawLabel_Draw(cv::Mat &img)
{
	cv::circle(img, keyPnts[0], 3, cv::Scalar(0, 0, 255));
}

#endif // _USE_OPENCV
