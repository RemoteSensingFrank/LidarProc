
// Created by wuwei on 18-1-24.


#include "LASColorMap.h"
#include <iostream>
#include "../LidarBase/LASReader.h"
#include "../LidarGeometry/Geometry.h"
#include "../LidarGeometry/GeometryAlgorithm.h"

#pragma comment(lib,"gdal_i.lib")

long LASColorMap::LASColorMap_Map(const char* pathLas, vector<string> pathImgs, const char* pathOutLas)
{
	FILE* fInLas = fopen(pathLas, "rb");
	FILE* fOutLas = fopen(pathOutLas, "wb");

	LidarPatchReader reader;
	LASHeader lasHeader;
	reader.LidarReader_ReadHeader(fInLas, lasHeader);
	reader.LidarReader_WriteHeader(fOutLas, lasHeader);

	//每个block取10w个点
	const int numBlock = 100000;
	int realRead = numBlock;
	LASPoint* points = NULL;
	try {
		points = new LASPoint[numBlock];
	}
	catch (std::bad_alloc e)
	{
		std::cerr << e.what() << endl;
	}
	int totalNum = 0;
	//需要反复的读取数据
	while (realRead != 0)
	{
		Rect2D rect = reader.LidarReader_ReadPatch(fInLas, lasHeader, points, realRead);
		totalNum += realRead;
		printf("process points:%d\n", totalNum);

		int realProcessPoints = 0;

		if (realRead == 0)
			break;

		//赋值标记
		char* labelPoint = new char[realRead];
		memset(labelPoint, 0, sizeof(char)*realRead);

		for (int i = 0; i<pathImgs.size(); ++i)
		{
			GDALAllRegister();
			GDALDatasetH m_dataset = GDALOpen(pathImgs[i].c_str(), GA_ReadOnly);
			double adfGeoTrans[6];
			GDALGetGeoTransform(m_dataset, adfGeoTrans);
			int xsize = GDALGetRasterXSize(m_dataset);
			int ysize = GDALGetRasterYSize(m_dataset);
			Rect2D rectImg = LASColorMap_ImageRange(m_dataset);

			if (GeometryRelation::IsRectIntersectRect(rectImg, rect))
			{
				unsigned char *data = NULL;
				data = new unsigned char[3 * xsize*ysize];
				for (int nb = 1; nb <= 3; ++nb)
					GDALRasterIO(GDALGetRasterBand(m_dataset, nb), GF_Read, 0, 0, xsize, ysize, data + (nb - 1)*xsize*ysize, xsize, ysize, GDT_Byte, 0, 0);

				//每一个点在影像上的坐标
				for (int j = 0; j<realRead; ++j)
				{
					if (!labelPoint[j]) {
						double x = points[j].m_vec3d.x;
						double y = points[j].m_vec3d.y;
						int ix, iy;
						LASColorMap_MapToImage(x, y, adfGeoTrans, ix, iy);
						if (ix < 0 || iy < 0 || ix >= xsize || iy >= ysize)
							continue;
						else {
							if (data[iy * xsize + ix] == 0 && data[iy * xsize + ix + xsize * ysize] == 0 && data[iy * xsize + ix + xsize * ysize] == 0)
								continue;
							points[j].m_colorExt.Red = data[iy * xsize + ix];
							points[j].m_colorExt.Green = data[iy * xsize + ix + xsize * ysize];
							points[j].m_colorExt.Blue = data[iy * xsize + ix + 2 * xsize * ysize];
							realProcessPoints++;
							labelPoint[j] = 1;
						}
					}
				}
				delete[]data; data = NULL;
			}
			GDALClose(m_dataset);
			if (realProcessPoints == realRead)
			{
				reader.LidarReader_WritePatch(fOutLas, lasHeader, points, realRead);
				break;
			}
		}

		//for test
		//测试哪些点没有颜色
//#ifdef _DEBUG
//		FILE* ofs = fopen("E:\\test.txt", "a+");
//		for (int i = 0; i < realRead; ++i)
//		{
//			labelPoint[i] == 0 ? fprintf(ofs, "%d:%lf,%lf,%lf\n", totalNum +i, points[i].m_vec3d.x, points[i].m_vec3d.y, points[i].m_vec3d.z): i = i ;
//		}
//		fclose(ofs);
//#endif // DEBUG
		
		delete[]labelPoint; labelPoint = NULL;
		
		if (realProcessPoints != realRead)
			reader.LidarReader_WritePatch(fOutLas, lasHeader, points, realRead);
	};


	delete[]points; points = NULL;
	fclose(fInLas);
	fclose(fOutLas);
	return 0;
}

long LASColorMap::LASColorMap_Map(const char* pathLas, const char* pathImg, const char* pathOutLas)
{
	FILE* fInLas = fopen(pathLas, "rb");
	FILE* fOutLas = fopen(pathOutLas, "wb");

	LidarPatchReader reader;
	LASHeader lasHeader;
	reader.LidarReader_ReadHeader(fInLas, lasHeader);
	reader.LidarReader_WriteHeader(fOutLas, lasHeader);

	//每个block取10w个点
	const int numBlock = 100000;
	int realRead = numBlock;
	GDALAllRegister();
	GDALDatasetH m_dataset = GDALOpen(pathImg, GA_ReadOnly);
	double adfGeoTrans[6];
	GDALGetGeoTransform(m_dataset, adfGeoTrans);
	int xsize = GDALGetRasterXSize(m_dataset);
	int ysize = GDALGetRasterYSize(m_dataset);

	Rect2D rectImg = LASColorMap_ImageRange(m_dataset);

	Rect2D rangeLas;
	rangeLas.minx = lasHeader.min_x;    rangeLas.maxx = lasHeader.max_x;
	rangeLas.miny = lasHeader.min_y;    rangeLas.maxy = lasHeader.max_y;
	if (!GeometryRelation::IsRectIntersectRect(rectImg, rangeLas))
		return -1;


	LASPoint* points = NULL;
	//
	unsigned char *data = NULL;

	try {
		data = new unsigned char[3 * xsize*ysize];
		points = new LASPoint[numBlock];
	}
	catch (std::bad_alloc e)
	{
		std::cerr << e.what() << endl;
	}

	//读取影像数据
	for (int i = 0; i<3; ++i)
	{
		GDALRasterIO(GDALGetRasterBand(m_dataset, i + 1), GF_Read, 0, 0, xsize, ysize, data + i*xsize*ysize, xsize, ysize, GDT_Byte, 0, 0);
	}

	while (realRead != 0)
	{
		Rect2D rect = reader.LidarReader_ReadPatch(fInLas, lasHeader, points, realRead);
		//每一个点在影像上的坐标
		for (int i = 0; i<realRead; ++i)
		{
			double x = points[i].m_vec3d.x;
			double y = points[i].m_vec3d.y;
			int ix, iy;
			LASColorMap_MapToImage(x, y, adfGeoTrans, ix, iy);
			if (ix<0 || iy<0 || ix>xsize || iy>ysize)
				continue;
			else {
				points[i].m_colorExt.Red = data[iy*xsize + ix];
				points[i].m_colorExt.Green = data[iy*xsize + ix + xsize*ysize];
				points[i].m_colorExt.Blue = data[iy*xsize + ix + 2 * xsize*ysize];
			}
		}
		reader.LidarReader_WritePatch(fOutLas, lasHeader, points, realRead);

	};

	delete[]points; points = NULL;
	delete[]data; data = NULL;

	fclose(fInLas);
	fclose(fOutLas);
	GDALClose(m_dataset);
	return 0;
}

Rect2D LASColorMap::LASColorMap_ImageRange(GDALDatasetH m_dataset)
{
	double adfGeoTransform[6];
	GDALGetGeoTransform(m_dataset, adfGeoTransform);

	int xsize = GDALGetRasterXSize(m_dataset);
	int ysize = GDALGetRasterYSize(m_dataset);

	double mx1, my1;
	double mx2, my2;
	LASColorMap_ImageToMap(0, 0, adfGeoTransform, mx1, my1);
	LASColorMap_ImageToMap(xsize, ysize, adfGeoTransform, mx2, my2);

	Rect2D range;
	range.minx = min(mx1, mx2);
	range.maxx = max(mx1, mx2);
	range.miny = min(my1, my2);
	range.maxy = max(my1, my2);

	return  range;
}

bool LASColorMap::LASColorMap_MapToImage(double mx, double my, double adfGeoTransform[6], int &ix, int &iy)
{
	ix = (mx - adfGeoTransform[0]) / adfGeoTransform[1];
	iy = (my - adfGeoTransform[3]) / adfGeoTransform[5];
	return true;
}

bool LASColorMap::LASColorMap_ImageToMap(int ix, int iy, double *adfGeoTransform, double &mx, double &my)
{
	mx = adfGeoTransform[0] + adfGeoTransform[1] * ix + adfGeoTransform[2] * iy;
	my = adfGeoTransform[3] + adfGeoTransform[4] * ix + adfGeoTransform[5] * iy;
	return true;
}

