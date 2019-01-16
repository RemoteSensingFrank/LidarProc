
//
// Created by wuwei on 18-1-14.
//
#include "../LidarBase/LASReader.h"
#include "../LidarGeometry/tsmToUTM.h"
#include "../LidarGeometry/GeometryAlgorithm.h"
#include "LASSimpleClassify.h"
#include <Eigen/Dense>

using namespace Eigen;

long LASSimpleClassify::LASVegetationByEcho(ILASDataset *dataset)
{
	const int &numpoints = dataset->m_totalReadLasNumber;
	for (int i = 0; i<dataset->m_numRectangles; ++i)
	{
		LASRectBlock &block = dataset->m_lasRectangles[i];
		for (int j = 0; j < block.m_lasPoints_numbers; ++j) {
			LASPoint &point = block.m_lasPoints[j];
			if ((GetReturnNumber(point.m_rnseByte)>1))
			{
				point.m_classify = elcLowVegetation;
			}
		}
	}
	return 0;
}

long LASSimpleClassify::LASClassifyByElevation(ILASDataset *dataset, float elevation, bool direct,
	eLASClassification eclass)
{
	const int &numpoints = dataset->m_totalReadLasNumber;
	for (int i = 0; i<dataset->m_numRectangles; ++i)
	{
		LASRectBlock &block = dataset->m_lasRectangles[i];
		for (int j = 0; j < block.m_lasPoints_numbers; ++j) {
			LASPoint &point = block.m_lasPoints[j];
			if (direct)
			{
				point.m_classify = point.m_vec3d.z>elevation ? eclass : elcUnclassified;
			}
			else {
				point.m_classify = point.m_vec3d.z<elevation ? eclass : elcUnclassified;
			}
		}
	}

	return 0;
}

long LASSimpleClassify::LASClassifyByIntensity(ILASDataset *dataset, float intensity, bool direct,
	eLASClassification eclass)
{
	const int &numpoints = dataset->m_totalReadLasNumber;
	for (int i = 0; i<dataset->m_numRectangles; ++i)
	{
		LASRectBlock &block = dataset->m_lasRectangles[i];
		for (int j = 0; j < block.m_lasPoints_numbers; ++j) {
			LASPoint &point = block.m_lasPoints[j];
			if (direct)
			{
				point.m_classify = point.m_intensity>intensity ? eclass : elcUnclassified;
			}
			else {
				point.m_classify = point.m_intensity<intensity ? eclass : elcUnclassified;
			}
		}
	}

	return 0;
}

long LASSimpleClassify::LASClassifyByColor(ILASDataset *dataset, std::vector<ColoInfo> colorInfo)
{
	if (!dataset->m_lasHeader.HasLASColorExt4() && !dataset->m_lasHeader.HasLASColorExt6())
		return -1;

	const int &numpoints = dataset->m_totalReadLasNumber;
	for (int i = 0; i<dataset->m_numRectangles; ++i)
	{
		LASRectBlock &block = dataset->m_lasRectangles[i];
		for (int j = 0; j < block.m_lasPoints_numbers; ++j) {
			LASPoint &point = block.m_lasPoints[j];
			int minIdx = 0;
			double minDis = 9999999;

			for (int i = 0; i<colorInfo.size(); ++i)
			{
				double dis = sqrt((point.m_colorExt.Red - colorInfo[i].red)*
					(point.m_colorExt.Red - colorInfo[i].red) +
					(point.m_colorExt.Green - colorInfo[i].green)*
					(point.m_colorExt.Green - colorInfo[i].green) +
					(point.m_colorExt.Blue - colorInfo[i].blue)*
					(point.m_colorExt.Blue - colorInfo[i].blue));
				minIdx = minDis>dis ? i : minIdx;
				minDis = min(minDis, dis);
			}
			point.m_classify = colorInfo[i].classType;
		}
	}

	return 0;
}

/////////////////////////////////////


long LASClassifyMemLimited::LASExportClassifiedPoints(const char* pathLas, eLASClassification type, const char* pathExport)
{
	FILE* fLasIn = nullptr, *fLasOut = nullptr;
	fLasIn = fopen(pathLas, "rb");
	fLasOut = fopen(pathExport, "wb");

	if (fLasIn == nullptr || fLasOut == nullptr)
		return -1;

	LASHeader    headerLas;
	LidarPatchReader patchReaderLine;
	patchReaderLine.LidarReader_ReadHeader(fLasIn, headerLas);
	int curPos = ftell(fLasIn);
	unsigned char buffer[34];
	int tmpsize=fread(buffer, 1, 34, fLasIn);
	int typesNumber = 0;
	int pointReserved = headerLas.number_of_point_records;
	const int readPatch = 100000;
	LASPoint* lasPnt = nullptr;
	try
	{
		lasPnt = new LASPoint[readPatch];
	}
	catch (bad_alloc e) {
		printf("%s\n", e.what());
		return -1;
	}

	double xmin = 99999999, xmax = -9999999;
	double ymin = 99999999, ymax = -9999999;
	double zmin = 99999999, zmax = -9999999;

	//get type number　and range first
	int realReadPoints = 0;
	int firstpointReserved = pointReserved;
	while (!feof(fLasIn) && firstpointReserved>0) {
		if (firstpointReserved < readPatch)
			realReadPoints = firstpointReserved;
		else
			realReadPoints = readPatch;

		Rect2D rect = patchReaderLine.LidarReader_ReadPatch(fLasIn, headerLas, lasPnt, realReadPoints);
		for (int i = 0; i<realReadPoints; ++i)
		{
			if (lasPnt[i].m_classify == type) {
				typesNumber++;
				xmin = min(xmin, lasPnt[i].m_vec3d.x);
				xmax = max(xmax, lasPnt[i].m_vec3d.x);
				ymin = min(ymin, lasPnt[i].m_vec3d.y);
				ymax = max(ymax, lasPnt[i].m_vec3d.y);
				zmin = min(zmin, lasPnt[i].m_vec3d.z);
				zmax = max(zmax, lasPnt[i].m_vec3d.z);
			}
		}
		firstpointReserved = firstpointReserved - realReadPoints;
	};
	fseek(fLasIn, curPos, SEEK_SET);
	headerLas.min_x = xmin; headerLas.max_x = xmax;
	headerLas.min_y = ymin; headerLas.max_y = ymax;
	headerLas.min_z = zmin; headerLas.max_z = zmax;
	headerLas.number_of_point_records = typesNumber;
	//export
	patchReaderLine.LidarReader_WriteHeader(fLasOut, headerLas);

	realReadPoints = 0;
	while (!feof(fLasIn) && (pointReserved>0)) {
		if (pointReserved < readPatch)
			realReadPoints = pointReserved;
		else
			realReadPoints = readPatch;

		Rect2D rect = patchReaderLine.LidarReader_ReadPatch(fLasIn, headerLas, lasPnt, realReadPoints);
		for (int i = 0; i<realReadPoints; ++i)
		{
			if (lasPnt[i].m_classify == type) {
				patchReaderLine.LidarReader_WritePatch(fLasOut, headerLas, &lasPnt[i], 1);
			}
		}
		pointReserved = pointReserved - realReadPoints;
	};
	delete[]lasPnt; lasPnt = nullptr;
	fclose(fLasIn); fLasIn = nullptr;
	fclose(fLasOut); fLasOut = nullptr;
	return 0;
}
