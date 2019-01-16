//
// Created by wuwei on 18-1-24.
//
#include <omp.h>
#include "gdal_priv.h"
#include "LASDangerPoints.h"

#include "PointProcAlgorithm.h"

static void SetPointColor(LASPoint &pnt, short r, short g, short b)
{
	pnt.m_colorExt.Red = r;
	pnt.m_colorExt.Green = g;
	pnt.m_colorExt.Blue = b;
}

long LASDangerPoints::LASDangerPoints_Range(const Point3D *pnt, float distance, ILASDataset* datasetVegterain, vector<int> &rectIds)
{
	Rect2D rect;
	rect.minx = pnt->x - distance; rect.maxx = pnt->x + distance;
	rect.miny = pnt->y - distance; rect.maxy = pnt->y + distance;
	datasetVegterain->LASDataset_Search(0, rect, rectIds);
	return 0;
}

long LASDangerPoints::LASDangerPoints_PerPoint(float distance, const Point3D *pnt, ILASDataset *datasetVegterian)
{
	vector<int> rectRangeIdx;
	LASDangerPoints_Range(pnt, distance, datasetVegterian, rectRangeIdx);

	for (int i = 0; i<rectRangeIdx.size(); ++i)
	{
		int ind = rectRangeIdx[i];
		for (int j = 0; j <datasetVegterian->m_lasRectangles[ind].m_lasPoints_numbers; ++j) {
			const Point3D tmpPnt = datasetVegterian->m_lasRectangles[ind].m_lasPoints[j].m_vec3d;
			if (pnt->Distance(tmpPnt)<distance) {
				datasetVegterian->m_lasRectangles[ind].m_lasPoints[j].m_classify = elcDanger;
			}
		}
	}

	return 0;
}

long LASDangerPoints::LASDangerPoints_PerPoint(float *distance, int dangerSectionNumber, const Point3D *pnt,ILASDataset *datasetVegterian) {

	vector<int> rectRangeIdx;
	LASDangerPoints_Range(pnt, distance[dangerSectionNumber - 1], datasetVegterian, rectRangeIdx);
	if (dangerSectionNumber != 3)
	{
		printf("plz input 3 number range\n");
		return -1;
	}

	//only consider 3 type of classes
	for (int i = 0; i<rectRangeIdx.size(); ++i)
	{
		int ind = rectRangeIdx[i];
		for (int j = 0; j <datasetVegterian->m_lasRectangles[ind].m_lasPoints_numbers; ++j) {

			LASPoint &laspnt = datasetVegterian->m_lasRectangles[ind].m_lasPoints[j];
			const Point3D tmpPnt = laspnt.m_vec3d;
			int classType = laspnt.m_classify;

			if (classType>elcDanger&&classType<elcDangerEnd)
			{
				if (pnt->Distance(tmpPnt)<distance[0] && classType>elcDanger + 1)
				{
					laspnt.m_classify = elcDangerLevel1;
					SetPointColor(laspnt, 255, 0, 0);
				}
				else if (pnt->Distance(tmpPnt)<distance[1] && classType>elcDanger + 2)
				{
					laspnt.m_classify = elcDangerLevel2;
					SetPointColor(laspnt, 255, 153, 0);
				}
				else if (pnt->Distance(tmpPnt)<distance[2] && classType>elcDanger + 3)
				{
					laspnt.m_classify = elcDanger;
					SetPointColor(laspnt, 153, 0, 255);
				}
			}
			else
			{
				double dis = pnt->Distance(tmpPnt);
				if (dis<distance[0])
				{
					laspnt.m_classify = elcDangerLevel1;
					SetPointColor(laspnt, 255, 0, 0);
				}
				else if (dis<distance[1])
				{
					laspnt.m_classify = elcDangerLevel2;
					SetPointColor(laspnt, 255, 153, 0);
				}
				else if (dis<distance[2])
				{
					laspnt.m_classify = elcDangerLevel3;
					SetPointColor(laspnt, 153, 0, 255);
				}
			}
		}
	}
	return 0;
}

long LASDangerPoints::LASDangerPoints_Detect(float distance, ILASDataset *datasetLine, ILASDataset *datasetVegterain)
{
	int rs = 0;
	int numLineRects = datasetLine->m_numRectangles;
	for (int i = 0; i<numLineRects; ++i)
	{
		int numPntInRect = datasetLine->m_lasRectangles[i].m_lasPoints_numbers;
		for (int j = 0; j<numPntInRect; ++j)
		{
			printf("\r%d-%d", numPntInRect, j);
			const Point3D linePnt = datasetLine->m_lasRectangles[i].m_lasPoints[j].m_vec3d;
			rs = rs | LASDangerPoints_PerPoint(distance, &linePnt, datasetVegterain);
		}
		printf("\n");
	}
	return rs;
}

long LASDangerPoints::LASDangerPoints_Detect(float* distance, int dangerSectionNumber, ILASDataset* datasetLine, ILASDataset* datasetVegterain) 
{
	int rs = 0;
	int numLineRects = datasetLine->m_numRectangles;
	for (int i = 0; i<numLineRects; ++i)
	{
		int numPntInRect = datasetLine->m_lasRectangles[i].m_lasPoints_numbers;
		printf("%d-%d", numLineRects, i);
		for (int j = 0; j<numPntInRect; ++j)
		{
			const Point3D linePnt = datasetLine->m_lasRectangles[i].m_lasPoints[j].m_vec3d;
			rs = rs | LASDangerPoints_PerPoint(distance, dangerSectionNumber, &linePnt, datasetVegterain);
		}
		printf("\n");
	}
	//trim to elcDanger to export and process
	for (int i = 0; i<datasetVegterain->m_numRectangles; ++i)
	{
		int numPntInRect = datasetVegterain->m_lasRectangles[i].m_lasPoints_numbers;
		for (int j = 0; j<numPntInRect; ++j)
		{
			LASPoint &pnt = datasetVegterain->m_lasRectangles[i].m_lasPoints[j];
			int classType = pnt.m_classify;
			if (classType>elcDanger&&classType<elcDangerEnd) {
				pnt.m_classify = elcDanger;
			}
		}
	}

	return rs;
}

/*
//deleted by Frank.Wu 弃用此函数
long LASDangerPoints::LADDangerPoints_SplitDanger(ILASDataset *datasetVegterain, const char *pathSplit)
{
	LidarMemReader lidarOpt;
	lidarOpt.LidarReader_Export(pathSplit, datasetVegterain, (int)elcDanger);
	return 0;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FLANN Danger Detect
long LASDangerPointsFlann::LASDangerPoints_PerPoint(float distance, const Point3D* pnt,kd_tree &treeVege, ILASDataset* datasetVegterain)
{
	double query_pt[3] = { pnt->x, pnt->y, pnt->z };
	//const size_t num_results = 10;
	//size_t ret_index[10];
	//double out_dist_sqr[10];
	//KNNResultSet<double> resultSet(num_results);

	const double radius = distance;
	std::vector<std::pair<size_t, double> > indices_dists;
	RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

	//resultSet.init(ret_index, out_dist_sqr);
	treeVege.findNeighbors(resultSet, &query_pt[0], SearchParams());

	for (int i = 0; i < resultSet.m_indices_dists.size(); ++i)
	{
		//for debug
		//printf("ret_index=%d out_dist_sqr=%lf\n", resultSet.m_indices_dists[i].first, resultSet.m_indices_dists[i].second);
		const LASIndex &idx = datasetVegterain->m_LASPointID[resultSet.m_indices_dists[i].first];
		LASPoint &pnt = datasetVegterain->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect];
		pnt.m_classify = elcDanger;
	}

	return 0;
}

long LASDangerPointsFlann::LASDangerPoints_PerPoint(float* distance, int dangerSectionNumber, const Point3D* pnt,  kd_tree &treeVege, ILASDataset* datasetVegterain)
{
	double query_pt[3] = { pnt->x, pnt->y, pnt->z };
	//const size_t num_results = 10;
	//size_t ret_index[10];
	//double out_dist_sqr[10];
	//KNNResultSet<double> resultSet(num_results);
	//目前只考虑三个段
	const double radius = distance[2];
	std::vector<std::pair<size_t, double> > indices_dists;
	RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

	//resultSet.init(ret_index, out_dist_sqr);
	treeVege.findNeighbors(resultSet, &query_pt[0], SearchParams());

	for (int i = 0; i < resultSet.m_indices_dists.size(); ++i)
	{
		//for debug
		//printf("ret_index=%d out_dist_sqr=%lf\n", resultSet.m_indices_dists[i].first, resultSet.m_indices_dists[i].second);
		const LASIndex &idx = datasetVegterain->m_LASPointID[resultSet.m_indices_dists[i].first];
		LASPoint &pnt = datasetVegterain->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect];
	
		int classType = pnt.m_classify;
		double dis = resultSet.m_indices_dists[i].second;
		if (classType>elcDanger&&classType<elcDangerEnd)
		{
			if (dis<distance[0] && classType>elcDanger + 1)
			{
				pnt.m_classify = elcDangerLevel1;
				SetPointColor(pnt, 255, 0, 0);
			}
			else if (dis<distance[1] && classType>elcDanger + 2)
			{
				pnt.m_classify = elcDangerLevel2;
				SetPointColor(pnt, 255, 153, 0);
			}
			else if (dis<distance[2] && classType>elcDanger + 3)
			{
				pnt.m_classify = elcDanger;
				SetPointColor(pnt, 153, 0, 255);
			}
		}
		else
		{
			double dis = resultSet.m_indices_dists[i].second;
			if (dis < distance[0])
			{
				pnt.m_classify = elcDangerLevel1;
				SetPointColor(pnt, 255, 0, 0);
			}
			else if (dis < distance[1])
			{
				pnt.m_classify = elcDangerLevel2;
				SetPointColor(pnt, 255, 153, 0);
			}
			else if (dis < distance[2])
			{
				pnt.m_classify = elcDangerLevel3;
				SetPointColor(pnt, 153, 0, 255);
			}
		}
	}//for 循环结束
	return 0;
}

long LASDangerPointsFlann::LASDangerPoints_Detect(float distance, ILASDataset* datasetLine, ILASDataset* datasetVegterain)
{
	double dis = distance * distance;
	//构造数据结构
	std::vector<Point3D> pntCloud;
	for (int i = 0; i < datasetVegterain->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = datasetVegterain->m_LASPointID[i];
		pntCloud.push_back(datasetVegterain->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
	}

	const PCAdaptor pcAdaptorPnts(pntCloud);
	kd_tree treeVegeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeVegeIndex.buildIndex();

	//每一个点进行做FLANN
	for (int i = 0; i < datasetLine->m_totalReadLasNumber; ++i)
	{
		printf("\rprocess points %d/%d", datasetLine->m_totalReadLasNumber, i + 1);
		const LASIndex &idx = datasetLine->m_LASPointID[i];
		const Point3D &pnt=datasetLine->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d;
		LASDangerPoints_PerPoint(dis, &pnt, treeVegeIndex, datasetVegterain);
	}
	printf("\n");

	pntCloud.clear();
	return 0;
}

long LASDangerPointsFlann::LASDangerPoints_Detect(float* distance, int dangerSectionNumber, ILASDataset* datasetLine, ILASDataset* datasetVegterain)
{
	float *dis = new float[dangerSectionNumber];
	for (int i = 0; i < dangerSectionNumber; ++i)
		dis[i] = distance[i] * distance[i];

	//构造数据结构
	std::vector<Point3D> pntCloud;
	for (int i = 0; i < datasetVegterain->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = datasetVegterain->m_LASPointID[i];
		pntCloud.push_back(datasetVegterain->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
	}

	const PCAdaptor pcAdaptorPnts(pntCloud);
	kd_tree treeVegeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeVegeIndex.buildIndex();

	//每一个点进行做FLANN
	for (int i = 0; i < datasetLine->m_totalReadLasNumber; ++i)
	{
		printf("\rprocess points %d/%d", datasetLine->m_totalReadLasNumber, i + 1);
		const LASIndex &idx = datasetLine->m_LASPointID[i];
		const Point3D &pnt = datasetLine->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d;
		LASDangerPoints_PerPoint(dis,dangerSectionNumber, &pnt, treeVegeIndex, datasetVegterain);
	}
	printf("\n");
	for (int i = 0; i < datasetVegterain->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = datasetVegterain->m_LASPointID[i];
		LASPoint &pnt = datasetVegterain->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect];
		int classType = pnt.m_classify;
		if (classType > elcDanger&&classType < elcDangerEnd)
			pnt.m_classify = elcDanger;
	}
	delete[]dis; dis = nullptr;
	pntCloud.clear();
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float LASFallingTreesDangerPoints::LASDangerPoints_Elevation(float *dataImg, int xsize, int ysize, double *adfGeoTrans,
	float gx, float gy) {
	double dTemp = adfGeoTrans[1] * adfGeoTrans[5] - adfGeoTrans[2] * adfGeoTrans[4];
	int Xpixel = (adfGeoTrans[5] * (gx - adfGeoTrans[0]) - adfGeoTrans[2] * (gy - adfGeoTrans[3])) / dTemp + 0.5;
	int Yline = (adfGeoTrans[1] * (gy - adfGeoTrans[3]) - adfGeoTrans[4] * (gx - adfGeoTrans[0])) / dTemp + 0.5;
	if (Xpixel>xsize || Xpixel<0 || Yline>ysize || Yline<0)
		return -9999;
	else
		return dataImg[Yline*xsize + Xpixel];

}

long LASFallingTreesDangerPoints::LASDangerPoints_FallingTree(float* distance, int dangerSectionNumber, const char* pathDEM, ILASDataset* datasetLine, ILASDataset* datasetVegterain) {
	int rs = 0;
	int numLineRects = datasetLine->m_numRectangles;
	//dem dataset
	GDALAllRegister();
	GDALDatasetH dataset = GDALOpen(pathDEM, GA_ReadOnly);
	int xsize = GDALGetRasterXSize(dataset);
	int ysize = GDALGetRasterYSize(dataset);
	float* dataDEM = new float[xsize*ysize];
	GDALRasterIO(GDALGetRasterBand(dataset, 1), GF_Read, 0, 0, xsize, ysize, dataDEM, xsize, ysize, GDT_Float32, 0, 0);
	double adfGeotransform[6];
	GDALGetGeoTransform(dataset, adfGeotransform);

	for (int i = 0; i<numLineRects; ++i)
	{
		int numPntInRect = datasetLine->m_lasRectangles[i].m_lasPoints_numbers;
		for (int j = 0; j<numPntInRect; ++j)
		{
			printf("\r%d-%d", numPntInRect, j);
			const Point3D linePnt = datasetLine->m_lasRectangles[i].m_lasPoints[j].m_vec3d;
			rs = rs | LASDangerPoints_FllingTree_PrePoint(distance, dangerSectionNumber, dataDEM, xsize, ysize, adfGeotransform, &linePnt, datasetVegterain);
		}
		printf("\n");
	}
	delete[]dataDEM; dataDEM = nullptr;
	return rs;
}

long LASFallingTreesDangerPoints::LASDangerPoints_FllingTree_PrePoint(float* distance, int dangerSectionNumber, 
	float *demData, int xsize, int ysize,double *adfGeotrans, const Point3D* pnt, ILASDataset* datasetVegterian) 
{
	vector<int> rectRangeIdx;
	LASDangerPoints_Range(pnt, 3 * distance[dangerSectionNumber - 1], datasetVegterian, rectRangeIdx);
	if (dangerSectionNumber != 3)
	{
		printf("plz input 3 number range\n");
		return -1;
	}
	
	Point3D pntGround;
	pntGround.x = pnt->x;
	pntGround.y = pnt->y;
	pntGround.z = LASDangerPoints_Elevation(demData, xsize, ysize, adfGeotrans, pnt->x, pnt->y);
	double height = fabs(pntGround.z - pnt->z);

	//only consider 3 type of classes
	for (int i = 0; i<rectRangeIdx.size(); ++i)
	{
		int ind = rectRangeIdx[i];
		for (int j = 0; j <datasetVegterian->m_lasRectangles[ind].m_lasPoints_numbers; ++j) {
			LASPoint &laspnt = datasetVegterian->m_lasRectangles[ind].m_lasPoints[j];
			const Point3D tmpPnt = laspnt.m_vec3d;
			int classType = laspnt.m_classify;
			if (classType == elcDanger)
				continue;

			if (classType>elcFallingTree&&classType<elcFallingTreeEnd)
			{
				if (pntGround.Distance(tmpPnt) - height<distance[0] && classType>elcFallingTreeLevel1 + 1)
				{
					laspnt.m_classify = elcFallingTreeLevel1;
					SetPointColor(laspnt, 255, 0, 0);
				}
				else if (pnt->Distance(tmpPnt) - height<distance[1] && classType>elcFallingTreeLevel2 + 2)
				{
					laspnt.m_classify = elcFallingTreeLevel2;
					SetPointColor(laspnt, 255, 255, 0);
				}
				else if (pnt->Distance(tmpPnt) - height<distance[2] && classType>elcFallingTreeLevel3 + 3)
				{
					laspnt.m_classify = elcFallingTreeLevel3;
					SetPointColor(laspnt, 0, 0, 255);
				}
			}
			else
			{
				if (pntGround.Distance(tmpPnt) - height<distance[0] && classType>elcFallingTreeLevel1 + 1)
				{
					laspnt.m_classify = elcFallingTreeLevel1;
					SetPointColor(laspnt, 255, 0, 0);
				}
				else if (pnt->Distance(tmpPnt) - height<distance[1] && classType>elcFallingTreeLevel2 + 2)
				{
					laspnt.m_classify = elcFallingTreeLevel2;
					SetPointColor(laspnt, 255, 255, 0);
				}
				else if (pnt->Distance(tmpPnt) - height<distance[2] && classType>elcFallingTreeLevel3 + 3)
				{
					laspnt.m_classify = elcFallingTreeLevel3;
					SetPointColor(laspnt, 0, 0, 255);
				}
			}
		}
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
long LASDangerPointsMergeArrgegate::LASDangerTestFlann()
{
	std::vector<Point3D> pntCloud;
	srand(time(NULL));
	//generated pnt cloud
	pntCloud.resize(10000);
	for (size_t i = 0; i < 10000; i++)
	{
		pntCloud[i].x = 10 * (rand() % 1000) / double(1000);
		pntCloud[i].y = 10 * (rand() % 1000) / double(1000);
		pntCloud[i].z = 10 * (rand() % 1000) / double(1000);
	}

	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pntCloud);

	//typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	// do a knn search
	double query_pt[3] = { 0.5, 0.5, 0.5 };
	const size_t num_results = 10;
	size_t ret_index[10];
	double out_dist_sqr[10];
	//KNNResultSet<double> resultSet(num_results);
	const double radius = 10;
	std::vector<std::pair<size_t, double> > indices_dists;
	RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

	//resultSet.init(ret_index, out_dist_sqr);
	treeIndex.findNeighbors(resultSet, &query_pt[0], SearchParams());

	//index.knnSearch(query, indices, dists, num_results, mrpt_flann::SearchParams(10));
	
	//printf("knnSearch(nn=%d)\n", num_results);

	for (int i = 0; i < resultSet.m_indices_dists.size();++i)
	{
		printf("ret_index=%d out_dist_sqr=%lf\n", resultSet.m_indices_dists[i].first, resultSet.m_indices_dists[i].second);
	}

	return 0;
}

long LASDangerPointsMergeArrgegate::LASDangerExtract(ILASDataset* lasDataset, Point3Ds &dangerPnts)
{
	int pointsNumber = lasDataset->m_totalReadLasNumber;
	dangerPnts.clear();
	try
	{
		dangerPnts.reserve(pointsNumber*0.1);

		for (int i = 0; i < pointsNumber; ++i)
		{
			LASIndex idxLas = lasDataset->m_LASPointID[i];
			const LASPoint &lasPnt = lasDataset->m_lasRectangles[idxLas.rectangle_idx].m_lasPoints[idxLas.point_idx_inRect];
			if(lasPnt.m_classify==32)
				dangerPnts.push_back(lasPnt.m_vec3d);
		}
	}
	catch (bad_alloc e)
	{
		printf(e.what());
	}
	return 0;
}

long LASDangerPointsMergeArrgegate::LASDangerExtractLinePoints(ILASDataset *lineDataset,
	Point3Ds dangerPnts, Point3Ds &linePnts)
{
	//构建树结构
	const PCAdaptor pcAdaptorPnts(dangerPnts);
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	const size_t num_results = 1;
	size_t ret_index;
	double out_dist_sqr;
	linePnts.resize(dangerPnts.size());

	for (int i = 0; i < lineDataset->m_totalReadLasNumber; ++i)
	{
		const LASIndex &lasIdx = lineDataset->m_LASPointID[i];

		double query_pt[3] = { 
			lineDataset->m_lasRectangles[lasIdx.rectangle_idx].m_lasPoints[lasIdx.point_idx_inRect].m_vec3d.x ,
			lineDataset->m_lasRectangles[lasIdx.rectangle_idx].m_lasPoints[lasIdx.point_idx_inRect].m_vec3d.y,
			lineDataset->m_lasRectangles[lasIdx.rectangle_idx].m_lasPoints[lasIdx.point_idx_inRect].m_vec3d.z
		};
		KNNResultSet<double> resultSet(num_results);
		resultSet.init(&ret_index, &out_dist_sqr);
		treeIndex.findNeighbors(resultSet, &query_pt[0],SearchParams(10));

		linePnts[ret_index].x = query_pt[0];
		linePnts[ret_index].y = query_pt[1];
		linePnts[ret_index].z = query_pt[2];
	}

	return 0;
}

long LASDangerPointsMergeArrgegate::LASDangerAggregate(Point3Ds dangerPnts, int *type, float knnRange)
{
	PointCloudSegment segmentAlg;
	return segmentAlg.PointCloudSegment_DBScan(dangerPnts, type, knnRange);
}

long LASDangerPointsMergeArrgegate::LASDangerMerge(Point3Ds dangerPnts, int *type, int typeNumbers,
	Point3Ds linePnts, int *correspondPairs)
{
	int numberPnts = dangerPnts.size();
	double *distance = new double[typeNumbers];
	for (int i = 0; i < typeNumbers; ++i)
	{
		distance[i] = _MAX_LIMIT_;//初始值
		correspondPairs[i] = 0;
	}
	for (int i = 0; i < numberPnts; ++i)
	{
		int idx = type[i];
		double dis = dangerPnts[i].Distance(linePnts[i]);
		if (dis < distance[idx])
		{
			distance[idx] = dis;
			correspondPairs[idx] = i;
		}
	}

	delete[]distance; distance = nullptr;
	return 0;
}