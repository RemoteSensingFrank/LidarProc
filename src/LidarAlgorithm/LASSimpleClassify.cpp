
//
// Created by wuwei on 18-1-14.
//
#include "../LidarBase/LASReader.h"
#include "LASSimpleClassify.h"
#include "../LidarGeometry/tsmToUTM.h"
#include"../LidarGeometry/GeometryAlgorithm.h"
#include"../LidarAlgorithm/PointProcAlgorithm.h"
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

	//get type number��and range first
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

/////////////////////////////////////

long classifyElectricPatrolFast::ElectricPatrolFast_Tower(ILASDataset* dataset, Point2D towerPnt, double range, LASColorExt color)
{
	Rect2D towerRect;
	towerRect.minx = towerPnt.x - range;
	towerRect.maxx = towerPnt.x + range;
	towerRect.miny = towerPnt.y - range;
	towerRect.maxy = towerPnt.y + range;
	try
	{
		vector<int> searchRect;
		int idPoint = 0;
		dataset->LASDataset_Search(idPoint, towerRect, searchRect);
		//if the height lower than the last 5% the point will be removed
		LASIndexDisList vecIndxDis;
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int rectInd = searchRect[j];
			if (dataset->m_lasRectangles[rectInd].m_lasPoints_numbers > 0)
			{
				for (int k = 0; k < dataset->m_lasRectangles[rectInd].m_lasPoints_numbers; ++k)
				{
					LASIndexDis idxDis;
					idxDis.indx.rectangle_idx = rectInd;
					idxDis.indx.point_idx_inRect = k;
					idxDis.distance = dataset->m_lasRectangles[rectInd].m_lasPoints[k].m_vec3d.z;
					vecIndxDis.push_back(idxDis);
				}
			}
		}

		int num = vecIndxDis.size()*0.7;
		sort(vecIndxDis.begin(), vecIndxDis.end(), [](LASIndexDis a, LASIndexDis b)-> bool {return a.distance < b.distance; });

		LASIndexDisList idxDisLists;
		for (int i = num; i < vecIndxDis.size(); ++i)
		{
			idxDisLists.push_back(vecIndxDis[i]);
		}
		return ElectrixPatrolFast_Seg(dataset, idxDisLists, color, 1);
	}
	catch (std::bad_alloc  *e)
	{
		printf("%s\n", e->what());
		return -1;
	}
}

long classifyElectricPatrolFast::ElectricPatrolFast_Tower(ILASDataset* dataset, Point2Ds towerPnt, double range, LASColorExt color)
{
	for(auto tPt:towerPnt){
		ElectricPatrolFast_Tower(dataset,tPt,range,color);
	}
	return 0;
}

long classifyElectricPatrolFast::ElectrixPatrolFast_Seg(ILASDataset* dataset, LASIndexDisList idxDisLists, LASColorExt color, double distance)
{
	LasAlgorithm::PointCloudSegment seg;
	int pointsNumber = idxDisLists.size();
	int* classify = new int[pointsNumber];
	Point3Ds points;
	try
	{
		points.reserve(pointsNumber*0.1);
		for (int i = 0; i < pointsNumber; ++i)
		{
			LASIndex idxLas = idxDisLists[i].indx;
			const LASPoint &lasPnt = dataset->m_lasRectangles[idxLas.rectangle_idx].m_lasPoints[idxLas.point_idx_inRect];
			points.push_back(lasPnt.m_vec3d);
		}
	}
	catch (bad_alloc e)
	{
		printf(e.what());
		return -1;
	}

	int numtype = seg.PointCloudSegment_DBScan(points, classify, distance);
	int *typesnumber = new int[numtype];
	memset(typesnumber, 0, sizeof(int)*numtype);
	for (int j = 0; j < pointsNumber; ++j)
		typesnumber[classify[j] - 1]++;

	int maxidx = 0;
	int maxdata = 0;
	for (int i = 0; i < numtype - 1; ++i)
	{
		if (maxdata < typesnumber[i])
		{
			maxidx = i + 1;
			maxdata = typesnumber[i];
		}
	}

	for (int j = 0; j < pointsNumber; ++j)
	{
		if (classify[j] == maxidx)
		{
			LASIndex idxLas = idxDisLists[j].indx;
			dataset->m_lasRectangles[idxLas.rectangle_idx].m_lasPoints[idxLas.point_idx_inRect].m_colorExt = color;
			dataset->m_lasRectangles[idxLas.rectangle_idx].m_lasPoints[idxLas.point_idx_inRect].m_classify = elcTowerRange;
		}
	}

	delete[]classify; classify = nullptr;
	delete[]typesnumber; typesnumber = nullptr;
	points.clear();
	return 0;
}

long classifyElectricPatrolFast::ElectricPatrolFast_Lines(ILASDataset* dataset, Point2Ds towerPnt, double range, double height, LASColorExt color)
{
	Eigen::MatrixXd rotMat;
	double theta = 0;
	if (towerPnt[1].x != towerPnt[0].x)
		theta = atan2(towerPnt[1].y - towerPnt[0].y, towerPnt[1].x - towerPnt[0].x);

	rotMat = Eigen::MatrixXd::Zero(2, 2);
	rotMat(0, 0) = cos(theta);
	rotMat(0, 1) = -sin(theta);
	rotMat(1, 0) = sin(theta);
	rotMat(1, 1) = cos(theta);

	//project
	Eigen::MatrixXd pnt1 = Eigen::MatrixXd::Zero(1, 2), pnt2 = Eigen::MatrixXd::Zero(1, 2);
	pnt1(0, 0) = towerPnt[0].x;
	pnt1(0, 1) = towerPnt[0].y;
	Eigen::MatrixXd rotPnt1 = pnt1 * rotMat;

	pnt2(0, 0) = towerPnt[1].x;
	pnt2(0, 1) = towerPnt[1].y;
	Eigen::MatrixXd rotPnt2 = pnt2 * rotMat;
	double avgHeight[2] = { 0,0 };
	Rect2D towerRect[2];

	for (int i = 0; i < 2; ++i)
	{
		int num = 0;
		towerRect[i].minx = towerPnt[i].x - range;
		towerRect[i].maxx = towerPnt[i].x + range;
		towerRect[i].miny = towerPnt[i].y - range;
		towerRect[i].maxy = towerPnt[i].y + range;

		vector<int> searchRect;
		int idPoint = 0;
		dataset->LASDataset_Search(idPoint, towerRect[i], searchRect);

		LASIndexDisList vecIndxDis;
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int rectInd = searchRect[j];
			if (dataset->m_lasRectangles[rectInd].m_lasPoints_numbers > 0)
			{
				for (int k = 0; k < dataset->m_lasRectangles[rectInd].m_lasPoints_numbers; ++k)
				{
					avgHeight[i] += dataset->m_lasRectangles[rectInd].m_lasPoints[k].m_vec3d.z;
					++num;
				}
			}
		}
		avgHeight[i] /= num;
	}

	//line func
	double axisk = (avgHeight[1] - avgHeight[0]) / (rotPnt2(0, 0) - rotPnt1(0, 0));
	double axisb = avgHeight[0] - axisk*rotPnt1(0, 0);
	
	Rect2D rect;
	Eigen::MatrixXd tp1(1, 2), tp2(1, 2);
	tp1(0, 0) = towerPnt[0].x; tp1(0, 1) = towerPnt[0].y;
	tp2(0, 0) = towerPnt[1].x; tp2(0, 1) = towerPnt[1].y;
	Eigen::MatrixXd trp1 = tp1 * rotMat;
	Eigen::MatrixXd trp2 = tp2 * rotMat;

	rect.minx = min(trp1(0, 0), trp2(0, 0)) - range;
	rect.miny = min(trp1(0, 1), trp2(0, 1)) - range;
	rect.maxx = max(trp1(0, 0), trp2(0, 0)) + range;
	rect.maxy = max(trp1(0, 1), trp2(0, 1)) + range;

	//line points
	std::vector<int> rectIds;
	//four corner points
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
	Point3Ds seedPnts;
	FILE *fs=fopen("../data/test.txt","w+");
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
			Eigen::MatrixXd temprotPnt = pnt * rotMat;
			
			if (GeometryRelation::IsPointInRect(temprotPnt(0, 0), temprotPnt(0, 1), rect.minx, rect.miny, rect.maxx, rect.maxy)&&
			    !GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y, towerRect[0].minx, towerRect[0].miny, towerRect[0].maxx, towerRect[0].maxy)&&
				!GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y, towerRect[1].minx, towerRect[1].miny, towerRect[1].maxx, towerRect[1].maxy))
			{
				double z = lasPnt.m_vec3d.z;
				if (temprotPnt(0, 0)*axisk + axisb + height < z&&lasPnt.m_classify == elcCreated)
				{
					fprintf(fs,"%lf,%lf,%lf\n",lasPnt.m_vec3d.x,lasPnt.m_vec3d.y,lasPnt.m_vec3d.z);
					seedPnts.push_back(lasPnt.m_vec3d);
				}
			}
		}
	}
	fclose(fs);
	ElectrixPatrolFast_Seed(dataset, seedPnts, 0.1, color);
	return 0;
}

long classifyElectricPatrolFast::ElectrixPatrolFast_Seed(ILASDataset* dataset, Point3Ds seedPoints, double range, LASColorExt color)
{
	Point3Ds pointSet;
	vector<LASIndex> idxLists;
	for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = dataset->m_LASPointID[i];
		idxLists.push_back(idx);
		Point2D pnt;
		pointSet.push_back(dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
	}

	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pointSet);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	std::vector<Point3D> preparePntSet;
	for (int i = 0; i < seedPoints.size(); ++i)
		preparePntSet.push_back(seedPoints[i]);

	//seed point grow algorithm
	do {
		//depth first
		int i = preparePntSet.size() - 1;
		double pnt[3] = { preparePntSet[i].x,preparePntSet[i].y,preparePntSet[i].z };
		preparePntSet.pop_back();

		std::vector<std::pair<size_t, double> > indices_dists;
		RadiusResultSet<double, size_t> resultSet((double)20, indices_dists);
		//resultSet.init(ret_index, out_dist_sqr);
		treeIndex.findNeighbors(resultSet, &pnt[0], SearchParams());

		for (int j = 0; j < resultSet.m_indices_dists.size(); ++j)
		{
			int idxPnt = resultSet.m_indices_dists[j].first;
			const LASIndex &idx = idxLists[idxPnt];
			LASPoint &lasPnt = dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect];
			if (DistanceComputation::Distance(pointSet[idxPnt], Point3D(pnt[0], pnt[1], pnt[2])) && lasPnt.m_classify == elcCreated)
			{
				lasPnt.m_classify = elcLine;
				lasPnt.m_colorExt = color;
				preparePntSet.push_back(pointSet[idxPnt]);
			}
		}
	} while (!preparePntSet.empty());
	pointSet.clear();
	idxLists.clear();
	return 0;
}

long classifyElectricPatrolFast::ElectricPatrolFast_Ground(ILASDataset* dataset, double rectRange, double disThres,double angle, LASColorExt color)
{
	std::vector<LASIndex> pntIdxs;
	ElectricPatrolFast_Angle(dataset, rectRange, angle*M_PI/180, pntIdxs);
	ElectricPatrolFast_GroundDis(dataset, rectRange/2, disThres, pntIdxs);
	for (int k = 0; k < pntIdxs.size(); ++k)
	{
		int i = pntIdxs[k].rectangle_idx;
		int j = pntIdxs[k].point_idx_inRect;
		LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
		if(pnt.ExtractNumberOfReturns()==pnt.ExtractReturnNumber())
		{
			pnt.m_classify = elcGround;
			pnt.m_colorExt = color;
		}
	}
	return 0;
}

long classifyElectricPatrolFast::ElectricPatrolFast_LocalMinNonMax(ILASDataset* dataset, std::vector<LASIndex> pntIdxs, double rectRange, eLASClassification cls, Point3Ds &localMin)
{
	double xmin = _MAX_LIMIT_, ymin = _MAX_LIMIT_, xmax = _MIN_LIMIT_, ymax = _MIN_LIMIT_;

	//range
	if (pntIdxs.empty())
	{
		for (int k = 0; k<dataset->m_totalReadLasNumber; ++k)
		{
			int i = dataset->m_LASPointID[k].rectangle_idx;
			int j = dataset->m_LASPointID[k].point_idx_inRect;
			LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
			if (pnt.m_classify == cls)
			{
				xmin = min(pnt.m_vec3d.x, xmin);
				ymin = min(pnt.m_vec3d.y, ymin);
				xmax = max(pnt.m_vec3d.x, xmax);
				ymax = max(pnt.m_vec3d.y, ymax);
			}
		}
	}
	else
	{
		for (int k = 0; k<pntIdxs.size(); ++k)
		{
			int i = pntIdxs[k].rectangle_idx;
			int j = pntIdxs[k].point_idx_inRect;
			LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
			xmin = min(pnt.m_vec3d.x, xmin);
			ymin = min(pnt.m_vec3d.y, ymin);
			xmax = max(pnt.m_vec3d.x, xmax);
			ymax = max(pnt.m_vec3d.y, ymax);
		}
	}
	
	int xsize = (xmax - xmin) / rectRange + 1;
	int ysize = (ymax - ymin) / rectRange + 1;

	//localmin
	Point3Ds localMinPts(xsize*ysize);
	for (int i = 0; i < xsize*ysize; ++i)
	{
		localMinPts[i].x = _MIN_LIMIT_;
		localMinPts[i].y = _MIN_LIMIT_;
		localMinPts[i].z = _MAX_LIMIT_;
	}

	for (int k = 0; k < dataset->m_totalReadLasNumber; ++k)
	{
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;
		LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
		if (pnt.m_classify == elcCreated)
		{
			int xidx = (pnt.m_vec3d.x - xmin) / rectRange;
			int yidx = (pnt.m_vec3d.y - ymin) / rectRange;
			if (localMinPts[yidx*xsize + xidx].z > pnt.m_vec3d.z)
			{
				localMinPts[yidx*xsize + xidx].x = pnt.m_vec3d.x;
				localMinPts[yidx*xsize + xidx].y = pnt.m_vec3d.y;
				localMinPts[yidx*xsize + xidx].z = min(localMinPts[yidx*xsize + xidx].z, pnt.m_vec3d.z);
			}
		}
	}

	Point3Ds localMinMax;
	double mx = 0, my = 0;
	for (int i = 0; i < xsize*ysize; ++i)
	{
		if (localMinPts[i].x > _MIN_LIMIT_&&localMinPts[i].y > _MIN_LIMIT_)
		{
			localMinMax.push_back(localMinPts[i]);
		}
	}

	//if empty
	if(localMinMax.empty())
	{
		return 0;
	}
	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(localMinMax);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();
#ifdef _DEBUG
	FILE* fptr = fopen("ground.txt", "w+");
#endif
	for (int k = 0; k<localMinMax.size(); ++k)
	{
		Point3D pnt = localMinMax[k];
		double pt[3] = { pnt.x,pnt.y,pnt.z };
		const size_t num_results = 8;
		size_t ret_index[num_results];
		double out_dist_sqr[num_results];
		KNNResultSet<double> resultSet(num_results);
		resultSet.init(ret_index, out_dist_sqr);
		treeIndex.findNeighbors(resultSet, &pt[0], SearchParams(10));
		double avgDis = 0;
		for (int i = 0; i < 8; ++i)
		{
			int idx1 = ret_index[i];
			avgDis += localMinMax[idx1].z / num_results;
		}

		if (fabs(pnt.z - avgDis) < 0.5)
		{
#ifdef _DEBUG
			fprintf(fptr, "%lf %lf %lf\n", pnt.x, pnt.y, pnt.z);
#endif
			localMin.push_back(pnt);
		}
	}
#ifdef _DEBUG
	fclose(fptr);
#endif
	return 0;
}

long classifyElectricPatrolFast::ElectricPatrolFast_GroundDis(ILASDataset* dataset, double rectRange, double disThres, std::vector<LASIndex> &pntIdxs)
{
	Point3Ds localMin;
	std::vector<LASIndex> tmpPntIdx(pntIdxs);
	pntIdxs.clear();
	ElectricPatrolFast_LocalMinNonMax(dataset, pntIdxs, rectRange,elcCreated ,localMin);

	if(localMin.empty())
	{
		return 0;
	}

	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(localMin);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	for (int k = 0; k<tmpPntIdx.size(); ++k)
	{
		int i = tmpPntIdx[k].rectangle_idx;
		int j = tmpPntIdx[k].point_idx_inRect;
		LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];

		if (pnt.m_classify == elcCreated)
		{
			double pt[3] = { pnt.m_vec3d.x,pnt.m_vec3d.y,pnt.m_vec3d.z };

			const size_t num_results = 3;
			size_t ret_index[num_results];
			double out_dist_sqr[num_results];
			KNNResultSet<double> resultSet(num_results);
			resultSet.init(ret_index, out_dist_sqr);
			treeIndex.findNeighbors(resultSet, &pt[0], SearchParams(10));

			int idx1 = ret_index[0];
			int idx2 = ret_index[1];
			int idx3 = ret_index[2];
			double dis = DistanceComputation::Distance(pnt.m_vec3d, localMin[idx1], localMin[idx2], localMin[idx3]);
			if (dis < disThres)
			{
				pntIdxs.push_back(tmpPntIdx[k]);
			}
		}
	}

	return 0;
}

long classifyElectricPatrolFast::ElectricPatrolFast_Angle(ILASDataset* dataset, double rectRange, double angle, std::vector<LASIndex> &pntIdxs)
{
	//old method
/* 	
	double xmin = _MAX_LIMIT_, ymin = _MAX_LIMIT_, xmax = _MIN_LIMIT_, ymax = _MIN_LIMIT_;

	//range
	if (pntIdxs.empty())
	{
		for (int k = 0; k<dataset->m_totalReadLasNumber; ++k)
		{
			int i = dataset->m_LASPointID[k].rectangle_idx;
			int j = dataset->m_LASPointID[k].point_idx_inRect;
			LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
			if (pnt.m_classify == elcCreated)
			{
				xmin = min(pnt.m_vec3d.x, xmin);
				ymin = min(pnt.m_vec3d.y, ymin);
				xmax = max(pnt.m_vec3d.x, xmax);
				ymax = max(pnt.m_vec3d.y, ymax);
			}
		}
	}
	else
	{
		for (int k = 0; k<pntIdxs.size(); ++k)
		{
			int i = pntIdxs[k].rectangle_idx;
			int j = pntIdxs[k].point_idx_inRect;
			LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
			xmin = min(pnt.m_vec3d.x, xmin);
			ymin = min(pnt.m_vec3d.y, ymin);
			xmax = max(pnt.m_vec3d.x, xmax);
			ymax = max(pnt.m_vec3d.y, ymax);
		}
	}


	int xsize = (xmax - xmin) / rectRange + 1;
	int ysize = (ymax - ymin) / rectRange + 1;

	//localmin
	Point3Ds localMinPts(xsize*ysize);
	for (int i = 0; i < xsize*ysize; ++i)
	{
		localMinPts[i].x = _MIN_LIMIT_;
		localMinPts[i].y = _MIN_LIMIT_;
		localMinPts[i].z = _MAX_LIMIT_;
	}

	for (int k = 0; k < dataset->m_totalReadLasNumber; ++k)
	{
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;
		LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
		if (pnt.m_classify == elcCreated)
		{
			int xidx = (pnt.m_vec3d.x - xmin) / rectRange;
			int yidx = (pnt.m_vec3d.y - ymin) / rectRange;
			if (localMinPts[yidx*xsize + xidx].z > pnt.m_vec3d.z)
			{
				localMinPts[yidx*xsize + xidx].x = pnt.m_vec3d.x;
				localMinPts[yidx*xsize + xidx].y = pnt.m_vec3d.y;
				localMinPts[yidx*xsize + xidx].z = min(localMinPts[yidx*xsize + xidx].z, pnt.m_vec3d.z);
			}
		}
	}


	for (int k = 0; k < dataset->m_totalReadLasNumber; ++k)
	{
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;
		LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
		if (pnt.m_classify == elcCreated)
		{
			int xidx = (pnt.m_vec3d.x - xmin) / rectRange;
			int yidx = (pnt.m_vec3d.y - ymin) / rectRange;

			double dz = pnt.m_vec3d.z - localMinPts[yidx*xsize + xidx].z;
			double dx = pnt.m_vec3d.x - localMinPts[yidx*xsize + xidx].x;
			double dy = pnt.m_vec3d.y - localMinPts[yidx*xsize + xidx].y;
			double theta = atan2(fabs(dz), sqrt(dx*dx + dy*dy));

			if (theta < angle)
			{
				pntIdxs.push_back(dataset->m_LASPointID[k]);
			}
		}
	}
	return 0; */

	/**
	 old version calculate the point angle with the local min point
	 the new version using the local min points construct the tirangle
	 and then calcualte the angle of the point with the triplane
	 */
	Point3Ds localMin;
	std::vector<LASIndex> tmpPntIdx(pntIdxs);
	pntIdxs.clear();
	ElectricPatrolFast_LocalMinNonMax(dataset, pntIdxs, rectRange,elcCreated ,localMin);
	if(localMin.empty())
	{
		return 0;
	}

	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(localMin);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();
	if(tmpPntIdx.empty())
	{
		for (int k = 0; k < dataset->m_totalReadLasNumber; ++k)
		{
			int i = dataset->m_LASPointID[k].rectangle_idx;
			int j = dataset->m_LASPointID[k].point_idx_inRect;
			LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];
			if (pnt.m_classify == elcCreated)
			{
				double pt[3] = { pnt.m_vec3d.x,pnt.m_vec3d.y,pnt.m_vec3d.z };

				const size_t num_results = 3;
				size_t ret_index[num_results];
				double out_dist_sqr[num_results];
				KNNResultSet<double> resultSet(num_results);
				resultSet.init(ret_index, out_dist_sqr);
				treeIndex.findNeighbors(resultSet, &pt[0], SearchParams(10));

				int idx1 = ret_index[0];
				int idx2 = ret_index[1];
				int idx3 = ret_index[2];
				double dis = DistanceComputation::Distance(pnt.m_vec3d, localMin[idx1], localMin[idx2], localMin[idx3]);
				double angle1 = asin(dis/DistanceComputation::Distance(pnt.m_vec3d, localMin[idx1]));
				double angle2 = asin(dis/DistanceComputation::Distance(pnt.m_vec3d, localMin[idx2]));
				double angle3 = asin(dis/DistanceComputation::Distance(pnt.m_vec3d, localMin[idx3]));

				if (min(min(angle1,angle2),angle3) < angle)
				{
					pntIdxs.push_back(dataset->m_LASPointID[k]);
				}
			}
		}
	}
	else{
		for (int k = 0; k<tmpPntIdx.size(); ++k)
		{
			int i = tmpPntIdx[k].rectangle_idx;
			int j = tmpPntIdx[k].point_idx_inRect;
			LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];

			if (pnt.m_classify == elcCreated)
			{
				double pt[3] = { pnt.m_vec3d.x,pnt.m_vec3d.y,pnt.m_vec3d.z };

				const size_t num_results = 3;
				size_t ret_index[num_results];
				double out_dist_sqr[num_results];
				KNNResultSet<double> resultSet(num_results);
				resultSet.init(ret_index, out_dist_sqr);
				treeIndex.findNeighbors(resultSet, &pt[0], SearchParams(10));

				int idx1 = ret_index[0];
				int idx2 = ret_index[1];
				int idx3 = ret_index[2];
				double dis = DistanceComputation::Distance(pnt.m_vec3d, localMin[idx1], localMin[idx2], localMin[idx3]);
				double angle1 = asin(dis/DistanceComputation::Distance(pnt.m_vec3d, localMin[idx1]));
				double angle2 = asin(dis/DistanceComputation::Distance(pnt.m_vec3d, localMin[idx2]));
				double angle3 = asin(dis/DistanceComputation::Distance(pnt.m_vec3d, localMin[idx3]));

				if (min(min(angle1,angle2),angle3) < angle)
				{
					pntIdxs.push_back(tmpPntIdx[k]);
				}
			}
		}
	}
	

	return 0;
}

long classifyElectricPatrolFast::ElectricPatrolFast_Vegetation(ILASDataset* dataset, double rectRange, double disThres, LASColorExt color)
{
	Point3Ds localMin;
	std::vector<LASIndex> pntIdxs;
	ElectricPatrolFast_LocalMinNonMax(dataset, pntIdxs, rectRange, elcGround, localMin);

	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(localMin);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	for (int k = 0; k<dataset->m_totalReadLasNumber; ++k)
	{
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;
		LASPoint &pnt = dataset->m_lasRectangles[i].m_lasPoints[j];

		if (pnt.m_classify == elcCreated)
		{
			double pt[3] = { pnt.m_vec3d.x,pnt.m_vec3d.y,pnt.m_vec3d.z };

			const size_t num_results = 3;
			size_t ret_index[num_results];
			double out_dist_sqr[num_results];
			KNNResultSet<double> resultSet(num_results);
			resultSet.init(ret_index, out_dist_sqr);
			treeIndex.findNeighbors(resultSet, &pt[0], SearchParams(10));

			int idx1 = ret_index[0];
			int idx2 = ret_index[1];
			int idx3 = ret_index[2];
			double dis = DistanceComputation::Distance(pnt.m_vec3d, localMin[idx1], localMin[idx2], localMin[idx3]);
			if (dis < disThres)
			{
				pnt.m_classify = elcVegetation;
				pnt.m_colorExt = color;
			}
		}
	}

	return 0;

}

long classifyElectricPatrolFast::ElectricPatrolFast_VegetationLast(ILASDataset* dataset,LASColorExt color)
{
	//get point of vegetation and line split
	for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = dataset->m_LASPointID[i];
		LASPoint &pt =dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect]; 
		if(pt.m_classify == elcCreated)
		{
			pt.m_classify=elcVegetation;
			pt.m_colorExt=color;
		}
	}
}


//GDALTriangulation* classifyElectricPatrolFast::ElectricPatrolFast_Triangle(Point3Ds localMinPts)
//{
//	try
//	{
//		int numPnts = localMinPts.size();
//		FILE* fptr = fopen("D:\\text.txt", "w+");
//
//		double mx = 0;
//		double my = 0;
//
//		double *padfX = new double[numPnts];
//		double *padfY = new double[numPnts];
//		memset(padfX, 0, sizeof(double)*numPnts);
//		memset(padfY, 0, sizeof(double)*numPnts);
//
//		for (int i = 0; i < numPnts; ++i)
//		{
//			fprintf(fptr, "%lf %lf %lf\n", localMinPts[i].x, localMinPts[i].y, localMinPts[i].z);
//			padfX[i] = localMinPts[i].x;
//			padfY[i] = localMinPts[i].x;
//		}
//		fclose(fptr);
//		GDALTriangulation *dTri = GDALTriangulationCreateDelaunay(numPnts, padfX, padfY);
//		delete[]padfX; padfX = nullptr;
//		delete[]padfY; padfY = nullptr;
//		return dTri;
//	}
//	catch (const std::exception&)
//	{
//		return nullptr;
//	}
//
//}