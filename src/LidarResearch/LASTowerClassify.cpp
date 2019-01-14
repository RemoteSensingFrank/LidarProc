#include"LASTowerClassify.h"
#include <Eigen/Dense>
#include"../LidarGeometry/Geometry.h"
#include"../LidarGeometry/GeometryAlgorithm.h"


using namespace Eigen;

#ifdef _ARTICLE__USE_

#include"../Utility/FileHelper.h"
#include"../LidarBase/LASReader.h"
#include"../LidarBase/tsmToUTM.h"
#include"../LidarAlgorithm/PointProcAlgorithm.h"

long LASClassifyTower::LASSetTowerLocation(double x, double y)
{
	Point2D perTower;
	perTower.x = x;
	perTower.y = y;
	m_towerLocate.push_back(perTower);
	return 0;
}

long LASClassifyTower::LASGetTowerLocation(const char* pathLocate)
{
	FILE* plf = fopen(pathLocate, "r+");
	if (plf == nullptr)
		return -1;
	char buffer[2048];
	while (!feof(plf))
	{
		Point2D perTower;
		fgets(buffer, 2048, plf);
		sscanf(buffer, "%lf%lf", &perTower.x, &perTower.y);
		int zone = 50;
		double mx, my;
		tsmLatLongToUTM(perTower.y, perTower.x, &zone, &mx, &my);
		perTower.x = mx;
		perTower.y = my;
		m_towerLocate.push_back(perTower);
	};
	fclose(plf);

	return 0;
}

long LASClassifyTower::LASGetTowerLocation()
{
	//m_towerLocate.push_back(Point2D(182175.6228, 2481545.586));
	//m_towerLocate.push_back(Point2D(182232.7469, 2481205.475));
	//m_towerLocate.push_back(Point2D(182706.1623, 2481000.313));
	m_towerLocate.push_back(Point2D(183021.9111, 2480776.1400));
	return 0;
}

long LASClassifyTower::LASTowerRect(ILASDataset* dataset, double range, vector<Rect2D> &rectTowers)
{
	if (m_towerLocate.empty())
		return 0;
	long numPnt = 0;
	for (int i = 0; i < m_towerLocate.size(); ++i)
	{
		Rect2D towerRect;
		towerRect.minx = m_towerLocate[i].x - range;
		towerRect.maxx = m_towerLocate[i].x + range;
		towerRect.miny = m_towerLocate[i].y - range;
		towerRect.maxy = m_towerLocate[i].y + range;

		vector<int> searchRect;
		int idPoint = 0;
		dataset->LASDataset_Search(idPoint, towerRect, searchRect);

		for (int j = 0; j < searchRect.size(); ++j)
		{
			int rectInd = searchRect[j];
			if (dataset->m_lasRectangles[rectInd].m_lasPoints_numbers > 0)
			{
				for (int k = 0; k < dataset->m_lasRectangles[rectInd].m_lasPoints_numbers; ++k)
				{
					dataset->m_lasRectangles[rectInd].m_lasPoints[k].m_classify = elcTowerRange;
					numPnt++;
				}
			}
		}
		rectTowers.push_back(towerRect);
	}
	return numPnt;
}

long LASClassifyTower::LASTowerRough(ILASDataset* dataset, vector<Rect2D> rectTowers, float heightThreshold /*= 10*/)
{
	for (int i = 0; i < rectTowers.size(); ++i)
	{
		vector<int> searchRect;
		int idPoint = 0;
		dataset->LASDataset_Search(idPoint, rectTowers[i], searchRect);
		double avgHeight = 0;
		int num = 0;
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int idxRect = searchRect[j];

			for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
			{
				LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
				if (lasPnt.m_classify == elcTowerRange)
				{
					++num;
					avgHeight = avgHeight + lasPnt.m_vec3d.z;
				}
			}
		}
		if (num > 0)
			avgHeight /= num;
		else
			avgHeight = 0;

		for (int j = 0; j < searchRect.size(); ++j)
		{
			int idxRect = searchRect[j];
			for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
			{
				LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
				if (lasPnt.m_vec3d.z - avgHeight > heightThreshold&&lasPnt.m_classify == elcTowerRange)
				{
					lasPnt.m_classify = elcTowerUp;
				}
			}
		}
	}
	return 0;
}

long LASClassifyTower::LASTowerPlaneFit(ILASDataset* dataset, vector<Rect2D> &rectTowers, double *abc)
{
	int numPoints = 0;
	//去中心化，统计中心坐标
	double centerx = 0, centery = 0, centerz = 0;
	//for (int i = 0; i < rectTowers.size(); ++i)
	//{
	// 先不考虑同时提取多个杆塔的情况
	//}
	vector<int> searchRect;
	dataset->LASDataset_Search(0, rectTowers[0], searchRect);
	for (int j = 0; j < searchRect.size(); ++j)
	{
		int idxRect = searchRect[j];
		for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
		{
			LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
			centerx += lasPnt.m_vec3d.x;
			centery += lasPnt.m_vec3d.y;
			centerz += lasPnt.m_vec3d.z;
			++numPoints;
		}
	}

	centerx /= numPoints;
	centery /= numPoints;
	centerz /= numPoints;

	//总点数
	MatrixXd Z(numPoints, 1), A(numPoints, 3), X(3, 1);
	numPoints = 0;
	//for (int i = 0; i < rectTowers.size(); ++i)
	//{
	//	先不考虑同时提取多个杆塔的情况
	//}

	searchRect.clear();
	dataset->LASDataset_Search(0, rectTowers[0], searchRect);
	for (int j = 0; j < searchRect.size(); ++j)
	{
		int idxRect = searchRect[j];
		for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
		{
			LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
			Z(numPoints, 0) = lasPnt.m_vec3d.z - centerz;
			A(numPoints, 0) = lasPnt.m_vec3d.x - centerx;
			A(numPoints, 1) = lasPnt.m_vec3d.y - centery;
			A(numPoints, 2) = 1;
			++numPoints;
		}
	}

	X = (A.transpose()*A).inverse()*(A.transpose()*Z);

	abc[0] = X(0, 0);
	abc[1] = X(1, 0);
	abc[2] = X(2, 0);

	return 0;
}

long LASClassifyTower::LASTowerRoughPlane(ILASDataset* dataset, vector<Rect2D> rectTowers, double* param, float heightThreshold /*= 10*/)
{
	//中心点
	int numPoints = 0;
	//去中心化，统计中心坐标
	double centerx = 0, centery = 0, centerz = 0;
	vector<int> searchRect;
	dataset->LASDataset_Search(0, rectTowers[0], searchRect);
	for (int j = 0; j < searchRect.size(); ++j)
	{
		int idxRect = searchRect[j];
		for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
		{
			LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
			centerx += lasPnt.m_vec3d.x;
			centery += lasPnt.m_vec3d.y;
			centerz += lasPnt.m_vec3d.z;
		}
		numPoints += dataset->m_lasRectangles[idxRect].m_lasPoints_numbers;
	}
	centerx /= numPoints;
	centery /= numPoints;
	centerz /= numPoints;

	searchRect.clear();
	int idPoint = 0;
	dataset->LASDataset_Search(idPoint, rectTowers[0], searchRect);
	for (int j = 0; j < searchRect.size(); ++j)
	{
		int idxRect = searchRect[j];
		for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
		{
			LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
			double z = param[0] * (lasPnt.m_vec3d.x - centerx) + param[1] * (lasPnt.m_vec3d.y - centery) + param[2];

			if (lasPnt.m_vec3d.z - centerz - z>heightThreshold)
			{
				lasPnt.m_classify = elcTowerUp;
			}
		}
	}
	return 0;
}

long LASClassifyTower::LASTowerRefine(ILASDataset* dataset, double range, float cubeDis/* = 0.5*/, float cubePoints/* = 20*/)
{
	for (int i = 0; i < m_towerLocate.size(); ++i)
	{
		Rect2D towerRect;
		towerRect.minx = m_towerLocate[i].x - range;
		towerRect.maxx = m_towerLocate[i].x + range;
		towerRect.miny = m_towerLocate[i].y - range;
		towerRect.maxy = m_towerLocate[i].y + range;

		vector<int> searchRect;
		int idPoint = 0;
		dataset->LASDataset_Search(idPoint, towerRect, searchRect);

		//get tower range point number
		int numPnt = 0;
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int rectInd = searchRect[j];
			numPnt += dataset->m_lasRectangles[rectInd].m_lasPoints_numbers;
		}

		//allocate memory
		//Point3D *pntPart = new Point3D[numPnt];
		int *idxpnt = new int[numPnt];
		memset(idxpnt, 0, sizeof(int)*numPnt);

		//calculate dense on each point
		idPoint = 0;
		for (int j = 0; j < searchRect.size(); ++j)
		{
#ifdef _DEBUG
			printf("process rect:%d-%d\n", searchRect.size(), j + 1);
#endif // DEBUG

			int idxRect = searchRect[j];
			for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
			{
				if (dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_classify == elcTowerUp)
				{
					++idPoint;
					continue;
				}

				double xmin = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.x - cubeDis;
				double xmax = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.x + cubeDis;
				double ymin = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.y - cubeDis;
				double ymax = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.y + cubeDis;
				double zmin = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.z - cubeDis * 20;
				double zmax = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.z + cubeDis * 20;

				for (int l = 0; l < searchRect.size(); ++l)
				{
					int idxRectIn = searchRect[l];
					if (GeometryRelation::IsRectIntersectRect(
						dataset->m_lasRectangles[idxRectIn].m_Rectangle,
						Rect2D(xmin, ymin, xmax, ymax)))
					{
						for (int m = 0; m < dataset->m_lasRectangles[idxRectIn].m_lasPoints_numbers; ++m)
						{
							Point3D &pntPart = dataset->m_lasRectangles[idxRectIn].m_lasPoints[m].m_vec3d;
							if (pntPart.x > xmin&&pntPart.x<xmax&&
								pntPart.y>ymin&&pntPart.y<ymax&&
								pntPart.z>zmin&&pntPart.z < zmax)
							{
								if (dataset->m_lasRectangles[idxRectIn].m_lasPoints[m].m_classify != elcTowerUp)
									idxpnt[idPoint]++;
							}
						}
					}
				}
				++idPoint;
			}
		}
		//classify
		idPoint = 0;
		int testNum = 0;
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int idxRectIn = searchRect[j];
			for (int k = 0; k < dataset->m_lasRectangles[idxRectIn].m_lasPoints_numbers; ++k)
			{
				if (dataset->m_lasRectangles[idxRectIn].m_lasPoints[k].m_classify == elcTowerUp)
				{
					continue;
					++idPoint;
				}
				if (dataset->m_lasRectangles[idxRectIn].m_lasPoints[k].m_classify != elcTowerUp && idxpnt[idPoint] < cubePoints)
				{
					dataset->m_lasRectangles[idxRectIn].m_lasPoints[k].m_classify = elcTowerUp;
					testNum++;
				}
				++idPoint;
			}
		}
		delete[]idxpnt;  idxpnt = NULL;
	}
	return 0;
}

long LASClassifyTower::LASTowerRefineClassified(ILASDataset* dataset, double range, float cubeDis/* = 0.5*/, float cubePoints/* = 20*/)
{
	for (int i = 0; i < m_towerLocate.size(); ++i)
	{
		Rect2D towerRect;
		towerRect.minx = m_towerLocate[i].x - range;
		towerRect.maxx = m_towerLocate[i].x + range;
		towerRect.miny = m_towerLocate[i].y - range;
		towerRect.maxy = m_towerLocate[i].y + range;

		vector<int> searchRect;
		int idPoint = 0;
		dataset->LASDataset_Search(idPoint, towerRect, searchRect);

		//get tower range point number
		int numPnt = 0;
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int rectInd = searchRect[j];
			numPnt += dataset->m_lasRectangles[rectInd].m_lasPoints_numbers;
		}

		//allocate memory
		//Point3D *pntPart = new Point3D[numPnt];
		int *idxpnt = new int[numPnt];
		memset(idxpnt, 0, sizeof(int)*numPnt);

		//calculate dense on each point
		idPoint = 0;
		for (int j = 0; j < searchRect.size(); ++j)
		{
#ifdef _DEBUG
			printf("process rect:%d-%d\n", searchRect.size(), j + 1);
#endif // DEBUG

			int idxRect = searchRect[j];
			for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
			{
				if (dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_classify == elcTowerUp)
				{
					++idPoint;
					continue;
				}

				double xmin = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.x - cubeDis;
				double xmax = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.x + cubeDis;
				double ymin = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.y - cubeDis;
				double ymax = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.y + cubeDis;
				double zmin = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.z - cubeDis;
				double zmax = dataset->m_lasRectangles[idxRect].m_lasPoints[k].m_vec3d.z + cubeDis;

				for (int l = 0; l < searchRect.size(); ++l)
				{
					int idxRectIn = searchRect[l];
					if (GeometryRelation::IsRectIntersectRect(
						dataset->m_lasRectangles[idxRectIn].m_Rectangle,
						Rect2D(xmin, ymin, xmax, ymax)))
					{
						for (int m = 0; m < dataset->m_lasRectangles[idxRectIn].m_lasPoints_numbers; ++m)
						{
							Point3D &pntPart = dataset->m_lasRectangles[idxRectIn].m_lasPoints[m].m_vec3d;
							if (pntPart.x > xmin&&pntPart.x<xmax&&
								pntPart.y>ymin&&pntPart.y<ymax&&
								pntPart.z>zmin&&pntPart.z < zmax)
							{
								if (dataset->m_lasRectangles[idxRectIn].m_lasPoints[m].m_classify == elcTowerUp)
									idxpnt[idPoint]++;
							}
						}
					}
				}
				++idPoint;
			}
		}
		//classify
		idPoint = 0;
		int testNum = 0;
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int idxRectIn = searchRect[j];
			for (int k = 0; k < dataset->m_lasRectangles[idxRectIn].m_lasPoints_numbers; ++k)
			{
				if (dataset->m_lasRectangles[idxRectIn].m_lasPoints[k].m_classify != elcTowerUp)
				{
					continue;
				}
				if (dataset->m_lasRectangles[idxRectIn].m_lasPoints[k].m_classify == elcTowerUp && idxpnt[idPoint] < cubePoints)
				{
					dataset->m_lasRectangles[idxRectIn].m_lasPoints[k].m_classify = elcTowerRange;
					testNum++;
				}
				++idPoint;
			}
		}
		delete[]idxpnt;  idxpnt = NULL;
	}
	return 0;
}

vector<string> LASClassifyTower::LASTowerRectExport(ILASDataset* dataset, const char* pathDir,vector<Rect2D> rectTowers, float cubeDis, float stepDis)
{
	//切块，并导出块中的点云
	double xmin = _MAX_LIMIT_, xmax = _MIN_LIMIT_, ymin = _MAX_LIMIT_, ymax = _MIN_LIMIT_, zmin = _MAX_LIMIT_, zmax = _MIN_LIMIT_;
	for (int i = 0; i < rectTowers.size(); ++i)
	{
		vector<int> searchRect;
		dataset->LASDataset_Search(0, rectTowers[i], searchRect);
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int idxRect = searchRect[j];
			for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
			{
				LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
				if (lasPnt.m_classify != elcTowerUp)
				{
					xmin = min(xmin, lasPnt.m_vec3d.x);
					xmax = max(xmax, lasPnt.m_vec3d.x);
					ymin = min(ymin, lasPnt.m_vec3d.y);
					ymax = max(ymax, lasPnt.m_vec3d.y);
					zmin = min(zmin, lasPnt.m_vec3d.z);
					zmax = max(zmax, lasPnt.m_vec3d.z);
				}
			}
		}
	}

	int xsize = (xmax - xmin) / cubeDis + 1;
	int ysize = (ymax - ymin) / cubeDis + 1;
	int zsize = (zmax - zmin) * 2.5 / cubeDis + 1;
	vector<vector<Point3D>> pnt(xsize*ysize*zsize);

	//export 
	for (int i = 0; i < rectTowers.size(); ++i)
	{
		vector<int> searchRect;
		dataset->LASDataset_Search(0, rectTowers[i], searchRect);
		for (int j = 0; j < searchRect.size(); ++j)
		{
			int idxRect = searchRect[j];
			for (int k = 0; k < dataset->m_lasRectangles[idxRect].m_lasPoints_numbers; ++k)
			{
				LASPoint &lasPnt = dataset->m_lasRectangles[idxRect].m_lasPoints[k];
				int idx = (lasPnt.m_vec3d.x - xmin) / cubeDis;
				int idy = (lasPnt.m_vec3d.y - ymin) / cubeDis;
				int idz = (lasPnt.m_vec3d.z - zmin) * 2.5 / cubeDis;
				if (idx > xsize|| idx<0 || idy>ysize|| idy<0 || idz>zsize|| idz < 0)
					continue;
				pnt[idz*xsize*ysize + idy * xsize + idx].push_back(lasPnt.m_vec3d);
			}
		}
	}
	vector<string> paths;
	for (int i = 0; i < pnt.size(); ++i)
	{
		char name[256];
		sprintf(name, "%s%d.txt", pathDir,i);
		if (pnt[i].size() > 10)
		{
			FILE* ef = fopen(name, "w+");
			for (int j = 0; j < pnt[i].size(); ++j)
			{
				fprintf(ef, "%lf  %lf  %lf\n", pnt[i][j].x, pnt[i][j].y, pnt[i][j].z);
			}
			fclose(ef); ef = NULL;
			paths.push_back(string(name));
		}
	}
	return paths;
}

long LASClassifyTower::LASTowerEigenClassify(const char* pathBlockDir, double threshold, const char* pathMerge)
{
	vector<string> path;
	FileHelper::listFiles(pathBlockDir, path, ".txt");

	FILE* ef = fopen(pathMerge, "w+");
	if (ef == nullptr)
		return -1;
	for (int i = 0; i < path.size(); ++i)
	{
		vector<double> eigen;
		long lError=calculateEigen(path[i], eigen);
		if (lError >= 0)
		{
			if (eigen[2] / eigen[1] > threshold)
			{
				FILE* rf = fopen(path[i].c_str(),"r+");
				if (rf == nullptr)
				{
					continue;
				}
				char line[1024];
				while (!feof(rf))
				{
					fgets(line, 1024, rf);
					double x, y, z;
					sscanf(line, "%lf%lf%lf", &x, &y, &z);
					fprintf(ef, "%lf  %lf  %lf\n", x,y, z);
				}
				fclose(rf); rf = nullptr;
			}
		}
	}
	fclose(ef); ef = nullptr;

	return 0;
}

long LASClassifyTower::LASTowerSegDBScan(ILASDataset *datasetMerge,double segDis, const char* pathSeedTower)
{
	PointCloudSegment seg;
	int pointsNumber = datasetMerge->m_totalReadLasNumber;
	int* classify = new int[pointsNumber];
	Point3Ds points;
	try
	{
		points.reserve(pointsNumber*0.1);
		for (int i = 0; i < pointsNumber; ++i)
		{
			LASIndex idxLas = datasetMerge->m_LASPointID[i];
			const LASPoint &lasPnt = datasetMerge->m_lasRectangles[idxLas.rectangle_idx].m_lasPoints[idxLas.point_idx_inRect];
			points.push_back(lasPnt.m_vec3d);
		}
	}
	catch (bad_alloc e)
	{
		printf(e.what());
	}
	int numtype=seg.PointCloudSegment_DBScan(points, classify, segDis);
	int *typesnumber = new int[numtype];
	memset(typesnumber, 0, sizeof(int)*numtype);
	for (int j = 0; j < pointsNumber; ++j)
	{
		typesnumber[classify[j]-1]++;
	}
	int maxidx = 0;
	int maxdata = 0;
	for (int i = 0; i < numtype-1; ++i)
	{
		if (maxdata < typesnumber[i])
		{
			maxidx = i+1;
			maxdata = typesnumber[i];
		}
	}

	Point3Ds pointVec;
	FILE* ef = fopen(pathSeedTower, "w+");
	for (int j = 0; j < pointsNumber; ++j)
	{
		if (classify[j] == maxidx)
		{
			LASIndex idxLas = datasetMerge->m_LASPointID[j];
			pointVec.push_back(datasetMerge->m_lasRectangles[idxLas.rectangle_idx].m_lasPoints[idxLas.point_idx_inRect].m_vec3d);
			const LASPoint &lasPnt = datasetMerge->m_lasRectangles[idxLas.rectangle_idx].m_lasPoints[idxLas.point_idx_inRect];
			fprintf(ef, "%lf  %lf  %lf\n", lasPnt.m_vec3d.x, lasPnt.m_vec3d.y, lasPnt.m_vec3d.z);
		}
	}
	fclose(ef);

	delete[]classify; classify = nullptr;
	delete[]typesnumber; typesnumber = nullptr;
	points.clear();
	return 0;
}

long LASClassifyTower::LASTowerSeed(ILASDataset* dataset, double hDis, double vDis, Point3Ds seedPoints, const char* pathOut)
{
	Point3Ds pointSet;
	for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = dataset->m_LASPointID[i];
		Point2D pnt;
		pointSet.push_back(dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
	}

	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pointSet);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	int* type = new int[pointSet.size()];
	memset(type, 0, sizeof(int)*pointSet.size());
	std::vector<Point3D> preparePntSet;
	for (int i = 0; i < seedPoints.size(); ++i)
		preparePntSet.push_back(seedPoints[i]);
	int typeNumber = 1;

	for (int i = 0; i < preparePntSet.size(); ++i)
	{
		printf("%ld-%ld\r", preparePntSet.size(), i);
		//depth first
		double pnt[3] = { preparePntSet[i].x,preparePntSet[i].y,preparePntSet[i].z };
		std::vector<std::pair<size_t, double> > indices_dists;
		RadiusResultSet<double, size_t> resultSet((double)20, indices_dists);
		//resultSet.init(ret_index, out_dist_sqr);
		treeIndex.findNeighbors(resultSet, &pnt[0], SearchParams());
		for (int j = 0; j < resultSet.m_indices_dists.size(); ++j)
		{
			int idxPnt = resultSet.m_indices_dists[j].first;
			if (type[idxPnt] == 0 && fabs(pointSet[idxPnt].x - pnt[0])<hDis&& fabs(pointSet[idxPnt].y - pnt[1])<hDis&&
				fabs(pointSet[idxPnt].z - pnt[2])<vDis)
			{
				type[idxPnt] = typeNumber;
			}
		}
	}
	//
	//export
	FILE *fs = fopen(pathOut, "w+");
	for (int i = 0; i < pointSet.size(); ++i)
	{
		if (type[i] == typeNumber)
		{
			fprintf(fs, "%lf  %lf  %lf\n", pointSet[i].x, pointSet[i].y, pointSet[i].z);
		}
	}
	pointSet.clear();
	delete[]type; type = nullptr;
	fclose(fs);
	return 0;
}

long LASClassifyTower::LASTowerSeed(ILASDataset* dataset,double hDis,double vDis, const char* seedPath, const char* pathOut)
{
	//import points
	FILE* strFile = fopen(seedPath, "r+");
	Point3Ds seedPoints;
	char line[2048];
	do {
		Point3D pnt;
		fgets(line, 2048, strFile);
		sscanf(line, "%lf%lf%lf", &pnt.x, &pnt.y, &pnt.z);
		seedPoints.push_back(pnt);
	} while (!feof(strFile));
	fclose(strFile);
	return LASTowerSeed(dataset,hDis,vDis ,seedPoints, pathOut);
}

long LASClassifyTower::LASTowerTotalMerge(ILASDataset *datasetUp, ILASDataset *datasetDown, const char* pathOut)
{
	FILE* ef = fopen(pathOut, "w+");
	if (ef == nullptr)
		return -1;
	for (int i = 0; i < datasetUp->m_totalReadLasNumber; ++i)
	{
		const LASIndex &indx = datasetUp->m_LASPointID[i];
		const LASPoint &pnt = datasetUp->m_lasRectangles[indx.rectangle_idx].m_lasPoints[indx.point_idx_inRect];
		fprintf(ef, "%lf  %lf  %lf\n", pnt.m_vec3d.x, pnt.m_vec3d.y, pnt.m_vec3d.z);
	}
	for (int i = 0; i < datasetDown->m_totalReadLasNumber; ++i)
	{
		const LASIndex &indx = datasetDown->m_LASPointID[i];
		const LASPoint &pnt = datasetDown->m_lasRectangles[indx.rectangle_idx].m_lasPoints[indx.point_idx_inRect];
		fprintf(ef, "%lf  %lf  %lf\n", pnt.m_vec3d.x, pnt.m_vec3d.y, pnt.m_vec3d.z);
	}
	fclose(ef);
}

long LASClassifyTower::calculateEigen(string strFile, vector<double> &eigen)
{
	FILE* file = fopen(strFile.c_str(), "r+");
	if (file == nullptr)
	{
		//printf("%s\n", strFile.c_str());
		return -1;
	}
	char line[1024];
	vector<double> matData;
	while (!feof(file))
	{
		fgets(line, 1024, file);
		double x, y, z;
		sscanf(line, "%lf%lf%lf", &x, &y, &z);
		matData.push_back(x);
		matData.push_back(y);
		matData.push_back(z);
	};

	int lines = matData.size() / 3;
	MatrixXd mat(lines, 3);
	for (int i = 0; i < lines; ++i)
	{
		mat(i, 0) = matData[3 * i + 0];
		mat(i, 1) = matData[3 * i + 1];
		mat(i, 2) = matData[3 * i + 2];
	}

	MatrixXd meanval = mat.colwise().mean();
	RowVectorXd meanvecRow = meanval;
	//样本均值化为0
	mat.rowwise() -= meanvecRow;

	//协方差
	MatrixXd covMat = mat.adjoint() * mat;

	//covMat = covMat.array() / mat.rows() - 1;

	SelfAdjointEigenSolver<MatrixXd> eig(covMat);
	MatrixXd values =  eig.eigenvalues();

	eigen.push_back(values(0,0));
	eigen.push_back(values(1,0));
	eigen.push_back(values(2,0));
	return 0;
}

long LASClassifyTower::LASTowerRoughTest()
{
	char* pathDown = "E:\\LidarData\\article_2018\\3\\towerup.txt";
	char* pathUp = "E:\\LidarData\\article_2018\\3\\downaccurate.txt";
	ILASDataset *lasDatasetDown = new ILASDataset();
	LidarReaderTxt *readerDown = new LidarReaderTxt();
	readerDown->LidarReader_Open(pathDown, lasDatasetDown);
	readerDown->LidarReader_Read(true, LASXYZ, lasDatasetDown);
	ILASDataset *lasDatasetUp = new ILASDataset();
	LidarReaderTxt *readerUp = new LidarReaderTxt();
	readerUp->LidarReader_Open(pathUp, lasDatasetUp);
	readerUp->LidarReader_Read(true, LASXYZ, lasDatasetUp);
	char* pathMerge = "E:\\LidarData\\article_2018\\3\\mergeaccurate.txt";
	LASTowerTotalMerge(lasDatasetUp, lasDatasetDown, pathMerge);


	//将提取的塔基点云作为种子点进行扩充，提取所有塔基点云
	
	/*
	char* pathLasRaw = "E:\\LidarData\\mergecolor.las";
	ILASDataset *lasDatasetRaw = new ILASDataset();
	LidarMemReader *readerRaw = new LidarMemReader();
	readerRaw->LidarReader_Open(pathLasRaw, lasDatasetRaw);
	readerRaw->LidarReader_Read(true, 1, lasDatasetRaw);

	//根据杆塔位置提取杆塔点云
	vector<Rect2D> rectTowers;
	LASGetTowerLocation();
	LASTowerRect(lasDatasetRaw, 10, rectTowers);

	//中间过程数据
	//readerRaw->LidarReader_Export("E:\\show.txt", lasDatasetRaw, elcTowerRange);

	double params[3];
	//z=ax+by+c
	//LASTowerPlaneFit(lasDatasetRaw, rectTowers, params);
	//LASTowerRoughPlane(lasDatasetRaw, rectTowers, params,10);
	LASTowerRough(lasDatasetRaw, rectTowers,6);
	//LASTowerRefineClassified(lasDatasetRaw, 20, 0.5, 25);
	char* pathUp = "E:\\LidarData\\article_2018\\towerup.txt";
	char* down = "E:\\LidarData\\article_2018\\down.txt";
	char* pathBlockDir = "E:\\LidarData\\article_2018\\block\\";
	char* pathEigen = "E:\\LidarData\\article_2018\\towereigen.txt";

	readerRaw->LidarReader_Export(pathUp, lasDatasetRaw, elcTowerUp);
	readerRaw->LidarReader_Export(down, lasDatasetRaw, elcTowerRange);
	printf("export tower up part~\n");
	char* pathEigenSeg = "E:\\LidarData\\article_2018\\eigenseg.txt";

	ILASDataset *lasDatasetD = new ILASDataset();
	LidarReaderTxt *readerD = new LidarReaderTxt();
	readerD->LidarReader_Open(down, lasDatasetD);
	readerD->LidarReader_Read(true, LASXYZ, lasDatasetD);
	//readerD->LidarReader_Write("E:\\LidarData\\article_2018\\test.las", lasDatasetD);
	LASTowerRectExport(lasDatasetD, pathBlockDir,rectTowers, 3,0);
	LASTowerEigenClassify(pathBlockDir, 4.0, pathEigen);
	printf("export PCA threshold result~\n");

	ILASDataset *lasDatasetEigen = new ILASDataset();
	LidarReaderTxt *readerEigen = new LidarReaderTxt();
	readerEigen->LidarReader_Open(pathEigen, lasDatasetEigen);
	readerEigen->LidarReader_Read(true, LASXYZ, lasDatasetEigen);
	LASTowerSegDBScan(lasDatasetEigen,1, pathEigenSeg);
	printf("export segment result~\n");
	ILASDataset *lasDatasetSeed = new ILASDataset();
	LidarReaderTxt *readerSeed = new LidarReaderTxt();
	readerSeed->LidarReader_Open(pathEigenSeg, lasDatasetSeed);
	readerSeed->LidarReader_Read(true, LASXYZ, lasDatasetSeed);

	char* pathDowntmp = "E:\\LidarData\\article_2018\\towerdowntmp.txt";
	LASTowerSeed(lasDatasetRaw,2,3, pathEigenSeg, pathDowntmp);
	char* pathDown = "E:\\LidarData\\article_2018\\towerdown.txt";
	LASTowerSeed(lasDatasetRaw,0.5,2, pathDowntmp, pathDown);
	//LASTowerSeed(lasDatasetRaw, pathEigenSeg, pathDown);

	printf("export tower down seed point~\n");
	ILASDataset *lasDatasetDown = new ILASDataset();
	LidarReaderTxt *readerDown = new LidarReaderTxt();
	readerDown->LidarReader_Open(pathDown, lasDatasetDown);
	readerDown->LidarReader_Read(true, LASXYZ, lasDatasetDown);
	ILASDataset *lasDatasetUp = new ILASDataset();
	LidarReaderTxt *readerUp = new LidarReaderTxt();
	readerUp->LidarReader_Open(pathUp, lasDatasetUp);
	readerUp->LidarReader_Read(true, LASXYZ, lasDatasetUp);
	char* pathMerge = "E:\\LidarData\\article_2018\\merge.txt";
	LASTowerTotalMerge(lasDatasetUp, lasDatasetDown, pathMerge);
	printf("export merge tower~\n");

	ILASDataset *lasDatasetTower = new ILASDataset();
	LidarReaderTxt *readerTower = new LidarReaderTxt();
	readerTower->LidarReader_Open(pathMerge, lasDatasetTower);
	readerTower->LidarReader_Read(true, LASXYZ, lasDatasetTower);
	char* pathTower = "E:\\LidarData\\article_2018\\tower.txt";
	LASTowerSegDBScan(lasDatasetTower, 0.2,pathTower);
	printf("export tower refine~\n");
	//清理内存
	delete lasDatasetRaw; delete readerRaw;
	delete lasDatasetEigen; delete readerEigen;
	delete lasDatasetSeed; delete readerSeed;
	delete lasDatasetDown; delete readerDown;
	delete lasDatasetUp; delete readerUp;
	delete lasDatasetTower; delete readerTower;
	*/
	return 0;
}


#endif
