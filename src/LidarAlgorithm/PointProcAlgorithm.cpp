#include"PointProcAlgorithm.h"
#include"../LidarGeometry/GeometryFlann.h"
#include"../LidarGeometry/GeometryAlgorithm.h"
#include"gdal_priv.h"
#include"Eigen/Dense"
#include<omp.h>
using namespace Eigen;
using namespace GeometryLas;
long PointCloudSegment::PointCloudSegment_DBScan(Point3Ds pointSet, int *type, float knnRange)
{
	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pointSet);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	memset(type, 0, sizeof(int)*pointSet.size());
	std::vector<Point3D> preparePntSet;
	preparePntSet.push_back(pointSet[0]);
	int typeNumber = 1;
	type[0] = typeNumber;
	bool bFlag = true;

	for (int i = 0; i < pointSet.size(); ++i)
	{
		printf("%ld-%ld\r", pointSet.size(), i);
		//depth first
		while (!preparePntSet.empty()) {
			int vecLen = preparePntSet.size();
			double pnt[3] = { preparePntSet[vecLen - 1].x,preparePntSet[vecLen - 1].y,preparePntSet[vecLen - 1].z };
			preparePntSet.pop_back();
			std::vector<std::pair<size_t, double> > indices_dists;
			RadiusResultSet<double, size_t> resultSet((double)knnRange, indices_dists);
			//resultSet.init(ret_index, out_dist_sqr);
			treeIndex.findNeighbors(resultSet, &pnt[0], SearchParams());
			for (int j = 0; j < resultSet.m_indices_dists.size(); ++j)
			{
				int idxPnt = resultSet.m_indices_dists[j].first;
				if (type[idxPnt] == 0)
				{
					preparePntSet.push_back(pointSet[idxPnt]);
					type[idxPnt] = typeNumber;
				}
			}
		};
		if (type[i] != 0)
		{
			bFlag = false;
			continue;
		}
		else
		{
			preparePntSet.push_back(pointSet[i]);
		}
		typeNumber++;
		bFlag = true;
	}
	if (bFlag)
	{
		typeNumber = typeNumber - 1;
	}
	printf("get %d number of segment types\n", typeNumber);
	return typeNumber;
}

long PointCloudSegment::PointCoudSegment_DBScan(ILASDataset *lasDataset, int **type, float knnRange)
{
	int pointNumber = lasDataset->m_totalReadLasNumber;
	for (int i = 0; i < lasDataset->m_numRectangles; ++i)
	{
		memset(type, 0, sizeof(int)*lasDataset->m_lasRectangles[i].m_lasPoints_numbers);
	}
	std::vector<LASIndex> preparePntSet;
	preparePntSet.push_back(lasDataset->m_LASPointID[0]);
	int typeNumber = 1;
	type[lasDataset->m_LASPointID[0].rectangle_idx][lasDataset->m_LASPointID[0].point_idx_inRect] = typeNumber;
	bool bFlag = true;

	for (int l = 0; l < lasDataset->m_numRectangles; ++l)
	{
		printf("%ld-%ld\r", lasDataset->m_numRectangles, l);
		for (int m = 0; m < lasDataset->m_lasRectangles[l].m_lasPoints_numbers;++m)
		{
			//depth first
			while (!preparePntSet.empty()) {
				int vecLen = preparePntSet.size();
				LASIndex idx = preparePntSet[vecLen - 1];
				preparePntSet.pop_back();
				//get range of point
				Rect2D rect;
				const Point3D &pnt = lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d;
				rect.minx = pnt.x - knnRange; rect.maxx = pnt.x + knnRange;
				rect.miny = pnt.y - knnRange; rect.maxy = pnt.y + knnRange;
				vector<int> rectIds;
				lasDataset->LASDataset_Search(0, rect, rectIds);

				//calculate in each rectangle
				for (int j = 0; j < rectIds.size(); ++j)
				{
					double pt[3] = { pnt.x,pnt.y,pnt.z };
					if (lasDataset->m_lasRectangles[rectIds[j]].m_lasPoints_numbers > 0)
					{
						std::vector<std::pair<size_t, double> > indices_dists;
						RadiusResultSet<double, size_t> resultSet((double)knnRange, indices_dists);
						lasDataset->m_lasRectangles[rectIds[j]].m_block_tree->findNeighbors(resultSet, &pt[0], SearchParams());
						for (int k = 0; k < resultSet.m_indices_dists.size(); ++k)
						{
							int idxPnt = resultSet.m_indices_dists[j].first;
							if (type[rectIds[j]][idxPnt] == 0)
							{
								LASIndex tmpLASIdx; tmpLASIdx.rectangle_idx = rectIds[j]; tmpLASIdx.point_idx_inRect = idxPnt;
								preparePntSet.push_back(tmpLASIdx);
								type[rectIds[j]][idxPnt] = typeNumber;
							}
						}
					}
				}//end rect iterator
			};
		
			if (type[l][m] != 0)
			{
				bFlag = false;
				continue;
			}
			else
			{
				LASIndex tmpLASIdx; tmpLASIdx.rectangle_idx = l; tmpLASIdx.point_idx_inRect = m;
				preparePntSet.push_back(tmpLASIdx);
			}
			typeNumber++;
			bFlag = true;
		}
	}
	if (bFlag)
	{
		typeNumber = typeNumber - 1;
	}
	printf("get %d number of segment types\n", typeNumber);
	return typeNumber;
}

long PointCloudSegment::PointCoudSegment_TypeExport(Point3Ds pointSet, int *type, int iType, const char* dir)
{
	string paths;
	char name[256];
	sprintf(name, "%s%d.txt", dir, iType);
	FILE* ef = fopen(name, "w+");

	for (int i = 0; i < pointSet.size(); ++i)
	{
		if(type[i]== iType)
			fprintf(ef, "%lf  %lf  %lf\n", pointSet[i].x, pointSet[i].y, pointSet[i].z);
	}
	fclose(ef); ef = NULL;
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
long PointCloudSegmentWithDirection::PointCloudSegmentDirect_SegmentPoly(Point3Ds pointSet, int *type, float directionRange, float knnRange)
{
	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pointSet);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	memset(type, 0, sizeof(int)*pointSet.size());
	std::vector<Point3D> preparePntSet;
	preparePntSet.push_back(pointSet[0]);
	int typeNumber = 1;
	type[0] = typeNumber;
	bool bFlag = true;
	const int num_results = 300;

	for (int i = 0; i < pointSet.size(); ++i)
	{
		printf("%ld-%ld\r", pointSet.size(), i);
		Point3Ds classPnts;
		//depth first
		while (!preparePntSet.empty()) {
			int vecLen = preparePntSet.size();
			double pnt[3] = { preparePntSet[vecLen - 1].x,preparePntSet[vecLen - 1].y,preparePntSet[vecLen - 1].z };
			preparePntSet.pop_back();
			std::vector<std::pair<size_t, double> > indices_dists;
			size_t ret_index[num_results];
			double out_dist_sqr[num_results];
			KNNResultSet<double> resultSet(num_results);
			resultSet.init(ret_index, out_dist_sqr);
			treeIndex.findNeighbors(resultSet, &pnt[0], SearchParams());
			Point3Ds partPnts;
			for (int j = 0; j < num_results; ++j)
			{
				if (out_dist_sqr[j] < knnRange&&type[ret_index[j]]==0)
					partPnts.push_back(pointSet[ret_index[j]]);
			}
			if (partPnts.size() > 5)
			{
				//计算角度判断角度差异
				if (classPnts.size() > 0)
				{
					Point3Ds directClass = PointCloudSegmentDirect_CalDirectVec(classPnts);
					Point3Ds directParts = PointCloudSegmentDirect_CalDirectVec(partPnts);
					if (fabs(cos(GeometryRelation::VectorAngle(directClass[0], directParts[0])) > directionRange))
					{
						for (int i = 0; i < partPnts.size(); ++i)
						{
							type[ret_index[i]] = typeNumber;
						}
						preparePntSet.insert(preparePntSet.end(), partPnts.begin(), partPnts.end());
					}
				}
				else {
					classPnts.insert(classPnts.end(), partPnts.begin(), partPnts.end());
					preparePntSet.insert(preparePntSet.end(), partPnts.begin(), partPnts.end());
					for (int i = 0; i < partPnts.size();++i)
					{
						type[ret_index[i]] = typeNumber;
					}
				}
			}
		};
		if (type[i] != 0)
		{
			bFlag = false;
			continue;
		}
		else
		{
			preparePntSet.push_back(pointSet[i]);
		}
		typeNumber++;
		bFlag = true;
	}
	if (bFlag)
	{
		typeNumber = typeNumber - 1;
	}

	printf("\nget %d number of segment types\n", typeNumber);
	return typeNumber;
}

Point3Ds PointCloudSegmentWithDirection::PointCloudSegmentDirect_CalDirectVec(Point3Ds pointSet, int *type, int iType)
{
	Point3Ds pointsetType;
	int numPoints = 0 ;
	for (int i = 0; i < pointSet.size(); ++i)
		if (type[i] == iType)
			++numPoints;

	//构建矩阵
	MatrixXd mat(numPoints, 3);
	for (int i = 0; i < pointSet.size(); ++i)
	{
		if (type[i] == iType)
		{
			mat(i, 0) = pointSet[i].x;
			mat(i, 1) = pointSet[i].y;
			mat(i, 2) = pointSet[i].z;
		}
	}

	MatrixXd meanval = mat.colwise().mean();
	RowVectorXd meanvecRow = meanval;
	//样本均值化为0
	mat.rowwise() -= meanvecRow;
	//协方差
	MatrixXd covMat = mat.adjoint() * mat;

	SelfAdjointEigenSolver<MatrixXd> eig(covMat);
	MatrixXd eigenVec = eig.eigenvectors();

	Point3Ds vecs;
	for (int i = 0; i < 3; ++i)
	{
		Point3D vec;
		vec.x = eigenVec(0,i);
		vec.y = eigenVec(1,i);
		vec.z = eigenVec(2,i);
		vecs.push_back(vec);
	}
	return vecs;
}

Point3Ds PointCloudSegmentWithDirection::PointCloudSegmentDirect_CalDirectVec(Point3Ds pointSet)
{
	Point3Ds pointsetType;
	int numPoints = pointSet.size();

	//构建矩阵
	MatrixXd mat(numPoints, 3);
	for (int i = 0; i < numPoints; ++i)
	{
		mat(i, 0) = pointSet[i].x;
		mat(i, 1) = pointSet[i].y;
		mat(i, 2) = pointSet[i].z;
	}

	MatrixXd meanval = mat.colwise().mean();
	RowVectorXd meanvecRow = meanval;
	//样本均值化为0
	mat.rowwise() -= meanvecRow;
	//协方差
	MatrixXd covMat = mat.adjoint() * mat;

	SelfAdjointEigenSolver<MatrixXd> eig(covMat);
	MatrixXd eigenVec = eig.eigenvectors();
	//cout << eigenVec << endl;
	Point3Ds vecs;
	for (int i = 0; i < 3; ++i)
	{
		Point3D vec;
		vec.x = eigenVec(0, i);
		vec.y = eigenVec(1, i);
		vec.z = eigenVec(2, i);
		vecs.push_back(vec);
	}
	return vecs;
}

/////////////////////////////////////
long PointCloudFilter::PointCloudFilter_Point2DEM(ILASDataset *lasDataset,float resolution, const char* pathChr,int filterTimes/* = 0*/)
{
	double maxx = lasDataset->m_lasHeader.max_x, minx = lasDataset->m_lasHeader.min_x\
		, maxy = lasDataset->m_lasHeader.max_y, miny = lasDataset->m_lasHeader.min_y;

	int xsize = (maxx - minx) / resolution + 1;
	int ysize = (maxy - miny) / resolution + 1;

	float *demData = nullptr;
	int   *idxData = nullptr;
	try {
		demData = new float[xsize*ysize];
		idxData = new int[xsize*ysize];
		memset(idxData, 0, sizeof(int)*xsize*ysize);
		memset(demData, 0, sizeof(float)*xsize*ysize);
	}
	catch (bad_alloc e)
	{
		printf("%s\n", e.what());
		return -1;
	}

	for (int i = 0; i < xsize-1; ++i)
	{
		printf("\r%d-\%d", xsize,i+1);
		for (int j = 0; j < ysize-1; ++j)
		{
			Rect2D rect;
			rect.maxx = (i+1) * resolution + minx;
			rect.minx = i * resolution + minx;
			rect.maxy = (j+1) * resolution + miny;
			rect.miny = j * resolution + miny;

			vector<int> rectIds;
			lasDataset->LASDataset_Search(0, rect, rectIds);

			for (int k = 0; k < rectIds.size(); ++k)
			{
				for (int m = 0; m < lasDataset->m_lasRectangles[rectIds[k]].m_lasPoints_numbers; ++m)
				{
					const LASPoint &lasPnt = lasDataset->m_lasRectangles[rectIds[k]].m_lasPoints[m];
					if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y,\
												rect.minx,rect.miny, rect.maxx, rect.maxy))
					{
						demData[j*xsize + i] += lasPnt.m_vec3d.z;
						idxData[j*xsize + i] ++;
					}
				}
			}
		}
	}


	for (int i = 0; i < xsize*ysize; ++i)
	{
		if (idxData[i] == 0)
			demData[i] = -1;
		else
			demData[i] /= idxData[i];
	}

	//filter to remove noisy point
	for (int i = 0; i < filterTimes; ++i)
		PointCloudFilter_DEMFilter(demData, xsize, ysize);

	//construct output data
	GDALAllRegister();
	GDALDatasetH m_dataset = GDALCreate(GDALGetDriverByName("GTiff"), pathChr, xsize, ysize, 1, GDT_Float32, nullptr);
	double adfGeoTransform[6] = { minx,resolution,0,miny,0,resolution };
	GDALRasterIO(GDALGetRasterBand(m_dataset, 1), GF_Write, 0, 0, xsize, ysize, demData, xsize, ysize, GDT_Float32,0, 0);
	GDALSetGeoTransform(m_dataset, adfGeoTransform);
	GDALClose(m_dataset);

	delete[]demData; demData = nullptr;
	delete[]idxData; idxData = nullptr;
}

long PointCloudFilter::PointCloudFilter_Point2DEMFlann(ILASDataset *lasDataset, float resolution, const char* pathChr)
{
	//construct tree
	Point2Ds pointSet;
	vector<double> height;
	for (int i = 0; i < lasDataset->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = lasDataset->m_LASPointID[i];
		Point2D pnt;
		pnt.x = lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d.x;
		pnt.y = lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d.y;
		pointSet.push_back(pnt);
		height.push_back(lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d.z);
	}

	typedef PointCloud2DAdaptor<std::vector<Point2D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pointSet);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 2> kd_tree;
	kd_tree treeIndex(2, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	//initial data
	double maxx = lasDataset->m_lasHeader.max_x, minx = lasDataset->m_lasHeader.min_x\
		, maxy = lasDataset->m_lasHeader.max_y, miny = lasDataset->m_lasHeader.min_y;

	int xsize = (maxx - minx) / resolution + 1;
	int ysize = (maxy - miny) / resolution + 1;

	float *demData = nullptr;
	try {
		demData = new float[xsize*ysize];
		memset(demData, 0, sizeof(float)*xsize*ysize);
	}
	catch (bad_alloc e)
	{
		printf("%s\n", e.what());
		return -1;
	}

	//nearest 5 point
	const size_t num_results = 5;

	for (int i = 0; i < xsize - 1; ++i)
	{
		printf("\r%d-\%d", xsize, i + 1);
		for (int j = 0; j < ysize - 1; ++j)
		{
			double query_pt[2] = { i * resolution + minx ,j * resolution + miny };
			size_t ret_index[5];
			double out_dist_sqr[5];
			KNNResultSet<double> resultSet(num_results);
			resultSet.init(ret_index, out_dist_sqr);
			treeIndex.knnSearch(query_pt, num_results, ret_index, out_dist_sqr);
			//treeIndex.findNeighbors(resultSet, &query_pt[0], SearchParams(10));

			double disTotal = 0;
			//inv distance weighted
			for (int i = 0; i < 5; ++i)
				disTotal += out_dist_sqr[i];
			
			float heightPre = 0;
			for (int i = 0; i < 5; ++i)
				heightPre +=out_dist_sqr[i] / disTotal*height[ret_index[i]];

			demData[j*xsize + i] = heightPre;
		}
	}
	//construct output data
	GDALAllRegister();
	GDALDatasetH m_dataset = GDALCreate(GDALGetDriverByName("GTiff"), pathChr, xsize, ysize, 1, GDT_Float32, nullptr);
	double adfGeoTransform[6] = { minx,resolution,0,miny,0,resolution };
	GDALRasterIO(GDALGetRasterBand(m_dataset, 1), GF_Write, 0, 0, xsize, ysize, demData, xsize, ysize, GDT_Float32, 0, 0);
	GDALSetGeoTransform(m_dataset, adfGeoTransform);
	GDALClose(m_dataset);

	delete[]demData; demData = nullptr;
	pointSet.clear();
	height.clear();
	return 0;
}

long PointCloudFilter::PointCloudFilter_DEMFilter(float *dataDEM, int xsize, int ysize)
{
	float *cpyData = new float[xsize*ysize];
	memcpy(cpyData, dataDEM, sizeof(float)*xsize*ysize);

	int ind[] = { -1,0,1 };
	for (int i = 0; i < xsize; ++i)
	{
		for (int j = 0; j < ysize; ++j)
		{
			double data = 0;
			int dataidx = 0;

			//left top
			if (i + ind[0] > 0 && i + ind[0] < xsize&&j+ind[0]>0&& j + ind[0]<ysize)
			{
				if (cpyData[(j + ind[0])*xsize + i + ind[0]] != -1.0f)
				{
					data += cpyData[(j + ind[0])*xsize + i + ind[0]];
					dataidx++;
				}
			}
			//center top
			if (i  > 0 && i< xsize&&j + ind[0] >0 && j + ind[0] <ysize)
			{
				if (cpyData[(j + ind[0])*xsize + i ] != -1.0f)
				{
					data += cpyData[(j + ind[0])*xsize + i];
					dataidx++;
				}
			}
			//right top
			if (i + ind[1] > 0 && i + ind[1] < xsize&&j + ind[0]>0 && j + ind[0]<ysize)
			{
				if (cpyData[(j + ind[0])*xsize + i + ind[1]] != -1.0f)
				{
					data+=cpyData[(j + ind[0])*xsize + i + ind[1]];
					dataidx++;
				}
			}
			//left
			if (i + ind[0] > 0 && i + ind[0] < xsize&&j>0 && j<ysize)
			{
				if (cpyData[(j)*xsize + i + ind[0]] != -1.0f)
				{
					data += cpyData[(j)*xsize + i + ind[0]];
					dataidx++;
				}
			}
			//right
			if (i + ind[1] > 0 && i + ind[1] < xsize&&j>0 && j<ysize)
			{
				if (cpyData[(j)*xsize + i + ind[1]] != -1.0f)
				{
					data += cpyData[(j)*xsize + i + ind[1]];
					dataidx++;
				}
			}
			//left bottom 
			if (i + ind[0] > 0 && i + ind[0] < xsize&&j + ind[1]>0 && j + ind[1]<ysize)
			{
				if (cpyData[(j + ind[1])*xsize + i + ind[0]] != -1.0f)
				{
					data += cpyData[(j + ind[1])*xsize + i + ind[0]];
					dataidx++;
				}
			}
			//center bottom 
			if (i> 0 && i< xsize&&j + ind[1]>0 && j + ind[1]<ysize)
			{
				if (cpyData[(j + ind[1])*xsize + i ] != -1.0f)
				{
					data += cpyData[(j + ind[1])*xsize + i ];
					dataidx++;
				}
			}
			//right bottom
			if (i + ind[1] > 0 && i + ind[1] < xsize&&j + ind[1]>0 && j + ind[1]<ysize)
			{
				if (cpyData[(j + ind[1])*xsize + i + ind[1]] != -1.0f)
				{
					data += cpyData[(j + ind[1])*xsize + i + ind[1]];
					dataidx++;
				}
			}
			if (dataidx == 0)
				dataDEM[j*xsize + i] = -1;
			if (dataDEM[j*xsize + i] != -1)
				continue;
			if (dataidx > 3)
				dataDEM[j*xsize + i] = data / dataidx;
		}
	}
	delete[]cpyData; cpyData = nullptr;
	return 0;
}