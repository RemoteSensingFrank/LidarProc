#include"PointProcAlgorithm.h"
#include"GeometryFlann.h"
#include"GeometryAlgorithm.h"
#include<gdal_priv.h>
#include<iostream>
#include<Eigen/Dense>
#include<omp.h>
using namespace Eigen;
namespace LasAlgorithm {
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
			for (int m = 0; m < lasDataset->m_lasRectangles[l].m_lasPoints_numbers; ++m)
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
			if (type[i] == iType)
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
					if (out_dist_sqr[j] < knnRange&&type[ret_index[j]] == 0)
						partPnts.push_back(pointSet[ret_index[j]]);
				}
				if (partPnts.size() > 5)
				{
					//����Ƕ��жϽǶȲ���
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
						for (int i = 0; i < partPnts.size(); ++i)
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
		int numPoints = 0;
		for (int i = 0; i < pointSet.size(); ++i)
			if (type[i] == iType)
				++numPoints;

		//��������
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
		//������ֵ��Ϊ0
		mat.rowwise() -= meanvecRow;
		//Э����
		MatrixXd covMat = mat.adjoint() * mat;

		SelfAdjointEigenSolver<MatrixXd> eig(covMat);
		MatrixXd eigenVec = eig.eigenvectors();

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

	Point3Ds PointCloudSegmentWithDirection::PointCloudSegmentDirect_CalDirectVec(Point3Ds pointSet)
	{
		Point3Ds pointsetType;
		int numPoints = pointSet.size();

		//��������
		MatrixXd mat(numPoints, 3);
		for (int i = 0; i < numPoints; ++i)
		{
			mat(i, 0) = pointSet[i].x;
			mat(i, 1) = pointSet[i].y;
			mat(i, 2) = pointSet[i].z;
		}

		MatrixXd meanval = mat.colwise().mean();
		RowVectorXd meanvecRow = meanval;
		//������ֵ��Ϊ0
		mat.rowwise() -= meanvecRow;
		//Э����
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

	void PointCloudSegmentWithDirection::Test()
	{
		ILASDataset *dataset = new ILASDataset();
		LidarMemReader *reader = new LidarMemReader();
		reader->LidarReader_Open("E:\\LidarData\\CEDD_Building.las", dataset);
		reader->LidarReader_Read(true, 1, dataset);

		Point3Ds pointSet;
		for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = dataset->m_LASPointID[i];
			pointSet.push_back(dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
		}
		int *segNum = new int[dataset->m_totalReadLasNumber];
		memset(segNum, 0, sizeof(int)*dataset->m_totalReadLasNumber);
		int typeNum = PointCloudSegmentDirect_SegmentPoly(pointSet, segNum, 0.866, 15);

		#pragma omp parallel for
		for (int i = 0; i < typeNum; ++i)
		{
			PointCoudSegment_TypeExport(pointSet, segNum, i + 1, "E:\\test\\");
		}

		delete[]segNum;
		delete dataset;
		delete reader;
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	long PointCloudSegmentWithKMeans::PointCloudSegment_KMeans(ILASDataset *lasDataset,int nType,int *type,double thresStop)
	{
		Point3Ds pointSet;
		for (int i = 0; i < lasDataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = lasDataset->m_LASPointID[i];
			pointSet.push_back(lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
		}
		return PointCloudSegment_KMeans(pointSet,nType,type,thresStop);
	}
	
	long PointCloudSegmentWithKMeans::PointCloudSegment_KMeans(Point3Ds pointSet,int nType,int *type,double thresStop)
	{
		Point3Ds initCenterPoints;
		Point3Ds iterCenterPoitns;

		//init center points
		for(int i=0;i<nType;++i)
		{
			initCenterPoints.push_back(pointSet[i]);
			iterCenterPoitns.push_back(pointSet[i]);
		}

		double iteratorDis=100;
		while (iteratorDis>thresStop)
		{
			printf("center point move distance:%lf\n",iteratorDis);
			PointCloudSegment_KMeansIterator(pointSet,nType,initCenterPoints,type);
			iteratorDis=0;
			for(int i=0;i<nType;++i)
			{
				iteratorDis+=DistanceComputation::Distance(iterCenterPoitns[i],initCenterPoints[i]);
			}
			iterCenterPoitns.clear();
			for(int i=0;i<nType;++i)
			{
				iterCenterPoitns.push_back(initCenterPoints[i]);
			}
		}
		return 0;
	}
	
	long PointCloudSegmentWithKMeans::PointCloudSegment_KMeansIterator(Point3Ds pointSet,int nType,Point3Ds &clusterCenter,int *type)
	{
		//calculate the classify of each point and recalculate the center point
		for(int i=0;i<pointSet.size();++i)
		{
			double minDis = _MAX_LIMIT_;
			int    minCls = 0;
			for(int j=0;j<nType;++j)
			{
				double dis=DistanceComputation::Distance(clusterCenter[j],pointSet[i]);
				minDis=minDis<dis?minDis:dis;
				minCls=minDis<dis?minCls:j;
			}
			type[i]=minCls;
		}


		//update center point
		for(int j=0;j<nType;++j)
		{
			clusterCenter[j].x=clusterCenter[j].y=clusterCenter[j].z=0;
		}

		int *numberType=new int[nType];
		memset(numberType,0,sizeof(int)*nType);

		for(int i=0;i<pointSet.size();++i)
		{
			clusterCenter[type[i]].x+=pointSet[i].x;
			clusterCenter[type[i]].y+=pointSet[i].y;
			clusterCenter[type[i]].z+=pointSet[i].z;
			numberType[type[i]]++;
		}
		for(int j=0;j<nType;++j)
		{
			clusterCenter[j].x/=double(numberType[j]);
			clusterCenter[j].y/=double(numberType[j]);
			clusterCenter[j].z/=double(numberType[j]);;
		}
		delete[]numberType;numberType=nullptr;
		
		return 0;
	}
}