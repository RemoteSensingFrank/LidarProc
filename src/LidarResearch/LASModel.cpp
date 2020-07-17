/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2020-06-16 14:28:11
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-07-15 17:07:23
 */ 
//
// Created by wuwei on 18-1-21.
//

#include "../LidarResearch/LASModel.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarGeometry/Geometry.h"
#include "../LidarGeometry/GeometryAlgorithm.h"

using namespace GeometryLas;

typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;

void LASModel::LASModel_PointsDense(ILASDataset* dataset, float cubeRange)
{
	//首先将空间进行分割
	float rangex = dataset->m_lasHeader.max_x - dataset->m_lasHeader.min_x;
	float rangey = dataset->m_lasHeader.max_y - dataset->m_lasHeader.min_y;
	float rangez = dataset->m_lasHeader.max_z - dataset->m_lasHeader.min_z;

	int xsize = rangex / cubeRange + 1;
	int ysize = rangey / cubeRange + 1;
	int zsize = rangez / cubeRange + 1;

	double stepx = rangex / xsize;
	double stepy = rangey / xsize;
	double stepz = rangez / xsize;

	//计算立方体点密度
	for (int i = 0; i<xsize; ++i)
	{
		for (int j = 0; j<ysize; ++j)
		{
			for (int k = 0; k<zsize; ++k)
			{
				DenseSt densePre;
				densePre.point.x = i; densePre.point.y = j; densePre.point.z = k;

				//范围
				Rect2D rect;
				rect.minx = i*stepx + dataset->m_lasHeader.min_x;
				rect.maxx = (i + 1)*stepx + dataset->m_lasHeader.min_x;

				rect.miny = j*stepy + dataset->m_lasHeader.min_y;
				rect.maxy = (j + 1)*stepy + dataset->m_lasHeader.min_y;

				int rectID = 0;
				vector<int> rectIDs;
				dataset->LASDataset_Search(rectID, rect, rectIDs);

				//获取点密度
				int numPoints = 0;
				for (int l = 0; l<rectIDs.size(); ++l)
				{
					for (int m = 0; m<dataset->m_lasRectangles[rectIDs[l]].m_lasPoints_numbers; ++m)
					{
						float x = dataset->m_lasRectangles[rectIDs[l]].m_lasPoints[m].m_vec3d.x;
						float y = dataset->m_lasRectangles[rectIDs[l]].m_lasPoints[m].m_vec3d.y;
						float z = dataset->m_lasRectangles[rectIDs[l]].m_lasPoints[m].m_vec3d.z;

						if (x>i*stepx + dataset->m_lasHeader.min_x&&
							x<(i + 1)*stepx + dataset->m_lasHeader.min_x&&
							y>j*stepy + dataset->m_lasHeader.min_y&&
							y<(j + 1)*stepy + dataset->m_lasHeader.min_y&&
							z>k*stepz + dataset->m_lasHeader.min_z&&
							z<(k + 1)*stepz + dataset->m_lasHeader.min_z)
							numPoints++;
					}
				}
				densePre.dense = numPoints;
				this->m_model.push_back(densePre);
			}
		}
	}
}

void LASModel::LASModel_Build(ILASDataset* dataset, float cubeRange, const char* pathExport)
{
	LASModel_PointsDense(dataset, cubeRange);

	//export
	FILE* fModel = fopen(pathExport, "w+");
	LASDense::iterator it = m_model.begin();
	while (it != m_model.end())
	{
		fprintf(fModel, "%d %d %d %d\n", it->point.x, it->point.y, it->point.z, it->dense);
		printf("%d %d %d %d\n", it->point.x, it->point.y, it->point.z, it->dense);
		it++;
	}
	fclose(fModel);
}

Point3Ds LASInvarianceFeatureExtract::LASInvariancePointsPart(ILASDataset* dataset,
															  int pointIdx,
															  int num)
{
	std::vector<Point3D> pntCloud;
	for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = dataset->m_LASPointID[i];
		pntCloud.push_back(dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
	}

	const PCAdaptor pcAdaptorPnts(pntCloud);
	kd_tree datasetIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	datasetIndex.buildIndex();

	const LASIndex &idxPnt = dataset->m_LASPointID[pointIdx];
	Point3D pnt = dataset->m_lasRectangles[idxPnt.rectangle_idx].m_lasPoints[idxPnt.point_idx_inRect].m_vec3d;
	double query_pt[3] = { pnt.x, pnt.y, pnt.z };

	size_t *ret_index   =new size_t[num];
	double *out_dist_sqr=new double[num];
	KNNResultSet<double> resultSet(num);
	// const double radius = range;
	// std::vector<std::pair<size_t, double> > indices_dists;
	// RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

	//resultSet.init(ret_index, out_dist_sqr);
	resultSet.init(ret_index, out_dist_sqr);
	datasetIndex.findNeighbors(resultSet, &query_pt[0], SearchParams());

	Point3Ds pntOutput;
	for (int i = 0; i < num; ++i)
	{
		//for debug
		//printf("ret_index=%d out_dist_sqr=%lf\n", resultSet.m_indices_dists[i].first, resultSet.m_indices_dists[i].second);
		int index = ret_index[i];
		const LASIndex &idx = dataset->m_LASPointID[index];
		pntOutput.push_back(dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
	}

	delete []ret_index;ret_index=nullptr;
	delete []out_dist_sqr;out_dist_sqr=nullptr;
	
	return pntOutput;
}

long LASInvarianceFeatureExtract::LASInvariancePointsLASDisHistroCal(Point3Ds part,double *histro)
{
	memset(histro,sizeof(double)*part.size(),0);
    Point3D pt2(part[0].x,part[0].y,part[0].z);
    for(int i=0;i<part.size();++i)
    {
        Point3D pt1(part[i].x,part[i].y,part[i].z);
        histro[i] = DistanceComputation::Distance(pt1,pt2);
    }
    std::sort(histro,histro+part.size());

	//print to console
	for(int i=0;i<part.size();++i)
	{
		printf("%0.4lf  ",histro[i]);
	}
	printf("\n");

    return 0;
}

long LASInvarianceFeatureExtract::LASInvariancePointsLASAngleHistroCal(Point3Ds part,double *histro)
{
	double *disHistro = new double[part.size()];
    memset(histro,sizeof(double)*part.size(),0);
    memset(disHistro,sizeof(double)*part.size(),0);
    Point3D pt2(part[0].x,part[0].y,part[0].z);
    for(int i=0;i<part.size();++i)
    {
        Point3D pt1(part[i].x,part[i].y,part[i].z);
        histro[i] = DistanceComputation::Distance(pt1,pt2);
    }

    vector<int> idx(part.size());
    for (int i = 0; i < part.size(); i++)idx[i] = i;
	
    //DEBUG
	// for (int i = 0; i < part.size(); i++) printf("%lf  ",histro[i]);
	// printf("\n");
	//EDEBUG

    sort(histro,histro+part.size());
    sort(idx.begin(),idx.end(),[histro](int i1, int i2) {return histro[i1] < histro[i2]; });

    // //DEBUG
	// for (int i = 0; i < part.size(); i++) printf("%d  ",idx[i]);
	// printf("\n");
	// //EDEBUG
	
    double vx1=part[idx[1]].x-part[idx[0]].x;
    double vy1=part[idx[1]].y-part[idx[0]].y;
    double vz1=part[idx[1]].z-part[idx[0]].z;

    double down1 = vx1*vx1+vy1*vy1+vz1*vz1;
    disHistro[0] = histro[0];
	disHistro[1] = 0;

    for (int i = 2; i < part.size(); i++)
    {

        double vx2=part[idx[i]].x-part[idx[0]].x;
        double vy2=part[idx[i]].y-part[idx[0]].y;
        double vz2=part[idx[i]].z-part[idx[0]].y;
        
        double up = vx1*vx2+vy1*vy2+vz1*vz2;
        double down2 = vx2*vx2+vy2*vy2+vz2*vz2;
        disHistro[i] = histro[i]*up/sqrt(down1)/sqrt(down2);
        // angleHistro[i-1]=up/sqrt(down1)*sqrt(down2);
    }
	
	memcpy(histro,disHistro,sizeof(double)*part.size());
	for(int i=0;i<part.size();++i)
	{
		printf("%0.4lf  ",histro[i]);
	}
	printf("\n");
    delete []disHistro;disHistro=nullptr;

}
