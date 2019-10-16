#include "LASFilter.h"
#include"GeometryAlgorithm.h"
#include"GeometryFlann.h"
#include<gdal_priv.h>
using namespace GeometryLas;
namespace LasAlgorithm{
/////////////////////////////////////
	long PointCloudFilterDEM::PointCloudFilter_Point2DEM(ILASDataset *lasDataset, float resolution, const char* pathChr, int filterTimes/* = 0*/)
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

		for (int i = 0; i < xsize - 1; ++i)
		{
			printf("\r%d-\%d", xsize, i + 1);
			for (int j = 0; j < ysize - 1; ++j)
			{
				Rect2D rect;
				rect.maxx = (i + 1) * resolution + minx;
				rect.minx = i * resolution + minx;
				rect.maxy = (j + 1) * resolution + miny;
				rect.miny = j * resolution + miny;

				vector<int> rectIds;
				lasDataset->LASDataset_Search(0, rect, rectIds);

				for (int k = 0; k < rectIds.size(); ++k)
				{
					for (int m = 0; m < lasDataset->m_lasRectangles[rectIds[k]].m_lasPoints_numbers; ++m)
					{
						const LASPoint &lasPnt = lasDataset->m_lasRectangles[rectIds[k]].m_lasPoints[m];
						if (GeometryRelation::IsPointInRect(lasPnt.m_vec3d.x, lasPnt.m_vec3d.y, \
							rect.minx, rect.miny, rect.maxx, rect.maxy))
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
		GDALRasterIO(GDALGetRasterBand(m_dataset, 1), GF_Write, 0, 0, xsize, ysize, demData, xsize, ysize, GDT_Float32, 0, 0);
		GDALSetGeoTransform(m_dataset, adfGeoTransform);
		GDALClose(m_dataset);

		delete[]demData; demData = nullptr;
		delete[]idxData; idxData = nullptr;
	}

	long PointCloudFilterDEM::PointCloudFilter_Point2DEMFlann(ILASDataset *lasDataset, float resolution, const char* pathChr)
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
					heightPre += out_dist_sqr[i] / disTotal*height[ret_index[i]];

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

	long PointCloudFilterDEM::PointCloudFilter_DEMFilter(float *dataDEM, int xsize, int ysize)
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
				if (i + ind[0] > 0 && i + ind[0] < xsize&&j + ind[0]>0 && j + ind[0] < ysize)
				{
					if (cpyData[(j + ind[0])*xsize + i + ind[0]] != -1.0f)
					{
						data += cpyData[(j + ind[0])*xsize + i + ind[0]];
						dataidx++;
					}
				}
				//center top
				if (i > 0 && i < xsize&&j + ind[0] >0 && j + ind[0] < ysize)
				{
					if (cpyData[(j + ind[0])*xsize + i] != -1.0f)
					{
						data += cpyData[(j + ind[0])*xsize + i];
						dataidx++;
					}
				}
				//right top
				if (i + ind[1] > 0 && i + ind[1] < xsize&&j + ind[0]>0 && j + ind[0] < ysize)
				{
					if (cpyData[(j + ind[0])*xsize + i + ind[1]] != -1.0f)
					{
						data += cpyData[(j + ind[0])*xsize + i + ind[1]];
						dataidx++;
					}
				}
				//left
				if (i + ind[0] > 0 && i + ind[0] < xsize&&j>0 && j < ysize)
				{
					if (cpyData[(j)*xsize + i + ind[0]] != -1.0f)
					{
						data += cpyData[(j)*xsize + i + ind[0]];
						dataidx++;
					}
				}
				//right
				if (i + ind[1] > 0 && i + ind[1] < xsize&&j>0 && j < ysize)
				{
					if (cpyData[(j)*xsize + i + ind[1]] != -1.0f)
					{
						data += cpyData[(j)*xsize + i + ind[1]];
						dataidx++;
					}
				}
				//left bottom 
				if (i + ind[0] > 0 && i + ind[0] < xsize&&j + ind[1]>0 && j + ind[1] < ysize)
				{
					if (cpyData[(j + ind[1])*xsize + i + ind[0]] != -1.0f)
					{
						data += cpyData[(j + ind[1])*xsize + i + ind[0]];
						dataidx++;
					}
				}
				//center bottom 
				if (i > 0 && i < xsize&&j + ind[1]>0 && j + ind[1] < ysize)
				{
					if (cpyData[(j + ind[1])*xsize + i] != -1.0f)
					{
						data += cpyData[(j + ind[1])*xsize + i];
						dataidx++;
					}
				}
				//right bottom
				if (i + ind[1] > 0 && i + ind[1] < xsize&&j + ind[1]>0 && j + ind[1] < ysize)
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

    
    long PointCloudFilterNoise::PointCloudFilter_Outlier(Point3Ds pointSet,int ptNumerThreshod,double rangeThreshod)
    {
        //构建kdTree
        typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
		const PCAdaptor pcAdaptorPnts(pointSet);

		typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
		kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
		treeIndex.buildIndex();

        //对每一个点计算其K近邻点
        const int num_results=ptNumerThreshod+1;
        for (int i = 0; i < pointSet.size(); ++i)
		{
            double pnt[3] = { pointSet[i].x,pointSet[i].y,pointSet[i].z };
            std::vector<std::pair<size_t, double> > indices_dists;
            size_t ret_index[num_results];
            double out_dist_sqr[num_results];
            KNNResultSet<double> resultSet(num_results);
            resultSet.init(ret_index, out_dist_sqr);
            treeIndex.findNeighbors(resultSet, &pnt[0], SearchParams());

            //计算是否为噪声点
            double disMean = 0;
            for(int j=0;j<num_results;++j){
                disMean+=DistanceComputation::Distance(pointSet[i],pointSet[ret_index[j]])/10.0;
            }
            
            if(disMean>rangeThreshod)
            {
                //标记为噪声点
            }


        }
        return 0;

    }

    long PointCloudFilterNoise::PointCloudFilter_RGBOutlier(ILASDataset *lasDataset,int r,int g,int b){
        return 0;
    }
}