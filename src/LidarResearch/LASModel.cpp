//
// Created by wuwei on 18-1-21.
//

#include"../LidarApplication/LASModel.h"
#include "../LidarBase/LASPoint.h"
#include "../LidarAlgorithm/Geometry.h"

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