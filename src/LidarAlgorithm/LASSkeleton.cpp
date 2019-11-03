#include "LASSeleton.h"
#include <set>

#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASReader.h"

#include"GeometryAlgorithm.h"
#include"GeometryFlann.h"

using namespace GeometryLas;
using namespace std;
namespace LasAlgorithm{

    int PointCloudShrinkSkeleton::PointCloudShrinkSkeleton_Centroid(Point3D pointSet int* clusterIdx,int clusterNum)
    {
        //判断重心的标准为点到点簇中所有点的距离之和最小
        double disMin=999999999;
        int idxMin=0;
        
        for(int i=0;i<clusterNum;++i)
        {
            double disSigma=0;
            for(int j=0;j<clusterNum;++j)
            {
                disSigma+=DistanceComputation::Distance(pointSet[clusterIdx[i]],pointSet[clusterIdx[j]]);
            }
            disMin=disSigma<disMin?disSigma:disMin;
            idxMin=disSigma<disMin?clusterIdx[i]:clusterIdx[idxMin];
        }
        return idxMin
    }

    Point3Ds PointCloudShrinkSkeleton::PointCloudShrinkSkeleton_Once(Point3Ds pointSet,int nearPointNum)
    {
        //构建kdtree
        typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
		const PCAdaptor pcAdaptorPnts(pointSet);

		typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
		kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
		treeIndex.buildIndex();
        Point3Ds pntSkeSet;
        
        size_t *ret_index=new size_t[nearPointNum];
        double *out_dist_sqrt = new double[nearPointNum];
        
        set<int> idxSet;
        
        for(int i=0;i<pointSet.size();++i)
        {
            double pnt[3]={pointSet[i].x,pointSet[i].y,pointSet[i].z};
            KNNResultSet<double> resultSet(nearPointNum);
            treeIndex.findNeighbors(resultSet,&pnt[0],searchParams());
            bool ifFind = false;
            for(int i=0;i<nearPointNum;++i)
            {
                ifFind = idxSet.find(ret_index[i]==idxSet.end()?true:false;
            } 
            if(!ifFind)
            {
                int idxCentId = PointCloudShrinkSkeleton_Centroid(pointSet,ret_index,nearPointNum);
                set<int>::iterator resultIdx = find(idxSet.begin(),idxSet.end(),idxCentid);
                pntSkeSet.push_back(pointSet[idxCentId]);
                idxSet.insert(idxCentId);
            }
        }
        delete[]ret_index;ret_index=nullptr;
        delete[]out_dist_sqrt;out_dist_sqrt=nullptr;
        idxSet.clear();
        
        return pntSkeSet;        
    }

    
    Point3Ds  PointCloudShrinkSkeleton::PointCloudShrinkSkeleton_Shrink(Point3Ds pointSet,int nearPointNum,int iteratorNum)
    {
        Point3Ds pnts(pointSet);
        Point3Ds pntTmp;
        for(int i=0;i<iteratorNum;++i)
        {
            pntTmp=PointCloudShrinkSkeleton_Once(pnts,nearPointNum);
            pnts.clear();
            pnts=pntTmp;
            pntTmp.clear();
        }
    
        return pnts;
    }    
    
    


    Point3Ds  PointCloudShrinkSkeleton::PointCloudShrinkSkeleton_Shrink(ILASDataset* lasDataset,int nearPointNum,int iteratorNum)
    {
        Point3Ds pointSet;
		for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = dataset->m_LASPointID[i];
			pointSet.push_back(dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
		}
		return PointCloudShrinkSkeleton_Shrink(pointSet,nearPointNum,iteratorNum);   
    }






}