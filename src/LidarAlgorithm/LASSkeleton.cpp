#include "LASSkeleton.h"
#include <set>

#include "../LidarBase/LASReader.h"
#include"../LidarGeometry/GeometryAlgorithm.h"
#include"../LidarGeometry/GeometryFlann.h"

using namespace GeometryLas;
using namespace std;
namespace LasAlgorithm
{
    int PointCloudShrinkSkeleton::PointCloudShrinkSkeleton_Centroid(Point3Ds pointSet, size_t* clusterIdx,int clusterNum)
    {
        //判断重心的标准为点到点簇中所有点的距离之和最小
        double disMin=999999999;
        int idxMin=0;

        //从性能上分析这个部分是可以进行优化的
        //TODO：不知道是不是有相应的算法，需要寻找
        for(int i=0;i<clusterNum;++i)
        {
            double disSigma=0;
            for(int j=0;j<clusterNum;++j)
            {
                disSigma+=DistanceComputation::Distance(pointSet[clusterIdx[i]],pointSet[clusterIdx[j]]);
            }
            idxMin=disSigma<disMin?i:idxMin;
            disMin=disSigma<disMin?disSigma:disMin;
            
        }
        return clusterIdx[idxMin];
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

        //看起来整个算法是串行算法，没法进行优化
        for(int i=0;i<pointSet.size();++i)
        {
            //首先找到K近邻的点
            double pnt[3]={pointSet[i].x,pointSet[i].y,pointSet[i].z};
            KNNResultSet<double> resultSet(nearPointNum);
            resultSet.init(ret_index, out_dist_sqrt);
            treeIndex.findNeighbors(resultSet,&pnt[0],SearchParams(10));

            //判断K近邻点中是否包含已有的中心点
            bool ifNotFind = false;
            for(int j=0;j<nearPointNum;++j)
            {
                ifNotFind = idxSet.find(ret_index[j])==idxSet.end()?true:false;
                if(!ifNotFind)
                    break;
            } 
            //如果不包含已有的中心点则在K近邻的点集中确定一个中心点
            if(ifNotFind)
            {
                int idxCentId = PointCloudShrinkSkeleton_Centroid(pointSet,ret_index,nearPointNum);
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
#ifdef _DEBUG
            printf("iterator: %d points count: %d\n",i+1,pntTmp.size());
#endif
            pnts.clear();
            pnts=pntTmp;
            pntTmp.clear();
        }
    
        return pnts;
    }    

    Point3Ds  PointCloudShrinkSkeleton::PointCloudShrinkSkeleton_Shrink(ILASDataset* lasDataset,int nearPointNum,int iteratorNum)
    {
        Point3Ds pointSet;
		for (int i = 0; i < lasDataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = lasDataset->m_LASPointID[i];
			pointSet.push_back(lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
		}
		return PointCloudShrinkSkeleton_Shrink(pointSet,nearPointNum,iteratorNum);   
    }

}