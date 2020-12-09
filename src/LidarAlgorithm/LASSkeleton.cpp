#include "LASSkeleton.h"
#include <set>
#include <iostream>
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
// #ifdef _DEBUG
//             printf("iterator: %d points count: %d\n",i+1,pntTmp.size());
// #endif
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
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * 直接根据直线拟合的方法能够获取线性特征比较强的块
     * 但是算法存在最大的问题在于在某些块中有线性特征比较强
     * 但是某些点偏差比较大，对于这些偏差大的点无法剔除
     **/
    bool PointCloudLineSkeleton::PointCloudLineSkeleton_LineExtractRaw(Point3Ds pointCluster,double threshold,MatrixXd &param)
    {
        //直线拟合
        //减去均值去中心化后平移的值为0，因此在处理过程中需要恢复，如何恢复平移的值还没有想好
        //先求缩放比例，然后代入回均值去求平移的值，如果只是判断是否为直线，则可以省略该过程
        double cx=0,cy=0,cz=0;
        for(int i=0;i<pointCluster.size();++i)
        {
            cx+=pointCluster[i].x/pointCluster.size();
            cy+=pointCluster[i].y/pointCluster.size();
            cz+=pointCluster[i].z/pointCluster.size();
        }
        for(int i=0;i<pointCluster.size();++i)
        {
            pointCluster[i].x-=cx;
            pointCluster[i].y-=cy;
            pointCluster[i].z-=cz;
        }

        //计算
        MatrixXd params = MatrixXd::Zero(2,2);
        MatrixXd paramM1= MatrixXd::Zero(2,2);
        MatrixXd paramM2= MatrixXd::Zero(2,2);
        for(int i=0;i<pointCluster.size();++i)
        {
            // printf("%lf-%lf-%lf\n",pointCluster[i].x,pointCluster[i].y,pointCluster[i].z);
            paramM1(0,0)+=pointCluster[i].x*pointCluster[i].z;
            paramM1(0,1)+=pointCluster[i].x;
            paramM1(1,0)+=pointCluster[i].y*pointCluster[i].z;
            paramM1(1,1)+=pointCluster[i].y;

            paramM2(0,0)+=pointCluster[i].z*pointCluster[i].z;
            paramM2(0,1)+=pointCluster[i].z;
            paramM2(1,0)+=pointCluster[i].z;
            paramM2(1,1)+=1;
        }
        params=paramM1*(paramM2.inverse());
        double residual=PointCloudLineSkeleton_LineResidual(pointCluster,params);
        param = params;
        //cout<<params<<endl;
        // direct.push_back(params(0,0));
        // direct.push_back(params(1,0));
        // direct.push_back(1.0);
        return residual<threshold;
    }

    //计算残差
    double PointCloudLineSkeleton::PointCloudLineSkeleton_LineResidual(Point3Ds pointCluster,MatrixXd lineParam)
    {
        double ex=0,ey=0,e=0;
        for(int i=0;i<pointCluster.size();++i)
        {
            MatrixXd mat(2,1);
            mat(0,0)=pointCluster[i].z;
            mat(1,0)=1;
            MatrixXd res=lineParam*mat;

            ex = res(0,0)-pointCluster[i].x;
            ey = res(1,0)-pointCluster[i].y;
            e+=sqrt(ex*ex+ey*ey);
        }
        return e/double(pointCluster.size());
    }


    Point3Ds PointCloudLineSkeleton::PointCloudLineSkeleton_Extract(Point3Ds pointSet,int nearPointNum,double lineResidual)
    {
        Point3Ds pnts(pointSet);
        Point3Ds pntTmp;
        for(int i=0;i<1;++i)
        {
            pntTmp=PointCloudLineSkeleton_Once(pointSet,nearPointNum,lineResidual);
#ifdef _DEBUG
            printf("iterator: %d points count: %d\n",i+1,pntTmp.size());
#endif
            pnts.clear();
            pnts=pntTmp;
            pntTmp.clear();
        }
        return pnts;
    }

    Point3Ds PointCloudLineSkeleton::PointCloudLineSkeleton_Extract(ILASDataset* lasDataset,int nearPointNum,double lineResidual)
    {
        Point3Ds pointSet;
		for (int i = 0; i < lasDataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = lasDataset->m_LASPointID[i];
			pointSet.push_back(lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
		}
		return PointCloudLineSkeleton_Extract(pointSet,nearPointNum,lineResidual); 
    }

    Point3Ds PointCloudLineSkeleton::PointCloudLineSkeleton_Once(Point3Ds pointSet,int nearPointNum,double lineResidual)
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
        vector<int> useLabel(pointSet.size());
        for(int i=0;i<pointSet.size();++i)
        {
            useLabel[i]=0;
        }

        //看起来整个算法是串行算法，没法进行优化
        MatrixXd direct;
        for(int i=0;i<pointSet.size();++i)
        {
            //首先找到K近邻的点
            Point3Ds temPtSet;
            double pnt[3]={pointSet[i].x,pointSet[i].y,pointSet[i].z};
            KNNResultSet<double> resultSet(nearPointNum);
            resultSet.init(ret_index, out_dist_sqrt);
            treeIndex.findNeighbors(resultSet,&pnt[0],SearchParams(10));
            for(int j=0;j<nearPointNum;++j)
            {
                temPtSet.push_back(pointSet[ret_index[j]]);
            }
            //判断是否具有线性特征
            if(PointCloudLineSkeleton_LineExtractRaw(temPtSet,lineResidual,direct))
            {
                for(int j=0;j<nearPointNum;++j)
                {
                    if(useLabel[ret_index[j]]==0)
                    {
                        useLabel[ret_index[j]]=1;
                        pntSkeSet.push_back(pointSet[ret_index[j]]);
                    }
                }
            }
        }
        delete[]ret_index;ret_index=nullptr;
        delete[]out_dist_sqrt;out_dist_sqrt=nullptr;
        useLabel.clear();
        return pntSkeSet;                
    }

    void PointCloudLineSkeleton::PointCloudShrinkSkeleton_LineTest()
    {
        Point3Ds points(50);
        for (int i = 0; i < 50; i++)
        {
            points[i].z = (double)i; 
            points[i].x = 3*points[i].z + 1 + double(rand())/double(RAND_MAX);  //引入噪声
            points[i].y = 2*points[i].z + 2 + double(rand())/double(RAND_MAX);  //引入噪声
        }
        MatrixXd direct;
        PointCloudLineSkeleton_LineExtractRaw(points,5,direct);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //精化线拟合
    vector<int> PointCloudLineRefineSkeleton::PointCloudLineSkeleton_LineRefine(Point3D ptCnt,Point3Ds pointCluster,double threshold)
    {
        vector<int> lineRefineIdxs;
        MatrixXd direct;
        GeometryRelation geoRel;
        double cx=0,cy=0,cz=0;
        for(int i=0;i<pointCluster.size();++i)
        {
            cx+=pointCluster[i].x/pointCluster.size();
            cy+=pointCluster[i].y/pointCluster.size();
            cz+=pointCluster[i].z/pointCluster.size();
        }
        for(int i=0;i<pointCluster.size();++i)
        {
            pointCluster[i].x-=cx;
            pointCluster[i].y-=cy;
            pointCluster[i].z-=cz;
        }
        if(PointCloudLineSkeleton_LineExtractRaw(pointCluster,threshold,direct))
        {
            //V1.0
            //整体的判断有问题，在这里给出的是残差而不是距离阈值，需要进一步思考
            //先去中心化，然后直接判断角度
            //在具有线性特征的情况下进一步获取具体的点
            //Point3D blockDirect(0,direct(1,1)-direct(1,0)*(direct(0,1)/direct(0,0)),-direct(0,1)/direct(0,0));
            // for(int i=0;i<pointCluster.size();++i)
            // {
            //     double distance=DistanceComputation::Distance(ptCnt,pointCluster[i]);
            //     //方向的计算有问题
            //     //虚拟于主方向上任何一个点理论上都可以，只需要了解到大小的趋势就行
            //     Point3D ptDirect(pointCluster[i].x-blockDirect.x,pointCluster[i].y-blockDirect.y,pointCluster[i].z-blockDirect.z);
            //     double vecterAngle = geoRel.VectorAngle(blockDirect,ptDirect);
            //     //判断(综合考虑方向因素后的距离是否小于原始距离的60%)
            //     //无法很好的判断与主方向的夹角的问题，因此这里在对角度的精化上存在没有明确物理意义的地方
            //     if(vecterAngle<0.2)
            //     {
            //         
            //     }
            // }

            //V2.0
            //按方向判断总是感觉有点问题，还是按残差判断比较靠谱
            //direct 中存储的是线性参数，通过此种方法是能够提取出线性特征，但是感觉没有太大意义
            //计算残差
            double ex=0,ey=0,e=0;
            for(int i=0;i<pointCluster.size();++i)
            {
                MatrixXd mat(2,1);
                mat(0,0)=pointCluster[i].z;
                mat(1,0)=1;
                MatrixXd res=direct*mat;

                ex = res(0,0)-pointCluster[i].x;
                ey = res(1,0)-pointCluster[i].y;
                double e=sqrt(ex*ex+ey*ey);
                if(e<threshold)
                {
                    lineRefineIdxs.push_back(i);
                }
            }
        }
        return lineRefineIdxs;
    }

    Point3Ds PointCloudLineRefineSkeleton::PointCloudLineSkeleton_Once(Point3Ds pointSet,int nearPointNum,double lineResidual)
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
        vector<int> useLabel(pointSet.size());
        for(int i=0;i<pointSet.size();++i)
        {
            useLabel[i]=0;
        }

        //看起来整个算法是串行算法，没法进行优化
        MatrixXd direct;
        for(int i=0;i<pointSet.size();++i)
        {
            //首先找到K近邻的点
            Point3Ds temPtSet;
            double pnt[3]={pointSet[i].x,pointSet[i].y,pointSet[i].z};
            KNNResultSet<double> resultSet(nearPointNum);
            resultSet.init(ret_index, out_dist_sqrt);
            treeIndex.findNeighbors(resultSet,&pnt[0],SearchParams(10));
            for(int j=0;j<nearPointNum;++j)
            {
                temPtSet.push_back(pointSet[ret_index[j]]);
            }
            //判断是否具有线性特征
            if(PointCloudLineSkeleton_LineExtractRaw(temPtSet,lineResidual,direct))
            {
                //获取强线性特征的点集
                vector<int> lineRef;
                lineRef=PointCloudLineSkeleton_LineRefine(pointSet[i],temPtSet,lineResidual);
                // printf("%d\n",lineRef.size());
                for(int j=0;j<lineRef.size();++j)
                {
                    int idx = lineRef[j];
                    if(useLabel[ret_index[idx]]==0)
                    {
                        useLabel[ret_index[idx]]=1;
                        pntSkeSet.push_back(pointSet[ret_index[idx]]);
                    }
                }
            }
        }
        delete[]ret_index;ret_index=nullptr;
        delete[]out_dist_sqrt;out_dist_sqrt=nullptr;
        useLabel.clear();
        return pntSkeSet;      
    }

    Point3Ds PointCloudLineRefineSkeleton::PointCloudLineSkeleton_Extract(Point3Ds pointSet,int nearPointNum,double lineResidual)
    {
        Point3Ds pnts(pointSet);
        Point3Ds pntTmp;
        for(int i=0;i<1;++i)
        {
            pntTmp=PointCloudLineSkeleton_Once(pointSet,nearPointNum,lineResidual);
// #ifdef _DEBUG
//             printf("iterator: %d points count: %d\n",i+1,pntTmp.size());
// #endif
            pnts.clear();
            pnts=pntTmp;
            pntTmp.clear();
        }
        return pnts;
    }

    Point3Ds PointCloudLineRefineSkeleton::PointCloudLineSkeleton_Extract(ILASDataset* lasDataset,int nearPointNum,double lineResidual)
    {
        Point3Ds pointSet;
		for (int i = 0; i < lasDataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = lasDataset->m_LASPointID[i];
			pointSet.push_back(lasDataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
		}
		return PointCloudLineSkeleton_Extract(pointSet,nearPointNum,lineResidual); 
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    long PointCloudLineInteractive::PointCloudLineInteractive_GetPointsRange(ILASDataset *dataset,double range,Point3Ds points,Point3Ds &innerPoints)
    {
        assert(points.size()>=2);

		for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = dataset->m_LASPointID[i];
            const LASPoint &lasPt = dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect];

            //点到线的距离
            bool inner=false;
            for(int j=1;j<points.size();++j)
            {   
                inner = DistanceComputation::Distance(lasPt.m_vec3d,points[j-1],points[j])<range?true:false;
                
                if(inner)
                {
                    innerPoints.push_back(lasPt.m_vec3d);
                    break;  //避免重复添加
                }
            }
		}
        return 0;
    }

    long PointCloudLineInteractive::PointCloudLineInteractive_FindNearestLinePoitns(Point3Ds innerPoints,Point3Ds &nearestPoints,Point3Ds mutiLines,int idx)
    {
        nearestPoints.clear();
        for (size_t i = 0; i < innerPoints.size(); i++)
        {
            /* code */
            double minDis = _MAX_LIMIT_;
            int idxMin = 0;
            for(int j=1;j<mutiLines.size();++j)
            {   
                double tmpMin = DistanceComputation::Distance(innerPoints[i],mutiLines[j-1],mutiLines[j]);
                if(minDis>tmpMin)
                {
                    minDis = tmpMin;
                    idxMin = j;
                }
            }
            if(idxMin==idx)
            {
                nearestPoints.push_back(innerPoints[i]);
            }
        }

        /**
         * export nearest point
         **/ 
        FILE* fs = fopen("../data/test/innerline.txt","w+");
        for(int i=0;i<innerPoints.size();++i)
        {
            fprintf(fs,"%lf,%lf,%lf\n",innerPoints[i].x,innerPoints[i].y,innerPoints[i].z);
        }
        fclose(fs);

        return 0;
    }


   
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool PointCloudLineInteractiveSimple::PointCloudLineInteractive_LineCheck(ILASDataset *dataset,double range,double density, Point3Ds points)
    {
        assert(points.size()>=2);
        int totalContainPoint=0;
        double dis = DistanceComputation::Distance(points[0],points[1]);
        if(dis<2.0)
        {
            return false;
        }
		for (int i = 0; i < dataset->m_totalReadLasNumber; ++i)
		{
			const LASIndex &idx = dataset->m_LASPointID[i];
            const LASPoint &lasPt = dataset->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect];

            //点到线的距离
            bool inner=false;
            for(int j=1;j<points.size();++j)
            {   
                double tmpdis2line=DistanceComputation::Distance(lasPt.m_vec3d,points[j-1],points[j]);
                inner = tmpdis2line<range?true:false;
                if(inner)
                {
                    totalContainPoint++;
                    break;  //避免重复添加
                }
            }
		}

        printf("%d\n",totalContainPoint);
        return totalContainPoint/dis>density;
    }

    long PointCloudLineInteractiveSimple::PointCloudLineInteractive_LineGet(ILASDataset *dataset,double range,double density, Point3Ds points,vector<Point3Ds> &lines)
    {
        //针对任意两点判断是为直线，如果为直线则获取直线
        for(int i=0;i<points.size();++i)
        {
            for(int j=i+1;j<points.size();++j)
            {
                Point3Ds pnts;
                pnts.push_back(points[i]);
                pnts.push_back(points[j]);

                if(PointCloudLineInteractive_LineCheck(dataset,range,density,pnts))
                {
                    lines.push_back(pnts);
                }
                
            }
        }
        return 0;
    }

    long PointCloudLineInteractiveSimple::PointCloudLineInteractive_LineFitOnce(Point3Ds nearestPoints,Point3Ds &mutiLines,int idx)
    {
        // reference: 薛丽红. 三维空间点中基于最小二乘法的分段直线拟合方法[J]. 齐齐哈尔大学学报(自然科学版), 2015(04):87-88+92.
        // 不能简单的对公式进行套用，需要分不同情况进行讨论
        
        double maxz=_MIN_LIMIT_,minz=_MAX_LIMIT_,
               maxx=_MIN_LIMIT_,minx=_MAX_LIMIT_,
               maxy=_MIN_LIMIT_,miny=_MAX_LIMIT_,
               cx=0,cy=0,cz=0;

        int idmaxz=0,idminz=0,
            idmaxx=0,idminx=0,
            idminy=0,idmaxy=0;

          //必须要去中心化避免出现计算越界的情况
        for (size_t i = 0; i < nearestPoints.size(); i++)
        {
            cx+=nearestPoints[i].x/nearestPoints.size();
            cy+=nearestPoints[i].y/nearestPoints.size();
            cz+=nearestPoints[i].z/nearestPoints.size();
        }
        for (size_t i = 0; i < nearestPoints.size(); i++)
        {
            nearestPoints[i].x-=cx;
            nearestPoints[i].y-=cy;
            nearestPoints[i].z-=cz;
        }

        for (size_t i = 0; i < nearestPoints.size(); i++)
        {
            /* code */
            if(nearestPoints[i].z>maxz)
            {
                maxz=nearestPoints[i].z;
                idmaxz = i;
            }
            if(nearestPoints[i].z<minz)
            {
                minz=nearestPoints[i].z;
                idminz = i;
            }

            if(nearestPoints[i].x>maxx)
            {
                maxx=nearestPoints[i].x;
                idmaxx = i;
            }
            if(nearestPoints[i].x<minx)
            {
                minx=nearestPoints[i].x;
                idminx = i;
            }

            if(nearestPoints[i].y>maxy)
            {
                maxy=nearestPoints[i].y;
                idmaxy = i;
            }
            if(nearestPoints[i].y<miny)
            {
                miny=nearestPoints[i].y;
                idminy = i;
            }
        }
        

        int type=0;
        double k1 =0 , b1 =0 ,k2 =0 ,b2 =0;
        if(((maxz-minz)>=(maxx-minx))&&((maxz-minz)>=(maxy-miny)))
        {
            // x=k1*z+b1
            // y=k2*z+b2
            double sigmaXi=0,sigmaYi=0,sigmaZi=0,
                   sigmaXiZi=0,sigmaYiZi=0,sigmaZiZi=0;

            for (size_t i = 0; i < nearestPoints.size(); i++)
            {
                /* code */
                sigmaXiZi+=nearestPoints[i].x*nearestPoints[i].z;
                sigmaYiZi+=nearestPoints[i].y*nearestPoints[i].z;
                sigmaZiZi+=nearestPoints[i].z*nearestPoints[i].z;

                sigmaXi+=nearestPoints[i].x;
                sigmaYi+=nearestPoints[i].y;
                sigmaZi+=nearestPoints[i].z;
            }
            k1 = (2*sigmaXiZi-sigmaXi*sigmaZi)/(2*sigmaZiZi-sigmaZi*sigmaZi);
            b1 = (sigmaXi-k1*sigmaZi)/2;
            k2 = (2*sigmaYiZi-sigmaYi*sigmaZi)/(2*sigmaZiZi-sigmaZi*sigmaZi);
            b2 = (sigmaYi-k2*sigmaZi)/2;
            type=1;
        }
        else if(((maxx-minx)>=(maxz-minz))&&((maxx-minx)>=(maxy-miny)))
        {
            //z = k1*x+b1
            //y = k2*x+b2
            double sigmaZi=0,sigmaYi=0,sigmaXi=0,
                   sigmaZiXi=0,sigmaYiXi=0,sigmaXiXi=0;
            for (size_t i = 0; i < nearestPoints.size(); i++)
            {
                /* code */
                sigmaZiXi+=nearestPoints[i].z*nearestPoints[i].x;
                sigmaYiXi+=nearestPoints[i].y*nearestPoints[i].x;
                sigmaXiXi+=nearestPoints[i].x*nearestPoints[i].x;

                sigmaXi+=nearestPoints[i].x;
                sigmaYi+=nearestPoints[i].y;
                sigmaZi+=nearestPoints[i].z;
            }                   
            k1 = (2*sigmaZiXi-sigmaZi*sigmaXi)/(2*sigmaXiXi-sigmaXi*sigmaXi);
            b1 = (sigmaZi-k1*sigmaXi)/2;
            k2 = (2*sigmaYiXi-sigmaYi*sigmaXi)/(2*sigmaXiXi-sigmaXi*sigmaXi);
            b2 = (sigmaYi-k2*sigmaXi)/2;

            type=2;
        }else if(((maxy-miny)>(maxz-minz))&&((maxy-miny)>(maxx-minx)))
        {
            // x=k1*y+b1
            // z=k2*y+b2
            double sigmaXi=0,sigmaZi=0,sigmaYi=0,
                   sigmaXiYi=0,sigmaZiYi=0,sigmaYiYi=0;

            for (size_t i = 0; i < nearestPoints.size(); i++)
            {
                /* code */
                sigmaXiYi+=nearestPoints[i].x*nearestPoints[i].y;
                sigmaZiYi+=nearestPoints[i].z*nearestPoints[i].y;
                sigmaYiYi+=nearestPoints[i].y*nearestPoints[i].y;

                sigmaXi+=nearestPoints[i].x;
                sigmaYi+=nearestPoints[i].y;
                sigmaZi+=nearestPoints[i].z;
            }
            k1 = (2*sigmaXiYi-sigmaXi*sigmaYi)/(2*sigmaYiYi-sigmaYi*sigmaYi);
            b1 = (sigmaXi-k1*sigmaYi)/2;
            k2 = (2*sigmaZiYi-sigmaZi*sigmaYi)/(2*sigmaYiYi-sigmaYi*sigmaYi);
            b2 = (sigmaZi-k2*sigmaYi)/2;
            type=3;
        }
        
        // for (size_t i = 0; i < nearestPoints.size(); i++)
        // {
        //     printf("RMS:%lf,%lf\n",fabs(nearestPoints[i].x-k1*nearestPoints[i].z-b1),fabs(nearestPoints[i].x-k2*nearestPoints[i].z-b2));
        // }
        //printf("k1: %lf,b1：%lf,k2：%lf,b2：%lf\n",k1,b1,k2,b2);

        //求解在这个直线上的极值点
        if(type==1){
            // x=k1*z+b1
            // y=k2*z+b2
            double x1 = k1*nearestPoints[idminz].z+b1;
            double y1 = k2*nearestPoints[idminz].z+b2;

            double x2 = k1*nearestPoints[idmaxz].z+b1;
            double y2 = k2*nearestPoints[idmaxz].z+b2;
            
            Point3D pt1,pt2;

            if(mutiLines[idx-1].z<mutiLines[idx].x)
            {
                pt1.x = x1+cx;
                pt1.y = y1+cy;
                pt1.z = minz+cz;

                pt2.x = x2+cx;
                pt2.y = y2+cy;
                pt2.z = maxz+cz;
                Point3Ds line;
                line.push_back(pt1);
                line.push_back(pt2);
                
                // mutiLines[idx]    = pt1;
                // mutiLines[idx-1]  = pt2;
                mutiLines[idx-1]=GeometryRelation::ProjectionPoint(mutiLines[idx-1],line);
                mutiLines[idx]=GeometryRelation::ProjectionPoint(mutiLines[idx],line);
            }
            else{
                pt2.x = x1+cx;
                pt2.y = y1+cy;
                pt2.z = minz+cz;

                pt1.x = x2+cx;
                pt1.y = y2+cy;
                pt1.z = maxz+cz;
                Point3Ds line;
                line.push_back(pt1);
                line.push_back(pt2);
                // mutiLines[idx-1]    =pt1;
                // mutiLines[idx]  =pt2;
                mutiLines[idx-1]=GeometryRelation::ProjectionPoint(mutiLines[idx-1],line);
                mutiLines[idx]=GeometryRelation::ProjectionPoint(mutiLines[idx],line);
            }
        }
        else if(type==2){
            //z = k1*x+b1
            //y = k2*x+b2
            double z1 = k1*nearestPoints[idminx].x+b1;
            double y1 = k2*nearestPoints[idminx].x+b2;

            double z2 = k1*nearestPoints[idmaxx].x+b1;
            double y2 = k2*nearestPoints[idmaxx].x+b2;
            
            Point3D pt1,pt2;
            if(mutiLines[idx-1].x<mutiLines[idx].x)
            {
                pt1.x = minx+cx;
                pt1.y = y1+cy;
                pt1.z = z1+cz;

                pt2.x = maxx+cx;
                pt2.y = y2+cy;
                pt2.z = z2+cz;
                Point3Ds line;
                line.push_back(pt1);
                line.push_back(pt2);
                
                // mutiLines[idx]    = pt1;
                // mutiLines[idx-1]  = pt2;
                mutiLines[idx-1]=GeometryRelation::ProjectionPoint(mutiLines[idx-1],line);
                mutiLines[idx]=GeometryRelation::ProjectionPoint(mutiLines[idx],line);
            }
            else{
                pt2.x = minx+cx;
                pt2.y = y1+cy;
                pt2.z = z1+cz;

                pt1.x = maxx+cx;
                pt1.y = y2+cy;
                pt1.z = z2+cz;
                Point3Ds line;
                line.push_back(pt1);
                line.push_back(pt2);
                // mutiLines[idx-1]    =pt1;
                // mutiLines[idx]  =pt2;
                mutiLines[idx-1]=GeometryRelation::ProjectionPoint(mutiLines[idx-1],line);
                mutiLines[idx]=GeometryRelation::ProjectionPoint(mutiLines[idx],line);
            }
 
        }
        else if(type==3){
            // x=k1*y+b1
            // z=k2*y+b2
            double x1 = k1*nearestPoints[idminy].x+b1;
            double z1 = k2*nearestPoints[idminy].x+b2;

            double x2 = k1*nearestPoints[idmaxy].x+b1;
            double z2 = k2*nearestPoints[idmaxy].x+b2;

            Point3D pt1,pt2;
            if(mutiLines[idx-1].y<mutiLines[idx].y)
            {
                pt1.x = x1+cx;
                pt1.y = miny+cy;
                pt1.z = z1+cz;

                pt2.x = x2+cx;
                pt2.y = maxy+cy;
                pt2.z = z2+cz;
                Point3Ds line;
                line.push_back(pt1);
                line.push_back(pt2);
                
                // mutiLines[idx]    = pt1;
                // mutiLines[idx-1]  = pt2;
                mutiLines[idx-1]=GeometryRelation::ProjectionPoint(mutiLines[idx-1],line);
                mutiLines[idx]=GeometryRelation::ProjectionPoint(mutiLines[idx],line);
            }
            else{
                pt2.x = x1+cx;
                pt2.y = miny+cy;
                pt2.z = z1+cz;

                pt1.x = x2+cx;
                pt1.y = maxy+cy;
                pt1.z = z2+cz;
                Point3Ds line;
                line.push_back(pt1);
                line.push_back(pt2);
                // mutiLines[idx-1]    =pt1;
                // mutiLines[idx]  =pt2;
                mutiLines[idx-1]=GeometryRelation::ProjectionPoint(mutiLines[idx-1],line);
                mutiLines[idx]=GeometryRelation::ProjectionPoint(mutiLines[idx],line);
            }
        }
        return 0;
    }

    long PointCloudLineInteractiveSimple::PointCloudLineInteractive_LineMerge(vector<Point3Ds> &mutiSimpleLines,double disThreshold/*=0.3*/)
    {
        //二维点变成一维点
        Point3Ds pointlink;
        for (size_t i = 0; i < mutiSimpleLines.size(); i++)
        {
            for (size_t j = 0; j < mutiSimpleLines[i].size(); j++)
            {
                pointlink.push_back(mutiSimpleLines[i][j]);
            }
        }

        //合并距离小于阈值的点
        for(size_t i=0;i<pointlink.size();++i)
        {   
            vector<int> pointindex;
            pointindex.push_back(i);
            for(size_t j=0;j<pointlink.size();++j)
            {
                if(DistanceComputation::Distance(pointlink[i],pointlink[j])<disThreshold)
                {
                    pointindex.push_back(j);
                }
            }
            
            Point3D pt;
            pt.x=pt.y=pt.z=0;
            for(int j=0;j<pointindex.size();++j)
            {
                pt.x+=pointlink[pointindex[j]].x/double(pointindex.size());
                pt.y+=pointlink[pointindex[j]].y/double(pointindex.size());
                pt.z+=pointlink[pointindex[j]].z/double(pointindex.size());
            }

            for(int j=0;j<pointindex.size();++j)
            {
                pointlink[pointindex[j]].x=pt.x;
                pointlink[pointindex[j]].y=pt.y;
                pointlink[pointindex[j]].z=pt.z;
            }

        }

        // 恢复为二维点，这样还是有问题
        // 如果点在直线上则可能出现连不上的情况
        // 另外如果每一条直线都是一个对象则可能对象太多，需要通过拓扑关系减少对象数量
        int countParam=0;
        for (size_t i = 0; i < mutiSimpleLines.size(); i++)
        {
            for (size_t j = 0; j < mutiSimpleLines[i].size(); j++)
            {
                mutiSimpleLines[i][j]=pointlink[countParam];
                countParam++;
            }
        }


        // for(size_t i = 0; i < mutiSimpleLines.size(); i++)
        // {   
        //     for(size_t j=0;j<mutiSimpleLines.size();++j)
        //     {
        //         if(i==j){
        //             continue;
        //         }else{
                    
        //         }
        //     }
        // }
        
    }

    long PointCloudLineInteractiveSimple::PointCloudLineInteractive_Trans2Simple(vector<Point3Ds> mutiComplexLines,vector<Point3Ds> &mutiSimpleLines)
    {
        mutiSimpleLines.clear();
        for (size_t i = 0; i < mutiComplexLines.size(); i++)
        {
            for(size_t j=0; j<mutiComplexLines[i].size()-1;++j)
            {
                Point3Ds line;
                line.push_back(mutiComplexLines[i][j]);
                line.push_back(mutiComplexLines[i][j+1]);
                mutiSimpleLines.push_back(line);
            }
        }
        return 0;
    }

    long PointCloudLineInteractiveSimple::PointCloudLineInteractive_Trans2Point(vector<Point3Ds> mutiComplexLines,Point3Ds &points)
    {
        points.clear();
        for (size_t i = 0; i < mutiComplexLines.size(); i++)
        {
            for(size_t j=0; j<mutiComplexLines[i].size();++j)
            {
                points.push_back(mutiComplexLines[i][j]);
            }
        }
        return 0;
    }

    long PointCloudLineInteractiveSimple::PointCloudLineInteractive_ModelRefine(ILASDataset *dataset,vector<Point3Ds> &featureLines)
    {

        vector<Point3Ds> simpleLine;
        Point3Ds points;

        //// 根据绘制得线进行拟合
        // PointCloudLineInteractive_Trans2Simple(featureLines,simpleLine);

        //根据点先check线，然后进行拟合
        PointCloudLineInteractive_Trans2Point(featureLines,points);
        PointCloudLineInteractive_LineGet(dataset,0.5,50,points,simpleLine);


        for(int i=0;i<simpleLine.size();++i)
        {
            Point3Ds innerPoints,nearestPoints;
            PointCloudLineInteractive_GetPointsRange(dataset,0.5,simpleLine[i],innerPoints);
            PointCloudLineInteractive_FindNearestLinePoitns(innerPoints,nearestPoints,simpleLine[i],1);
            PointCloudLineInteractive_LineFitOnce(nearestPoints,simpleLine[i],1);
        }
        PointCloudLineInteractive_LineMerge(simpleLine,2);


        featureLines.swap(simpleLine);
        return 0;
    }
}