#ifdef _USE_PCL_
#include "LidarFeaturePoints.h"
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>


typedef pcl::PointXYZ PointType;
/**
 * @brief  重构一下取值函数，默认取值为去强度值，没有这个部分则报错：
           error: ‘const struct pcl::PointXYZ’ has no member named ‘intensity’
 * @note   
 * @retval None
 */
namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
    return p.z;
      }
    };
}

/**
 * @brief  代码运行过程会有内存泄露的问题，感觉应该是库的问题，但是不知道问题在哪里
            经过检查是RangeImage这段代码的问题，感觉跟内核有关，估计是虚拟机运行
            不支持，或者是指令集的问题
            2019-07-21:经过查看资料发现是PCL库编译的问题，存在Boost库与STD冲突的问题
            编译的时候添加C++11标准用Release编译就可以解决这个问题
 * @note   
 * @param  input_cloud:  输入点云
 * @param  &narfIndex:   特征点点云
 * @param  &narfDesc:    特征点描述
 * @retval 
 */
long LidarFeaturePoints::LidarFeature_NARF(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::PointCloud<int> &narfIndex,pcl::PointCloud<pcl::Narf36> &narfDesc)
{
    pcl::PointCloud<PointType>& point_cloud = *input_cloud;
    float angular_resolution = 0.5f;           ////angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
    float support_size = 4.0f;                 //点云大小的设置
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;     //设置坐标系
    bool setUnseenToMaxRange = false;
    bool rotation_invariant = true;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage());
    pcl::RangeImage& range_image = *range_image_ptr;  
    range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges); 

    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange ();

    /*提取特征点*/
    pcl::RangeImageBorderExtractor range_image_border_extractor;   //用来提取边缘
    pcl::NarfKeypoint narf_keypoint_detector;      //用来检测关键点
    narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);   //
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;    //设置NARF的参数
    narf_keypoint_detector.compute (narfIndex);

    /*提取特征描述*/
    std::vector<int> keypoint_indices2;
    keypoint_indices2.resize (narfIndex.points.size ());
    for (unsigned int i=0; i<narfIndex.size (); ++i) // This step is necessary to get the right vector type
        keypoint_indices2[i]=narfIndex.points[i];
    pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
    narf_descriptor.getParameters ().support_size = support_size;
    narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
    narf_descriptor.compute (narfDesc);
    printf("feature points count:%d\n",narfIndex.size());
    return 0;
}

long LidarFeaturePoints::LidarFeature_Sift(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                           pcl::PointCloud<int> &siftPointIdx,
                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr siftFPFH)
{
    // Parameters for sift computation
    float min_scale = 0.5f;       //the standard deviation of the smallest scale in the scale space
    int n_octaves   = 10;        //the number of octaves (i.e. doublings of scale) to compute
    int n_scales_per_octave = 4;//the number of scales to compute within each octave
    float min_contrast = 0.2f;  //the minimum contrast required for detection

    pcl::console::TicToc time;
    time.tic();

    // Estimate the sift interest points using z values from xyz as the Intensity variants
    //pcl::PointCloud<pcl::PointXYZ>::Ptr feauture_cloud
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());

    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(input_cloud);
    sift.compute(result);

    //pcl::PointIndicesConstPtr keypoints_indices = sift.getKeypointsIndices();
    //计算点云特征
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> est_normal;
    est_normal.setInputCloud(input_cloud);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(10);          //近邻10个点
    est_normal.compute(*normals);       //计算法线

    //创建FPFH估计对象fpfh，并把输入数据集cloud和法线normals传递给它。
    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeFPFH(new pcl::search::KdTree<pcl::PointXYZ> ());
    fpfh.setInputCloud(input_cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(treeFPFH);
    //fpfh.setIndices(keypoints_indices);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh.setKSearch(10);        //近邻10个点
    fpfh.compute(*fpfhs);       //计算特征
    
    //判断SIFT点在原始点云中的index
    const int searchNum=1;
    std::vector<int> pointIdxNKNSearch(searchNum);
    std::vector<float> pointNKNSquaredDistance(searchNum);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud);
    for(auto resPoint : result.points)
    {
        pcl::PointXYZ searchPoint;
        searchPoint.x=resPoint.x;
        searchPoint.y=resPoint.y;
        searchPoint.z=resPoint.z;

        if (kdtree.nearestKSearch(searchPoint, searchNum, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            siftPointIdx.push_back(pointIdxNKNSearch[0]);
            siftFPFH->push_back(fpfhs->points[pointIdxNKNSearch[0]]);
        }
    }

    std::cout << "Computing Points: "<<input_cloud->size()<<std::endl;
    std::cout << "Computing FPFH Features: "<<siftFPFH->size()<<std::endl;
    std::cout << "Computing the SIFT points takes "<<time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "No. of SIFT points in the result are " << siftPointIdx.size () << std::endl;

/*  std::cout << "Computing Points: "<<input_cloud->size()<<std::endl;
    std::cout << "Computing FPFH Features: "<<fpfhs->size()<<std::endl;
    std::cout << "Computing Normals: "<<normals->size()<<std::endl;
    std::cout << "Computing the SIFT points takes "<<time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "No. of SIFT points in the result are " << result.points.size () << std::endl;
 */
    // Copying the pointwithscale to pointxyz so as visualize the cloud
    //copyPointCloud(result, *feauture_cloud);
    //std::cout << "SIFT points in the result are " << feauture_cloud->points.size () << std::endl;

#ifdef _DEBUG
    FILE *fs = fopen("../data/test/sift.txt","w+");
    for(int i=0;i<siftPointIdx.points.size ();++i)
        fprintf(fs,"%lf  %lf  %lf\n",input_cloud->points[siftPointIdx[i]].x,
                                     input_cloud->points[siftPointIdx[i]].y,
                                     input_cloud->points[siftPointIdx[i]].z);
    fclose(fs);
#endif

    return 0;
}


#endif