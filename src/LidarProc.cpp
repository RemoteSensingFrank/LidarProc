// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include<stdio.h>
#include<gtest/gtest.h>
#include "./LidarAlgorithm/LASProfile.h"

int main(int argc ,char* argv[])
{

    LASProfile profile;
    ProfileDecorate decorateParam;
    decorateParam.lbType = LABEL_SQUARE;

    Point2D pntTower[2];
    pntTower[0].x = /* 204742.7500 */182716.690002;
    pntTower[0].y = /* 2477649.12 */2481004.000000;
	

    pntTower[1].x = /* 204781.66 */183023.250000;
    pntTower[1].y = /* 2477626.81 */2480775.869999;


	//根据大小计算需要的分辨率，控制图像大小
	double xscale = fabs(pntTower[0].x-pntTower[1].x)/600;
	double yscale = fabs(pntTower[0].y -pntTower[1].y)/800;
	double scale = min(xscale,yscale);
	double hspan = max(fabs(pntTower[0].x-pntTower[1].x)/10,fabs(pntTower[0].y -pntTower[1].y)/10);

    decorateParam.hspan_dis = hspan;
    decorateParam.vspan_dis = 10;
    decorateParam.labelPnts.push_back(Point3D(182766.440002,2480992.250000,66.100002));

    profile.LASProfile_Verticle("../data/default/color.las",pntTower,20,scale,"../data/profile/v.jpg",&decorateParam);
    profile.LASProfile_Horizontal("../data/default/color.las",pntTower,20,scale,"../data/profile/h.jpg",&decorateParam);
    profile.LASProfile_Front("../data/default/color.las",pntTower,20,scale,"../data/profile/f.jpg",&decorateParam);

/*  testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();  */
}

