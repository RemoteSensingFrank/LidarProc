#pragma once
#ifndef _WORKING_CONDITION_SIMULATION_H_
#define _WORKING_CONDITION_SIMULATION_H_

#include <vector>
#include"Geometry.h"
//#include "../LidarApplication/LASVectorClassify.h"
using namespace GeometryLas;

namespace LasAlgorithm {

#pragma region 导线参数

	struct WireParam
	{
		double wire_external_D;                  //导线外径(m)
		double E1;                               //辐射散热系数
		double alphaS;                            //导线表面的吸热系数
		double R20;                              //导线温度为20 ℃时的直流电阻(Ω/m)
		double alpha20;                          //20 ℃时的电阻温度系数(℃–1)
		int number_of_aluminum_layers;           //钢芯铝绞线的铝层数(层)
		double steel_core_d;                     //钢芯直径(m)
		double f;                                //电源的频率(Hz)
		double A;                                //导线截面积(mm2)
		double maxTemper;                        //导线设计最大温度(℃)
		//double maxI;                             //当前工况下的设计最大载流量(A)

		double unitWeight;                        //电线单位质量(kg/m)
		double shapeFactor;                       //导线体型系数

		double E;                                 //导线的弹性系数（N/mm2）
		double alpha;                             //导线的温度伸长系数（1/C。）
		double insulatorLength;                   //绝缘子串长度（m）
		double insulatorArea;                     //绝缘子串受风面积(m2)
		double insulatorWeight;                   //绝缘子串重量(kg)
		double hammerWeight;                      //重锤重量(kg)
	};

#pragma endregion

#pragma region 环境参数

	struct EnvironmentParam
	{
		double theltaA;                           //当前环境温度(℃)
		double vertical_wind_speed;               //垂直于导线的风速(m/s)	
		double Js;                                //日光对导线的日照强度(W/m2)

		double iceThickness;                      //覆冰厚度(mm)
		double windSpeedInAverageAltitude;        //导线平均高度处的风速（m/s）
		double windPressureFactor;                //导线风压不均匀系数
		int isWindDeviate;                        //是否考虑风偏，0：否，1：是
	};

#pragma endregion

#pragma region 导线信息计算

	class WireInformation
	{
	public:
		double thelta;                            //温升(℃)
		double I;                                 //载流量(A)
		double Wr;                                //辐射散热功率(W/m)
		double Wf;                                //对流散热功率(W/m)
		double Ws;                                //日照吸热功率(W/m)
		double Rdc;                               //导线的直流电阻(Ω/m)
		double K1;                                //集肤效应系数
		double K2;                                //铁损系数 K1*K2 为交直流电阻比

		double gamma;                             //导线比载(N/(m*mm2))
		double wireStress;                        //导线水平应力(N/mm2)

		WireParam wireParam;                      //导线参数
		EnvironmentParam environmentParam;        //环境参数    
		int calculateType;                        //计算类型，0：正算载流量，1：反算温升

		WireInformation();
		/**
		* 构造函数,设定正反算类型
		* @param wParam  导线参数
		* @param eParam  环境参数
		* @param knownQuantity  已知量
		* @param calType 计算类型，0：正算载流量，1：反算温升
		* @return
		*/
		WireInformation(WireParam wParam, EnvironmentParam eParam, double knownQuantity, int calType);

		/**
		* 计算辐射散热功率,默认温升固定
		* @return
		*/
		void CalWr();

		/**
		* 计算辐射散热功率,根据温升计算
		* @param tmpThelta  温升
		* @return tmpWr 根据温升计算得到的辐射散热功率
		*/
		double CalWr(double tmpThelta);

		/**
		* 计算对流散热功率
		* @return
		*/
		void CalWf();

		/**
		* 计算对流散热功率,根据温升计算
		* @param tmpThelta  温升
		* @return tmpWf 根据温升计算得到的对流散热功率
		*/
		double CalWf(double tmpThelta);

		/**
		* 计算日照吸热功率
		* @return
		*/
		void CalWs();

		/**
		* 计算导线的直流电阻,默认温升固定
		* @return
		*/
		void CalRdc();

		/**
		* 计算导线的直流电阻,根据温升计算
		* @param tmpThelta  温升
		* @return tmpRdc 根据温升计算得到的直流电阻
		*/
		double CalRdc(double tmpThelta);

		/**
		* 计算导线的集肤效应系数,默认温升固定
		* @return
		*/
		void CalK1();

		/**
		* 计算导线的集肤效应系数,根据温升计算
		* @param tmpThelta  温升
		* @return tmpK1 根据温升计算得到的集肤效应系数
		*/
		double CalK1(double tmpThelta);

		/**
		* 计算导线的铁损系数
		* @return
		*/
		void CalK2();

		/**
		* 计算导线的载流量
		* @param I0  载流量初始值
		* @param tolerance  载流量迭代误差限值
		* @return
		*/
		void CalI(double I0, double tolerance);

		/**
		* 计算导线的载流量,整体流程
		* @param I0  载流量初始值
		* @param tolerance  载流量迭代误差限值
		* @return
		*/
		bool CalIProgram(double I0, double tolerance);

		/**
		* 计算导线的温升
		* @param thelta0  温升初始值
		* @param tolerance  温升迭代误差限值
		* @return
		*/
		void CalThelta(double thelta0, double tolerance);

		/**
		* 计算导线的温升,整体流程
		* @param I0  温升初始值
		* @param tolerance  温升迭代误差限值
		* @return
		*/
		bool CalTheltaProgram(double I0, double tolerance);

		/**
		* 计算导线比载,返回导线风偏角
		* @return
		*/
		double CalGamma();

		/**
		* 计算导线体型系数
		* @param wireExternalD  导线外径(m)
		* @param iceThick  覆冰厚度(mm)
		* @return
		*/
		static double CalShapeFactor(double wireExternalD, double iceThick);

		/**
		* 计算导线风压不均匀系数;
		* @param windSpeed  导线的风速(m/s)
		* @return
		*/
		static double CalWindPressureFactor(double windSpeed);

		/**
		* 计算绝缘子风偏角
		* @param hTowerSpan  杆塔水平档距(m)
		* @param vTowerSpan  杆塔垂直档距(m)
		* @return
		*/
		double CalInsulatorWindDefAngle(double hTowerSpan, double vTowerSpan);
	};

#pragma endregion

#pragma region 导线模型计算

	class WireModel
	{
	public:
		double towerSpan;                            //档距(m)
		double representSpan;                        //代表档距(m)
		double elevationAngle;                       //高差角
		double wireSag;                              //弧垂
		Point3D hangPointA;                          //悬挂点A(小比例尺端点)
		Point3D hangPointB;                          //悬挂点B(大比例尺端点)
		vector <double> planeLineFactors;            //平面直线方程
		vector <double> sectionLineFactors;          //剖面抛物线方程
		Point2D oFootPoint;                          //原点到二维直线的垂足(坐标原点)
		int sectionLineType;                         //0:z-s(尺度因子),1：z-sqrt(x2 + y2)

		//double k;                                    //斜抛物线悬链线导线模型，y = k * x2, k = gamma / (2 * wireStress * cos (elevationAngle))
		//double kx;                                   //kx = tanf(elevationAngle) - (gamma * towerSpan) / (2 * wireStress * cos (elevationAngle))
		//double kx2;                                  //kx2 = gamma / (2 * wireStress * cos (elevationAngle)) ,y = kx * x + kx2 * x2，以悬挂点A为O点
		int insulatorType;                             //绝缘子型号，0：悬垂串，1：耐张串
		int splitNum;                                  //导线分裂数
		double splitDistance;                          //导线分裂间距

		double wireWindDefAngle;                       //导线风偏角
		double insulatorWindDefAngle;                  //绝缘子风偏角
		double insulatorLength;                        //绝缘子串长度（m）

		bool isWindDeviate;                            //是否考虑风偏

		WireModel();

		/**
		* 根据应力和比载计算悬链线方程式(无风偏)
		* @param lineModel     无风偏导线模型
		* @param lineWeight    导线比载
		* @param lineStress    导线应力
		* @return
		*/
		void CalUnKnownLineModel(WireModel lineModel, double lineWeight, double lineStress);

		/**
		* 初始化导线模型信息
		* @param lineModel    已知导线模型
		* @return
		*/
		void GetModelInfo(WireModel lineModel);

		/**
		* 计算风偏的x,y,z偏离距离
		* @param spanX      距离一侧杆塔的水平距离
		* @param curvePoint 已知直线上的点
		* @param calType    计算类型：0:绝缘子和导线风偏都考虑,1:只考虑导线风偏,2:只考虑绝缘子风偏
		* @param outType    输出类型：0:左右风偏点都输出,1:只输出左风偏点,2:只输出右风偏点
		* @return
		*/
		vector<Point3D> getWindOffsetPoint(double spanX, Point3D curvePoint, int calType = 0, int outType = 0);
	};

#pragma endregion

#pragma region 杆塔耐张段
	struct TensionSegment
	{
		vector <Point2D> tsTowerPoints;            //耐张段的杆塔坐标,始终两塔为耐张塔,中间为直线塔,始终是从一个耐张塔到另一个耐张塔
		vector <int> linePointsIndex;              //每档导线点云的下标,将杆塔段与导线点云对应
		vector <WireModel> tsLineModel;            //耐张段中的所有导线模型
		vector <WireModel> simLineModel;           //耐张段中的所有导线模拟模型
		vector <vector <int>> wholeLineIndex;      //一整条导线的所有导线模型
												   //double representSpan;                      //代表档距(m)
		long turnNum;                              //耐张段的回路数
	};
#pragma endregion

#pragma region 导线工况模拟

	class WireSimulation
	{
	public:
		WireInformation knownWireInfo;                    //已知导线信息
		WireModel knownWireModel;                         //已知导线模型
		WireInformation unKnownWireInfo;                  //待求导线信息
		WireModel unKnownWireModel;                       //待求导线模型

		std::vector <WireModel> tensionSegment;           //所在耐张段		

		WireSimulation();

		/**
		* 构造函数,设定正反算类型
		* @param knWireInfo    已知导线信息
		* @param knWireModel   已知导线模型
		* @param unKnWireInfo  待求导线信息
		* @param unKnWireModel 待求导线模型
		* @param tnSegment     所在耐张段
		* @return
		*/
		WireSimulation(WireInformation knWireInfo, WireModel knWireModel, WireInformation unKnWireInfo, WireModel unKnWireModel, std::vector <WireModel> tnSegment);

		/**
		* 通过弧垂反算导线的应力
		* @param sag  弧垂
		* @param weight  比载
		* @param span    档距
		* @param angle   高差角
		* @return 应力
		*/
		double CalWireStressBySag(double sag, double weight, double span, double angle);

		/**
		* 求解应力方程计算导线的应力
		* @param sigmaCM  已知情况下的电线水平应力
		* @param angle    高差角
		* @param E        电线的弹性系数
		* @param gammaM   已知情况下的电线比载
		* @param span     档距
		* @param gamma    待求比载
		* @param alpha    电线的温度伸长系数（1/C。）
		* @param tM       已知情况下的电线温度
		* @param t        待求温度
		* @return 水平应力
		*/
		double CalWireStress(double sigmaCM, double angle, double E, double gammaM, double span, double gamma, double alpha, double tM, double t);

		/**
		* 计算导线代表档距
		* @return 代表档距
		*/
		double CalRepresentSpan();

		/**
		* 导线模拟程序
		* @return
		*/
		void SimulationProgram(TensionSegment tensionSeg, int lineIndex);

		/**
		* 计算杆塔的水平档距和垂直档距
		* @param tensionSeg  耐张段模型
		* @param lineIndex   线的下标
		* @param wWeight     导线比载
		* @param wStress     导线应力
		* @param hTowerSpan  杆塔水平档距(m)
		* @param vTowerSpan  杆塔垂直档距(m)
		* @return
		*/
		void GetTowerHVSpan(TensionSegment tensionSeg, int lineIndex, double wWeight, double wStress, double &hTowerSpan, double &vTowerSpan);

		/**
		* 导线模拟信息初始化
		* @param xmlConfig 配置文件
		* @param wParam    导线信息
		* @param eParam    已知环境信息
		* @param knownI    已知电流
		* @param unEParam  模拟环境信息
		* @param unKnownI  模拟电流
		* @return
		*/
		static void InitSimulationInfoFromXml(const char* xmlConfig, WireParam &wParam, EnvironmentParam &eParam, double &knownI, EnvironmentParam &unEParam, double &unKnownI);
	};

#pragma endregion

#endif
}