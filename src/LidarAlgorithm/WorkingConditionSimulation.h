#pragma once
#ifndef _WORKING_CONDITION_SIMULATION_H_
#define _WORKING_CONDITION_SIMULATION_H_

#include <vector>
#include"Geometry.h"
//#include "../LidarApplication/LASVectorClassify.h"
using namespace GeometryLas;

namespace LasAlgorithm {

#pragma region ���߲���

	struct WireParam
	{
		double wire_external_D;                  //�����⾶(m)
		double E1;                               //����ɢ��ϵ��
		double alphaS;                            //���߱��������ϵ��
		double R20;                              //�����¶�Ϊ20 ��ʱ��ֱ������(��/m)
		double alpha20;                          //20 ��ʱ�ĵ����¶�ϵ��(��C1)
		int number_of_aluminum_layers;           //��о�����ߵ�������(��)
		double steel_core_d;                     //��оֱ��(m)
		double f;                                //��Դ��Ƶ��(Hz)
		double A;                                //���߽����(mm2)
		double maxTemper;                        //�����������¶�(��)
		//double maxI;                             //��ǰ�����µ�������������(A)

		double unitWeight;                        //���ߵ�λ����(kg/m)
		double shapeFactor;                       //��������ϵ��

		double E;                                 //���ߵĵ���ϵ����N/mm2��
		double alpha;                             //���ߵ��¶��쳤ϵ����1/C����
		double insulatorLength;                   //��Ե�Ӵ����ȣ�m��
		double insulatorArea;                     //��Ե�Ӵ��ܷ����(m2)
		double insulatorWeight;                   //��Ե�Ӵ�����(kg)
		double hammerWeight;                      //�ش�����(kg)
	};

#pragma endregion

#pragma region ��������

	struct EnvironmentParam
	{
		double theltaA;                           //��ǰ�����¶�(��)
		double vertical_wind_speed;               //��ֱ�ڵ��ߵķ���(m/s)	
		double Js;                                //�չ�Ե��ߵ�����ǿ��(W/m2)

		double iceThickness;                      //�������(mm)
		double windSpeedInAverageAltitude;        //����ƽ���߶ȴ��ķ��٣�m/s��
		double windPressureFactor;                //���߷�ѹ������ϵ��
		int isWindDeviate;                        //�Ƿ��Ƿ�ƫ��0����1����
	};

#pragma endregion

#pragma region ������Ϣ����

	class WireInformation
	{
	public:
		double thelta;                            //����(��)
		double I;                                 //������(A)
		double Wr;                                //����ɢ�ȹ���(W/m)
		double Wf;                                //����ɢ�ȹ���(W/m)
		double Ws;                                //�������ȹ���(W/m)
		double Rdc;                               //���ߵ�ֱ������(��/m)
		double K1;                                //����ЧӦϵ��
		double K2;                                //����ϵ�� K1*K2 Ϊ��ֱ�������

		double gamma;                             //���߱���(N/(m*mm2))
		double wireStress;                        //����ˮƽӦ��(N/mm2)

		WireParam wireParam;                      //���߲���
		EnvironmentParam environmentParam;        //��������    
		int calculateType;                        //�������ͣ�0��������������1����������

		WireInformation();
		/**
		* ���캯��,�趨����������
		* @param wParam  ���߲���
		* @param eParam  ��������
		* @param knownQuantity  ��֪��
		* @param calType �������ͣ�0��������������1����������
		* @return
		*/
		WireInformation(WireParam wParam, EnvironmentParam eParam, double knownQuantity, int calType);

		/**
		* �������ɢ�ȹ���,Ĭ�������̶�
		* @return
		*/
		void CalWr();

		/**
		* �������ɢ�ȹ���,������������
		* @param tmpThelta  ����
		* @return tmpWr ������������õ��ķ���ɢ�ȹ���
		*/
		double CalWr(double tmpThelta);

		/**
		* �������ɢ�ȹ���
		* @return
		*/
		void CalWf();

		/**
		* �������ɢ�ȹ���,������������
		* @param tmpThelta  ����
		* @return tmpWf ������������õ��Ķ���ɢ�ȹ���
		*/
		double CalWf(double tmpThelta);

		/**
		* �����������ȹ���
		* @return
		*/
		void CalWs();

		/**
		* ���㵼�ߵ�ֱ������,Ĭ�������̶�
		* @return
		*/
		void CalRdc();

		/**
		* ���㵼�ߵ�ֱ������,������������
		* @param tmpThelta  ����
		* @return tmpRdc ������������õ���ֱ������
		*/
		double CalRdc(double tmpThelta);

		/**
		* ���㵼�ߵļ���ЧӦϵ��,Ĭ�������̶�
		* @return
		*/
		void CalK1();

		/**
		* ���㵼�ߵļ���ЧӦϵ��,������������
		* @param tmpThelta  ����
		* @return tmpK1 ������������õ��ļ���ЧӦϵ��
		*/
		double CalK1(double tmpThelta);

		/**
		* ���㵼�ߵ�����ϵ��
		* @return
		*/
		void CalK2();

		/**
		* ���㵼�ߵ�������
		* @param I0  ��������ʼֵ
		* @param tolerance  ���������������ֵ
		* @return
		*/
		void CalI(double I0, double tolerance);

		/**
		* ���㵼�ߵ�������,��������
		* @param I0  ��������ʼֵ
		* @param tolerance  ���������������ֵ
		* @return
		*/
		bool CalIProgram(double I0, double tolerance);

		/**
		* ���㵼�ߵ�����
		* @param thelta0  ������ʼֵ
		* @param tolerance  �������������ֵ
		* @return
		*/
		void CalThelta(double thelta0, double tolerance);

		/**
		* ���㵼�ߵ�����,��������
		* @param I0  ������ʼֵ
		* @param tolerance  �������������ֵ
		* @return
		*/
		bool CalTheltaProgram(double I0, double tolerance);

		/**
		* ���㵼�߱���,���ص��߷�ƫ��
		* @return
		*/
		double CalGamma();

		/**
		* ���㵼������ϵ��
		* @param wireExternalD  �����⾶(m)
		* @param iceThick  �������(mm)
		* @return
		*/
		static double CalShapeFactor(double wireExternalD, double iceThick);

		/**
		* ���㵼�߷�ѹ������ϵ��;
		* @param windSpeed  ���ߵķ���(m/s)
		* @return
		*/
		static double CalWindPressureFactor(double windSpeed);

		/**
		* �����Ե�ӷ�ƫ��
		* @param hTowerSpan  ����ˮƽ����(m)
		* @param vTowerSpan  ������ֱ����(m)
		* @return
		*/
		double CalInsulatorWindDefAngle(double hTowerSpan, double vTowerSpan);
	};

#pragma endregion

#pragma region ����ģ�ͼ���

	class WireModel
	{
	public:
		double towerSpan;                            //����(m)
		double representSpan;                        //������(m)
		double elevationAngle;                       //�߲��
		double wireSag;                              //����
		Point3D hangPointA;                          //���ҵ�A(С�����߶˵�)
		Point3D hangPointB;                          //���ҵ�B(������߶˵�)
		vector <double> planeLineFactors;            //ƽ��ֱ�߷���
		vector <double> sectionLineFactors;          //���������߷���
		Point2D oFootPoint;                          //ԭ�㵽��άֱ�ߵĴ���(����ԭ��)
		int sectionLineType;                         //0:z-s(�߶�����),1��z-sqrt(x2 + y2)

		//double k;                                    //б�����������ߵ���ģ�ͣ�y = k * x2, k = gamma / (2 * wireStress * cos (elevationAngle))
		//double kx;                                   //kx = tanf(elevationAngle) - (gamma * towerSpan) / (2 * wireStress * cos (elevationAngle))
		//double kx2;                                  //kx2 = gamma / (2 * wireStress * cos (elevationAngle)) ,y = kx * x + kx2 * x2�������ҵ�AΪO��
		int insulatorType;                             //��Ե���ͺţ�0����������1�����Ŵ�
		int splitNum;                                  //���߷�����
		double splitDistance;                          //���߷��Ѽ��

		double wireWindDefAngle;                       //���߷�ƫ��
		double insulatorWindDefAngle;                  //��Ե�ӷ�ƫ��
		double insulatorLength;                        //��Ե�Ӵ����ȣ�m��

		bool isWindDeviate;                            //�Ƿ��Ƿ�ƫ

		WireModel();

		/**
		* ����Ӧ���ͱ��ؼ��������߷���ʽ(�޷�ƫ)
		* @param lineModel     �޷�ƫ����ģ��
		* @param lineWeight    ���߱���
		* @param lineStress    ����Ӧ��
		* @return
		*/
		void CalUnKnownLineModel(WireModel lineModel, double lineWeight, double lineStress);

		/**
		* ��ʼ������ģ����Ϣ
		* @param lineModel    ��֪����ģ��
		* @return
		*/
		void GetModelInfo(WireModel lineModel);

		/**
		* �����ƫ��x,y,zƫ�����
		* @param spanX      ����һ�������ˮƽ����
		* @param curvePoint ��ֱ֪���ϵĵ�
		* @param calType    �������ͣ�0:��Ե�Ӻ͵��߷�ƫ������,1:ֻ���ǵ��߷�ƫ,2:ֻ���Ǿ�Ե�ӷ�ƫ
		* @param outType    ������ͣ�0:���ҷ�ƫ�㶼���,1:ֻ������ƫ��,2:ֻ����ҷ�ƫ��
		* @return
		*/
		vector<Point3D> getWindOffsetPoint(double spanX, Point3D curvePoint, int calType = 0, int outType = 0);
	};

#pragma endregion

#pragma region �������Ŷ�
	struct TensionSegment
	{
		vector <Point2D> tsTowerPoints;            //���Ŷεĸ�������,ʼ������Ϊ������,�м�Ϊֱ����,ʼ���Ǵ�һ������������һ��������
		vector <int> linePointsIndex;              //ÿ�����ߵ��Ƶ��±�,���������뵼�ߵ��ƶ�Ӧ
		vector <WireModel> tsLineModel;            //���Ŷ��е����е���ģ��
		vector <WireModel> simLineModel;           //���Ŷ��е����е���ģ��ģ��
		vector <vector <int>> wholeLineIndex;      //һ�������ߵ����е���ģ��
												   //double representSpan;                      //������(m)
		long turnNum;                              //���ŶεĻ�·��
	};
#pragma endregion

#pragma region ���߹���ģ��

	class WireSimulation
	{
	public:
		WireInformation knownWireInfo;                    //��֪������Ϣ
		WireModel knownWireModel;                         //��֪����ģ��
		WireInformation unKnownWireInfo;                  //��������Ϣ
		WireModel unKnownWireModel;                       //������ģ��

		std::vector <WireModel> tensionSegment;           //�������Ŷ�		

		WireSimulation();

		/**
		* ���캯��,�趨����������
		* @param knWireInfo    ��֪������Ϣ
		* @param knWireModel   ��֪����ģ��
		* @param unKnWireInfo  ��������Ϣ
		* @param unKnWireModel ������ģ��
		* @param tnSegment     �������Ŷ�
		* @return
		*/
		WireSimulation(WireInformation knWireInfo, WireModel knWireModel, WireInformation unKnWireInfo, WireModel unKnWireModel, std::vector <WireModel> tnSegment);

		/**
		* ͨ���������㵼�ߵ�Ӧ��
		* @param sag  ����
		* @param weight  ����
		* @param span    ����
		* @param angle   �߲��
		* @return Ӧ��
		*/
		double CalWireStressBySag(double sag, double weight, double span, double angle);

		/**
		* ���Ӧ�����̼��㵼�ߵ�Ӧ��
		* @param sigmaCM  ��֪����µĵ���ˮƽӦ��
		* @param angle    �߲��
		* @param E        ���ߵĵ���ϵ��
		* @param gammaM   ��֪����µĵ��߱���
		* @param span     ����
		* @param gamma    �������
		* @param alpha    ���ߵ��¶��쳤ϵ����1/C����
		* @param tM       ��֪����µĵ����¶�
		* @param t        �����¶�
		* @return ˮƽӦ��
		*/
		double CalWireStress(double sigmaCM, double angle, double E, double gammaM, double span, double gamma, double alpha, double tM, double t);

		/**
		* ���㵼�ߴ�����
		* @return ������
		*/
		double CalRepresentSpan();

		/**
		* ����ģ�����
		* @return
		*/
		void SimulationProgram(TensionSegment tensionSeg, int lineIndex);

		/**
		* ���������ˮƽ����ʹ�ֱ����
		* @param tensionSeg  ���Ŷ�ģ��
		* @param lineIndex   �ߵ��±�
		* @param wWeight     ���߱���
		* @param wStress     ����Ӧ��
		* @param hTowerSpan  ����ˮƽ����(m)
		* @param vTowerSpan  ������ֱ����(m)
		* @return
		*/
		void GetTowerHVSpan(TensionSegment tensionSeg, int lineIndex, double wWeight, double wStress, double &hTowerSpan, double &vTowerSpan);

		/**
		* ����ģ����Ϣ��ʼ��
		* @param xmlConfig �����ļ�
		* @param wParam    ������Ϣ
		* @param eParam    ��֪������Ϣ
		* @param knownI    ��֪����
		* @param unEParam  ģ�⻷����Ϣ
		* @param unKnownI  ģ�����
		* @return
		*/
		static void InitSimulationInfoFromXml(const char* xmlConfig, WireParam &wParam, EnvironmentParam &eParam, double &knownI, EnvironmentParam &unEParam, double &unKnownI);
	};

#pragma endregion

#endif
}