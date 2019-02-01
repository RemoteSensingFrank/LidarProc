#include "WorkingConditionSimulation.h"
#include <cmath>
#include <vector>
#include "../Utility/tinyxml2.h"
using namespace tinyxml2;

namespace LasAlgorithm {

#pragma region 常量定义

#ifndef Pi
#define Pi 3.14159265                       //π
#endif // !Pi

#define StefanBoundConstant 5.67e-8         //斯特凡- 包尔茨曼常数(W/m2) S1

#define GravitationalAcceleration 9.80665   //重力加速度(m/s2)
#define AirDensity 1.25                     //空气密度(kg/m3)

#pragma endregion


#pragma region 导线温度载流量正反算


const double Tolearance = 1e-5;   //迭代误差
const double InitThelta = 50;     //初始温度
const double InitEpsino = 1.5;    //初始载流量，double I0 = InitEpsino * wParam.A


	WireInformation::WireInformation()
	{

	}

	WireInformation::WireInformation(WireParam wParam, EnvironmentParam eParam, double knownQuantity, int calType)
	{
		wireParam = wParam;
		environmentParam = eParam;
		calculateType = calType;
		if (calculateType == 0)
		{
			thelta = knownQuantity;
		}
		else if (calculateType == 1)
		{
			I = knownQuantity;
		}
	}

	void WireInformation::CalWr()
	{
		Wr = Pi * wireParam.wire_external_D * wireParam.E1 * StefanBoundConstant *
			(pow((environmentParam.theltaA + thelta + 273), 4) -
				pow((environmentParam.theltaA + 273), 4));
	}

	double WireInformation::CalWr(double tmpThelta)
	{
		double tmpWr = Pi * wireParam.wire_external_D * wireParam.E1 * StefanBoundConstant *
			(pow((environmentParam.theltaA + tmpThelta + 273), 4) -
				pow((environmentParam.theltaA + 273), 4));
		return tmpWr;
	}

	void WireInformation::CalWf()
	{
		double numdaF = 2.42e-2 + 7 * (environmentParam.theltaA + thelta / 2.0) * 1e-5;
		double v = 1.32e-5 + 9.6 * (environmentParam.theltaA + thelta / 2.0) * 1e-8;  //1.32e-5 + 7 * (environmentParam.theltaA + thelta / 2.0) * 1e-5
		double Re = environmentParam.vertical_wind_speed * wireParam.wire_external_D / v;
		Wf = 0.57 * Pi * numdaF * thelta * pow(Re, 0.485);
	}

	double WireInformation::CalWf(double tmpThelta)
	{
		double numdaF = 2.42e-2 + 7 * (environmentParam.theltaA + tmpThelta / 2.0) * 1e-5;
		double v = 1.32e-5 + 9.6 * (environmentParam.theltaA + tmpThelta / 2.0) * 1e-8;
		double Re = environmentParam.vertical_wind_speed * wireParam.wire_external_D / v;
		double tmpWf = 0.57 * Pi * numdaF * tmpThelta * pow(Re, 0.485);
		return tmpWf;
	}

	void WireInformation::CalWs()
	{
		Ws = environmentParam.Js * wireParam.alphaS * wireParam.wire_external_D;
	}

	void WireInformation::CalRdc()
	{
		Rdc = wireParam.R20 * (1 + wireParam.alpha20 * (environmentParam.theltaA + thelta - 20));
	}

	double WireInformation::CalRdc(double tmpThelta)
	{
		double tmpRdc = wireParam.R20 * (1 + wireParam.alpha20 * (environmentParam.theltaA + tmpThelta - 20));
		return tmpRdc;
	}

	void WireInformation::CalK1()
	{
		double x = ((wireParam.wire_external_D + wireParam.steel_core_d * 2) / (wireParam.wire_external_D + wireParam.steel_core_d)) * 0.01 *
			sqrt((8 * Pi * wireParam.f * (wireParam.wire_external_D - wireParam.steel_core_d)) / ((wireParam.wire_external_D + wireParam.steel_core_d) * 1000 * Rdc));
		K1 = 0.99609 + 0.018578 * x - 0.030263 * pow(x, 2) + 0.020735 * pow(x, 3);
	}

	double WireInformation::CalK1(double tmpThelta)
	{
		double x = ((wireParam.wire_external_D + wireParam.steel_core_d * 2) / (wireParam.wire_external_D + wireParam.steel_core_d)) * 0.01 *
			sqrt((8 * Pi * wireParam.f * (wireParam.wire_external_D - wireParam.steel_core_d)) / ((wireParam.wire_external_D + wireParam.steel_core_d) * 1000 * CalRdc(tmpThelta)));
		double tmpK1 = 0.99609 + 0.018578 * x - 0.030263 * pow(x, 2) + 0.020735 * pow(x, 3);
		return tmpK1;
	}

	void WireInformation::CalK2()
	{
		double epsino = I / wireParam.A;
		K2 = 0.99947 + 0.028895 * epsino - 0.0059348 * pow(epsino, 2) + 0.00042259 * pow(epsino, 3);
	}

	void WireInformation::CalI(double I0, double tolerance)
	{
		if (wireParam.number_of_aluminum_layers % 2 == 0)
		{
			double K2 = 1;
			double Rac = K1 * K2 * Rdc;
			I = sqrt(((Wr + Wf - Ws) / Rac));
		}
		else
		{
			double C1 = 0.00042259 * K1 * Rdc / pow(wireParam.A, 3);
			double C2 = -0.0059348 * K1 * Rdc / pow(wireParam.A, 2);
			double C3 = 0.028895 * K1 * Rdc / wireParam.A;
			double C4 = 0.99947 * K1 * Rdc;
			double B = Wr + Wf - Ws;

			double In, In1, FI, FId;
			In1 = I0;
			do
			{
				In = In1;
				FI = C1 * pow(In, 5) + C2 * pow(In, 4) + C3 * pow(In, 3) + C4 * pow(In, 2) - B;
				FId = 5 * C1 * pow(In, 4) + 4 * C2 * pow(In, 3) + 3 * C3 * pow(In, 2) + 2 * C4 * In;
				In1 = In - FI / FId;
			} while (abs(FI) >= tolerance);

			I = In;
		}
	}

	bool WireInformation::CalIProgram(double I0, double tolerance)
	{
		if (environmentParam.theltaA + thelta > wireParam.maxTemper)
		{
			return false;
		}
		CalWr();
		CalWf();
		CalWs();
		if (Wr + Wf - Ws <= 0)
		{
			return false;
		}
		CalRdc();
		CalK1();
		CalI(I0, tolerance);
		CalK2();
		return true;
	}

	void WireInformation::CalThelta(double thelta0, double tolerance)
	{
		double belta2 = 1.995 * Pi * 1e-5;
		double belta3 = 0.013794 * Pi + 3.99 * Pi * environmentParam.theltaA * 1e-5;
		double belta4 = environmentParam.vertical_wind_speed * wireParam.wire_external_D + 1e-12;//+ 1e-12 防止出现nan值
		double belta5 = 4.8e-8;
		double belta6 = 1.32e-5 + 9.6 * environmentParam.theltaA * 1e-8;
		double Xthelta, belta1, Gthelta, GtheltaD, theltaN, theltaN1;
		theltaN1 = thelta0;
		do
		{
			theltaN = theltaN1;
			Xthelta = ((wireParam.wire_external_D + wireParam.steel_core_d * 2) / (wireParam.wire_external_D + wireParam.steel_core_d)) * 0.01 *
				sqrt((8 * Pi * wireParam.f * (wireParam.wire_external_D - wireParam.steel_core_d)) / ((wireParam.wire_external_D + wireParam.steel_core_d) * 1000 * CalRdc(theltaN)));

			belta1 = (0.018578 - 0.060526 * Xthelta + 0.062205 * pow(Xthelta, 2)) *
				((wireParam.wire_external_D + wireParam.steel_core_d * 2) / (wireParam.wire_external_D + wireParam.steel_core_d)) -
				0.05 * sqrt((80 * Pi * wireParam.f * (wireParam.wire_external_D - wireParam.steel_core_d)) / ((wireParam.wire_external_D + wireParam.steel_core_d))) *
				pow(CalRdc(theltaN), -0.5);

			Gthelta = pow(I, 2) * CalK1(theltaN) * K2 * CalRdc(theltaN) - CalWr(theltaN) - CalWf(theltaN) + Ws;

			GtheltaD = pow(I, 2) * K2 * wireParam.R20 * wireParam.alpha20 * (belta1 + CalK1(theltaN)) -
				4 * Pi * wireParam.wire_external_D * wireParam.E1 * StefanBoundConstant * pow((theltaN + environmentParam.theltaA + 273), 3) -
				(2 * belta2 * theltaN + belta3) * pow((belta4 / (belta5 * theltaN + belta6)), 0.485) + 0.485 * (belta2 * theltaN + belta3) *
				theltaN * belta4 * belta5 * pow((belta4 / (belta5 * theltaN + belta6)), -0.515) * pow((belta5 * theltaN + belta6), -2);

			theltaN1 = theltaN - Gthelta / GtheltaD;
		} while (abs(Gthelta) >= tolerance);
		thelta = theltaN;
	}

	bool WireInformation::CalTheltaProgram(double thelta0, double tolerance)
	{
		/*double tmpThelta = wireParam.maxTemper - environmentParam.theltaA;
		if (CalIProgram(tmpThelta, tolerance))
		{
			if (I < wireParam.maxI)
			{
				CalThelta(thelta0,tolerance);
				return true;
			}
		}
		return false;*/

		CalWs();
		CalK2();
		CalThelta(thelta0, tolerance);
		return true;
	}

	double WireInformation::CalGamma()
	{
		double g1 = GravitationalAcceleration * wireParam.unitWeight;                                          //自重力荷载
		double g2 = GravitationalAcceleration * 0.9 * Pi * environmentParam.iceThickness *
			(environmentParam.iceThickness + wireParam.wire_external_D);// * 1e-3                              //冰重力荷载
		double g3 = g1 + g2;                                                                                   //自重力加冰重力荷载
		double g4 = 0.625 * pow(environmentParam.windSpeedInAverageAltitude, 2) *
			wireParam.wire_external_D * environmentParam.windPressureFactor * wireParam.shapeFactor;// * 1e-3  //无冰时风荷载
		double g5 = 0.625 * pow(environmentParam.windSpeedInAverageAltitude, 2) *
			(environmentParam.iceThickness * 2 + wireParam.wire_external_D) *
			environmentParam.windPressureFactor * wireParam.shapeFactor;// * 1e-3                              //覆冰时风荷载
		double g6 = sqrt(pow(g1, 2) + pow(g4, 2));                                                             //无冰时综合荷载
		double g7 = sqrt(pow(g3, 2) + pow(g5, 2));                                                             //覆冰时综合荷载

		if (environmentParam.iceThickness > 0)
		{
			gamma = g7 / wireParam.A;
		}
		else
		{
			gamma = g6 / wireParam.A;
		}

		return atan(g5 / g1);
	}

	double WireInformation::CalShapeFactor(double wireExternalD, double iceThick)
	{
		double sf = 0;
		if (iceThick > 0)
		{
			sf = 1.2;
		}
		else
		{
			if (wireExternalD < 0.017)
			{
				sf = 1.2;
			}
			else
			{
				sf = 1.1;
			}
		}
		return sf;
	}

	double WireInformation::CalWindPressureFactor(double windSpeed)
	{
		//double wpf = 5.543 * pow(windSpeed,-0.737);
		double wpf = 1.0;
		if (windSpeed <= 10)
		{
			wpf = 1.0;
		}
		else if (windSpeed > 10 && windSpeed <= 15)
		{
			wpf = 0.75;
		}
		else if (windSpeed > 15)
		{
			wpf = 0.61;
		}
		return wpf;
	}

	double WireInformation::CalInsulatorWindDefAngle(double hTowerSpan, double vTowerSpan)
	{
		double iWindDefAngle = 0.0;
		if (!environmentParam.isWindDeviate || (hTowerSpan == 0.0 && vTowerSpan == 0.0))
		{
			return iWindDefAngle;
		}
		double beltaC = 1.0;
		if (environmentParam.windSpeedInAverageAltitude < 20)
		{
			beltaC = 1.0;
		}
		else if (environmentParam.windSpeedInAverageAltitude >= 20 && environmentParam.windSpeedInAverageAltitude < 30)
		{
			beltaC = 1.1;
		}
		else if (environmentParam.windSpeedInAverageAltitude >= 30 && environmentParam.windSpeedInAverageAltitude < 35)
		{
			beltaC = 1.2;
		}
		else if (environmentParam.windSpeedInAverageAltitude >= 35)
		{
			beltaC = 1.3;
		}
		double P = 0.625 * pow(environmentParam.windSpeedInAverageAltitude, 2) *
			(environmentParam.iceThickness * 2 + wireParam.wire_external_D) *
			environmentParam.windPressureFactor * wireParam.shapeFactor * beltaC;
		double W1 = GravitationalAcceleration * wireParam.unitWeight;
		double P1 = 9.81 * wireParam.insulatorArea * pow(environmentParam.windSpeedInAverageAltitude, 2) / 16;
		double G1 = GravitationalAcceleration * (wireParam.insulatorWeight + wireParam.hammerWeight);
		iWindDefAngle = atan((P1 / 2 + P * hTowerSpan) / (G1 / 2 + W1 * vTowerSpan));
		return iWindDefAngle;
	}
#pragma 

#pragma region 导线模型计算

	WireModel::WireModel()
	{

	}

	void WireModel::CalUnKnownLineModel(WireModel lineModel, double lineWeight, double lineStress)
	{
		splitNum = lineModel.splitNum;
		splitDistance = lineModel.splitDistance;
		sectionLineType = 1;

		towerSpan = lineModel.towerSpan;
		elevationAngle = lineModel.elevationAngle;
		hangPointA = lineModel.hangPointA;
		hangPointB = lineModel.hangPointB;
		double z = 0;
		if (lineModel.hangPointA.z > lineModel.hangPointB.z)
		{
			oFootPoint = Point2D(lineModel.hangPointB.x, lineModel.hangPointB.y);
			z = lineModel.hangPointB.z;
		}
		else
		{
			oFootPoint = Point2D(lineModel.hangPointA.x, lineModel.hangPointA.y);
			z = lineModel.hangPointA.z;
		}
		//oFootPoint = lineModel.oFootPoint;
		planeLineFactors = lineModel.planeLineFactors;
		//sectionLineFactors.push_back(hangPointA.z);
		sectionLineFactors.push_back(z);
		double kx = tan(elevationAngle) - ((lineWeight * towerSpan) / (2 * lineStress * cos(elevationAngle)));
		double kx2 = lineWeight / (2 * lineStress * cos(elevationAngle));
		sectionLineFactors.push_back(kx);
		sectionLineFactors.push_back(kx2);
	}

	void WireModel::GetModelInfo(WireModel lineModel)
	{
		towerSpan = lineModel.towerSpan;
		elevationAngle = lineModel.elevationAngle;
		wireSag = lineModel.wireSag;
		hangPointA = lineModel.hangPointA;
		hangPointB = lineModel.hangPointB;
		oFootPoint = lineModel.oFootPoint;
		planeLineFactors = lineModel.planeLineFactors;
		sectionLineFactors = lineModel.sectionLineFactors;
	}

	vector<Point3D> WireModel::getWindOffsetPoint(double spanX, Point3D curvePoint, int calType, int outType)
	{
		double signFx = 0.0;
		double signIn = 0.0;
		if (calType == 0)
		{
			signFx = 1.0;
			signIn = 1.0;
		}
		else if (calType = 1)
		{
			signFx = 1.0;
			signIn = 0.0;
		}
		else if (calType = 2)
		{
			signFx = 0.0;
			signIn = 1.0;
		}
		vector<Point3D> resPoints;
		double fx = 4 * wireSag * ((spanX / towerSpan) - pow((spanX / towerSpan), 2));
		double hOffset = fx * sin(wireWindDefAngle) * signFx + insulatorLength * sin(insulatorWindDefAngle) * signIn;
		double vOffset = fx * (1 - cos(wireWindDefAngle)) * signFx + insulatorLength * (1 - cos(insulatorWindDefAngle)) * signIn;
		Point3D leftPoint;
		Point3D rightPoint;
		leftPoint.x = (curvePoint.x + planeLineFactors[1] * curvePoint.y - planeLineFactors[1] * planeLineFactors[0] +
			planeLineFactors[1] * hOffset * sqrt(pow(planeLineFactors[1], 2) + 1)) / (pow(planeLineFactors[1], 2) + 1);
		leftPoint.y = (planeLineFactors[1] * curvePoint.x + pow(planeLineFactors[1], 2) * curvePoint.y + planeLineFactors[0] -
			hOffset * sqrt(pow(planeLineFactors[1], 2) + 1)) / (pow(planeLineFactors[1], 2) + 1);
		leftPoint.z = curvePoint.z + vOffset;
		rightPoint.x = (curvePoint.x + planeLineFactors[1] * curvePoint.y - planeLineFactors[1] * planeLineFactors[0] -
			planeLineFactors[1] * hOffset * sqrt(pow(planeLineFactors[1], 2) + 1)) / (pow(planeLineFactors[1], 2) + 1);
		rightPoint.y = (planeLineFactors[1] * curvePoint.x + pow(planeLineFactors[1], 2) * curvePoint.y + planeLineFactors[0] +
			hOffset * sqrt(pow(planeLineFactors[1], 2) + 1)) / (pow(planeLineFactors[1], 2) + 1);
		rightPoint.z = curvePoint.z + vOffset;

		if (outType == 1)
		{
			resPoints.push_back(leftPoint);
		}
		else if (outType == 2)
		{
			resPoints.push_back(rightPoint);
		}
		else
		{
			resPoints.push_back(leftPoint);
			resPoints.push_back(rightPoint);
		}

		return resPoints;
	}

#pragma endregion

#pragma region 导线工况模拟

	WireSimulation::WireSimulation()
	{

	}

	WireSimulation::WireSimulation(WireInformation knWireInfo, WireModel knWireModel, WireInformation unKnWireInfo, WireModel unKnWireModel, std::vector <WireModel> tnSegment)
	{
		knownWireInfo = knWireInfo;
		knownWireModel = knWireModel;
		unKnownWireInfo = unKnWireInfo;
		unKnownWireModel = unKnWireModel;
		tensionSegment = tnSegment;
	}

	double WireSimulation::CalWireStressBySag(double sag, double weight, double span, double angle)
	{
		double sigma = 0;
		sigma = weight * pow(span, 2) / (8 * sag * cos(angle));
		return sigma;
	}

	double WireSimulation::CalWireStress(double sigmaCM, double angle, double E, double gammaM, double span, double gamma, double alpha, double tM, double t)
	{
		double sigma = 0;
		sigmaCM /= cos(angle);
		double Fm = pow(span, 2) * pow(gammaM, 2) * E / (24 * pow(sigmaCM, 2)) - (sigmaCM + alpha * E * tM);
		double b = pow(gamma, 2) * pow(span, 2) * E / 24;
		double a = Fm + alpha * E * t;
		double A = abs(a);
		if (A == 0)
		{
			sigma = pow(b, 1.0 / 3);
		}
		else
		{
			double C = a / abs(a);
			double delta = 13.5 * b / pow(A, 3) - C;
			if (delta >= 1)
			{
				double thelta = acosh(delta);
				sigma = A / 3 * (2 * cosh(thelta / 3.0) - C);
			}
			else
			{
				double thelta = acos(delta);
				sigma = A / 3 * (2 * cos(thelta / 3.0) - C);
			}
		}
		sigma *= cos(angle);
		return sigma;
	}

	double WireSimulation::CalRepresentSpan()
	{
		if (tensionSegment.size() < 2)
		{
			return knownWireModel.towerSpan;
		}
		double repreSpan = 0;
		double sumUp = 0;
		double sumDown = 0;
		double sumT = 0;
		for (size_t i = 0; i < tensionSegment.size(); i++)
		{
			sumUp += (pow(tensionSegment[i].towerSpan, 3) * cos(tensionSegment[i].elevationAngle));
			sumDown += (tensionSegment[i].towerSpan / cos(tensionSegment[i].elevationAngle));
			sumT += (tensionSegment[i].towerSpan / pow(cos(tensionSegment[i].elevationAngle), 2));
		}
		repreSpan = pow(sumUp / sumDown, 1 / 2.0) / (sumDown / sumT);
		return repreSpan;
	}

	void WireSimulation::SimulationProgram(TensionSegment tensionSeg, int lineIndex)
	{
		WireModel lineModel = tensionSeg.tsLineModel[lineIndex];
		knownWireInfo.CalTheltaProgram(InitThelta, Tolearance);
		knownWireInfo.CalGamma();
		unKnownWireInfo.CalTheltaProgram(InitThelta, Tolearance);
		/*if (unKnownWireInfo.environmentParam.theltaA >= 75)
		{
			unKnownWireInfo.thelta = unKnownWireInfo.environmentParam.theltaA - knownWireInfo.environmentParam.theltaA;
		}*/
		unKnownWireModel.wireWindDefAngle = unKnownWireInfo.CalGamma();

		knownWireModel.GetModelInfo(lineModel);
		knownWireModel.representSpan = CalRepresentSpan();
		knownWireInfo.wireStress = CalWireStressBySag(knownWireModel.wireSag, knownWireInfo.gamma, knownWireModel.towerSpan, knownWireModel.elevationAngle);
		unKnownWireInfo.wireStress = CalWireStress(knownWireInfo.wireStress, knownWireModel.elevationAngle, knownWireInfo.wireParam.E, knownWireInfo.gamma,
			knownWireModel.representSpan, unKnownWireInfo.gamma, knownWireInfo.wireParam.alpha, knownWireInfo.thelta, unKnownWireInfo.thelta);
		//unKnownWireInfo.wireStress = knownWireInfo.wireStress + 0.3;//
		unKnownWireModel.CalUnKnownLineModel(knownWireModel, unKnownWireInfo.gamma, unKnownWireInfo.wireStress);

		double htSpan = 0.0;
		double vtSpan = 0.0;
		GetTowerHVSpan(tensionSeg, lineIndex, unKnownWireInfo.gamma, unKnownWireInfo.wireStress, htSpan, vtSpan);
		unKnownWireModel.insulatorLength = unKnownWireInfo.wireParam.insulatorLength;
		unKnownWireModel.isWindDeviate = unKnownWireInfo.environmentParam.isWindDeviate == 0 ? false : true;
		unKnownWireModel.insulatorWindDefAngle = unKnownWireInfo.CalInsulatorWindDefAngle(htSpan, vtSpan);//0 45。角0.7854
	}

	void WireSimulation::GetTowerHVSpan(TensionSegment tensionSeg, int lineIndex, double wWeight, double wStress, double &hTowerSpan, double &vTowerSpan)
	{
		hTowerSpan = 0.0;
		vTowerSpan = 0.0;
		int lineIndex2 = 0;
		if (tensionSeg.tsTowerPoints.size() < 3)
		{
			return;
		}
		int rangeIndex = lineIndex / (tensionSeg.turnNum * 3);
		Point2D towerPt;
		if (lineIndex >= tensionSeg.turnNum * 3 * (tensionSeg.tsTowerPoints.size() - 2))
		{
			lineIndex2 = tensionSeg.wholeLineIndex[lineIndex][rangeIndex - 1];
			towerPt = tensionSeg.tsTowerPoints[rangeIndex - 1];
		}
		else
		{
			lineIndex2 = tensionSeg.wholeLineIndex[lineIndex][rangeIndex + 1];
			towerPt = tensionSeg.tsTowerPoints[rangeIndex + 1];
		}
		hTowerSpan = (tensionSeg.tsLineModel[lineIndex].towerSpan / cos(tensionSeg.tsLineModel[lineIndex].elevationAngle) +
			tensionSeg.tsLineModel[lineIndex2].towerSpan / cos(tensionSeg.tsLineModel[lineIndex2].elevationAngle)) / 2;
		double sign1 = 1.0;
		double sign2 = 1.0;
		Point2D ptA1 = Point2D(tensionSeg.simLineModel[lineIndex].hangPointA.x, tensionSeg.simLineModel[lineIndex].hangPointA.y);
		Point2D ptB1 = Point2D(tensionSeg.simLineModel[lineIndex].hangPointB.x, tensionSeg.simLineModel[lineIndex].hangPointB.y);
		if (towerPt.Distance(ptA1) < towerPt.Distance(ptB1))
		{
			if (tensionSeg.simLineModel[lineIndex].hangPointA.z < tensionSeg.simLineModel[lineIndex].hangPointB.z)
			{
				sign1 = -1.0;
			}
		}
		else
		{
			if (tensionSeg.simLineModel[lineIndex].hangPointA.z > tensionSeg.simLineModel[lineIndex].hangPointB.z)
			{
				sign1 = -1.0;
			}
		}
		Point2D ptA2 = Point2D(tensionSeg.simLineModel[lineIndex2].hangPointA.x, tensionSeg.simLineModel[lineIndex2].hangPointA.y);
		Point2D ptB2 = Point2D(tensionSeg.simLineModel[lineIndex2].hangPointB.x, tensionSeg.simLineModel[lineIndex2].hangPointB.y);
		if (towerPt.Distance(ptA2) < towerPt.Distance(ptB2))
		{
			if (tensionSeg.simLineModel[lineIndex2].hangPointA.z < tensionSeg.simLineModel[lineIndex2].hangPointB.z)
			{
				sign2 = -1.0;
			}
		}
		else
		{
			if (tensionSeg.simLineModel[lineIndex2].hangPointA.z > tensionSeg.simLineModel[lineIndex2].hangPointB.z)
			{
				sign2 = -1.0;
			}
		}
		vTowerSpan = hTowerSpan + wStress / wWeight * (tan(tensionSeg.tsLineModel[lineIndex].elevationAngle) * sign1 + tan(tensionSeg.tsLineModel[lineIndex2].elevationAngle) * sign2);
		return;
	}

	void WireSimulation::InitSimulationInfoFromXml(const char* xmlConfig, WireParam &wParam, EnvironmentParam &eParam, double &knownI, EnvironmentParam &unEParam, double &unKnownI)
	{
		XMLDocument xmlDoc;
		xmlDoc.LoadFile(xmlConfig);
		if (xmlDoc.Error())
		{
			printf("Open WireSimulationConfig.xml Error:");
			return;
		}

		string wireType = xmlDoc.FirstChildElement("config")->FirstChildElement("wireType")->GetText();
		XMLElement * wireInfo = xmlDoc.FirstChildElement("config")->FirstChildElement("WireInfo")->FirstChildElement("wireInfo");
		while (wireInfo->Attribute("wireType") != wireType && wireInfo->NextSiblingElement("wireInfo") != NULL)
		{
			wireInfo = wireInfo->NextSiblingElement("wireInfo");
		}
		wParam.A = atof(wireInfo->FirstChildElement("wireArea")->GetText());
		wParam.alpha = atof(wireInfo->FirstChildElement("expansionCoeff")->GetText());
		wParam.alpha20 = atof(wireInfo->FirstChildElement("RTemperCoeff")->GetText());
		wParam.alphaS = atof(wireInfo->FirstChildElement("heatCoeff")->GetText());
		wParam.E = atof(wireInfo->FirstChildElement("elasticCoeff")->GetText());
		wParam.E1 = atof(wireInfo->FirstChildElement("radiativeCoeff")->GetText());
		wParam.f = atof(wireInfo->FirstChildElement("frequency")->GetText());
		wParam.number_of_aluminum_layers = atoi(wireInfo->FirstChildElement("aluminumLayers")->GetText());
		wParam.R20 = atof(wireInfo->FirstChildElement("R20")->GetText());
		wParam.steel_core_d = atof(wireInfo->FirstChildElement("steelCoreD")->GetText());
		wParam.unitWeight = atof(wireInfo->FirstChildElement("unitWeight")->GetText());
		wParam.wire_external_D = atof(wireInfo->FirstChildElement("externalD")->GetText());
		wParam.insulatorLength = atof(wireInfo->FirstChildElement("insulatorLength")->GetText());
		wParam.insulatorArea = atof(wireInfo->FirstChildElement("insulatorArea")->GetText());
		wParam.insulatorWeight = atof(wireInfo->FirstChildElement("insulatorWeight")->GetText());
		wParam.hammerWeight = atof(wireInfo->FirstChildElement("hammerWeight")->GetText());

		XMLElement * curEnviParam = xmlDoc.FirstChildElement("config")->FirstChildElement("EnviParam")->FirstChildElement("CurEnviParam");
		eParam.iceThickness = atof(curEnviParam->FirstChildElement("iceThickness")->GetText());
		eParam.Js = atof(curEnviParam->FirstChildElement("irradiance")->GetText());
		eParam.theltaA = atof(curEnviParam->FirstChildElement("enviTemper")->GetText());
		eParam.vertical_wind_speed = atof(curEnviParam->FirstChildElement("vWindSpeed")->GetText());
		eParam.windSpeedInAverageAltitude = eParam.vertical_wind_speed;

		XMLElement * simuEnviParam = xmlDoc.FirstChildElement("config")->FirstChildElement("EnviParam")->FirstChildElement("SimuEnviParam");
		unEParam.iceThickness = atof(simuEnviParam->FirstChildElement("iceThickness")->GetText());
		unEParam.Js = atof(simuEnviParam->FirstChildElement("irradiance")->GetText());
		unEParam.theltaA = atof(simuEnviParam->FirstChildElement("enviTemper")->GetText());
		unEParam.vertical_wind_speed = atof(simuEnviParam->FirstChildElement("vWindSpeed")->GetText());
		unEParam.windSpeedInAverageAltitude = unEParam.vertical_wind_speed;
		unEParam.isWindDeviate = atoi(simuEnviParam->FirstChildElement("isWindDeviate")->GetText());;

		knownI = atof(xmlDoc.FirstChildElement("config")->FirstChildElement("ElectricCurrent")->FirstChildElement("knownI")->GetText());
		unKnownI = atof(xmlDoc.FirstChildElement("config")->FirstChildElement("ElectricCurrent")->FirstChildElement("simulationI")->GetText());

		eParam.windPressureFactor = WireInformation::CalWindPressureFactor(eParam.vertical_wind_speed);
		unEParam.windPressureFactor = WireInformation::CalWindPressureFactor(unEParam.vertical_wind_speed);
		wParam.shapeFactor = WireInformation::CalShapeFactor(wParam.wire_external_D, eParam.iceThickness);
	}
#pragma endregion


}