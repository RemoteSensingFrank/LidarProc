#include "LASVectorClassify.h"

#include "../LidarBase/LASPoint.h"
#include "../LidarBase/LASReader.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "../LidarAlgorithm/GeometryAlgorithm.h"
#include"../LidarAlgorithm/PointProcAlgorithm.h"
//#include "../LidarAlgorithm/WorkingConditionSimulation.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include "../Utility/tinyxml2.h"

#include <fstream>

using namespace Eigen;
using namespace tinyxml2;

OGREnvelope LASVectorClassify::LASRectToEnvelope(Rect2D rect)
{
	OGREnvelope envelope;
	envelope.MinX = rect.minx;
	envelope.MinY = rect.miny;
	envelope.MaxX = rect.maxx;
	envelope.MaxY = rect.maxy;
	return envelope;
}

Rect2D LASVectorClassify::LASEnvelopToRect(OGREnvelope *m_Envelope)
{
	Rect2D rect;
	rect.minx = m_Envelope->MinX;
	rect.miny = m_Envelope->MinY;
	rect.maxx = m_Envelope->MaxX;
	rect.maxy = m_Envelope->MaxY;
	
	return rect;
}

bool  LASVectorClassify::LASSetMaps(vector<string> nameKey, vector<eLASClassification>typeKey)
{
	if (nameKey.size() != typeKey.size())
		return false;

	for (int i = 0; i < nameKey.size(); ++i)
		name_type_maps.insert(pair<string, int>(nameKey[i], typeKey[i]));

	return true;
}

void LASVectorClassify::VectorPointsToPolygon(Point2Ds points, OGRPolygon* poPolygon)
{
	OGRLinearRing ring;
	for(auto pnt:points){
		ring.addPoint(pnt.x,pnt.y);
	}
	ring.closeRings();
	poPolygon->addRing(&ring);
}

void LASVectorClassify::VectorPolygonToPoints(OGRPolygon *poPolygon, Point2Ds &points)
{
	OGRLinearRing* ring=poPolygon->getExteriorRing();
	for(int i=0;i<ring->getNumPoints();++i)
	{
		OGRPoint opt;
		ring->getPoint(i,&opt);
		Point2D vpt;
		vpt.x = opt.getX();
		vpt.y = opt.getY();
		points.push_back(vpt);
	}
}

bool LASVectorClassify::TestGeos() {
	OGRPolygon  poPolygon ;
	OGRLinearRing ring;
	ring.addPoint(0, 10);
	ring.addPoint(-10, -10);
	ring.addPoint(10, -10);
	ring.closeRings();

	poPolygon.addRing(&ring);
	OGRPoint *pnt = new OGRPoint(0, 0);
	bool test1 = ring.isPointInRing(pnt, 0);
	bool test2 = poPolygon.IsPointOnSurface(pnt);
	return (test1&&test2);
}

#pragma region 导线点云处理
bool LASVectorClassify::ReadTowerPointsByTxt(const char* pathTowerPoints, vector <pair <Point2D, int>> &towerPoints, vector <TensionSegment> &tensionSeg, int &towerSpanNum)
{
	ifstream fileIn;
	if (pathTowerPoints == NULL || pathTowerPoints == "") {
		return false;
	}
	fileIn.open(pathTowerPoints,ios::in);
	if (!fileIn.is_open()) {
		printf("open towerPoints File Failed.\r\n");
		return false;
	}

	vector <bool> isContinuity;
	/*string txtName = string(pathTowerPoints).substr(string(pathTowerPoints).find_last_of("\\") + 1);
	int startNo = atoi(txtName.substr(txtName.find_last_of("_") + 1,txtName.find_first_of("-") - txtName.find_last_of("_")).c_str());
	int endNo = atoi(txtName.substr(txtName.find_last_of("-") + 1, txtName.find_first_of(".") - txtName.find_last_of("-")).c_str());*/
	int splitInx = -1;//-1
	//bool firstFlag = true;

	string strOne;
	bool lFlag = false;
	while (getline(fileIn, strOne)) {
		vector <string> strVec = FileHelper::SplitString(strOne, "\t");
		double x = atof(strVec[1].c_str());
		double y = atof(strVec[2].c_str());
		int towerType = 0;
		if (strVec[5]._Equal("L"))
		{
			towerType = 1;
		}
		else if (strVec[5]._Equal("Z"))
		{
			towerType = 0;
			lFlag = true;
		}
		towerPoints.push_back(make_pair(Point2D(x, y), towerType));

		isContinuity.push_back(true);
		/*splitInx++;
		if (strVec[6].find(strVec[0]) == string::npos && firstFlag)
		{
			isContinuity[splitInx - 1] = false;
			firstFlag = false;
		}*/
	}

	bool isNeedSplit = true;
	/*if (endNo - startNo + 1 > towerPoints.size())
	{
		isNeedSplit = true;
	}*/
	if (splitInx > -1)
	{
		isContinuity[splitInx] = false;
	}
	towerSpanNum = towerPoints.size() - 1;

	TensionSegment tenSeg;
	for (size_t i = 0; i < towerPoints.size(); i++)
	{
		if (towerPoints[i].second == 0 || i == 0)
		{
			tenSeg.tsTowerPoints.push_back(towerPoints[i].first);
			continue;
		}
		tenSeg.tsTowerPoints.push_back(towerPoints[i].first);
		tensionSeg.push_back(tenSeg);
		tenSeg.tsTowerPoints.clear();
		vector<Point2D>().swap(tenSeg.tsTowerPoints);
		if (isNeedSplit && !isContinuity[i])
		{
			i++;
			towerSpanNum--;
			if (i >= towerPoints.size())
			{
				break;
			}
		}
		tenSeg.tsTowerPoints.push_back(towerPoints[i].first);
	}
	
	return lFlag;
}

//vector <int> CountLineType(int * lineType, int size)
//{
//	vector <int> type;
//	for (size_t i = 0; i < size; i++)
//	{
//		bool flag = false;
//		for (size_t j = 0; j < type.size(); j++)
//		{
//			if (type[j] == lineType[i])
//			{
//				flag = true;
//			}
//		}
//		if (!flag)
//		{
//			type.push_back(lineType[i]);
//		}
//	}
//	return type;
//}

void LASVectorClassify::ErrorAnalysis(vector <Point3Ds> vecPoints)
{
	Point3Ds pointCloud;
	for (size_t i = 0; i < vecPoints.size(); i++)
	{
		for (size_t j = 0; j < vecPoints[i].size(); j++)
		{
			pointCloud.push_back(vecPoints[i][j]);
		}
	}
	Point3Ds knownPoints;
	Point3D kp1(190131.827, 2483150.205, 39.745);
	knownPoints.push_back(kp1);
	Point3D kp2(190175.318, 2483134.463, 43.022);
	knownPoints.push_back(kp2);
	Point3D kp3(190263.522, 2483102.640, 53.946);
	knownPoints.push_back(kp3);
	Point3D kp4(190166.381, 2483115.828, 43.018);
	knownPoints.push_back(kp4);
	Point3D kp5(190218.499, 2483097.054, 48.886);
	knownPoints.push_back(kp5);
	Point3D kp6(190253.439, 2483084.431, 53.979);
	knownPoints.push_back(kp6);
	Point3Ds lidarPoints = FindNearestPoint(knownPoints, pointCloud);
	PrintPointsError(knownPoints, lidarPoints);
}


vector <TensionSegment> LASVectorClassify::LidarToVector(const char * pathFileConfig, string towerSpanName)
{
	XMLDocument xmlDoc;
	xmlDoc.LoadFile(pathFileConfig);

	//const char* pathLas = xmlDoc.FirstChildElement("config")->FirstChildElement("pathLas")->GetText();
    //const char* pathTxt = xmlDoc.FirstChildElement("config")->FirstChildElement("pathTowerTxt")->GetText();
	const string pathLas = string(xmlDoc.FirstChildElement("config")->FirstChildElement("pathDir")->GetText()) + "\\las\\" + towerSpanName + "_conductor.las";
	const string pathTxt = string(xmlDoc.FirstChildElement("config")->FirstChildElement("pathDir")->GetText()) + "\\txt\\" + towerSpanName + ".txt";
	long turnNumber = atol(xmlDoc.FirstChildElement("config")->FirstChildElement("lineTurnNumber")->GetText());
	double sampleSpacing = atof(xmlDoc.FirstChildElement("config")->FirstChildElement("pointSampleSpacing")->GetText());
	double lineSpacing = atof(xmlDoc.FirstChildElement("config")->FirstChildElement("lineClassifyDistance")->GetText());
	double initResidual = atof(xmlDoc.FirstChildElement("config")->FirstChildElement("initResidual")->GetText());
	long splitNumber = atol(xmlDoc.FirstChildElement("config")->FirstChildElement("lineSplitNumber")->GetText());
	//const string fileName = xmlDoc.FirstChildElement("config")->FirstChildElement("outFileName")->GetText();
	//const string filePath = xmlDoc.FirstChildElement("config")->FirstChildElement("outFilePath")->GetText();
	//const string outFileName = fileName;

	ILASDataset *datasetLine = new ILASDataset();
	LidarMemReader memReaderLine;
	memReaderLine.LidarReader_Open(pathLas.c_str(), datasetLine);
	memReaderLine.LidarReader_Read(true, 1, datasetLine);

	std::vector<Point3D> pntCloudLine;
	for (int i = 0; i < datasetLine->m_totalReadLasNumber; ++i)
	{
		const LASIndex &idx = datasetLine->m_LASPointID[i];
		pntCloudLine.push_back(datasetLine->m_lasRectangles[idx.rectangle_idx].m_lasPoints[idx.point_idx_inRect].m_vec3d);
	}
	PointCloudSegment segmentAlg;
	int * lineType = new int[pntCloudLine.size()];
	long lineCount = segmentAlg.PointCloudSegment_DBScan(pntCloudLine, lineType, lineSpacing);   //按类别分出每一相导线
	vector <Point3Ds> segCloudLine(lineCount);//
	for (size_t i = 0; i < pntCloudLine.size(); i++)
	{
		for (size_t j = 0; j < lineCount; j++)
		{
			if (lineType[i] == j+1)
			{
				segCloudLine[j].push_back(pntCloudLine[i]);
				/*if (j == 3 || j == 6)
				{
					segCloudLine[3].push_back(pntCloudLine[i]);
				}
				else if (j == 4 || j == 7)
				{
					segCloudLine[4].push_back(pntCloudLine[i]);
				}
				else if (j == 5 || j == 8)
				{
					segCloudLine[5].push_back(pntCloudLine[i]);
				}
				else
				{
					segCloudLine[j].push_back(pntCloudLine[i]);
				}*/
			}
		}
	}
	/*for (int i = 0; i < lineCount; i++)
	{
		vector <Point3Ds> tmp;
		tmp.push_back(segCloudLine[i]);
		OutPoint3DsToTxt(std::to_string(i),"D:\\Program\\Data\\PointCloud\\33KV\\ATB-NTM\\out", tmp);
	}*/
	//ErrorAnalysis(segCloudLine);
	//vector <int> cType = CountLineType(lineType, pntCloudLine.size());

	vector <pair <Point2D, int>> towerPoints;
	vector <TensionSegment> tensionSeg;
	int towerSpanNum = 0;
	ReadTowerPointsByTxt(pathTxt.c_str(), towerPoints, tensionSeg, towerSpanNum);
    segCloudLine = SegmentTangentTowerPoints(segCloudLine, tensionSeg, turnNumber);
	lineCount = segCloudLine.size();
	long realLineCount = turnNumber * 3 * towerSpanNum;

	if (realLineCount < lineCount)
	{
		segCloudLine = ClearSmallPoints(segCloudLine,realLineCount);
		lineCount = realLineCount;
	}
	else if (realLineCount > lineCount)
	{
		printf("Line Count Lost Error!!!Classify Line Count : %d",lineCount);
		vector <TensionSegment>().swap(tensionSeg);
		return tensionSeg;
	}
	//ErrorAnalysis(segCloudLine);
	pntCloudLine.clear();
	delete[]lineType;
	lineType = NULL;
	delete datasetLine;
	datasetLine = NULL;
	
	Point2D oPoint(0,0);
	vector <WireModel> lineModels(lineCount);
	vector <Point3Ds> resultCloudLine(lineCount);
	vector <double> residualVec(lineCount);
	for (size_t i = 0; i < lineCount; i++)     //XY平面直线拟合与原点的起始垂足计算
	{
		residualVec[i] = initResidual;
		//lineModels[i].planeLineFactors = LeastSquare(segCloudLine[i],1);
		lineModels[i].planeLineFactors = GeometryComputation::RobustLeastSquare(segCloudLine[i], residualVec[i], 1);
		lineModels[i].oFootPoint = GeometryComputation::GetFootOfPerpendicular(lineModels[i].planeLineFactors, oPoint);
		lineModels[i].splitNum = splitNumber;
		GetSectionLineModel(lineModels[i], segCloudLine[i]);
		for (size_t m = 0; m < tensionSeg.size(); m++)
		{
			for (size_t n = 0; n < tensionSeg[m].linePointsIndex.size(); n++)
			{
				if (tensionSeg[m].linePointsIndex[n] == i)
				{
					tensionSeg[m].tsLineModel.push_back(lineModels[i]);
					break;
				}
			}
			tensionSeg[m].simLineModel = vector <WireModel> (tensionSeg[m].tsLineModel.size());
		}
		resultCloudLine[i] = GetPointsByLineModel(lineModels[i],sampleSpacing,1);
	}

	//OutPoint3DsToTxt(towerSpanName + "_conductor.txt", string(xmlDoc.FirstChildElement("config")->FirstChildElement("pathDir")->GetText()) + "\\out\\", resultCloudLine);
	//ErrorAnalysis(resultCloudLine);
	
	//GetWholeLineSegment(tensionSeg);
	return tensionSeg;
}

vector <Point3Ds> LASVectorClassify::ClearSmallPoints(vector <Point3Ds> &pointClouds, long realLineCount)
{
	vector <Point3Ds> resPoints;
	long redundantCount = pointClouds.size() - realLineCount;
	if (redundantCount == 0)
	{
		return pointClouds;
	}
	while (redundantCount > 0)
	{
		size_t smallIndex = 0;
		int pointSize = 99999999999;
		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			if (pointClouds[i].size() < pointSize && pointClouds[i].size() != 0)
			{
				pointSize = pointClouds[i].size();
				smallIndex = i;
			}
		}
		pointClouds[smallIndex].clear();
		redundantCount--;
	}
	for (size_t i = 0; i < pointClouds.size(); i++)
	{
		if (pointClouds[i].size() < 1)
		{
			continue;
		}
		resPoints.push_back(pointClouds[i]);
	}
	vector <Point3Ds>().swap(pointClouds);
	return resPoints;
}

vector <Point3Ds> LASVectorClassify::SegmentTangentTowerPoints(vector <Point3Ds> &pointClouds, vector <TensionSegment> &tensionSeg, long turnNumber)
{
	vector <Point3Ds> resPoints;
	double rectError = 20.0;//25
	for (size_t i = 0; i < tensionSeg.size(); i++)
	{
		tensionSeg[i].turnNum = turnNumber;
		int towerNum = tensionSeg[i].tsTowerPoints.size();
		if (towerNum < 2)
		{
			printf("耐张段划分不正确");
			assert(-1);
		}
		if (towerNum < 3)
		{
			Rect2D lTowerRect(tensionSeg[i].tsTowerPoints[0].x - rectError, tensionSeg[i].tsTowerPoints[0].y - rectError, tensionSeg[i].tsTowerPoints[0].x + rectError, tensionSeg[i].tsTowerPoints[0].y + rectError);
			Rect2D rTowerRect(tensionSeg[i].tsTowerPoints[towerNum - 1].x - rectError, tensionSeg[i].tsTowerPoints[towerNum - 1].y - rectError, 
				tensionSeg[i].tsTowerPoints[towerNum - 1].x + rectError, tensionSeg[i].tsTowerPoints[towerNum - 1].y + rectError);
			int count = 0;
			for (size_t j = 0; j < pointClouds.size(); j++)
			{
				bool lInRect = false;
				bool rInRect = false;
				for (size_t k = 0; k < pointClouds[j].size(); k++)
				{
					if (GeometryRelation::IsPointInRect(pointClouds[j][k].x, pointClouds[j][k].y, lTowerRect.minx,lTowerRect.miny,lTowerRect.maxx,lTowerRect.maxy))
					{
						lInRect = true;
					}
					if (GeometryRelation::IsPointInRect(pointClouds[j][k].x, pointClouds[j][k].y, rTowerRect.minx, rTowerRect.miny, rTowerRect.maxx, rTowerRect.maxy))
					{
						rInRect = true;
					}
					if (lInRect && rInRect)
					{
						break;
					}
				}
				if (lInRect && rInRect)
				{
					resPoints.push_back(pointClouds[j]);
					pointClouds[j].clear();
					tensionSeg[i].linePointsIndex.push_back(resPoints.size() - 1);
					vector <int> oneLineIndex;
					oneLineIndex.push_back(resPoints.size() - 1);
					tensionSeg[i].wholeLineIndex.push_back(oneLineIndex);
					count++;
					if (count == (towerNum - 1) * tensionSeg[i].turnNum * 3)
					{
						break;
					}
				}
			}
		}
		else
		{
			int count = 0;
			for (size_t j = 0; j < pointClouds.size(); j++)
			{
				if (pointClouds[j].size() < 1)
				{
					continue;
				}
				int * highPointIdx = new int[towerNum];
				int * lowPointIdx = new int[towerNum];
				double * highZ = new double[towerNum];
				double * lowZ = new double[towerNum];
				bool * inRect = new bool[towerNum];
				for (size_t k = 0; k < pointClouds[j].size(); k++)
				{
					for (size_t m = 0; m < towerNum; m++)
					{
						if (k == 0)
						{
							highZ[m] = -999;
							lowZ[m] = 9999999999;
							inRect[m] = false;
						}
						Rect2D towerRect(tensionSeg[i].tsTowerPoints[m].x - rectError, tensionSeg[i].tsTowerPoints[m].y - rectError, tensionSeg[i].tsTowerPoints[m].x + rectError, tensionSeg[i].tsTowerPoints[m].y + rectError);
						if (GeometryRelation::IsPointInRect(pointClouds[j][k].x, pointClouds[j][k].y, towerRect.minx, towerRect.miny, towerRect.maxx, towerRect.maxy))
						{
							if (pointClouds[j][k].z > highZ[m])
							{
								highZ[m] = pointClouds[j][k].z;
								highPointIdx[m] = k;
							}
							if (pointClouds[j][k].z < lowZ[m])
							{
								lowZ[m] = pointClouds[j][k].z;
								lowPointIdx[m] = k;
							}
							inRect[m] = true;
						}
					}
				}

				int inRectCount = 0;
				for (size_t m = 0; m < towerNum; m++)
				{
					if (inRect[m])
					{
						inRectCount++;
					}
				}
				if (inRectCount < 2)
				{
					continue;
				}
				else if (inRectCount == 2)
				{
					resPoints.push_back(pointClouds[j]);
					pointClouds[j].clear();
					tensionSeg[i].linePointsIndex.push_back(resPoints.size() - 1);
					/*vector <int> oneLineIndex;
					oneLineIndex.push_back(resPoints.size() - 1);
					tensionSeg[i].wholeLineIndex.push_back(oneLineIndex);*/
					count++;
					if (count == (towerNum - 1) * tensionSeg[i].turnNum * 3)
					{
						break;
					}
				}
				else if (inRectCount > 2)
				{
					//vector <Point3Ds> leftPointsVec;
					vector <Point3D> rightPoints = pointClouds[j];
					vector <int> oneLineIndex;
					for (size_t m = 1; m < towerNum - 1; m++)//找到杆塔点所在矩形框内的最高最低点连成线，以其过最高点的垂直线分割点云
					{
						vector <Point3D> tmpRightPoints;
						vector <Point3D> leftPoints;
						double lineEquA = rightPoints[lowPointIdx[m]].x - rightPoints[highPointIdx[m]].x;
						double lineEquB = rightPoints[lowPointIdx[m]].y - rightPoints[highPointIdx[m]].y;
						double lineEquC = rightPoints[highPointIdx[m]].x * (rightPoints[highPointIdx[m]].x - rightPoints[lowPointIdx[m]].x) +
							rightPoints[highPointIdx[m]].y * (rightPoints[highPointIdx[m]].y - rightPoints[lowPointIdx[m]].y);
						double leftFlag = lineEquA * tensionSeg[i].tsTowerPoints[0].x + lineEquB * tensionSeg[i].tsTowerPoints[0].y + lineEquC;
						for (size_t n = 0; n < rightPoints.size(); n++)
						{
							double lineEquValue = lineEquA * rightPoints[n].x + lineEquB * rightPoints[n].y + lineEquC;
							if (lineEquValue * leftFlag >= 0)
							{
								leftPoints.push_back(rightPoints[n]);
							}
							else
							{
								tmpRightPoints.push_back(rightPoints[n]);
							}
						}
						vector <Point3D>().swap(rightPoints);
						rightPoints = tmpRightPoints;
						vector <Point3D>().swap(tmpRightPoints);
						resPoints.push_back(leftPoints);
						count++;
						tensionSeg[i].linePointsIndex.push_back(resPoints.size() - 1);
						oneLineIndex.push_back(resPoints.size() - 1);
						vector <Point3D>().swap(leftPoints);
					}
					resPoints.push_back(rightPoints);
					count++;
					tensionSeg[i].linePointsIndex.push_back(resPoints.size() - 1);
					oneLineIndex.push_back(resPoints.size() - 1);
					for (size_t r = 0; r < towerNum - 1; r++)
					{
						tensionSeg[i].wholeLineIndex.push_back(oneLineIndex);
					}
				}

				delete inRect; inRect = NULL;
				delete highPointIdx; highPointIdx = NULL;
				delete lowPointIdx; lowPointIdx = NULL;
				delete highZ; highZ = NULL;
				delete lowZ; lowZ = NULL;
				pointClouds[j].clear();
				if (count == (towerNum - 1) * tensionSeg[i].turnNum * 3)
				{
					break;
				}
			}
		}
	}

	vector <Point3Ds>().swap(pointClouds);
	return resPoints;
}

//根据first的值降序排序
bool ComparisionPair(pair<double, int>a, pair<double, int>b)
{
	return a.first < b.first;
}

//怎么从耐张段中所有的聚类直线提取一相导线的耐张段所有直线？
void LASVectorClassify::GetWholeLineSegment(vector <TensionSegment> &tensionSeg)
{
	for (size_t i = 0; i < tensionSeg.size(); i++)
	{
		if (tensionSeg[i].wholeLineIndex.size() > 0)
		{
			continue;
		}
		for (size_t j = 0; j < tensionSeg[i].linePointsIndex.size(); j++)
		{
			WireModel curLineModel = tensionSeg[i].tsLineModel[j];
			vector <pair <double, int>> sameLine;
			for (size_t l = 0; l < tensionSeg[i].linePointsIndex.size(); l++)
			{
				WireModel tmpLineModel = tensionSeg[i].tsLineModel[l];

				double parallelK = abs((curLineModel.planeLineFactors[0] - tmpLineModel.planeLineFactors[0]) / curLineModel.planeLineFactors[0]);
				double parallelB = abs((curLineModel.planeLineFactors[1] - tmpLineModel.planeLineFactors[1]) / curLineModel.planeLineFactors[1]);
				sameLine.push_back(make_pair(parallelK * parallelB, tensionSeg[i].linePointsIndex[l]));
			}
			sort(sameLine.begin(),sameLine.end(), ComparisionPair);
			vector <int> oneLineIndex;
			//oneLineIndex.push_back(tensionSeg[i].linePointsIndex[j]);
			for (size_t m = 0; m < (tensionSeg[i].linePointsIndex.size() / (3 * tensionSeg[i].turnNum)); m++)
			{
				oneLineIndex.push_back(sameLine[m].second);
			}
			sort(oneLineIndex.begin(),oneLineIndex.end());
			tensionSeg[i].wholeLineIndex.push_back(oneLineIndex);
		}
		
	}
}

void LASVectorClassify::GetSectionLineModel(WireModel &lineModel, Point3Ds linePoints)
{
	double lineAngle = atan(lineModel.planeLineFactors[1]);
	Point2Ds sectionPoints;
	double minscaleFactor = 0;
	double maxscaleFactor = 0;
	Point3D maxPoint;//
	Point3D minPoint;//
	Point2D maxFootPoint;//
	Point2D minFootPoint;//
	double maxError = 0;//
	double minError = 9999999999999;//
	double totalError = 0;//
	for (size_t i = 0; i < linePoints.size(); i++)   //根据比例因子的极值求端点
	{
		Point2D planePoint(linePoints[i].x, linePoints[i].y);
		//double error = linePoints[i].y - GetYByLineFactors(lineModel.planeLineFactors, linePoints[i].x);
		double error = GeometryComputation::GetPointToLineDistance(lineModel.planeLineFactors, planePoint);
		if (abs(minError) > abs(error))
		{
			minError = error;
		}
		if (abs(maxError) < abs(error))
		{
			maxError = error;
		}
		totalError += abs(error);

		Point2D strightLinePoint = GeometryComputation::GetFootOfPerpendicular(lineModel.planeLineFactors, planePoint);
		double scaleFactor = Point2DToScaleFactor(strightLinePoint, lineModel.oFootPoint, lineAngle);
		//Point2D curveLinePoint(scaleFactor, linePoints[i].z);
		//sectionPoints.push_back(curveLinePoint);
		if (i == 0)
		{
			minscaleFactor = scaleFactor;
			maxscaleFactor = scaleFactor;
			lineModel.hangPointA = Point3D(minscaleFactor, 0, linePoints[i].z);
			minPoint = linePoints[i];
			minFootPoint = strightLinePoint;
			lineModel.hangPointB = Point3D(maxscaleFactor, 0, linePoints[i].z);
			maxPoint = linePoints[i];
			maxFootPoint = strightLinePoint;
		}
		else
		{
			if (minscaleFactor > scaleFactor)  
			{
				minscaleFactor = scaleFactor;
				lineModel.hangPointA = Point3D(minscaleFactor, 0,linePoints[i].z);
				minPoint = linePoints[i];
				minFootPoint = strightLinePoint;
			}
			if (maxscaleFactor < scaleFactor)
			{
				maxscaleFactor = scaleFactor;
				lineModel.hangPointB = Point3D(maxscaleFactor, 0,linePoints[i].z);
				maxPoint = linePoints[i];
				maxFootPoint = strightLinePoint;
			}
		}
	}
	double avgError = totalError / linePoints.size();//
	//printf("AverageXYerror: %f\r\n",avgError);

	Point2D hangPointA2D = ScaleFactorToPoint2D(lineModel.hangPointA.x, lineModel.oFootPoint, lineAngle, lineModel.planeLineFactors);
	Point2D hangPointB2D = ScaleFactorToPoint2D(lineModel.hangPointB.x, lineModel.oFootPoint, lineAngle, lineModel.planeLineFactors);

	double tmpErrorA = hangPointA2D.y - GeometryComputation::GetYByLineFactors(lineModel.planeLineFactors, hangPointA2D.x);//
	double tmpErrorB = hangPointB2D.y - GeometryComputation::GetYByLineFactors(lineModel.planeLineFactors, hangPointB2D.x);//

	lineModel.oFootPoint = hangPointA2D;
	for (size_t i = 0; i < linePoints.size(); i++)   //将原点垂足改为极小值端点，减小数值大小,数值太大最小二乘拟合矩阵求解精度不够
	{
		Point2D planePoint(linePoints[i].x, linePoints[i].y);
		Point2D strightLinePoint = GeometryComputation::GetFootOfPerpendicular(lineModel.planeLineFactors, planePoint);
		double scaleFactor = Point2DToScaleFactor(strightLinePoint, lineModel.oFootPoint, lineAngle);
		Point2D curveLinePoint(scaleFactor, linePoints[i].z);
		sectionPoints.push_back(curveLinePoint);
	}
	lineModel.sectionLineFactors = GeometryComputation::LeastSquare(sectionPoints, 2);//LeastSquare

	double maxZError = 0;//
	double minZError = 9999999999999;//
	double totalZError = 0;//
	for (size_t i = 0; i < sectionPoints.size(); i++)
	{
		double error = sectionPoints[i].y - GeometryComputation::GetYByLineFactors(lineModel.sectionLineFactors, sectionPoints[i].x);
		if (abs(minZError) > abs(error))
		{
			minZError = error;
		}
		if (abs(maxZError) < abs(error))
		{
			maxZError = error;
		}
		totalZError += abs(error);
	}
	double avgZError = totalZError / sectionPoints.size();//
	//printf("AverageZerror: %f\r\n", avgZError);

	lineModel.sectionLineType = 0;
	lineModel.splitDistance = avgError;
	lineModel.hangPointA.x = hangPointA2D.x;
	lineModel.hangPointA.y = hangPointA2D.y;
	lineModel.hangPointA.z = GeometryComputation::GetYByLineFactors(lineModel.sectionLineFactors, Point2DToScaleFactor(hangPointA2D, lineModel.oFootPoint, lineAngle));
	lineModel.hangPointB.x = hangPointB2D.x;
	lineModel.hangPointB.y = hangPointB2D.y;
	lineModel.hangPointB.z = GeometryComputation::GetYByLineFactors(lineModel.sectionLineFactors, Point2DToScaleFactor(hangPointB2D, lineModel.oFootPoint, lineAngle));
	
	lineModel.towerSpan = sqrt(pow((hangPointA2D.x - hangPointB2D.x),2.0) + pow((hangPointA2D.y - hangPointB2D.y),2.0));
	lineModel.elevationAngle = atan(abs(lineModel.hangPointA.z - lineModel.hangPointB.z) / lineModel.towerSpan);
	lineModel.wireSag = GetWireSagByLineModel(lineModel);

	lineModel.wireWindDefAngle = 0;//////
	lineModel.isWindDeviate = false;/////
}

double LASVectorClassify::Point2DToScaleFactor(Point2D point2D, Point2D OFootPoint, double lineAngle)
{
	if (lineAngle == 0)
	{
		assert(-1);
	}
	double scaleFactor = 0;
	if (abs(sin(lineAngle)) >= abs(cos(lineAngle)))
	{
		scaleFactor = (point2D.x - OFootPoint.x) / (-1.0 * sin(lineAngle));   
	}
	else
	{
		scaleFactor = (point2D.y - OFootPoint.y) / cos(lineAngle);
	}
	return scaleFactor;
}

Point2D LASVectorClassify::ScaleFactorToPoint2D(double scaleFactor, Point2D OFootPoint, double lineAngle, vector <double> lineFactors)
{
	if (lineFactors[1] == 0)
	{
		assert(-1);
	}
	Point2D resPoint;
	if (abs(sin(lineAngle)) >= abs(cos(lineAngle)))
	{
		resPoint.x = OFootPoint.x - scaleFactor * sin(lineAngle);
		resPoint.y = lineFactors[0] + lineFactors[1] * resPoint.x;
	}
	else
	{
		resPoint.y = OFootPoint.y + scaleFactor * cos(lineAngle);
		resPoint.x = (resPoint.y - lineFactors[0]) / lineFactors[1];
	}
	return resPoint;
}

double LASVectorClassify::Point2DToSectionX(Point2D point2D, Point2D OPoint)
{
	double sectionX;
	sectionX = sqrt(pow((point2D.x - OPoint.x),2) + pow((point2D.y - OPoint.y),2));
	return sectionX;
}

Point2D LASVectorClassify::SectionXToPoint2D(double sectoinX, Point2D OPoint, double lineAngle)
{
	Point2D resPoint;
	resPoint.x = OPoint.x + sectoinX * cos(lineAngle);
	resPoint.y = OPoint.y + sectoinX * sin(lineAngle);
	return resPoint;
}

Point3Ds LASVectorClassify::GetPointsByLineModel(WireModel lineModel, double sampleSpacing, int isNeedSplit, int pointOutType)
{
	if (lineModel.planeLineFactors[1] == 0 && isNeedSplit == 1)
	{
		assert(-1);
	}
	Point3Ds linePoints;
	double lineAngle = atan(lineModel.planeLineFactors[1]);
	double signX = 1.0;////
	double signY = 1.0;////
	if ((lineModel.hangPointA.x > lineModel.hangPointB.x && cos(lineAngle) > 0)
		|| (lineModel.hangPointA.x < lineModel.hangPointB.x && cos(lineAngle) < 0))
	{
		signX = -1.0;
	}
	if ((lineModel.hangPointA.y > lineModel.hangPointB.y && sin(lineAngle) > 0)
		|| (lineModel.hangPointA.y < lineModel.hangPointB.y && sin(lineAngle) < 0))
	{
		signY = -1.0;
	}
	if (lineModel.oFootPoint.x - lineModel.hangPointB.x == 0 && lineModel.oFootPoint.y - lineModel.hangPointB.y == 0)
	{
		signX *= -1.0;
		signY *= -1.0;
	}

	//bool isWindDeviate = false;//此处风偏计算未考虑一端是耐张串一端是悬垂串的情况,暂弃
	//if (lineModel.wireWindDefAngle > 0.01)
	//{
	//	isWindDeviate = true;
	//}
	for (double i = 0; i < lineModel.towerSpan; i += sampleSpacing)
	{
		//Point2D strightLinePoint(lineModel.hangPointA.x + i * cos(lineAngle) * signX, lineModel.hangPointA.y + i * sin(lineAngle) * signY);
		Point2D strightLinePoint(lineModel.oFootPoint.x + i * cos(lineAngle) * signX, lineModel.oFootPoint.y + i * sin(lineAngle) * signY);
		double s = 0;
		if (lineModel.sectionLineType == 0)
		{
			s = Point2DToScaleFactor(strightLinePoint, lineModel.oFootPoint, lineAngle);
		}
		else if (lineModel.sectionLineType == 1)
		{
			s = Point2DToSectionX(strightLinePoint, lineModel.oFootPoint);
		}
		double z = GeometryComputation::GetYByLineFactors(lineModel.sectionLineFactors,s);
		if (isNeedSplit == 0 || lineModel.splitNum == 1)
		{
			Point3D curvePoint(strightLinePoint.x, strightLinePoint.y, z);
			if (!lineModel.isWindDeviate)
			{
				linePoints.push_back(curvePoint);
			}
			else
			{
				vector<Point3D> devPoints = lineModel.getWindOffsetPoint(i,curvePoint,0, pointOutType);//wind deviate
				for (size_t j = 0; j < devPoints.size(); j++)
				{
					linePoints.push_back(devPoints[j]);
				}
			}
		}
		else if (isNeedSplit == 1)
		{
			double valueChange = 0.0;
			if (lineModel.splitNum == 2)
			{
				valueChange = 0.0;
			}
			else if (lineModel.splitNum == 4)
			{
				valueChange = 1.0;
			}
			double X1 = (strightLinePoint.x + lineModel.planeLineFactors[1] * strightLinePoint.y - lineModel.planeLineFactors[1] * lineModel.planeLineFactors[0] +
				lineModel.planeLineFactors[1] * lineModel.splitDistance * sqrt(pow(lineModel.planeLineFactors[1],2) + 1)) / (pow(lineModel.planeLineFactors[1], 2) + 1);
			double Y1 = (lineModel.planeLineFactors[1] * strightLinePoint.x + pow(lineModel.planeLineFactors[1],2) * strightLinePoint.y + lineModel.planeLineFactors[0] -
				lineModel.splitDistance * sqrt(pow(lineModel.planeLineFactors[1], 2) + 1)) / (pow(lineModel.planeLineFactors[1], 2) + 1);
			double X2 = (strightLinePoint.x + lineModel.planeLineFactors[1] * strightLinePoint.y - lineModel.planeLineFactors[1] * lineModel.planeLineFactors[0] -
				lineModel.planeLineFactors[1] * lineModel.splitDistance * sqrt(pow(lineModel.planeLineFactors[1], 2) + 1)) / (pow(lineModel.planeLineFactors[1], 2) + 1);
			double Y2 = (lineModel.planeLineFactors[1] * strightLinePoint.x + pow(lineModel.planeLineFactors[1], 2) * strightLinePoint.y + lineModel.planeLineFactors[0] +
				lineModel.splitDistance * sqrt(pow(lineModel.planeLineFactors[1], 2) + 1)) / (pow(lineModel.planeLineFactors[1], 2) + 1);
			/*double dis1 = sqrt(pow((X1 - strightLinePoint.x),2) + pow((Y1 - strightLinePoint.y), 2)) - lineModel.splitDistance;
			double dis2 = sqrt(pow((X2 - strightLinePoint.x), 2) + pow((Y2 - strightLinePoint.y), 2)) - lineModel.splitDistance;
			if (dis1 > 0.00001 || dis2 > 0.00001)
			{
				printf("ERROR");
			}*/
			Point3D ltCurvePoint(X1, Y1, z + lineModel.splitDistance * valueChange);
			if (valueChange == 1.0)
			{
				Point3D lbCurvePoint(X1, Y1, z - lineModel.splitDistance * valueChange);
				Point3D rtCurvePoint(X2, Y2, z + lineModel.splitDistance * valueChange);
				if (!lineModel.isWindDeviate)
				{
					linePoints.push_back(lbCurvePoint);
					linePoints.push_back(rtCurvePoint);
				}
				else
				{
					vector<Point3D> lbdevPoints = lineModel.getWindOffsetPoint(i, lbCurvePoint,0, pointOutType);//wind deviate
					for (size_t j = 0; j < lbdevPoints.size(); j++)
					{
						linePoints.push_back(lbdevPoints[j]);
					}
					vector<Point3D> rtdevPoints = lineModel.getWindOffsetPoint(i, rtCurvePoint,0, pointOutType);//wind deviate
					for (size_t j = 0; j < rtdevPoints.size(); j++)
					{
						linePoints.push_back(rtdevPoints[j]);
					}
				}
			}
			Point3D rbCurvePoint(X2, Y2, z - lineModel.splitDistance * valueChange);
			if (!lineModel.isWindDeviate)
			{
				linePoints.push_back(ltCurvePoint);
				linePoints.push_back(rbCurvePoint);
			}
			else
			{
				vector<Point3D> ltdevPoints = lineModel.getWindOffsetPoint(i, ltCurvePoint,0, pointOutType);//wind deviate
				for (size_t j = 0; j < ltdevPoints.size(); j++)
				{
					linePoints.push_back(ltdevPoints[j]);
				}
				vector<Point3D> rbdevPoints = lineModel.getWindOffsetPoint(i, rbCurvePoint,0, pointOutType);//wind deviate
				for (size_t j = 0; j < rbdevPoints.size(); j++)
				{
					linePoints.push_back(rbdevPoints[j]);
				}
			}
		}
	}
	return linePoints;
}

Point3Ds LASVectorClassify::GetPointsByWindLineModel(TensionSegment tensionSeg, size_t lineIndex, double sampleSpacing, int isNeedSplit)
{
	if (tensionSeg.tsTowerPoints.size() < 3)
	{
		//tensionSeg.simLineModel[lineIndex].isWindDeviate = false;
		return GetPointsByLineModel(tensionSeg.simLineModel[lineIndex],sampleSpacing,isNeedSplit);
	}

	tensionSeg.simLineModel[lineIndex].isWindDeviate = true;
	Point3Ds linePoints;
	vector <Point3Ds> linePointsVec;
	Point2D ptA = Point2D(tensionSeg.simLineModel[lineIndex].hangPointA.x, tensionSeg.simLineModel[lineIndex].hangPointA.y);
	Point2D ptB = Point2D(tensionSeg.simLineModel[lineIndex].hangPointB.x, tensionSeg.simLineModel[lineIndex].hangPointB.y);
	Point2D towerPt;
	if (lineIndex < tensionSeg.turnNum * 3 || lineIndex >= tensionSeg.turnNum * 3 * (tensionSeg.tsTowerPoints.size() - 1))
	{
		if (lineIndex < tensionSeg.turnNum * 3)
		{
			towerPt = tensionSeg.tsTowerPoints[0];
		}
		else if (lineIndex >= tensionSeg.turnNum * 3 * (tensionSeg.tsTowerPoints.size() - 1))
		{
			towerPt = tensionSeg.tsTowerPoints[tensionSeg.tsTowerPoints.size() - 1];
		}
		if (towerPt.Distance(ptA) < towerPt.Distance(ptB))
		{
			Point3Ds hangPoints = tensionSeg.simLineModel[lineIndex].getWindOffsetPoint(tensionSeg.simLineModel[lineIndex].towerSpan, tensionSeg.simLineModel[lineIndex].hangPointB, 2);
			for (size_t i = 0; i < hangPoints.size(); i++)
			{
				tensionSeg.simLineModel[lineIndex].hangPointB = hangPoints[i];
				tensionSeg.simLineModel[lineIndex].planeLineFactors[0] = tensionSeg.simLineModel[lineIndex].hangPointA.y - tensionSeg.simLineModel[lineIndex].hangPointA.x *
					(tensionSeg.simLineModel[lineIndex].hangPointB.y - tensionSeg.simLineModel[lineIndex].hangPointA.y) / (tensionSeg.simLineModel[lineIndex].hangPointB.x - tensionSeg.simLineModel[lineIndex].hangPointA.x);
				tensionSeg.simLineModel[lineIndex].planeLineFactors[1] = (tensionSeg.simLineModel[lineIndex].hangPointB.y - tensionSeg.simLineModel[lineIndex].hangPointA.y) /
					(tensionSeg.simLineModel[lineIndex].hangPointB.x - tensionSeg.simLineModel[lineIndex].hangPointA.x);
				linePointsVec.push_back(GetPointsByLineModel(tensionSeg.simLineModel[lineIndex], sampleSpacing, isNeedSplit,i + 1));
			}
		}
		else
		{
			Point3Ds hangPoints = tensionSeg.simLineModel[lineIndex].getWindOffsetPoint(tensionSeg.simLineModel[lineIndex].towerSpan, tensionSeg.simLineModel[lineIndex].hangPointA, 2);
			for (size_t i = 0; i < hangPoints.size(); i++)
			{
				tensionSeg.simLineModel[lineIndex].hangPointA = hangPoints[i];
				tensionSeg.simLineModel[lineIndex].oFootPoint = Point2D(tensionSeg.simLineModel[lineIndex].hangPointA.x, tensionSeg.simLineModel[lineIndex].hangPointA.y);
				tensionSeg.simLineModel[lineIndex].planeLineFactors[0] = tensionSeg.simLineModel[lineIndex].hangPointA.y - tensionSeg.simLineModel[lineIndex].hangPointA.x *
					(tensionSeg.simLineModel[lineIndex].hangPointB.y - tensionSeg.simLineModel[lineIndex].hangPointA.y) / (tensionSeg.simLineModel[lineIndex].hangPointB.x - tensionSeg.simLineModel[lineIndex].hangPointA.x);
				tensionSeg.simLineModel[lineIndex].planeLineFactors[1] = (tensionSeg.simLineModel[lineIndex].hangPointB.y - tensionSeg.simLineModel[lineIndex].hangPointA.y) /
					(tensionSeg.simLineModel[lineIndex].hangPointB.x - tensionSeg.simLineModel[lineIndex].hangPointA.x);
				linePointsVec.push_back(GetPointsByLineModel(tensionSeg.simLineModel[lineIndex], sampleSpacing, isNeedSplit, i + 1));
			}
		}
	}
	else
	{
		Point3Ds hangPointsA = tensionSeg.simLineModel[lineIndex].getWindOffsetPoint(tensionSeg.simLineModel[lineIndex].towerSpan, tensionSeg.simLineModel[lineIndex].hangPointA, 2);
		Point3Ds hangPointsB = tensionSeg.simLineModel[lineIndex].getWindOffsetPoint(tensionSeg.simLineModel[lineIndex].towerSpan, tensionSeg.simLineModel[lineIndex].hangPointB, 2);
		for (size_t i = 0; i < hangPointsA.size(); i++)
		{
			tensionSeg.simLineModel[lineIndex].hangPointA = hangPointsA[i];
			tensionSeg.simLineModel[lineIndex].hangPointB = hangPointsB[i];
			tensionSeg.simLineModel[lineIndex].oFootPoint = Point2D(tensionSeg.simLineModel[lineIndex].hangPointA.x, tensionSeg.simLineModel[lineIndex].hangPointA.y);
			tensionSeg.simLineModel[lineIndex].planeLineFactors[0] = tensionSeg.simLineModel[lineIndex].hangPointA.y - tensionSeg.simLineModel[lineIndex].hangPointA.x *
				(tensionSeg.simLineModel[lineIndex].hangPointB.y - tensionSeg.simLineModel[lineIndex].hangPointA.y) / (tensionSeg.simLineModel[lineIndex].hangPointB.x - tensionSeg.simLineModel[lineIndex].hangPointA.x);
			tensionSeg.simLineModel[lineIndex].planeLineFactors[1] = (tensionSeg.simLineModel[lineIndex].hangPointB.y - tensionSeg.simLineModel[lineIndex].hangPointA.y) /
				(tensionSeg.simLineModel[lineIndex].hangPointB.x - tensionSeg.simLineModel[lineIndex].hangPointA.x);
			linePointsVec.push_back(GetPointsByLineModel(tensionSeg.simLineModel[lineIndex], sampleSpacing, isNeedSplit, i + 1));
		}
		
	}
	for (size_t i = 0; i < linePointsVec.size(); i++)
	{
		for (size_t j = 0; j < linePointsVec[i].size(); j++)
		{
			linePoints.push_back(linePointsVec[i][j]);
		}
	}
	return linePoints;
}

double LASVectorClassify::GetWireSagByLineModel(WireModel lineModel)
{
	double resultSag = 0;
	double lineAngle = atan(lineModel.planeLineFactors[1]);
	Point2D midPoint((lineModel.hangPointA.x + lineModel.hangPointB.x) / 2.0, (lineModel.hangPointA.y + lineModel.hangPointB.y) / 2.0);
	double midZ = (lineModel.hangPointA.z + lineModel.hangPointB.z) / 2.0;
	double midS = 0;
	if (lineModel.sectionLineType == 0)
	{
		midS = Point2DToScaleFactor(midPoint, lineModel.oFootPoint, lineAngle);
	}
	else if(lineModel.sectionLineType == 1)
	{
		midS = Point2DToSectionX(midPoint, lineModel.oFootPoint);
	}
	double midZOnCurve = GeometryComputation::GetYByLineFactors(lineModel.sectionLineFactors, midS);
	resultSag = midZ - midZOnCurve;
	return resultSag;
}

void LASVectorClassify::OutPoint3DsToTxt(string filename, string filePath, vector<Point3Ds> pntClouds)
{
	string file = filePath + "\\" + filename + ".txt";
	FILE* fOutTxt = fopen(file.c_str(), "wb");
	for (size_t i = 0; i < pntClouds.size(); i++)
	{
		for (size_t j = 0; j < pntClouds[i].size(); j++)
		{
			fprintf(fOutTxt, "%f %f %f\r\n", pntClouds[i][j].x, pntClouds[i][j].y, pntClouds[i][j].z);
		}
	}
	fclose(fOutTxt);
}

Point3Ds LASVectorClassify::FindNearestPoint(Point3Ds knownPoints, Point3Ds pointCloud)
{
	Point3Ds resPoints;
	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pointCloud);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	for (size_t i = 0; i < knownPoints.size(); i++)
	{
		double pnt[3] = { knownPoints[i].x,knownPoints[i].y,knownPoints[i].z };
		std::vector<std::pair<size_t, double> > indices_dists;
		size_t * out_indices = new size_t[1];
		double * out_distances = new double[1];
		treeIndex.knnSearch(&pnt[0],1, out_indices, out_distances);
		resPoints.push_back(pointCloud[out_indices[0]]);
		delete[]out_indices; out_indices = NULL;
		delete[]out_distances; out_distances = NULL;
	}
	return resPoints;
}

void LASVectorClassify::PrintPointsError(Point3Ds knownPoints, Point3Ds lidarPoints)
{
	printf("\r\n");
	double maxD = -99999999999;
	double minD = 99999999999;
	double totalD = 0;
	for (size_t i = 0; i < knownPoints.size(); i++)
	{
		double dx = knownPoints[i].x - lidarPoints[i].x;
		double dy = knownPoints[i].y - lidarPoints[i].y;
		double dz = knownPoints[i].z - lidarPoints[i].z;
		double D = sqrt(pow(dx,2) + pow(dy, 2) + pow(dz, 2));
		printf("Point ID %d:  %3f   %3f   %3f , D: %2f\r\n",i, dx, dy, dz, D);
		if (maxD < D)
		{
			maxD = D;
		}
		if (minD > D)
		{
			minD = D;
		}
		totalD += pow(D,2);
	}
	printf("RMS: %2f,MaxError: %2f,MinError: %2f\r\n",sqrt(totalD / knownPoints.size()),maxD,minD);
}

//void LASVectorClassify::OutLineModelToShp(string fileName, string filePath, WireModel lineModel, double sampleSpacing)
//{
//	const char *pszDriverName = "ESRI Shapefile";
//	GDALDriver *poDriver;
//	GDALAllRegister();
//	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
//	if (poDriver == NULL)
//	{
//		printf("%s driver not available.\n", pszDriverName);
//		return;
//	}
//	GDALDataset *poDS;
//
//	string file = filePath + "\\" + fileName + ".shp";
//	poDS = poDriver->Create(file.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建shp文件
//	if (poDS == NULL)
//	{
//		printf("Creation of output file failed.\n");
//		return;
//	}
//	OGRLayer *poLayer;
//	poLayer = poDS->CreateLayer("Line", NULL, wkbMultiLineString, NULL);
//	if (poLayer == NULL)
//	{
//		printf("Layer creation failed.\n");
//		return;
//	}
//	OGRFieldDefn firstField("X", OFTReal);
//	OGRFieldDefn secondField("Y", OFTReal);
//	OGRFieldDefn thirdField("Z", OFTReal);
//	firstField.SetWidth(32);
//	secondField.SetWidth(32);
//	thirdField.SetWidth(32);
//	poLayer->CreateField(&firstField);
//	poLayer->CreateField(&secondField);
//	poLayer->CreateField(&thirdField);
//
//	OGRFeature *poFeature;
//	poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
//	OGRMultiLineString multiCurve;
//	double lineAngle = atan(lineModel.planeLineFactors[1]);
//	for (double i = 0; i < lineModel.towerSpan; i += sampleSpacing)
//	{
//		Point2D strightLinePoint(lineModel.hangPointA.x + i * cos(lineAngle), lineModel.hangPointA.y + i * sin(lineAngle));
//		double s = Point2DToScaleFactor(strightLinePoint, lineModel.oFootPoint, lineAngle);
//		double z = GetYByLineFactors(lineModel.sectionLineFactors, s);
//		OGRPoint curvePoint(strightLinePoint.x, strightLinePoint.y, z);
//		multiCurve.addGeometry(curvePoint);
//	}
//
//	int x, y;
//	int a = 10, b = 100;
//	for (int i = 1; i <= 10; i++)
//	{
//		OGRFeature *poFeature;
//		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
//		poFeature->SetField("ID", i);
//		poFeature->SetField("NAME", i);
//		x = (rand() % (b - a)) + a;
//		y = (rand() % (b - a)) + a;
//		poFeature->SetField("X", x);
//		poFeature->SetField("Y", y);
//		OGRPoint pt;
//		pt.setX(x);
//		pt.setY(y);
//		poFeature->SetGeometry(&pt);
//		if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
//		{
//			printf("Failed to create feature in shapefile.\n");
//			return;
//		}
//		OGRFeature::DestroyFeature(poFeature);
//	}
//	GDALClose(poDS);
//}
#pragma endregion

///////////////////////////////////////////////////////////////////////////////////////////////////
void LASShpClassify::LASClassifyByVector(const char* pathLas, map<string,string> pShpPath, const char* pathOutLas)
{
	//???LAS
	//ILASDataset *datasetLas = new ILASDataset();
	FILE* fLasIn = nullptr, *fLasOut = nullptr;
	fLasIn = fopen(pathLas, "rb");
	fLasOut = fopen(pathOutLas, "wb");

	if (fLasIn == nullptr || fLasOut == nullptr)
		return;

	LASHeader    headerLas;
	LidarPatchReader patchReaderLine;
	patchReaderLine.LidarReader_ReadHeader(fLasIn, headerLas);
	patchReaderLine.LidarReader_WriteHeader(fLasOut, headerLas);

	//FILE* fOutTxt = fopen("D:\\Program\\Data\\TestShp\\DRIVEWAY2.txt", "wb"); //////

	int pointReserved = headerLas.number_of_point_records;
	const int readPatch = 100000;
	LASPoint* lasPnt = nullptr;
	try
	{
		lasPnt = new LASPoint[readPatch];
	}
	catch (bad_alloc e) {
		printf("%s\n", e.what());
		return;
	}
	
	GDALAllRegister();
	CPLSetConfigOption("SHAPE_ENCODING", "");

	int realReadPoints = 0;
	while (!feof(fLasIn)) {
		if(pointReserved<readPatch)
			realReadPoints = pointReserved;
		else
			realReadPoints = readPatch;

		Rect2D rect=patchReaderLine.LidarReader_ReadPatch(fLasIn, headerLas, lasPnt, realReadPoints);

		if (realReadPoints == 0) { break; }   //add by lan

		for(auto iterShp : pShpPath){
			string path = iterShp.first;
			string type = iterShp.second;

			GDALDataset *pShpData = (GDALDataset*)GDALOpenEx(path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
			if (pShpData == NULL)
			{
				printf("SHP File Open Failed!\n");
				return;
			}
			for (size_t j = 0; j < pShpData->GetLayerCount(); ++j) {
				OGRLayer *pLayer = pShpData->GetLayer(j);
				pLayer->ResetReading();
				pLayer->SetSpatialFilterRect(rect.minx, rect.miny, rect.maxx, rect.maxy);
				int count = pLayer->GetFeatureCount();

				if (count <= 0)
					continue;

				OGRFeature *pFeature;
				while ((pFeature = pLayer->GetNextFeature()) != NULL)
				{
					OGRFeatureDefn *poFDefn = pLayer->GetLayerDefn();
					OGRGeometry *poGeometry;
					poGeometry = pFeature->GetGeometryRef();
					OGRwkbGeometryType geoWkbType = poGeometry->getGeometryType();
					if (geoWkbType == wkbPolygon)
					{
						OGRPolygon *polygon = (OGRPolygon*)poGeometry;
						for (int k = 0; k < realReadPoints; ++k) {
							OGRPoint *pnt = new OGRPoint(lasPnt[k].m_vec3d.x, lasPnt[k].m_vec3d.y);
							if (polygon->IsPointOnSurface(pnt))
							{
								//fprintf(fOutTxt, "%f %f\r\n", pnt->getX(), pnt->getY());   /////
								std::map<std::string, int>::iterator it = name_type_maps.find(type);
								if (it != name_type_maps.end())
									lasPnt[k].m_classify = it->second;
								else
									lasPnt[k].m_classify = elcUnclassified;
							}
						}
					}
					else if (geoWkbType == wkbMultiPolygon) {
						OGRMultiPolygon *polygonM = (OGRMultiPolygon*)poGeometry;
						for (int n = 0; n < polygonM->getNumGeometries(); n++)
						{
							OGRGeometry * FirstGeometry = polygonM->getGeometryRef(n);
							if (FirstGeometry == NULL)
							{
								continue;
							}
							OGRPolygon * poMultiPolygon = (OGRPolygon *)FirstGeometry;
							if (poMultiPolygon->getExteriorRing() == NULL)
							{
								continue;
							}
							OGRLinearRing * poMultiLinearRing = poMultiPolygon->getExteriorRing();

							for (int k = 0; k < realReadPoints; ++k) {
								OGRPoint *pnt = new OGRPoint(lasPnt[k].m_vec3d.x, lasPnt[k].m_vec3d.y);
								if (poMultiLinearRing->isPointInRing(pnt))
								{
									//fprintf(fOutTxt, "%f %f\r\n", pnt->getX(), pnt->getY());   /////
									std::map<std::string, int>::iterator it = name_type_maps.find(type);
									if (it != name_type_maps.end())
										lasPnt[k].m_classify = it->second;
									else
										lasPnt[k].m_classify = elcUnclassified;
								}
							}
						}
					}
					else
					{
						printf("not supportted polygon type\n");
					}
				}
			}

		}
		patchReaderLine.LidarReader_WritePatch(fLasOut, headerLas, lasPnt, realReadPoints);
		pointReserved=pointReserved-realReadPoints;
	};//while

	delete[]lasPnt; lasPnt = nullptr;
	fclose(fLasIn); fLasIn = nullptr;
	fclose(fLasOut); fLasOut = nullptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void LASShpClassify::LASClassifyBySingleTypeVector(std::vector <string> lasPaths, string lasOutPaths, string pShpPath, const char classifyType, int algorithmType)
{
	FILE *fLasOut = nullptr;
	fLasOut = fopen(lasOutPaths.c_str(), "wb");
	if (fLasOut == nullptr)
		return;
	for (int lasPathCount = 0; lasPathCount < lasPaths.size(); lasPathCount++)
	{
		FILE* fLasIn = nullptr;
		fLasIn = fopen(lasPaths.at(lasPathCount).c_str(), "rb");

		if (fLasIn == nullptr)
			continue;

		LASHeader    headerLas;
		LidarPatchReader patchReaderLine;
		patchReaderLine.LidarReader_ReadHeader(fLasIn, headerLas);
		patchReaderLine.LidarReader_WriteHeader(fLasOut, headerLas);

		//FILE* fOutTxt = fopen("D:\\Program\\Data\\TestShp\\DRIVEWAY3.txt", "wb"); //////

		int pointReserved = headerLas.number_of_point_records;
		const int readPatch = 100000;
		LASPoint* lasPnt = nullptr;
		try
		{
			lasPnt = new LASPoint[readPatch];
		}
		catch (bad_alloc e) {
			printf("%s\n", e.what());
			return;
		}

		GDALAllRegister();
		CPLSetConfigOption("SHAPE_ENCODING", "");

		GeometryRelation geometryRelation;

		int realReadPoints = 0;
		while (!feof(fLasIn)) {
			if (pointReserved<readPatch)
				realReadPoints = pointReserved;
			else
				realReadPoints = readPatch;

			Rect2D rect = patchReaderLine.LidarReader_ReadPatch(fLasIn, headerLas, lasPnt, realReadPoints);

			if (realReadPoints == 0) { break; }   //add by lan

			GDALDataset *pShpData = (GDALDataset*)GDALOpenEx(pShpPath.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
			if (pShpData == NULL)
			{
				printf("SHP File Open Failed!\n");
				return;
			}
			for (size_t j = 0; j < pShpData->GetLayerCount(); ++j) {
				OGRLayer *pLayer = pShpData->GetLayer(j);
				pLayer->ResetReading();
				pLayer->SetSpatialFilterRect(rect.minx, rect.miny, rect.maxx, rect.maxy);
				int count = pLayer->GetFeatureCount();

				if (count <= 0)
					continue;

				OGRFeature *pFeature;
				while ((pFeature = pLayer->GetNextFeature()) != NULL)
				{
					OGRFeatureDefn *poFDefn = pLayer->GetLayerDefn();
					OGRGeometry *poGeometry;
					poGeometry = pFeature->GetGeometryRef();
					OGRwkbGeometryType geoWkbType = poGeometry->getGeometryType();
					if (geoWkbType == wkbPolygon)
					{
						if (algorithmType == 0)
						{
							OGRPolygon *polygon = (OGRPolygon*)poGeometry;
							OGRLinearRing * pOGRLinearRing = polygon->getExteriorRing();
							int point_count = 0;
							point_count = pOGRLinearRing->getNumPoints();
							if (point_count < 3) { continue; }
							OGRRawPoint *Gpoints = new OGRRawPoint[point_count];
							pOGRLinearRing->getPoints(Gpoints, NULL);
							Point2Ds polygonPoints;
							for (int k = 0; k <point_count; k++)
							{
								Point2D point2D;
								point2D.x = Gpoints[k].x;
								point2D.y = Gpoints[k].y;
								polygonPoints.push_back(point2D);
							}

							for (int k = 0; k < realReadPoints; ++k) {
								if (GeometryRelation::IsPointInPolygon(lasPnt[k].m_vec3d.x, lasPnt[k].m_vec3d.y, polygonPoints))
								{
									//fprintf(fOutTxt, "%f %f\r\n", lasPnt[k].m_vec3d.x, lasPnt[k].m_vec3d.y);   /////
									lasPnt[k].m_classify = classifyType;
								}
							}

							delete[]Gpoints; Gpoints = NULL;
						}
						else if(algorithmType == 1)
						{
							OGRPolygon *polygon = (OGRPolygon*)poGeometry;
							for (int k = 0; k < realReadPoints; ++k) {
								OGRPoint *pnt = new OGRPoint(lasPnt[k].m_vec3d.x, lasPnt[k].m_vec3d.y);
								if (polygon->IsPointOnSurface(pnt))
								{
									//fprintf(fOutTxt, "%f %f\r\n", pnt->getX(), pnt->getY());   /////
									lasPnt[k].m_classify = classifyType;
								}
							}
						}
					}
					else if (geoWkbType == wkbMultiPolygon) {
						if (algorithmType == 0)
						{
							OGRMultiPolygon *polygonM = (OGRMultiPolygon*)poGeometry;
							for (int n = 0; n < polygonM->getNumGeometries(); n++)
							{
								OGRGeometry * FirstGeometry = polygonM->getGeometryRef(n);
								if (FirstGeometry == NULL)
								{
									continue;
								}
								OGRPolygon * poMultiPolygon = (OGRPolygon *)FirstGeometry;
								if (poMultiPolygon->getExteriorRing() == NULL)
								{
									continue;
								}
								OGRLinearRing * poMultiLinearRing = poMultiPolygon->getExteriorRing();
								int point_count = 0;
								point_count = poMultiLinearRing->getNumPoints();
								if (point_count < 3) { continue; }
								OGRRawPoint *Gpoints = new OGRRawPoint[point_count];
								poMultiLinearRing->getPoints(Gpoints, NULL);
								Point2Ds polygonPoints;
								for (int k = 0; k <point_count; k++)
								{
									Point2D point2D;
									point2D.x = Gpoints[k].x;
									point2D.y = Gpoints[k].y;
									polygonPoints.push_back(point2D);
								}

								for (int k = 0; k < realReadPoints; ++k) {
									if (GeometryRelation::IsPointInPolygon(lasPnt[k].m_vec3d.x, lasPnt[k].m_vec3d.y, polygonPoints))
									{
										lasPnt[k].m_classify = classifyType;
									}
								}

								delete[]Gpoints; Gpoints = NULL;
							}
						}
						else if (algorithmType == 1)
						{
							OGRMultiPolygon *polygonM = (OGRMultiPolygon*)poGeometry;
							for (int n = 0; n < polygonM->getNumGeometries(); n++)
							{
								OGRGeometry * FirstGeometry = polygonM->getGeometryRef(n);
								if (FirstGeometry == NULL)
								{
									continue;
								}
								OGRPolygon * poMultiPolygon = (OGRPolygon *)FirstGeometry;
								if (poMultiPolygon->getExteriorRing() == NULL)
								{
									continue;
								}
								OGRLinearRing * poMultiLinearRing = poMultiPolygon->getExteriorRing();

								for (int k = 0; k < realReadPoints; ++k) {
									OGRPoint *pnt = new OGRPoint(lasPnt[k].m_vec3d.x, lasPnt[k].m_vec3d.y);
									if (poMultiLinearRing->isPointInRing(pnt))
									{
										//fprintf(fOutTxt, "%f %f\r\n", pnt->getX(), pnt->getY());   /////
										lasPnt[k].m_classify = classifyType;
									}
								}
							}
						}
					}
					else
					{
						printf("not supportted polygon type\n");
					}
				}
			}
			patchReaderLine.LidarReader_WritePatch(fLasOut, headerLas, lasPnt, realReadPoints);
			pointReserved = pointReserved - realReadPoints;
		};//while

		delete[]lasPnt; lasPnt = nullptr;
		fclose(fLasIn); fLasIn = nullptr;
	}
	fclose(fLasOut); fLasOut = nullptr;
}