//
// Created by wuwei on 17-12-26.
//

#include "LASReader.h"
#include <stdio.h>

#include "LASPoint.h"

#define LargestMemoryToRead 2147483648
#define BlockPointNumbers 2048

long LidarMemReader::LidarReader_Open(const char* pathLidar, _OUT_ ILASDataset* dataset)
{
	assert(dataset != nullptr);

	if (m_lasFile != nullptr)
		fclose(m_lasFile);
	m_lasFile = fopen(pathLidar, "rb");
	if (m_lasFile == nullptr)
		return -1;

	//读取头文件
	LASHeader &refHeader = dataset->m_lasHeader;
	refHeader.ReadHeader(m_lasFile);
	if (refHeader.number_of_variable_length_records == 0)
	{
		//没有变长字段
	}
	else
	{
		//读取变长字段
		dataset->m_lasvarHeader = new LASVariableRecord[refHeader.number_of_variable_length_records];
		for (int i = 0; i < refHeader.number_of_variable_length_records; ++i)
		{
			dataset->m_lasvarHeader[i].Read(m_lasFile);
		}
	}
	m_isDatasetOpen = true;
	return 0;
}

long LidarMemReader::LidarReader_Read(bool inMemory, int skip, _OUT_ ILASDataset* dataset)
{
	if (!m_isDatasetOpen)
	{
		printf("do not open file\n");
		return -1;
	}

	LASHeader &refLasHeader = dataset->m_lasHeader;
	fseek(m_lasFile, refLasHeader.offset_to_point_data, SEEK_SET);
	int totalLasNumber = 0;
	long long  memoryUsed = 0;
	totalLasNumber = refLasHeader.number_of_point_records / skip;
	memoryUsed = totalLasNumber*refLasHeader.point_data_record_length;

	if (inMemory&&memoryUsed > LargestMemoryToRead)
	{
		printf("too large memory to allocate\n");
		printf("advice to upper samples\n");
		return -1;
	}

	int *pointsRect = NULL;
	try {
		double width = (refLasHeader.max_x - refLasHeader.min_x);
		double height = (refLasHeader.max_y - refLasHeader.min_y);
		double scaleWH = width / height;
		double sqrt_2 = (sqrt(double(totalLasNumber) / double(BlockPointNumbers) / scaleWH));
		//每个块的大小
		double widthPer = width / sqrt_2 + 0.5;;
		double heightPer = height / sqrt_2 + 0.5;;
		int widthNum = ceil(width / widthPer)+1;
		int heightNum = ceil(height / heightPer)+1;

		//反正先统计信息
		//分配内存
		dataset->LASDataset_AllocateMemory(widthNum*heightNum);
		//构建R树结构
		for (int i = 0; i < widthNum; ++i)
		{
			for (int j = 0; j < heightNum; ++j)
			{
				dataset->m_lasRectangles[j*widthNum + i].m_Rectangle.Set(i*widthPer + refLasHeader.min_x,
					j*heightPer + refLasHeader.min_y,
					(i + 1)*widthPer + refLasHeader.min_x,
					(j + 1)*heightPer + refLasHeader.min_y);
			}
		}
		dataset->LASDataset_BuildTree();
		dataset->m_totalReadLasNumber = totalLasNumber;
		dataset->m_LASPointID = new LASIndex[totalLasNumber];

		/*
		//每一个矩形区域中的点的索引
		//deleted by Frank.Wu
		//以前的时候为了省内存通过LASPoint* 进行存储，因此需要有两遍遍历，
		//第一遍先获取到每个Rect内的点的数目，然后申请内存
		//第二遍才能够获取每个点，现在采用vector动态分配内存，提高读取效率但是内存利用率下降了
		//pointsRect = new int[widthNum*heightNum];
		//memset(pointsRect, 0, sizeof(int)*widthNum*heightNum);
		//LidarReader_RectNumbers(m_lasFile, dataset, widthPer, heightPer, widthNum, heightNum, skip, pointsRect);

		//回到数据起点
		fseek(m_lasFile, refLasHeader.offset_to_point_data, SEEK_SET);

		//第二遍遍历获取数据
		for (int j = 0; j < widthNum*heightNum; ++j)
			dataset->m_lasRectangles[j].LASRect_AllocateMemory(pointsRect[j], inMemory, dataset->m_lasRectangles[j].m_Rectangle);
		memset(pointsRect, 0, sizeof(int)*widthNum*heightNum);
		int totalRead = 0;
		LidarReader_RectPoints(m_lasFile, dataset, widthPer, heightPer, widthNum, heightNum, inMemory, skip, totalRead, pointsRect);
		*/
		totalLasNumber = 0;
		LidarReader_RectPoints(m_lasFile, dataset, widthPer, heightPer, widthNum, heightNum, inMemory, skip, totalLasNumber);
	}
	catch (bad_alloc &e)
	{
		printf("%s\n", e.what());
		return -1;
	}

	dataset->LASDataset_Trim(inMemory);
	if (pointsRect != NULL)
		delete[]pointsRect;
	pointsRect = NULL;

	return 0;
}

long LidarMemReader::LidarReader_Write(const char *pathLidar, ILASDataset* dataset)
{
	assert(dataset != nullptr);

	if (dataset->m_lasRectangles == nullptr)
	{
		printf("no las data\n");
		return -1;
	}
	//新建一个LASHeader
	LASHeader &refHeader = dataset->m_lasHeader;
	LASHeader lasHeader(refHeader);
	int totalPoints = 0;

	//写数据之前重新检查一下以免出现不一致的现象
	for (size_t i = 0; i < dataset->m_numRectangles; i++)
	{
		for (size_t j = 0; j < dataset->m_lasRectangles[i].m_lasPoints_numbers; ++j)
		{
			lasHeader.max_x = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x, lasHeader.max_x);
			lasHeader.min_x = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x, lasHeader.min_x);
			lasHeader.max_y = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y, lasHeader.max_y);
			lasHeader.min_y = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y, lasHeader.min_y);
			lasHeader.max_z = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z, lasHeader.max_z);
			lasHeader.min_z = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z, lasHeader.min_z);
		}
		totalPoints += dataset->m_lasRectangles[i].m_lasPoints_numbers;
	}
	lasHeader.number_of_point_records = totalPoints;
	lasHeader.point_data_record_length = 34;
	lasHeader.point_data_format = 3;
	lasHeader.version_major = 1;
	lasHeader.version_minor = 2;
	FILE* fLasOut = fopen(pathLidar, "wb");
	if (fLasOut == nullptr)
	{
		printf("create las file failed!\n");
		return -1;
	}
	lasHeader.WriteHeader(fLasOut);

	//中间有问题 不知道怎么搞 变长字段还没有进行处理
	int sizeBuff = lasHeader.offset_to_point_data - sizeof(LASHeader);
	if (sizeBuff != 0)
	{
		char* buffer = new char[sizeBuff];
		memset(buffer, 0, sizeof(char) * sizeBuff);
		fwrite(buffer, 1, sizeBuff, fLasOut);
		delete[]buffer; buffer = NULL;
	}

	//写数据
	for (int k = 0; k<dataset->m_totalReadLasNumber; ++k)
	{
		//printf("\r%d", k);
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;

		int x = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x - lasHeader.x_offset) / lasHeader.x_scale_factor;
		int y = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y - lasHeader.y_offset) / lasHeader.y_scale_factor;
		int z = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z - lasHeader.z_offset) / lasHeader.z_scale_factor;
		fwrite(&x, sizeof(int), 1, fLasOut);
		fwrite(&y, sizeof(int), 1, fLasOut);
		fwrite(&z, sizeof(int), 1, fLasOut);

		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_intensity, sizeof(unsigned short), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_rnseByte, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_classify, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_scanAngle, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_userdata, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_flightID, sizeof(unsigned short), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_gpsTime, sizeof(double), 1, fLasOut);
		//printf("%d\n",&dataset->m_lasRectangles[i].m_lasPoints[j].m_colorExt.Red);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_colorExt, sizeof(LASColorExt), 1, fLasOut);
	}
	fclose(fLasOut);
	return 0;
}

long LidarMemReader::LidarReader_Write(const char* pathLidar, ILASDataset* dataset, eLASClassification classType)
{
	assert(dataset != nullptr);

	if (dataset->m_lasRectangles == nullptr)
	{
		printf("no las data\n");
		return -1;
	}
	//新建一个LASHeader
	LASHeader &refHeader = dataset->m_lasHeader;

	LASHeader lasHeader(refHeader);
	int totalPoints = 0;

	for (size_t i = 0; i < dataset->m_numRectangles; i++)
	{
		for (size_t j = 0; j < dataset->m_lasRectangles[i].m_lasPoints_numbers; ++j)
		{
			if (dataset->m_lasRectangles[i].m_lasPoints[j].m_classify == classType)
			{
				lasHeader.max_x = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x, lasHeader.max_x);
				lasHeader.min_x = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x, lasHeader.min_x);
				lasHeader.max_y = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y, lasHeader.max_y);
				lasHeader.min_y = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y, lasHeader.min_y);
				lasHeader.max_z = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z, lasHeader.max_z);
				lasHeader.min_z = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z, lasHeader.min_z);
				++totalPoints;
			}
		}
	}
	lasHeader.number_of_point_records = totalPoints;
	lasHeader.point_data_record_length = 34;
	lasHeader.point_data_format = 3;
	FILE* fLasOut = fopen(pathLidar, "wb");
	if (fLasOut == nullptr)
		return -1;
	lasHeader.number_of_point_records = totalPoints;
	lasHeader.version_major = 1;
	lasHeader.version_minor = 2;
	lasHeader.WriteHeader(fLasOut);
	//中间有问题 不知道怎么搞 变长字段还没有进行处理
	int sizeBuff = lasHeader.offset_to_point_data - sizeof(LASHeader);
	if (sizeBuff != 0)
	{
		char* buffer = new char[sizeBuff];
		memset(buffer, 0, sizeof(char) * sizeBuff);
		fwrite(buffer, 1, sizeBuff, fLasOut);
		delete[]buffer; buffer = NULL;
	}

	for (int k = 0; k<dataset->m_totalReadLasNumber; ++k)
	{
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;
		if (dataset->m_lasRectangles[i].m_lasPoints[j].m_classify == classType)
		{
			int x = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x - lasHeader.x_offset) / lasHeader.x_scale_factor;
			int y = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y - lasHeader.y_offset)/ lasHeader.y_scale_factor;
			int z = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z - lasHeader.z_offset) / lasHeader.z_scale_factor;
			
			fwrite(&x, sizeof(int), 1, fLasOut);
			fwrite(&y, sizeof(int), 1, fLasOut);
			fwrite(&z, sizeof(int), 1, fLasOut);

			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_intensity, sizeof(unsigned short), 1, fLasOut);
			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_rnseByte, sizeof(unsigned char), 1, fLasOut);
			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_classify, sizeof(unsigned char), 1, fLasOut);
			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_scanAngle, sizeof(unsigned char), 1, fLasOut);
			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_userdata, sizeof(unsigned char), 1, fLasOut);
			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_flightID, sizeof(unsigned short), 1, fLasOut);
			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_gpsTime, sizeof(double), 1, fLasOut);
			fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_colorExt, sizeof(LASColorExt), 1, fLasOut);
		}
	}
	fclose(fLasOut);
	return 0;
}

long LidarMemReader::LidarReader_WriteWithColor(const char* pathLidar, ILASDataset* dataset, LASColorExt color)
{
	assert(dataset != nullptr);

	if (dataset->m_lasRectangles == nullptr)
	{
		printf("no las data\n");
		return -1;
	}
	//新建一个LASHeader
	LASHeader &refHeader = dataset->m_lasHeader;

	LASHeader lasHeader(refHeader);
	int totalPoints = 0;

	for (size_t i = 0; i < dataset->m_numRectangles; i++)
	{
		for (size_t j = 0; j < dataset->m_lasRectangles[i].m_lasPoints_numbers; ++j)
		{
			lasHeader.max_x = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x, lasHeader.max_x);
			lasHeader.min_x = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x, lasHeader.min_x);
			lasHeader.max_y = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y, lasHeader.max_y);
			lasHeader.min_y = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y, lasHeader.min_y);
			lasHeader.max_z = max(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z, lasHeader.max_z);
			lasHeader.min_z = min(dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z, lasHeader.min_z);
		}
		totalPoints += dataset->m_lasRectangles[i].m_lasPoints_numbers;
	}
	FILE* fLasOut = fopen(pathLidar, "wb+");
	if (fLasOut == nullptr)
		return -1;
	lasHeader.number_of_point_records = totalPoints;
	lasHeader.point_data_record_length = 34;
	lasHeader.point_data_format = 3;
	lasHeader.number_of_point_records = totalPoints;
	lasHeader.version_major = 1;
	lasHeader.version_minor = 2;
	lasHeader.WriteHeader(fLasOut);
	//中间有问题 不知道怎么搞 变长字段还没有进行处理
	int sizeBuff = lasHeader.offset_to_point_data - sizeof(LASHeader);
	if (sizeBuff != 0)
	{
		char* buffer = new char[sizeBuff];
		memset(buffer, 0, sizeof(char) * sizeBuff);
		fwrite(buffer, 1, sizeBuff, fLasOut);
		delete[]buffer; buffer = NULL;
	}

	for (int k = 0; k<dataset->m_totalReadLasNumber; ++k)
	{
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;

		int x = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x - lasHeader.x_offset) / lasHeader.x_scale_factor;
		int y = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y - lasHeader.y_offset) / lasHeader.y_scale_factor;
		int z = (dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z - lasHeader.z_offset) / lasHeader.z_scale_factor;
		fwrite(&x, sizeof(int), 1, fLasOut);
		fwrite(&y, sizeof(int), 1, fLasOut);
		fwrite(&z, sizeof(int), 1, fLasOut);

		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_intensity, sizeof(unsigned short), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_rnseByte, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_classify, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_scanAngle, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_userdata, sizeof(unsigned char), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_flightID, sizeof(unsigned short), 1, fLasOut);
		fwrite(&dataset->m_lasRectangles[i].m_lasPoints[j].m_gpsTime, sizeof(double), 1, fLasOut);
		//if (isColorEx)
		fwrite(&color, sizeof(LASColorExt), 1, fLasOut);
	}
	fclose(fLasOut);
	return 0;
}

long LidarMemReader::LidarReader_ReadMerge(std::vector<string> lasFiles, _OUT_ ILASDataset* dataset)
{
	assert(dataset != nullptr);
	int pointDataRecord = 20;
	int totalRecordPointNumber = 0;
	//file header
	LASHeader &refHeader = dataset->m_lasHeader;
	for (int i = 0; i < lasFiles.size(); ++i)
	{
		FILE* fs = fopen(lasFiles[i].c_str(), "rb");
		LASHeader tmpHeader;
		tmpHeader.ReadHeader(fs);
		refHeader.max_x = max(tmpHeader.max_x, refHeader.max_x);
		refHeader.max_y = max(tmpHeader.max_y, refHeader.max_y);
		refHeader.max_z = max(tmpHeader.max_z, refHeader.max_z);
		refHeader.min_x = min(tmpHeader.min_x, refHeader.min_x);
		refHeader.min_y = min(tmpHeader.min_y, refHeader.min_y);
		refHeader.min_z = min(tmpHeader.min_z, refHeader.min_z);
		totalRecordPointNumber += tmpHeader.number_of_point_records;
		fclose(fs);
	}

	//构建树结构
	int *pointsRect = NULL;
	double width = (refHeader.max_x - refHeader.min_x);
	double height = (refHeader.max_y - refHeader.min_y);
	double scaleWH = width / height;
	double sqrt_2 = (sqrt(double(totalRecordPointNumber) / double(BlockPointNumbers) / scaleWH));
	//每个块的大小
	double widthPer = width / sqrt_2 + 0.5;;
	double heightPer = height / sqrt_2 + 0.5;;
	int widthNum = ceil(width / widthPer)+1;
	int heightNum = ceil(height / heightPer)+1;
	dataset->LASDataset_AllocateMemory(widthNum*heightNum);
	dataset->m_LASPointID = new LASIndex[totalRecordPointNumber];
	//构建R树结构
	for (int i = 0; i < widthNum; ++i)
	{
		for (int j = 0; j < heightNum; ++j)
		{
			dataset->m_lasRectangles[j*widthNum + i].m_Rectangle.Set(i*widthPer + refHeader.min_x,
				j*heightPer + refHeader.min_y,
				(i + 1)*widthPer + refHeader.min_x,
				(j + 1)*heightPer + refHeader.min_y);
		}
	}


	//pointsRect = new int[widthNum*heightNum];
	//memset(pointsRect, 0, sizeof(int)*widthNum*heightNum);
	//for (int i = 0; i < lasFiles.size(); ++i)
	//{
	//	FILE* fs = fopen(lasFiles[i].c_str(), "rb");
	//	LASHeader tmpHeader;
	//	tmpHeader.ReadHeader(fs);
	//	fseek(fs, tmpHeader.offset_to_point_data, SEEK_SET);
	//	pointDataRecord = tmpHeader.point_data_record_length;

	//	// 主要是为了解决不同数据集不规范引起的问题（下同）
	//	dataset->m_lasHeader.point_data_record_length = pointDataRecord;
	//	dataset->m_lasHeader.number_of_point_records = tmpHeader.number_of_point_records;
	//	dataset->m_lasHeader.x_offset = tmpHeader.x_offset;
	//	dataset->m_lasHeader.y_offset = tmpHeader.y_offset;
	//	dataset->m_lasHeader.z_offset = tmpHeader.z_offset;
	//	dataset->m_lasHeader.x_scale_factor = tmpHeader.x_scale_factor;
	//	dataset->m_lasHeader.y_scale_factor = tmpHeader.y_scale_factor;
	//	dataset->m_lasHeader.z_scale_factor = tmpHeader.z_scale_factor;
	//	LidarReader_RectNumbers(fs, dataset, widthPer, heightPer, widthNum, heightNum, 1, pointsRect);
	//	fclose(fs);
	//}

	//for (int j = 0; j < widthNum*heightNum; ++j)
	//	dataset->m_lasRectangles[j].LASRect_AllocateMemory(pointsRect[j], true, dataset->m_lasRectangles[j].m_Rectangle);
	//memset(pointsRect, 0, sizeof(int)*widthNum*heightNum);
	
	//dataset->m_LASPointID = new LASIndex[totalRecordPointNumber];
	int totalRead = 0;

	for (int i = 0; i < lasFiles.size(); ++i)
	{
		FILE* fs = fopen(lasFiles[i].c_str(), "rb");
		LASHeader tmpHeader;
		tmpHeader.ReadHeader(fs);
		fseek(fs, tmpHeader.offset_to_point_data, SEEK_SET);
		pointDataRecord = tmpHeader.point_data_record_length;

		// 主要是为了解决不同数据集不规范引起的问题（下同）
		dataset->m_lasHeader.point_data_record_length = pointDataRecord;
		dataset->m_lasHeader.number_of_point_records = tmpHeader.number_of_point_records;
		dataset->m_lasHeader.x_offset = tmpHeader.x_offset;
		dataset->m_lasHeader.y_offset = tmpHeader.y_offset;
		dataset->m_lasHeader.z_offset = tmpHeader.z_offset;
		dataset->m_lasHeader.x_scale_factor = tmpHeader.x_scale_factor;
		dataset->m_lasHeader.y_scale_factor = tmpHeader.y_scale_factor;
		dataset->m_lasHeader.z_scale_factor = tmpHeader.z_scale_factor;
		LidarReader_RectPoints(fs, dataset, widthPer, heightPer, widthNum, heightNum, true,1, totalRead);
		fclose(fs);
	}
	dataset->m_lasHeader.number_of_point_records = totalRecordPointNumber;
	dataset->m_totalReadLasNumber = totalRecordPointNumber;
	dataset->m_lasHeader.point_data_record_length = 34;
	dataset->m_lasHeader.x_offset = 0;
	dataset->m_lasHeader.y_offset = 0;
	dataset->m_lasHeader.z_offset = 0;
	dataset->m_lasHeader.x_scale_factor = 0.01;
	dataset->m_lasHeader.y_scale_factor = 0.01;
	dataset->m_lasHeader.z_scale_factor = 0.01;
	dataset->LASDataset_Trim(true);
	if (pointsRect != NULL)
		delete[]pointsRect;
	pointsRect = NULL;

	return 0;
}

long LidarMemReader::LidarReader_Export(const char* pathLidar, ILASDataset* dataset, int classType)
{
	FILE* fw = fopen(pathLidar, "w+");
	if (fw == nullptr)
		return -1;

	bool color = false;
	if (dataset->m_lasHeader.HasLASColorExt4() || dataset->m_lasHeader.HasLASColorExt6())
	{
		color = true;
	}
	color = true;
	for (int k = 0; k<dataset->m_totalReadLasNumber; ++k)
	{
		int i = dataset->m_LASPointID[k].rectangle_idx;
		int j = dataset->m_LASPointID[k].point_idx_inRect;

		if (dataset->m_lasRectangles[i].m_lasPoints[j].m_classify == classType)
		{
			if (color)
			{
				double x = dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x,
					y = dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y,
					z = dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z;

				int r = dataset->m_lasRectangles[i].m_lasPoints[j].m_colorExt.Red,
					g = dataset->m_lasRectangles[i].m_lasPoints[j].m_colorExt.Green,
					b = dataset->m_lasRectangles[i].m_lasPoints[j].m_colorExt.Blue;
				fprintf(fw, "%8.4lf  %8.4lf  %8.4lf  %d  %d %d\n", x, y, z, r, g, b);
				//printf("%8.4lf  %8.4lf  %8.4lf  %d  %d %d\n", x, y, z, r, g, b);
			}
			else {
				double x = dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.x,
					y = dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.y,
					z = dataset->m_lasRectangles[i].m_lasPoints[j].m_vec3d.z;
				fprintf(fw, "%8.4lf  %8.4lf  %8.4lf\n", x, y, z);
				//printf("%8.4lf  %8.4lf  %8.4lf\n", x, y, z);
			}
		}
	}
	fclose(fw);
	return 0;
}

long LidarMemReader::LidarReader_RectNumbers(FILE* fs, ILASDataset* dataset, double widthPre, double heightPre,
	int widthNum, int heightNum,int skip, _OUT_ int *numPtsRect)
{
	LASHeader &refLasHeader = dataset->m_lasHeader;
	int read_once_max = BlockPointNumbers;
	int read_once = refLasHeader.number_of_point_records;
	long bytelen = read_once_max * refLasHeader.point_data_record_length;

	unsigned char *readOnce = new unsigned char[bytelen];
	int totalLasNumber = 0;
	int alread_read = 0;
	while (alread_read<refLasHeader.number_of_point_records)
	{
		if (read_once>read_once_max)
			read_once = read_once_max;
		int readLen = read_once * refLasHeader.point_data_record_length;
		fread(readOnce, readLen, 1, fs);

		//先读取，然后进行处理
		for (size_t i = 0; i < read_once; i += skip)
		{
			Point3D vex;
			const unsigned char* data = readOnce + i * refLasHeader.point_data_record_length;
			int size = sizeof(int);
			int x, y, z;
			memcpy(&x, data, size); memcpy(&y, data + size, size); memcpy(&z, data + size * 2, size);
			vex.x = x * refLasHeader.x_scale_factor + refLasHeader.x_offset;
			vex.y = y * refLasHeader.y_scale_factor + refLasHeader.y_offset;
			vex.z = z * refLasHeader.z_scale_factor + refLasHeader.z_offset;

			dataset->m_xrange[0] = max(dataset->m_xrange[0], vex.x); dataset->m_xrange[1] = min(dataset->m_xrange[1], vex.x);
			dataset->m_yrange[0] = max(dataset->m_yrange[0], vex.y); dataset->m_yrange[1] = min(dataset->m_yrange[1], vex.y);
			dataset->m_zrange[0] = max(dataset->m_zrange[0], vex.z); dataset->m_zrange[1] = min(dataset->m_zrange[1], vex.z);

			int widtnIdx = int((vex.x - refLasHeader.min_x) / widthPre);
			int heighIdx = int((vex.y - refLasHeader.min_y) / heightPre);
			if (widtnIdx < 0 || heighIdx < 0||widtnIdx>widthNum||heighIdx>heightNum)
				return -1;
			//计算没一个Rect中有多少个点
			numPtsRect[heighIdx*widthNum + widtnIdx]++;
			totalLasNumber++;
		}
		alread_read += read_once;
		read_once_max = min(read_once_max, int(refLasHeader.number_of_point_records - alread_read));
	}
	delete[]readOnce; readOnce = nullptr;
	return 0;
}

long LidarMemReader::LidarReader_RectPoints(FILE* fs, ILASDataset* dataset, double widthPre, double heightPre,
	int widthNum, int heightNum, bool inMemory, int skip, int &totallasPnts, _OUT_ int *pointsRect)
{
	LASHeader &refLasHeader = dataset->m_lasHeader;
	int read_once_max = BlockPointNumbers;
	int read_once = refLasHeader.number_of_point_records;
	long bytelen = read_once_max * refLasHeader.point_data_record_length;

	unsigned char *readOnce = new unsigned char[bytelen];
	int totalLasNumber = 0;
	int alread_read = 0;

	while (alread_read<refLasHeader.number_of_point_records)
	{
		if (read_once>read_once_max)
			read_once = read_once_max;
		int readLen = read_once * refLasHeader.point_data_record_length;
		fread(readOnce, readLen, 1, fs);
		//先读取，然后进行处理
		for (size_t i = 0; i < read_once; i += skip)
		{
			const unsigned char* data = readOnce + i * refLasHeader.point_data_record_length;
			LASPoint lasPnts;
			lasPnts.ExtractFromBuffer(data, refLasHeader);

			int widtnIdx = int((lasPnts.m_vec3d.x - refLasHeader.min_x) / widthPre);
			int heighIdx = int((lasPnts.m_vec3d.y - refLasHeader.min_y) / heightPre);
			//用vector导致兼容性有问题
			//if (inMemory)
			//	memcpy(dataset->m_lasRectangles[heighIdx*widthNum + widtnIdx].m_lasPoints + pointsRect[heighIdx*widthNum + widtnIdx], &lasPnts, sizeof(LASPoint));
			dataset->m_LASPointID[totallasPnts].rectangle_idx = heighIdx * widthNum + widtnIdx;
			dataset->m_LASPointID[totallasPnts].point_idx_inRect = pointsRect[heighIdx*widthNum + widtnIdx];
			pointsRect[heighIdx*widthNum + widtnIdx]++;
			totallasPnts++;
		}
		alread_read += read_once;
		read_once_max = min(read_once_max, int(refLasHeader.number_of_point_records - alread_read));
	}
	delete[]readOnce; readOnce = nullptr;
	return 0;
}

long LidarMemReader::LidarReader_RectPoints(FILE* fs, ILASDataset* dataset, double widthPre, double heightPre,
	int widthNum, int heightNum, bool inMemory, int skip, int &totallasPnts)
{
	LASHeader &refLasHeader = dataset->m_lasHeader;
	int read_once_max = BlockPointNumbers;
	int read_once = refLasHeader.number_of_point_records;
	long bytelen = read_once_max * refLasHeader.point_data_record_length;

	//反复申请小内存可能出现内存碎片影响效率需要内存池优化
	//memory pool
	unsigned char *readOnce = new unsigned char[bytelen];
	int totalLasNumber = 0;
	int alread_read = 0;

	int *pointsRect = new int[widthNum*heightNum];
	memset(pointsRect, 0, sizeof(int)*widthNum*heightNum);
	int readlasPnts = 0;
	while (alread_read<refLasHeader.number_of_point_records)
	{
		if (read_once>read_once_max)
			read_once = read_once_max;
		int readLen = read_once * refLasHeader.point_data_record_length;
		fread(readOnce, readLen, 1, fs);
		//先读取，然后进行处理
		for (size_t i = 0; i < read_once; i += skip)
		{
			const unsigned char* data = readOnce + i * refLasHeader.point_data_record_length;
			LASPoint lasPnts;
			lasPnts.ExtractFromBuffer(data, refLasHeader);

			int widtnIdx = int((lasPnts.m_vec3d.x - refLasHeader.min_x) / widthPre);
			int heighIdx = int((lasPnts.m_vec3d.y - refLasHeader.min_y) / heightPre);
			if (widtnIdx > widthNum || widtnIdx<0 || heighIdx>heightNum || heighIdx < 0)
				continue;

			if (inMemory)
				dataset->m_lasRectangles[heighIdx*widthNum + widtnIdx].m_lasPoints.push_back(lasPnts);
			dataset->m_lasRectangles[heighIdx*widthNum + widtnIdx].m_lasPoints_numbers++;
			dataset->m_LASPointID[totallasPnts+ readlasPnts].point_idx_inRect = pointsRect[heighIdx*widthNum + widtnIdx];
			dataset->m_LASPointID[totallasPnts+ readlasPnts].rectangle_idx = heighIdx * widthNum + widtnIdx;
			pointsRect[heighIdx*widthNum + widtnIdx]++;
			readlasPnts++;
		}
		alread_read += read_once;
		read_once_max = min(read_once_max, int(refLasHeader.number_of_point_records - alread_read));
	}
	totallasPnts += readlasPnts;
	for (size_t i = 0; i < widthNum*heightNum; ++i)
	{
		if (dataset->m_lasRectangles[i].m_lasPoints_numbers > 0)
		{
			dataset->m_lasRectangles[i].LASRectBuildTree();
		}
	}
	delete[]pointsRect; pointsRect = nullptr;
	delete[]readOnce; readOnce = nullptr;
	return 0;
}



long LidarPatchReader::LidarReader_ReadHeader(FILE* lf, LASHeader &lasHeader)
{
	lasHeader.ReadHeader(lf);
	//变长头
	for (int i = 0; i<lasHeader.number_of_variable_length_records; ++i)
	{
		LASVariableRecord variableRecord;
		variableRecord.Read(lf);
	}
	fseek(lf, lasHeader.offset_to_point_data, SEEK_SET);
	return 0;
}

long LidarPatchReader::LidarReader_WriteHeader(FILE *lf, LASHeader lasHeader)
{
	lasHeader.number_of_variable_length_records = 0;
	lasHeader.version_major = 1;
	lasHeader.version_minor = 2;
	lasHeader.point_data_format = 3;
	lasHeader.point_data_record_length = LASHeader::Data_Record_Length_of_Format3;
	lasHeader.WriteHeader(lf);
	//中间有问题 不知道怎么搞 变长字段还没有进行处理
	int sizeBuff = lasHeader.offset_to_point_data - sizeof(LASHeader);
	if (sizeBuff != 0)
	{
		char* buffer = new char[sizeBuff];
		memset(buffer, 0, sizeof(char) * sizeBuff);
		fwrite(buffer, 1, sizeBuff, lf);
		delete[]buffer; buffer = NULL;
	}
	return 0;
}

Rect2D LidarPatchReader::LidarReader_ReadPatch(FILE *lf, LASHeader lasHeader, LASPoint *points, int &number)
{
	double dxMin = _MAX_LIMIT_, dxMax = _MIN_LIMIT_;
	double dyMin = _MAX_LIMIT_, dyMax = _MIN_LIMIT_;

	//懒得适配直接读取
	int numreader = 0;
	unsigned char buffer[34];
	//int length = lasHeader.point_data_record_length;
	//buffer = new unsigned char[length];
	while (!feof(lf)&&numreader<number) {
		fread(buffer, lasHeader.point_data_record_length, 1, lf);
		points[numreader].ExtractFromBuffer(buffer, lasHeader);

		dxMin = min(points[numreader].m_vec3d.x, dxMin);
		dxMax = max(points[numreader].m_vec3d.x, dxMax);

		dyMin = min(points[numreader].m_vec3d.y, dyMin);
		dyMax = max(points[numreader].m_vec3d.y, dyMax);

		++numreader;
	};
	//delete[]buffer; buffer = nullptr;

	Rect2D rect;
	rect.minx = dxMin; rect.maxx = dxMax;
	rect.miny = dyMin; rect.maxy = dyMax;
	number = numreader;
	return rect;
}

long LidarPatchReader::LidarReader_WritePatch(FILE* lf, LASHeader lasHeader, LASPoint* points, int number)
{
	lasHeader.number_of_variable_length_records = 0;
	lasHeader.version_major = 1;
	lasHeader.version_minor = 2;
	lasHeader.point_data_format = 3;
	lasHeader.point_data_record_length = LASHeader::Data_Record_Length_of_Format3;

	for (int i = 0; i<number; ++i)
	{
		//points[i].ExportToBuffer(data, lasHeader);
		int x = int((points[i].m_vec3d.x - lasHeader.x_offset) / lasHeader.x_scale_factor);
		int y = int((points[i].m_vec3d.y - lasHeader.y_offset) / lasHeader.y_scale_factor);
		int z = int((points[i].m_vec3d.z - lasHeader.z_offset) / lasHeader.z_scale_factor);
		fwrite(&x, 1,sizeof(int), lf);
		fwrite(&y, 1,sizeof(int), lf);
		fwrite(&z, 1,sizeof(int), lf);
		fwrite(&points[i].m_intensity, sizeof(unsigned short), 1, lf);
		fwrite(&points[i].m_rnseByte, sizeof(unsigned char), 1, lf);
		fwrite(&points[i].m_classify, sizeof(unsigned char), 1, lf);
		fwrite(&points[i].m_scanAngle, sizeof(unsigned char), 1, lf);
		fwrite(&points[i].m_userdata, sizeof(unsigned char), 1, lf);
		fwrite(&points[i].m_flightID, sizeof(unsigned short), 1, lf);
		if (lasHeader.HasGPSTime())
			fwrite(&points[i].m_gpsTime, sizeof(double), 1, lf);
		if (lasHeader.HasLASColorExt6())
		{
			int num = sizeof(LASColorExt);
			fwrite(&points[i].m_colorExt, sizeof(LASColorExt), 1, lf);
		}
	}
	return 0;
}

/////////////////////////////////////////////////////////////////////////////
long LidarReaderTxt::LidarReader_Open(const char* pathLidar, ILASDataset* dataset)
{
	m_lasFile = fopen(pathLidar, "r+");
	dataset->m_lasHeader.version_major = 1;
	dataset->m_lasHeader.version_minor = 2;
	dataset->m_lasHeader.point_data_record_length = 34;
	dataset->m_lasHeader.point_data_format = 3;
	dataset->m_lasHeader.file_signature[0] = 'L';
	dataset->m_lasHeader.file_signature[1] = 'A';
	dataset->m_lasHeader.file_signature[2] = 'S';
	dataset->m_lasHeader.file_signature[3] = 'F';

	return 0;
}

long LidarReaderTxt::LidarReader_Read(bool inMemory, eTxtLASType type, ILASDataset* dataset)
{
	//header and build tree
	char line[2048];
	int totalPointNumber = 0;
	LASHeader &header = dataset->m_lasHeader;
	header.max_x = header.max_y = header.max_z = _MIN_LIMIT_;
	header.min_x = header.min_y = header.min_z = _MAX_LIMIT_;
	
	//first
	while (!feof(m_lasFile))
	{
		fgets(line, 2048, m_lasFile);
		double x, y, z;
		int    r, g, b;
		if(type==LASXYZ)
			sscanf(line, "%lf%lf%lf", &x, &y, &z);
		else if (type == LASXYZRGB)
			sscanf(line, "%lf%lf%lf%d%d%d", &x, &y, &z, &r, &g, &b);

		header.max_x = max(header.max_x, x);
		header.max_y = max(header.max_y, y);
		header.max_z = max(header.max_z, z);
		header.min_x = min(header.min_x, x);
		header.min_y = min(header.min_y, y);
		header.min_z = min(header.min_z, z);
		totalPointNumber++;
	};
	header.point_data_record_length = 34;
	header.number_of_point_records = totalPointNumber;
	fseek(m_lasFile, 0, SEEK_SET);
	double width = (header.max_x - header.min_x);
	double height = (header.max_y - header.min_y);
	double scaleWH = width / height;
	double sqrt_2 = (sqrt(double(totalPointNumber) / double(BlockPointNumbers) / scaleWH));
	//每个块的大小
	double widthPer = width / sqrt_2 + 0.5;;
	double heightPer = height / sqrt_2 + 0.5;;
	int widthNum = ceil(width / widthPer);
	int heightNum = ceil(height / heightPer);

	//反正先统计信息
	//分配内存
	dataset->LASDataset_AllocateMemory(widthNum*heightNum);
	//构建R树结构
	for (int i = 0; i < widthNum; ++i)
		for (int j = 0; j < heightNum; ++j)
			dataset->m_lasRectangles[j*widthNum + i].m_Rectangle.Set(i*widthPer + header.min_x, j*heightPer + header.min_y, (i + 1)*widthPer + header.min_x, (j + 1)*heightPer + header.min_y);

	dataset->LASDataset_BuildTree();

	//每一个矩形区域中的点的索引
	//deleted by Frank.Wu
	//以前的时候为了省内存通过LASPoint* 进行存储，因此需要有两遍遍历，
	//第一遍先获取到每个Rect内的点的数目，然后申请内存
	//第二遍才能够获取每个点，现在采用vector动态分配内存，提高读取效率但是内存利用率下降了
	dataset->m_LASPointID = new LASIndex[totalPointNumber];
	dataset->m_totalReadLasNumber = totalPointNumber;
	int totallasPnts = 0;
	int *pointsRect = new int[widthNum*heightNum];
	memset(pointsRect, 0, sizeof(int)*widthNum*heightNum);
	while (!feof(m_lasFile))
	{
		fgets(line, 2048, m_lasFile);
		double x, y, z;
		int    r, g, b;
		if (type == LASXYZ)
			sscanf(line, "%lf%lf%lf", &x, &y, &z);
		else if (type == LASXYZRGB)
			sscanf(line, "%lf%lf%lf%d%d%d", &x, &y, &z, &r, &g, &b);
		LASPoint lasPnts;
		lasPnts.m_classify = 1;
		if(type==LASXYZRGB)
			lasPnts.m_colorExt.Red = r; lasPnts.m_colorExt.Green = g; lasPnts.m_colorExt.Blue = b;
		lasPnts.m_vec3d.x = x; lasPnts.m_vec3d.y = y; lasPnts.m_vec3d.z = z;
		lasPnts.m_flightID = lasPnts.m_intensity = 0;
		lasPnts.m_rnseByte = 0;
		lasPnts.m_gpsTime = 0;
		lasPnts.m_scanAngle = 0;
		lasPnts.m_userdata = 0;

		int widtnIdx = int((x - header.min_x) / widthPer);
		int heighIdx = int((y - header.min_y) / heightPer);
		if (widtnIdx < 0 || heighIdx < 0)
			return -1;
		if (inMemory)
			dataset->m_lasRectangles[heighIdx*widthNum + widtnIdx].m_lasPoints.push_back(lasPnts);
		dataset->m_lasRectangles[heighIdx*widthNum + widtnIdx].m_lasPoints_numbers++;
		dataset->m_LASPointID[totallasPnts].rectangle_idx = heighIdx*widthNum + widtnIdx;
		dataset->m_LASPointID[totallasPnts].point_idx_inRect = pointsRect[heighIdx*widthNum + widtnIdx];
		pointsRect[heighIdx*widthNum + widtnIdx]++;
		++totallasPnts;
	};
	for (size_t i = 0; i < widthNum*heightNum; ++i)
	{
		if (dataset->m_lasRectangles[i].m_lasPoints_numbers > 0)
		{
			dataset->m_lasRectangles[i].LASRectBuildTree();
		}
	}
	delete[]pointsRect; pointsRect = nullptr;
	fclose(m_lasFile); m_lasFile = nullptr;
	return 0;
}