#pragma once
//
// Created by wuwei on 17-12-25.
//

#ifndef LASLIB_LASHEADER_H
#define LASLIB_LASHEADER_H


#include <stdio.h>
#include <memory.h>
using namespace std;

#ifndef _MAX_LIMIT_
#define _MAX_LIMIT_ 99999999
#endif

#ifndef _MIN_LIMIT_
#define _MIN_LIMIT_ -99999999
#endif



// LAS头结构
#pragma pack(1)
class  LASHeader
{
public:

	// LASHeader在文件中的大小dpw
	// sizeof(LASHeader)计算的结果可能比这个大
	// 因为编译器可能会做一些优化(如边界对齐等)
	//加上颜色扩展
	enum { HEADER_SIZE = 227 };

public:												// 必须 说明		默认值	字节编号
	char file_signature[4];							// *	文件标志	"LASF"	1-4
													//unsigned short file_source_id;				// *	文件编号	0
													/*
													* This data field is reserved and must be zero filled by generating generating_software.
													* 1.0版las文件为4字节；1.1版为2字节，与上面的File_Source_ID合用4个字节
													*/
	unsigned int  reserved;							// -	保存使用	5-8		5-8
	unsigned int  project_ID_GUID_data_1;			// -	4字节		0		9-12
	unsigned short project_ID_GUID_data_2;			// -	2字节		0		13-14
	unsigned short project_ID_GUID_data_3;			// -	2字节		0		15-16
	unsigned char project_ID_GUID_data_4[8];		// -	8字节		0		17-24
	unsigned char version_major;					// *	主版本号			25
	unsigned char version_minor;					// *	副版本号			26
	unsigned char system_id[32];					// *	系统标识			27-58
	char generating_software[32];					// *	生成软件			59-90
	unsigned short file_creation_day;				// -	创建时间			91-92
	unsigned short file_creation_year;				// -						93-94
	unsigned short header_size;						// *	Head大小			95-96
	unsigned int offset_to_point_data;				// *	数据地址			97-100
	unsigned int number_of_variable_length_records; // *	变长记录数目		101-104
	unsigned char point_data_format;				// *	点数据格式GPS		105
	unsigned short point_data_record_length;		// *	点数据长度			106-107
	unsigned int number_of_point_records;			// *	点的数目			108-111
	unsigned int number_of_points_by_return[5];	    // *	回波返回数			112-131

													/*
													* Xcoordinate = (Xrecord * x_scale_factor) + x_offset
													* Ycoordinate = (Yrecord * y_scale_factor) + y_offset
													* Zcoordinate = (Zrecord * z_scale_factor) + z_offset
													*/
	double x_scale_factor;							// *	缩放系数			132-139
	double y_scale_factor;							// *	缩放系数			140-147
	double z_scale_factor;							// *	缩放系数			148-155

	double x_offset;								// *	相对偏移			156-163
	double y_offset;								// *	相对偏移			170-177
	double z_offset;								// *	相对偏移			164-169

	double max_x;									// *	x最大值				178-195
	double min_x;									// *	x最小值				186-193
	double max_y;									// *						194-201
	double min_y;									// *						202-209
	double max_z;									// *						210-217
	double min_z;									// *						218-225

public:
	/*
	构造和析构
	*/
	LASHeader();
	LASHeader(const LASHeader& header);
	LASHeader& operator=(const LASHeader& header);

	/*
	数据集ID
	*/
	inline unsigned short GetFile_Source_ID() { return (version_minor == 1 && version_minor == 1) ? reserved >> 16 : 0; }
	
	/*
	数据集保留位
	*/
	inline unsigned int Getreserved() { return (version_minor == 1 && version_minor == 1) ? reserved &= 0x0000FFFF : reserved; }
	inline bool HasPoint()		   const { return point_data_record_length >= 12 ? true : false; }
	inline bool HasIntensity()	   const { return point_data_record_length >= 14 ? true : false; }
	inline bool HasReturnNumber() const { return point_data_record_length >= 15 ? true : false; }
	inline bool HasNumberofReturn()const { return point_data_record_length >= 15 ? true : false; }
	inline bool HasScanDirectionFlag() const { return point_data_record_length >= 15 ? true : false; }
	inline bool HasEdgeofFlightLine()const { return point_data_record_length >= 15 ? true : false; }
	inline bool HasScanAgnleRank()	const { return point_data_record_length >= 17 ? true : false; }
	inline bool HasFlightLine()const { return point_data_record_length >= 17 ? true : false; }

	void Setreserved(unsigned int reservedi);
	void SetFile_Source_ID(unsigned short id);

	bool HasGPSTime()const;
	bool HasLASColorExt4() const;//是否有颜色 四字节定义
	bool HasLASColorExt6() const;
	int  HasLASPointExt() const; //数据格式是否有扩展
	bool HasPointDataStartfile_signature() const { return version_minor == 0 && version_minor == 1; }

	//读写数据头文件
	void ReadHeader(FILE *fs);
	void WriteHeader(FILE *fs) const;

	const static unsigned short Data_Record_Length_of_Format0; // 20，不扩展激光点结构时的0格式点标准长度
	const static unsigned short Data_Record_Length_of_Format1; // 28，不扩展激光点结构时的1格式点标准长度
	const static unsigned short Data_Record_Length_of_Format2; // 26，1.2不扩展激光点结构时的1格式点标准长度
	const static unsigned short Data_Record_Length_of_Format3; // 34，1.2不扩展激光点结构时的1格式点标准长度
	const static unsigned short HeaderSize_Def;				   // 227，标准头文件长度

	const static char ErrorPointCnt[];			 // "number of point is more than LasFileHead.Number_of_point_records";
	const static char ErrorVarLenRecordCnt[];	 // "number of variable Length Record is more than LasFileHead.Number_of_variable_length_records";
	const static char ErrorOffsettoData[];		 // "number of bytes of Variable Length Record is more than LasFileHead.Offset_to_data";
};
#pragma pack()

// 变长记录头部
#define min_las_record_after_header 54;
class  LASVariableRecord
{
public:											// 必须	描述	默认值
	LASVariableRecord() {
		reserved = 0xAABB;
		memset(user_id, 0, sizeof(char) * 16);
		record_id = 0;
		record_length_after_header = min_las_record_after_header;
		memset(description, 0, sizeof(char));
		record_buffer = NULL;
	}
	~LASVariableRecord()
	{
		if (record_buffer != NULL)
			delete[]record_buffer;
	}
	unsigned short reserved;					// -	保留	0
	char user_id[16];							// *			LASF_Specl/LASF_Projection
	unsigned short record_id;					// *	编号
	unsigned short record_length_after_header;	// *	后续记录大小
	char description[32];						// -	数据说明
	unsigned char* record_buffer;

	void Read(FILE* fs);
	void Write(FILE* fs) const;

	//Two bytes after the last variable length record, and before the point data
	const static unsigned short Min_Record_Length_After_Header;  // 54
	const static short Point_Data_Start_Signature;
};

// 官方定义的变长记录结构
// User ID: LASF_Projection
// Record ID: 34735
class  LASVariable_header_geo_keys
{
public:
	unsigned short key_directory_version;
	unsigned short key_revision;
	unsigned short minor_revision;
	unsigned short number_of_keys;

	LASVariable_header_geo_keys()
	{
		key_directory_version = 1;	// Always
		key_revision = 1;			// Always
		minor_revision = 0;			// Always
	}
};
class  LASVariable_header_key_entry
{
public:
	unsigned short key_id;
	unsigned short tiff_tag_location;
	unsigned short count;
	unsigned short value_offset;
};

// 地面类型
enum  LAS_CLASSIFICATION_TYPE
{
	CLASSIFICATION_NULL = 0,	// 未设置类别
	CLASSIFICATION_UNCLASSIFIED = 1,
	CLASSIFICATION_GROUND = 2,
	CLASSIFICATION_LOW_VEGETATION = 3,
	CLASSIFICATION_MEDIUM_VEGETATION = 4,
	CLASSIFICATION_HIGH_VEGETATION = 5,
	CLASSIFICATION_BUILDING = 6,
	CLASSIFICATION_LOW_POINT_NOISE = 7,
	CLASSIFICATION_MEDEL_KEYPOINT = 8,
	CLASSIFICATION_WATER = 9,
	CLASSIFICATION_OVERLAP_POINTS2 = 12,
	CLASSIFICATION_reserved						// 保留使用
};
#endif


