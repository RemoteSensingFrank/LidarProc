![](https://img.shields.io/github/issues/RemoteSensingFrank/LidarProc)
![](https://img.shields.io/github/forks/RemoteSensingFrank/LidarProc)
![](https://img.shields.io/github/stars/RemoteSensingFrank/LidarProc)
![](https://img.shields.io/github/license/RemoteSensingFrank/LidarProc)
![](https://www.travis-ci.org/RemoteSensingFrank/LidarProc.svg?branch=master)
# LidarProc
## LAS Data Format:
.las format is the standard data format for lidar data. And a .las dataset can be divided into 3 part: **1.PUBLIC HEADER BLOCK**,**2.VARIABLE LENGTH RECORDS** and **3.POINT DATA RECORD**, in **PUBLIC HEADER BLOCK** part the descriptions of the dataset is recorded. The descriptions included number of points, length of each point, las dataset type and so on. So this part should be processed carefully to get the informations of the dataset; and in **VARIABLE LENGTH RECORDS** this part do not needed necessarily and in most dataset this part does not exist; the final part is **POINT DATA RECORD**. This part recorded all points informations.  
By the way I will introduce the struct of the program along with the dataset struct in detail:

```c++
#pragma pack(1)
class  LASHeader
{
public:
	enum { HEADER_SIZE = 227 };

public:												
	char file_signature[4];		
	unsigned int  reserved;						
	unsigned int  project_ID_GUID_data_1;			// -			9-12
	unsigned short project_ID_GUID_data_2;			// -		    13-14
	unsigned short project_ID_GUID_data_3;			// -		    15-16
	unsigned char project_ID_GUID_data_4[8];		// -		    17-24
	unsigned char version_major;					// *			25
	unsigned char version_minor;					// *			26
	unsigned char system_id[32];					// *			27-58
	char generating_software[32];					// *			59-90
	unsigned short file_creation_day;				// -			91-92
	unsigned short file_creation_year;				// -			93-94
	unsigned short header_size;						// *			95-96
	unsigned int offset_to_point_data;				// *			97-100
	unsigned int number_of_variable_length_records;// *				101-104
	unsigned char point_data_format;				// *			105
	unsigned short point_data_record_length;		// *			106-107
	unsigned int number_of_point_records;			// *			108-111
	unsigned int number_of_points_by_return[5];	    // *			112-131

													/*
													* Xcoordinate = (Xrecord * x_scale_factor) + x_offset
													* Ycoordinate = (Yrecord * y_scale_factor) + y_offset
													* Zcoordinate = (Zrecord * z_scale_factor) + z_offset
													*/
	double x_scale_factor;							// *			132-139
	double y_scale_factor;							// *			140-147
	double z_scale_factor;							// *			148-155

	double x_offset;								// *			156-163
	double y_offset;								// *			170-177
	double z_offset;								// *			164-169

	double max_x;									// *			178-195
	double min_x;									// *			186-193
	double max_y;									// *			194-201
	double min_y;									// *			202-209
	double max_z;									// *			210-217
	double min_z;									// *			218-225

public:
	LASHeader();
	LASHeader(const LASHeader& header);
	LASHeader& operator=(const LASHeader& header);


	inline unsigned short GetFile_Source_ID() { return (version_minor == 1 && version_minor == 1) ? reserved >> 16 : 0; }

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
	bool HasLASColorExt4() const;//
	bool HasLASColorExt6() const;
	int  HasLASPointExt() const; //
	bool HasPointDataStartfile_signature() const { return version_minor == 0 && version_minor == 1; }
	void ReadHeader(FILE *fs);
  void WriteHeader(FILE *fs) const;

	const static unsigned short Data_Record_Length_of_Format0; //
	const static unsigned short Data_Record_Length_of_Format1; //
	const static unsigned short Data_Record_Length_of_Format2; //
	const static unsigned short Data_Record_Length_of_Format3; //
	const static unsigned short HeaderSize_Def;				   //

	const static char ErrorPointCnt[];			 // "number of point is more than LasFileHead.Number_of_point_records";
	const static char ErrorVarLenRecordCnt[];	 // "number of variable Length Record is more than LasFileHead.Number_of_variable_length_records";
	const static char ErrorOffsettoData[];		 // "number of bytes of Variable Length Record is more than LasFileHead.Offset_to_data";
};
#pragma pack()

```

header struct is as the code above:
as usual the size of the header is 277 bytes and the means of each byte is in the code, and be careful that usful data only occupy 225 bytes, so for jump to the data record directly you should seek 227 bytes.   

```C
enum  eLASEchos
{
	eLidarEchoOnly = 0,
	eLidarEchoFirst = 1,
	eLidarEchoMidian = 2,
	eLidarEchoLast = 3
};

#pragma pack(1)
enum  eLASClassification
{
	elcCreated			 = 0,	
	elcUnclassified		 = 1,	
	elcGround			 = 2,	
	elcLowVegetation	 = 3,	
	elcMediumVegetation  = 4,	
	elcHighVegetation	 = 5,	
	elcBuilding			 = 6,	
	elcLowPoint			 = 7,	
	elcModelKeyPoint	 = 8,	
	elcWater			 = 9,	
	elcOverlapPoint		 = 12,	
	elcDanger			 = 13,	
	elcTowerRange		 = 14,	
	elcDeletedPoint		 = -1	
};
#pragma pack()

static eLASClassification GetLidarClassification(unsigned char clsType)
{
	return (eLASClassification)clsType;
}
/*Las1.2��ɫ��չ*/
#pragma pack(1)
struct LASColorExt
{
	unsigned short Red;
	unsigned short Green;
	unsigned short Blue;
};
#pragma pack()

//las Point def
#pragma pack(1)
class LASPoint
{
public:
	/*
	* constructor 
	*/
	void Write(FILE *fs, const LASHeader& info) const;
	void Read(FILE *fs, const LASHeader& info);

	/**
	* extract from binnary data
	* @param data
	* @param info
	*/
	void ExtractFromBuffer(const unsigned char* data, const LASHeader& info);
	void ExportToBuffer(unsigned char* data, const LASHeader& info) const;

public:
	Point3D			m_vec3d;
	unsigned short  m_intensity;
	unsigned char   m_rnseByte;
	char			m_classify;
	char			m_scanAngle;
	unsigned char	m_userdata;
	unsigned short  m_flightID;
	double			m_gpsTime;
	LASColorExt		m_colorExt;
};
#pragma pack()
```
from the code we can know the each point data struct:  
point(x,y,z);intensity;rnse;classify;scanAngle;userdata;flightID;gpstime(not necessary);color(not necessary).  
there are some import things we should take care: 1.the point is saved as type int to save memory,the scale and offset is recorded in header; 2.rnse refers to return number(3 byte), number of returns(3 byte), scan angle direction(1 byte), edge of flight line(1 byte); the four part is recorded together to save memory; 3. the color part is not recorded by 3 shorts data in format 1.1 in format 1.1 the color part is recorded into a int.

## Program Struct with Zh-cn
<center><img src="https://blogimage-1251632003.cos.ap-guangzhou.myqcloud.com/%E6%BF%80%E5%85%89%E7%82%B9%E4%BA%91%E6%95%B0%E6%8D%AE%E5%A4%84%E7%90%86%E8%BD%AF%E4%BB%B6%E7%A0%94%E5%8F%91.svg"/></center>  

