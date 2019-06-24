//
// Created by wuwei on 18-3-13.
//

#include "LASFormatTransform.h"
#include "../LidarUtil/FileHelper.h"
#include "../LidarBase/LASPoint.h"

struct PotreeArguments {
	bool help = false;
	StoreOption storeOption = StoreOption::ABORT_IF_EXISTS;
	vector<string> source;
	string outdir;
	float spacing;
	int levels;
	string format;
	double scale;
	int diagonalFraction;
	Potree::OutputFormat outFormat;
	vector<double> colorRange;
	vector<double> intensityRange;
	vector<string> outputAttributes;
	bool generatePage;
	bool pageTemplate;
	string pageTemplatePath = "";
	vector<double> aabbValues;
	string pageName = "";
	string projection = "";
	bool sourceListingOnly = false;
	string listOfFiles = "";
	ConversionQuality conversionQuality = ConversionQuality::DEFAULT;
	string conversionQualityString = "";
	string title = "PotreeViewer";
	string description = "";
	bool edlEnabled = false;
	bool showSkybox = false;
	string material = "RGB";
    string executablePath;
	int storeSize;
	int flushLimit;
};




long LASTransToPotree::LASTransToPotree_Trans(const char* lasPath,const char* transdir)
{
    try{
		PotreeArguments a;
        a.outdir = transdir;
        a.spacing = 0.0;
        a.storeSize = 200'000;
        a.flushLimit= 10'000'000;
        a.diagonalFraction = 300;
        a.levels = (-1);
        a.outFormat = Potree::OutputFormat::BINARY;
        a.outputAttributes = { "RGB" };
        a.scale = (0.0);
        a.storeOption = StoreOption::ABORT_IF_EXISTS;
        vector<string> source;
        source.push_back(lasPath);
        a.source=source;

        PotreeConverter pc(a.executablePath, a.outdir, a.source);
		pc.spacing = a.spacing;
		pc.diagonalFraction = a.diagonalFraction;
		pc.maxDepth = a.levels;
		pc.format = a.format;
		pc.colorRange = a.colorRange;
		pc.intensityRange = a.intensityRange;
		pc.scale = a.scale;
		pc.outputFormat = a.outFormat;
		pc.outputAttributes = a.outputAttributes;
		pc.aabbValues = a.aabbValues;
		pc.pageName = a.pageName;
		pc.pageTemplatePath = a.pageTemplatePath;
		pc.storeOption = a.storeOption;
		pc.projection = a.projection;
		pc.sourceListingOnly = a.sourceListingOnly;
		pc.quality = a.conversionQuality;
		pc.title = a.title;
		pc.description = a.description;
		pc.edlEnabled = a.edlEnabled;
		pc.material = a.material;
		pc.showSkybox = a.showSkybox;
		pc.storeSize = a.storeSize;
		pc.flushLimit = a.flushLimit;

		pc.convert();
	}catch(exception &e){
		cout << "ERROR: " << e.what() << endl;
		return 1;
	}
}

long LASTransToPotree::LASTransToPotree_Trans(const char* lasPath)
{
    try{
		PotreeArguments a;

        a.outdir = "./www/pointclouds/"+FileHelper::getFileName(lasPath)+"/";
        a.spacing = 0.0;
        a.storeSize = 200'000;
        a.flushLimit= 10'000'0000;
        a.diagonalFraction = 300.0;
        a.levels = (-1);
        a.outFormat = Potree::OutputFormat::LAS;
        a.outputAttributes = { "RGB" };
        a.scale = (0.0);
        a.storeOption = StoreOption::ABORT_IF_EXISTS;
        vector<string> source;
        source.push_back(lasPath);
        a.source=source;

        PotreeConverter pc(a.executablePath, a.outdir, a.source);
		pc.spacing = a.spacing;
		pc.diagonalFraction = a.diagonalFraction;
		pc.maxDepth = a.levels;
		pc.format = a.format;
		pc.colorRange = a.colorRange;
		pc.intensityRange = a.intensityRange;
		pc.scale = a.scale;
		pc.outputFormat = a.outFormat;
		pc.outputAttributes = a.outputAttributes;
		pc.aabbValues = a.aabbValues;
		pc.pageName = a.pageName;
		pc.pageTemplatePath = a.pageTemplatePath;
		pc.storeOption = a.storeOption;
		pc.projection = a.projection;
		pc.sourceListingOnly = a.sourceListingOnly;
		pc.quality = a.conversionQuality;
		pc.title = a.title;
		pc.description = a.description;
		pc.edlEnabled = a.edlEnabled;
		pc.material = a.material;
		pc.showSkybox = a.showSkybox;
		pc.storeSize = a.storeSize;
		pc.flushLimit = a.flushLimit;

		pc.convert();
	}catch(exception &e){
		cout << "ERROR: " << e.what() << endl;
		return 1;
	}
}