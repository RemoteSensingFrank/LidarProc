//
// Created by wuwei on 18-3-13.
//

#include "LASFormatTransform.h"
#include <zlib.h>
#include "../LidarBase/LASPoint.h"

void LASFormatTransform3DTiles::LASFormatTransform_3DTilesJson(Point3D *center,std::string pathOut){
    string jsTemplate="{\n\
    \"asset\" : {\n  \
        \"version:\" \"1.0\"\n\
    },\n\
    \"geometricError\":10000,\n\
    \"refine\":\"add\",\n\
    \"root\":\n\
    { \n\
        \"boundingVolume\":{\n  \
        \"sphere\":[%lf,%lf,%lf,100]\n\
        },\n\
        \"geometricError\": 0,\n\
        \"content\":{\n  \
        \"url\":\"%s.pnts\"\n  \
         },\n\
         \"children\":[]\n\
   }\n\
}";
    int pos=pathOut.find_last_of('/');
    string s(pathOut.substr(pos+1));
    char buffer[2048];
    sprintf(buffer,jsTemplate.c_str(),center->x,center->y,center->z,s.c_str());
    string writeStr = pathOut+".json";

    FILE* fjson = fopen(writeStr.c_str(),"w+");
    fprintf(fjson,"%s",buffer);
    fclose(fjson);
}

void LASFormatTransform3DTiles::LASFormatTransform_3DTilesPnts(LASPoint *pnts, int pntNumber, std::string pathOut) {
    //process
    //write binnary data
    //gzip binnary data
    //remove binnary pnt;
    //rename gzip data as .pnt
    FILE* fs=fopen(pathOut.c_str(),"wb");

    char head[4]={'p','n','t','s'};
    unsigned  int version =1;
    //15 byte include 3 float xyz 3 char rgb
    unsigned int byteLength = pntNumber*15+16;
    unsigned int numberPnts = pntNumber;

    //header part
    fwrite(head,sizeof(char),4,fs);
    fwrite(&version,sizeof(int),1,fs);
    fwrite(&byteLength,sizeof(int),1,fs);
    fwrite(&numberPnts,sizeof(int),1,fs);

    //data part
    for(int i=0;i<pntNumber;++i){
        float x = pnts[i].m_vec3d.x;
        float y = pnts[i].m_vec3d.y;
        float z = pnts[i].m_vec3d.z;

        fwrite(&x,sizeof(float),1,fs);
        fwrite(&y,sizeof(float),1,fs);
        fwrite(&z,sizeof(float),1,fs);
    }
    for(int i=0;i<pntNumber;++i){
        char r = pnts[i].m_colorExt.Red;
        char g = pnts[i].m_colorExt.Green;
        char b = pnts[i].m_colorExt.Blue;

        fwrite(&r,sizeof(char),1,fs);
        fwrite(&g,sizeof(char),1,fs);
        fwrite(&b,sizeof(char),1,fs);
    }
    fclose(fs);
}

void LASFormatTransform3DTiles::LASFormatTransform_3DTilesPntsGZip(const char* pathBin,const char* pathPnts){
    std::string gzipFile = "gzip "+ string(pathBin) + ";mv " + string(pathBin) + ".gz " + string(pathPnts)+";";
    std::string cmdRmRaw = "rm -rf "+string(pathBin);

    system(gzipFile.c_str());
    system(cmdRmRaw.c_str());
}

void LASFormatTransform_Tileset(const char* pathDir,const char* pathTileset,const char* jsonName) {
    string cmd="tileset.sh "+ string(pathDir)+" "+string(jsonName);
    system(cmd.c_str());
}
