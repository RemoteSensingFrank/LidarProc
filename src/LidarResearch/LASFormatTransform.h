//
// Created by wuwei on 18-3-13.
//

#ifndef LIDARPROC_LASFORMATTRANSFORM_H
#define LIDARPROC_LASFORMATTRANSFORM_H

#include <string>
#include "../LidarGeometry/Geometry.h"
#include "../LidarBase/LASPoint.h"

class LASFormatTransform3DTiles {
public:
    /**
     * 3D tiles pre json
     * @param center xyz coordinate
     * @param pathOut
     */
    void LASFormatTransform_3DTilesJson(Point3D *center,std::string pathOut);

    /**
     * 3D tiles points
     * @param pnts  //centered points xyz coordinate
     * @param pntNumber
     * @param pathOut
     */
    void LASFormatTransform_3DTilesPnts(LASPoint* pnts,int pntNumber,std::string pathOut);

    /**
     * zip point and remove raw binnary file
     * @param pathBin
     * @param pathPnts
     */
    void LASFormatTransform_3DTilesPntsGZip(const char* pathBin,const char* pathPnts);

    /**
     * trans to tile set
     * @param pathDir
     * @param pathTileset
     */
    void LASFormatTransform_Tileset(const char* pathDir,const char* pathTileset,const char* jsonName);
};


#endif //LIDARPROC_LASFORMATTRANSFORM_H
