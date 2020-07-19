/*
 * @Descripttion: 
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2020-07-19 12:13:47
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-07-19 14:31:57
 */ 
#pragma once
#include <string>
#include <vector>

#include "../LidarBase/LASPoint.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

using namespace std;
using namespace rapidjson;

/**
 * @name: 序列化与反序列化几何对象
 * @msg: 
 * @param {type} 
 * @return: 
 */
class GeoJsonGeometryUtil
{
public:
    template <typename Writer>
    void Serialize(Writer& writer){
        // This base class just write out name-value pairs, without wrapping within an object.
        writer.String("type");
        writer.String(type);

        writer.String(("coordinates"));
        writer.StartArray();
        for (Point3Ds::const_iterator pntItr = points.begin(); pntItr != points.end(); ++pntItr)
        {
            writer.StartArray();
            writer.Double(pntItr->x);
            writer.Double(pntItr->y);
            writer.Double(pntItr->z);
            writer.EndArray();
        }
        writer.EndArray();
    }

    void DeSerialize(const Value &geoJson){
        if(geoJson.HasMember("type") && geoJson["type"].IsString())
        {
            const rapidjson::Value& childValue = geoJson["type"];
            type=childValue.GetString();
        }
        if(geoJson.HasMember("coordinates") && geoJson["coordinates"].IsArray())
        {
            const Value& coords = geoJson["coordinates"];
            for (SizeType i = 0; i < coords.Size(); i++) // 使用 SizeType 而不是 size_t
            {
                const Value& pnt = coords[0];
                if(pnt.Size()==3)
                {
                    Point3D pt(pnt[0].GetDouble(),pnt[1].GetDouble(),pnt[2].GetDouble());
                    points.push_back(pt);
                }
            }
        }
    }
public:
    string  type;
    Point3Ds points;
};

/**
 * @name: 序列化与反序列化线特征
 * @msg: 
 * @param {type} 
 * @return: 
 */
class GeoJsonLineStringJsonUtil
{
public:
    template <typename Writer>
    void Serialize(Writer& writer){
        writer.StartObject();
        
        // This base class just write out name-value pairs, without wrapping within an object.
        writer.String("type");
        writer.String(type);

        writer.String("properties");
        writer.StartObject();
        writer.String("name");
        writer.String(name);
        writer.EndObject();

        writer.String("geometry");
        writer.StartObject();
        geometryUtil.Serialize(writer);
        writer.EndObject();
        writer.EndObject();
    }

    void DeSerialize(const Value &lineFeature)
    {
        if(lineFeature.HasMember("type") && lineFeature["type"].IsString())
        {
            const rapidjson::Value& childValue = lineFeature["type"];
            type=childValue.GetString();
        }
        if(lineFeature.HasMember("properties") && lineFeature["properties"].IsString())
        {
            const rapidjson::Value& childValue = lineFeature["properties"];
            if(childValue.HasMember("name") && childValue["name"].IsString())
            {
                name = childValue["name"].GetString();
            }
        }
        if(lineFeature.HasMember("geometry") && lineFeature["geometry"].IsObject())
        {
            const Value& geoJson = lineFeature["geometry"];
            geometryUtil.DeSerialize(geoJson);
        }
    }
public:
    string type;
    string name;
    GeoJsonGeometryUtil geometryUtil;
};