// Minimal stub of Graphisoft Archicad API for headless compilation
#ifndef ACAPINC_STUB_H
#define ACAPINC_STUB_H

// Basic types and constants used in this project
using Int32 = int;
using UInt32 = unsigned int;
using GSErrCode = int;
constexpr GSErrCode NoError = 0;

struct API_AttributeIndex { short index; };

enum API_AttributeID {
    API_LayerID = 1
};

struct API_AttributeHeader {
    short typeID;
    API_AttributeIndex index;
    char name[64];
};

struct API_Attribute {
    API_AttributeHeader header;
};

struct API_StoryType {
    short index;
    double level;
    char uName[64];
};

struct API_StoryInfo {
    API_StoryType* data;
    short firstStory;
    short lastStory;
};

// stub functions
inline GSErrCode ACAPI_Attribute_GetNum(API_AttributeID, UInt32& count) { count = 0; return NoError; }
inline API_AttributeIndex ACAPI_CreateAttributeIndex(short i) { API_AttributeIndex idx; idx.index = i; return idx; }
inline GSErrCode ACAPI_Attribute_Get(API_Attribute*) { return NoError; }
inline GSErrCode ACAPI_ProjectSetting_GetStorySettings(API_StoryInfo*) { return NoError; }
inline void BMKillHandle(void*) {}

// qsort/pop? not needed

#endif // ACAPINC_STUB_H
