#include "TopoMeshHelper.hpp"

#include "APIEnvir.h"
#include "ACAPinc.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <limits>

// =============================================================================
// Внутренние структуры
// =============================================================================

namespace {

struct TopoPoint { double x, y, z; };

struct TopoParams {
	Int32         layerIdx;
	double        radiusMm;
	char          separator;
	Int32         storyIdx;
	double        bboxOffsetMm;
	GS::UniString meshName;
	Int32         meshLayerIdx;
};

struct ArcPoint { double x, y; };
struct TextItem { double x, y; GS::UniString text; };

// =============================================================================
// JSON парсер
// =============================================================================

static GS::UniString JsonGetString(const GS::UniString& json, const char* key)
{
	GS::UniString searchKey = GS::UniString("\"") + key + "\":";
	const char* src   = json.ToCStr().Get();
	const char* found = std::strstr(src, searchKey.ToCStr().Get());
	if (found == nullptr) return GS::EmptyUniString;

	const char* val = found + std::strlen(searchKey.ToCStr().Get());
	while (*val == ' ') ++val;

	if (*val == '"') {
		++val;
		const char* end = std::strchr(val, '"');
		if (end == nullptr) return GS::EmptyUniString;
		char buf[512] = {};
		std::strncpy(buf, val, (size_t)(end - val));
		return GS::UniString(buf);
	} else {
		char buf[64] = {};
		int i = 0;
		while (*val && *val != ',' && *val != '}' && *val != ' ' && i < 63)
			buf[i++] = *val++;
		return GS::UniString(buf);
	}
}

static double JsonGetDouble(const GS::UniString& json, const char* key, double def = 0.0)
{
	GS::UniString s = JsonGetString(json, key);
	if (s.IsEmpty()) return def;
	double out = def; std::sscanf(s.ToCStr().Get(), "%lf", &out); return out;
}

static Int32 JsonGetInt(const GS::UniString& json, const char* key, Int32 def = 0)
{
	GS::UniString s = JsonGetString(json, key);
	if (s.IsEmpty()) return def;
	Int32 out = def; std::sscanf(s.ToCStr().Get(), "%d", &out); return out;
}

// =============================================================================
// Парсинг параметров
// =============================================================================

static bool ParseTopoParams(const GS::UniString& json, TopoParams& p)
{
	p.layerIdx     = JsonGetInt   (json, "layerIdx",   -1);
	p.radiusMm     = JsonGetDouble(json, "radius",   3000.0);
	p.storyIdx     = JsonGetInt   (json, "storyIdx",    0);
	p.bboxOffsetMm = JsonGetDouble(json, "bboxOffset", 1000.0);
	p.meshName     = JsonGetString(json, "meshName");
	p.meshLayerIdx = JsonGetInt   (json, "meshLayer",   0);
	if (p.meshName.IsEmpty()) p.meshName = "TopoMesh";
	GS::UniString sep = JsonGetString(json, "separator");
	p.separator = (sep == ",") ? ',' : '.';
	if (p.layerIdx < 0) {
		ACAPI_WriteReport("[TopoMesh] Ошибка: layerIdx не задан", false);
		return false;
	}
	return true;
}

// =============================================================================
// Парсинг высотной отметки
// =============================================================================

static bool ParseElevation(const char* text, char sep, double& outMm)
{
	if (text == nullptr || *text == '\0') return false;
	while (*text == ' ') ++text;
	bool neg = false;
	if      (*text == '+') ++text;
	else if (*text == '-') { neg = true; ++text; }
	char buf[64] = {};
	std::strncpy(buf, text, 63);
	for (int i = 0; buf[i]; ++i) if (buf[i] == sep) buf[i] = '.';
	double val = 0.0;
	if (std::sscanf(buf, "%lf", &val) != 1) return false;
	outMm = val * 1000.0;
	if (neg) outMm = -outMm;
	return true;
}

// =============================================================================
// Индекс атрибута слоя
// =============================================================================

static API_AttributeIndex GetLayerAttrIdx(Int32 listIndex)
{
	GS::UInt32 layerCount = 0;
	if (ACAPI_Attribute_GetNum(API_LayerID, layerCount) != NoError || layerCount == 0)
		return ACAPI_CreateAttributeIndex(1);

	Int32 current = 0;
	for (Int32 i = 1; i <= static_cast<Int32>(layerCount); ++i) {
		API_Attribute attr = {};
		attr.header.typeID = API_LayerID;
		attr.header.index  = ACAPI_CreateAttributeIndex(i);
		if (ACAPI_Attribute_Get(&attr) != NoError) continue;
		if (current == listIndex) return attr.header.index;
		++current;
	}
	return ACAPI_CreateAttributeIndex(1);
}

// =============================================================================
// Высота этажа
// =============================================================================

static double GetStoryElevM(Int32 storyIdx)
{
	API_StoryInfo si = {};
	if (ACAPI_ProjectSetting_GetStorySettings(&si) != NoError)
		return 0.0;
	if (si.data == nullptr)
		return 0.0;

	double elev = 0.0;
	const Int32 cnt = (Int32)(BMGetHandleSize((GSHandle)si.data) / sizeof(API_StoryType));
	const Int32 idx = storyIdx - si.firstStory;
	if (idx >= 0 && idx < cnt)
		elev = (*si.data)[idx].level;

	BMKillHandle((GSHandle*)&si.data);
	return elev;
}

// =============================================================================
// Сбор Arc и Text со слоя
// =============================================================================

static void CollectOnLayer(API_AttributeIndex layerAttrIdx,
	std::vector<ArcPoint>& arcs,
	std::vector<TextItem>& texts)
{
	GS::Array<API_Guid> arcList;
	ACAPI_Element_GetElemList(API_ArcID, &arcList);
	for (UIndex i = 0; i < arcList.GetSize(); ++i) {
		API_Element elem = {};
		elem.header.guid = arcList[i];
		if (ACAPI_Element_Get(&elem) != NoError) continue;
		if (elem.header.layer != layerAttrIdx) continue;
		arcs.push_back({ elem.arc.origC.x, elem.arc.origC.y });
	}

	GS::Array<API_Guid> textList;
	ACAPI_Element_GetElemList(API_TextID, &textList);
	for (UIndex i = 0; i < textList.GetSize(); ++i) {
		API_Element elem = {};
		elem.header.guid = textList[i];
		if (ACAPI_Element_Get(&elem) != NoError) continue;
		if (elem.header.layer != layerAttrIdx) continue;
		API_ElementMemo memo = {};
		if (ACAPI_Element_GetMemo(textList[i], &memo, APIMemoMask_TextContent) != NoError) continue;
		TextItem ti;
		ti.x = elem.text.loc.x;
		ti.y = elem.text.loc.y;
		if (memo.textContent != nullptr && *memo.textContent != nullptr)
			ti.text = GS::UniString(*memo.textContent);
		ACAPI_DisposeElemMemoHdls(&memo);
		if (!ti.text.IsEmpty()) texts.push_back(ti);
	}
}

// =============================================================================
// Сопоставление Arc <-> Text
// =============================================================================

static std::vector<TopoPoint> MatchPoints(
	const std::vector<ArcPoint>& arcs,
	const std::vector<TextItem>& texts,
	double radiusMm, char sep)
{
	const double r2 = (radiusMm / 1000.0) * (radiusMm / 1000.0);
	std::vector<TopoPoint> result;
	result.reserve(arcs.size());

	for (size_t ai = 0; ai < arcs.size(); ++ai) {
		double bestD2 = std::numeric_limits<double>::max();
		const TextItem* best = nullptr;
		for (size_t ti = 0; ti < texts.size(); ++ti) {
			const double dx = arcs[ai].x - texts[ti].x;
			const double dy = arcs[ai].y - texts[ti].y;
			const double d2 = dx*dx + dy*dy;
			if (d2 <= r2 && d2 < bestD2) { bestD2 = d2; best = &texts[ti]; }
		}
		if (best == nullptr) continue;
		double elevMm = 0.0;
		if (!ParseElevation(best->text.ToCStr().Get(), sep, elevMm)) continue;
		result.push_back({ arcs[ai].x, arcs[ai].y, elevMm / 1000.0 });
	}
	return result;
}

// =============================================================================
// Экранирование для JSON
// =============================================================================

static GS::UniString JsonEscape(const GS::UniString& s)
{
	GS::UniString out;
	for (UInt32 i = 0; i < s.GetLength(); ++i) {
		if      (s[i] == '"')  out += "\\\"";
		else if (s[i] == '\\') out += "\\\\";
		else                   out += s[i];
	}
	return out;
}

// =============================================================================
// Создание Mesh
// =============================================================================

static GSErrCode BuildMesh(const std::vector<TopoPoint>& pts,
	const TopoParams& p, double storyElevM)
{
	if (pts.size() < 3) {
		ACAPI_WriteReport("[TopoMesh] Менее 3 точек (%d)", false, (int)pts.size());
		return Error;
	}

	// Все топо-точки идут как вершины контура — ArchiCAD сам триангулирует
	const Int32 nUnique = (Int32)pts.size();

	API_Element     elem = {};
	API_ElementMemo memo = {};

	elem.header.type = API_MeshID;
	GSErrCode err = ACAPI_Element_GetDefaults(&elem, nullptr);
	if (err != NoError) {
		ACAPI_WriteReport("[TopoMesh] GetDefaults failed: %d", false, (int)err);
		return err;
	}

	elem.header.layer    = GetLayerAttrIdx(p.meshLayerIdx);
	elem.header.floorInd = (short)p.storyIdx;
	elem.mesh.level      = 0.0;

	// Контур — прямоугольный bounding box (4 угла + замыкающая = 5 точек)
	const Int32 nC      = 4;
	const Int32 nCoords = nC + 1; // с замыкающей

	// Bounding box
	double minX = pts[0].x, maxX = pts[0].x;
	double minY = pts[0].y, maxY = pts[0].y;
	const double offM = p.bboxOffsetMm / 1000.0;
	for (const auto& pt : pts) {
		if (pt.x < minX) minX = pt.x; if (pt.x > maxX) maxX = pt.x;
		if (pt.y < minY) minY = pt.y; if (pt.y > maxY) maxY = pt.y;
	}
	minX -= offM; minY -= offM; maxX += offM; maxY += offM;

	elem.mesh.poly.nCoords   = nCoords;
	elem.mesh.poly.nSubPolys = 1;
	elem.mesh.poly.nArcs     = 0;

	memo.coords    = reinterpret_cast<API_Coord**>   (BMAllocateHandle((nCoords+1)*(GSSize)sizeof(API_Coord),        ALLOCATE_CLEAR, 0));
	memo.pends     = reinterpret_cast<Int32**>        (BMAllocateHandle(2          *(GSSize)sizeof(Int32),             ALLOCATE_CLEAR, 0));
	memo.parcs     = reinterpret_cast<API_PolyArc**>  (BMAllocateHandle(1          *(GSSize)sizeof(API_PolyArc),      ALLOCATE_CLEAR, 0));
	memo.meshPolyZ = reinterpret_cast<double**>       (BMAllocateHandle((nCoords+1)*(GSSize)sizeof(double),           ALLOCATE_CLEAR, 0));

	// Внутренние точки рельефа — meshLevelCoords
	const Int32 nLvl = nUnique;
	memo.meshLevelCoords = reinterpret_cast<API_MeshLevelCoord**>(
		BMAllocateHandle(nLvl*(GSSize)sizeof(API_MeshLevelCoord), ALLOCATE_CLEAR, 0));

	if (!memo.coords || !memo.meshPolyZ || !memo.pends || !memo.parcs || !memo.meshLevelCoords) {
		ACAPI_WriteReport("[TopoMesh] Ошибка памяти", false);
		ACAPI_DisposeElemMemoHdls(&memo);
		return Error;
	}

	// Контур: обход BL->BR->TR->TL->BL
	const double cx4[4] = { minX, maxX, maxX, minX };
	const double cy4[4] = { minY, minY, maxY, maxY };
	for (Int32 i = 0; i < nC; ++i) {
		(*memo.coords)   [i+1].x = cx4[i];
		(*memo.coords)   [i+1].y = cy4[i];
		(*memo.meshPolyZ)[i+1]   = 0.0; // высота контура = 0 (level точки дадут рельеф)
	}
	(*memo.coords)   [nCoords] = (*memo.coords)[1];
	(*memo.meshPolyZ)[nCoords] = 0.0;

	(*memo.pends)[0] = 0;
	(*memo.pends)[1] = nCoords;

	// meshLevelEnds — обязателен вместе с meshLevelCoords
	// один контур level-точек: ends[0]=0, ends[1]=nLvl
	memo.meshLevelEnds = reinterpret_cast<Int32**>(BMAllocateHandle(2*(GSSize)sizeof(Int32), ALLOCATE_CLEAR, 0));
	if (!memo.meshLevelEnds) {
		ACAPI_WriteReport("[TopoMesh] Ошибка памяти (meshLevelEnds)", false);
		ACAPI_DisposeElemMemoHdls(&memo);
		return Error;
	}
	(*memo.meshLevelEnds)[0] = 0;
	(*memo.meshLevelEnds)[1] = nLvl;

	// Внутренние точки рельефа
	for (Int32 i = 0; i < nLvl; ++i) {
		(*memo.meshLevelCoords)[i].c.x = pts[i].x;
		(*memo.meshLevelCoords)[i].c.y = pts[i].y;
		(*memo.meshLevelCoords)[i].c.z = pts[i].z - storyElevM;
	}

	ACAPI_WriteReport("[TopoMesh] bbox=(%.2f,%.2f)-(%.2f,%.2f) nLvl=%d storyElevM=%.3f",
		false, minX, minY, maxX, maxY, nLvl, storyElevM);

	err = ACAPI_Element_Create(&elem, &memo);

	ACAPI_DisposeElemMemoHdls(&memo);

	if (err != NoError) ACAPI_WriteReport("[TopoMesh] Create failed: %d", false, (int)err);
	else                ACAPI_WriteReport("[TopoMesh] Mesh создан (%d level-точек)", false, nLvl);
	return err;
}
} // namespace

// =============================================================================
// Публичный API
// =============================================================================

namespace TopoMeshHelper {

GS::UniString GetLayerListJson()
{
	GS::UInt32 layerCount = 0;
	if (ACAPI_Attribute_GetNum(API_LayerID, layerCount) != NoError)
		return "[]";

	GS::UniString json = "[";
	bool  first   = true;
	Int32 listIdx = 0;

	for (Int32 i = 1; i <= static_cast<Int32>(layerCount); ++i) {
		API_Attribute attr = {};
		attr.header.typeID = API_LayerID;
		attr.header.index  = ACAPI_CreateAttributeIndex(i);
		if (ACAPI_Attribute_Get(&attr) != NoError) continue;
		if (!first) json += ",";
		first = false;
		json += GS::UniString::Printf("{\"name\":\"%s\",\"index\":%d}",
			JsonEscape(GS::UniString(attr.header.name)).ToCStr().Get(),
			(int)listIdx);
		++listIdx;
	}
	json += "]";
	return json;
}

GS::UniString GetStoryListJson()
{
	API_StoryInfo si = {};
	if (ACAPI_ProjectSetting_GetStorySettings(&si) != NoError)
		return "[]";
	if (si.data == nullptr)
		return "[]";

	GS::UniString json = "[";
	const Int32 cnt = (Int32)(BMGetHandleSize((GSHandle)si.data) / sizeof(API_StoryType));
	for (Int32 i = 0; i < cnt; ++i) {
		if (i > 0) json += ",";
		json += GS::UniString::Printf("{\"name\":\"%s\",\"index\":%d}",
			JsonEscape(GS::UniString((*si.data)[i].uName)).ToCStr().Get(),
			(int)(*si.data)[i].index);
	}
	BMKillHandle((GSHandle*)&si.data);
	json += "]";
	return json;
}

GS::UniString GetSampleElevationText(Int32 layerIdx)
{
	API_AttributeIndex layerAttrIdx = GetLayerAttrIdx(layerIdx);

	GS::Array<API_Guid> textList;
	ACAPI_Element_GetElemList(API_TextID, &textList);

	for (UIndex i = 0; i < textList.GetSize(); ++i) {
		API_Element elem = {};
		elem.header.guid = textList[i];
		if (ACAPI_Element_Get(&elem) != NoError) continue;
		if (elem.header.layer != layerAttrIdx) continue;
		API_ElementMemo memo = {};
		if (ACAPI_Element_GetMemo(textList[i], &memo, APIMemoMask_TextContent) != NoError) continue;
		GS::UniString result;
		if (memo.textContent != nullptr && *memo.textContent != nullptr)
			result = GS::UniString(*memo.textContent);
		ACAPI_DisposeElemMemoHdls(&memo);
		if (!result.IsEmpty()) return result;
	}
	return "(текстов на слое не найдено)";
}

bool CreateTopoMesh(const GS::UniString& jsonPayload)
{
	TopoParams params = {};
	if (!ParseTopoParams(jsonPayload, params)) return false;

	GS::UInt32 layerCount = 0;
	ACAPI_Attribute_GetNum(API_LayerID, layerCount);
	ACAPI_WriteReport("[TopoMesh] srcIdx=%d dstIdx=%d radius=%.0f sep=%c story=%d bbox=%.0f layers=%u name='%s'",
		false,
		params.layerIdx, params.meshLayerIdx,
		params.radiusMm, params.separator,
		params.storyIdx, params.bboxOffsetMm,
		layerCount, params.meshName.ToCStr().Get());

	if (params.layerIdx < 0 || params.layerIdx >= (Int32)layerCount) {
		ACAPI_WriteReport("[TopoMesh] Неверный исходный слой %d", false, params.layerIdx);
		return false;
	}
	if (params.meshLayerIdx < 0 || params.meshLayerIdx >= (Int32)layerCount) {
		ACAPI_WriteReport("[TopoMesh] Неверный слой для Mesh %d", false, params.meshLayerIdx);
		return false;
	}

	if (params.storyIdx <= 0) {
		API_StoryInfo si = {};
		if (ACAPI_ProjectSetting_GetStorySettings(&si) == NoError && si.data != nullptr) {
			const Int32 cnt = (Int32)(BMGetHandleSize((GSHandle)si.data) / sizeof(API_StoryType));
			for (Int32 i = 0; i < cnt; ++i) {
				if ((*si.data)[i].index > 0) { params.storyIdx = (*si.data)[i].index; break; }
			}
			BMKillHandle((GSHandle*)&si.data);
		}
		if (params.storyIdx <= 0) params.storyIdx = 1;
		ACAPI_WriteReport("[TopoMesh] storyIdx fallback -> %d", false, params.storyIdx);
	}

	API_AttributeIndex layerAttrIdx = GetLayerAttrIdx(params.layerIdx);

	std::vector<ArcPoint> arcs;
	std::vector<TextItem> texts;
	CollectOnLayer(layerAttrIdx, arcs, texts);

	ACAPI_WriteReport("[TopoMesh] Дуг: %d, текстов: %d", false, (int)arcs.size(), (int)texts.size());
	if (arcs.empty())  { ACAPI_WriteReport("[TopoMesh] Нет Arc на слое",     false); return false; }
	if (texts.empty()) { ACAPI_WriteReport("[TopoMesh] Нет текстов на слое", false); return false; }

	std::vector<TopoPoint> topo = MatchPoints(arcs, texts, params.radiusMm, params.separator);
	ACAPI_WriteReport("[TopoMesh] Сопоставлено: %d", false, (int)topo.size());
	if (topo.size() < 3) { ACAPI_WriteReport("[TopoMesh] Мало точек", false); return false; }

	const double storyElevM = GetStoryElevM(params.storyIdx);
	{
		API_StoryInfo si2 = {};
		if (ACAPI_ProjectSetting_GetStorySettings(&si2) == NoError && si2.data != nullptr) {
			ACAPI_WriteReport("[TopoMesh] firstStory=%d lastStory=%d storyIdx=%d storyElevM=%.3f",
				false, si2.firstStory, si2.lastStory, params.storyIdx, storyElevM);
			BMKillHandle((GSHandle*)&si2.data);
		}
	}

	// CEF thread — нельзя вызывать API напрямую.
	// Передаём данные через static и запрашиваем вызов из главного потока.
	static GS::UniString s_pendingPayload;
	static std::vector<TopoPoint> s_pts;
	static TopoParams s_params;
	static double s_storyElevM;

	s_pendingPayload = jsonPayload;
	s_pts   = topo;
	s_params = params;
	s_storyElevM = storyElevM;

	GSErrCode evErr = ACAPI_Command_CallFromEventLoop(
		[](void*) -> GSErrCode {
			return ACAPI_CallUndoableCommand("Create Topo Mesh", [&]() -> GSErrCode {
				return BuildMesh(s_pts, s_params, s_storyElevM);
			});
		},
		nullptr, false);

	if (evErr != NoError)
		ACAPI_WriteReport("[TopoMesh] CallFromEventLoop failed: %d", false, (int)evErr);
	else
		ACAPI_WriteReport("[TopoMesh] CallFromEventLoop scheduled ok", false);

	return evErr == NoError;
}

void GetLayerList(GS::Array<GS::Pair<GS::UniString, Int32>>& outLayers)
{
	outLayers.Clear();
	GS::UInt32 layerCount = 0;
	if (ACAPI_Attribute_GetNum(API_LayerID, layerCount) != NoError || layerCount == 0)
		return;

	Int32 listIdx = 0;
	for (GS::UInt32 i = 1; i <= layerCount; ++i) {
		API_Attribute attr = {};
		attr.header.typeID = API_LayerID;
		attr.header.index  = ACAPI_CreateAttributeIndex((short)i);
		if (ACAPI_Attribute_Get(&attr) == NoError) {
			outLayers.Push(GS::Pair<GS::UniString, Int32>{ attr.header.name, listIdx });
			++listIdx;
		}
	}
}

void GetStoryList(GS::Array<GS::Pair<GS::UniString, Int32>>& outStories)
{
	outStories.Clear();
	API_StoryInfo si = {};
	if (ACAPI_ProjectSetting_GetStorySettings(&si) != NoError)
		return;
	if (si.data == nullptr)
		return;

	const Int32 cnt = (Int32)(BMGetHandleSize((GSHandle)si.data) / sizeof(API_StoryType));
	for (Int32 i = 0; i < cnt; ++i) {
		const API_StoryType& st = (*si.data)[i];
		outStories.Push(GS::Pair<GS::UniString, Int32>{ GS::UniString(st.uName), (Int32)st.index });
	}
	BMKillHandle((GSHandle*)&si.data);
}

} // namespace TopoMeshHelper
