#include "TopoMeshHelper.hpp"

#include "APIEnvir.h"
#include "ACAPinc.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <string>
#include <limits>

// =============================================================================
// Внутренние структуры
// =============================================================================

namespace {

struct TopoPoint {
	double x;   // ArchiCAD meters
	double y;
	double z;
};

struct ParseParams {
	Int32          layerIdx;
	double         radiusMm;
	char           separator;   // '.' или ','
	Int32          mFactor;
	Int32          mmFactor;
	Int32          storyIdx;
	double         bboxOffsetMm;
	GS::UniString  meshName;
	Int32          meshLayerIdx;
};

// =============================================================================
// Простой JSON парсер (без внешних зависимостей)
// =============================================================================

// Найти значение ключа в плоском JSON объекте
// Возвращает строку значения (без кавычек для строк)
static GS::UniString JsonGetString(const GS::UniString& json, const char* key)
{
	// Ищем "key":
	GS::UniString searchKey = GS::UniString("\"") + key + "\":";
	const char* src = json.ToCStr().Get();
	const char* found = std::strstr(src, searchKey.ToCStr().Get());
	if (found == nullptr) return GS::EmptyUniString;

	const char* val = found + searchKey.GetLength();
	while (*val == ' ') ++val;

	if (*val == '"') {
		// строковое значение
		++val;
		const char* end = std::strchr(val, '"');
		if (end == nullptr) return GS::EmptyUniString;
		char buf[512] = {};
		std::strncpy(buf, val, (size_t)(end - val));
		return GS::UniString(buf);
	} else {
		// числовое / булево значение
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
	double out = def;
	std::sscanf(s.ToCStr().Get(), "%lf", &out);
	return out;
}

static Int32 JsonGetInt(const GS::UniString& json, const char* key, Int32 def = 0)
{
	GS::UniString s = JsonGetString(json, key);
	if (s.IsEmpty()) return def;
	Int32 out = def;
	std::sscanf(s.ToCStr().Get(), "%d", &out);
	return out;
}

// =============================================================================
// Парсинг параметров из JSON
// =============================================================================

static bool ParseParams(const GS::UniString& json, ParseParams& p)
{
	p.layerIdx    = JsonGetInt(json, "layerIdx", -1);
	p.radiusMm    = JsonGetDouble(json, "radius", 3000.0);
	p.mFactor     = JsonGetInt(json, "mFactor", 1);
	p.mmFactor    = JsonGetInt(json, "mmFactor", 1000);
	p.storyIdx    = JsonGetInt(json, "storyIdx", 0);
	p.bboxOffsetMm = JsonGetDouble(json, "bboxOffset", 1000.0);
	p.meshName    = JsonGetString(json, "meshName");
	p.meshLayerIdx = JsonGetInt(json, "meshLayer", 0);

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
// Парсинг высотной отметки из строки
// Форматы: "14.200", "14,200", "+14.2", "-0.450"
// Возвращает высоту в мм
// =============================================================================

static bool ParseElevationText(const char* text, char separator, Int32 mFactor, Int32 mmFactor, double& outMm)
{
	if (text == nullptr || text[0] == '\0') return false;

	// Убираем пробелы и знак +
	while (*text == ' ') ++text;
	bool negative = false;
	if (*text == '+') ++text;
	else if (*text == '-') { negative = true; ++text; }

	// Заменяем разделитель на '.' для sscanf
	char buf[64] = {};
	std::strncpy(buf, text, 63);
	for (int i = 0; buf[i]; ++i)
		if (buf[i] == separator) buf[i] = '.';

	double val = 0.0;
	if (std::sscanf(buf, "%lf", &val) != 1) return false;

	// val в метрах → переводим в мм
	// mFactor и mmFactor: по умолчанию 1 и 1000 → val * 1000
	// Пользователь может настроить если данные в другом масштабе
	outMm = val * (double)mFactor * (double)mmFactor;
	if (negative) outMm = -outMm;

	return true;
}

// =============================================================================
// Получение API_AttributeIndex слоя по номеру в списке (1-based)
// =============================================================================

static API_AttributeIndex GetLayerAttributeIndex(Int32 listIndex)
{
	// listIndex — порядковый номер из нашего списка (0-based)
	// Перебираем слои и возвращаем нужный
	API_Attribute attrib = {};
	attrib.header.typeID = API_LayerID;

	Int32 count = 0;
	Int32 current = 0;

	while (true) {
		attrib.header.index = current + 1;
		if (ACAPI_Attribute_Get(&attrib) != NoError) break;

		if (current == listIndex)
			return attrib.header.index;

		++current;
		if (current > 1000) break; // защита от бесконечного цикла
	}
	return 1; // fallback: первый слой
}

// =============================================================================
// Сбор элементов на слое
// =============================================================================

struct ArcPoint {
	double x, y; // центр Arc/Circle в метрах ArchiCAD
};

struct TextItem {
	double x, y;
	GS::UniString text;
};

static void CollectElementsOnLayer(API_AttributeIndex layerAttrIdx,
	std::vector<ArcPoint>& arcs,
	std::vector<TextItem>& texts)
{
	// Перебираем все элементы через ACAPI_Element_GetElemList
	GS::Array<API_Guid> elemList;
	ACAPI_Element_GetElemList(API_ArcID, &elemList);
	ACAPI_Element_GetElemList(API_CircleID, &elemList); // Circle — тот же Arc с полным углом

	for (const API_Guid& guid : elemList) {
		API_Element elem = {};
		elem.header.guid = guid;
		if (ACAPI_Element_Get(&elem) != NoError) continue;
		if (elem.header.layer != layerAttrIdx) continue;

		ArcPoint ap;
		ap.x = elem.arc.origC.x;
		ap.y = elem.arc.origC.y;
		arcs.push_back(ap);
	}

	// Тексты
	GS::Array<API_Guid> textList;
	ACAPI_Element_GetElemList(API_TextID, &textList);

	for (const API_Guid& guid : textList) {
		API_Element elem = {};
		elem.header.guid = guid;
		API_ElementMemo memo = {};
		if (ACAPI_Element_Get(&elem) != NoError) continue;
		if (elem.header.layer != layerAttrIdx) continue;
		if (ACAPI_Element_GetMemo(guid, &memo, APIMemoMask_TextContent) != NoError) continue;

		TextItem ti;
		ti.x = elem.text.loc.x;
		ti.y = elem.text.loc.y;

		// Получаем текст из memo
		if (memo.textContent != nullptr) {
			// textContent — handle на строку в кодировке системы
			const char* raw = *memo.textContent;
			if (raw != nullptr)
				ti.text = GS::UniString(raw);
		}
		ACAPI_DisposeElemMemoHdls(&memo);

		if (!ti.text.IsEmpty())
			texts.push_back(ti);
	}
}

// =============================================================================
// Пространственное сопоставление: Arc ↔ ближайший Text в радиусе
// =============================================================================

static double Dist2(double ax, double ay, double bx, double by)
{
	const double dx = ax - bx;
	const double dy = ay - by;
	return dx * dx + dy * dy;
}

static std::vector<TopoPoint> MatchPointsWithElevations(
	const std::vector<ArcPoint>& arcs,
	const std::vector<TextItem>& texts,
	double radiusMm,
	char separator,
	Int32 mFactor,
	Int32 mmFactor)
{
	// radiusMm → meters (ArchiCAD внутри в метрах)
	const double radiusM = radiusMm / 1000.0;
	const double r2 = radiusM * radiusM;

	std::vector<TopoPoint> result;
	result.reserve(arcs.size());

	for (const ArcPoint& arc : arcs) {
		double bestDist2 = std::numeric_limits<double>::max();
		const TextItem* bestText = nullptr;

		for (const TextItem& ti : texts) {
			const double d2 = Dist2(arc.x, arc.y, ti.x, ti.y);
			if (d2 <= r2 && d2 < bestDist2) {
				bestDist2 = d2;
				bestText = &ti;
			}
		}

		if (bestText == nullptr) continue; // не нашли текст

		double elevMm = 0.0;
		if (!ParseElevationText(bestText->text.ToCStr().Get(), separator, mFactor, mmFactor, elevMm))
			continue; // не смогли распарсить

		TopoPoint tp;
		tp.x = arc.x;
		tp.y = arc.y;
		tp.z = elevMm / 1000.0; // мм → метры для ArchiCAD
		result.push_back(tp);
	}

	return result;
}

// =============================================================================
// Построение Mesh через ArchiCAD API
// =============================================================================

static bool BuildMesh(const std::vector<TopoPoint>& points,
	const ParseParams& p,
	double storyElevM)
{
	if (points.size() < 3) {
		ACAPI_WriteReport("[TopoMesh] Ошибка: менее 3 точек (%d), Mesh не создаётся", false, (int)points.size());
		return false;
	}

	// --- Bounding box ---
	const double offsetM = p.bboxOffsetMm / 1000.0;

	double minX = points[0].x, maxX = points[0].x;
	double minY = points[0].y, maxY = points[0].y;
	double minZ = points[0].z, maxZ = points[0].z;

	for (const TopoPoint& pt : points) {
		if (pt.x < minX) minX = pt.x;
		if (pt.x > maxX) maxX = pt.x;
		if (pt.y < minY) minY = pt.y;
		if (pt.y > maxY) maxY = pt.y;
		if (pt.z < minZ) minZ = pt.z;
		if (pt.z > maxZ) maxZ = pt.z;
	}

	minX -= offsetM; minY -= offsetM;
	maxX += offsetM; maxY += offsetM;

	// Углы bounding box получают minZ
	// Контур: 4 угла (+ повтор первого для замкнутости)
	// Итого: 4 угловые точки + N топо точек

	const int nCorners = 4;
	const int nTopoPoints = (int)points.size();
	const int nTotal = nCorners + nTopoPoints;

	// coords — 1-based, размер (nTotal + 1)
	// meshPolyZ — 1-based, размер (nTotal + 1)

	API_Element    elem    = {};
	API_ElementMemo memo   = {};

	// Defaults
	if (ACAPI_Element_GetDefaults(&elem, &memo) != NoError) {
		ACAPI_WriteReport("[TopoMesh] Ошибка: GetDefaults", false);
		ACAPI_DisposeElemMemoHdls(&memo);
		return false;
	}

	// Настройка элемента
	elem.header.typeID = API_MeshID;
	elem.header.layer  = (short)GetLayerAttributeIndex(p.meshLayerIdx);

	// Этаж
	elem.mesh.head.floorInd = (short)p.storyIdx;
	// level — высота нижней грани относительно этажа (берём minZ - storyElevM)
	elem.mesh.level = minZ - storyElevM;

	// Имя (elemID)
	GS::ucscpy(elem.mesh.head.hasMemo ? 
		reinterpret_cast<GS::uchar_t*>(elem.mesh.head.elemID.elemIDStr) : 
		reinterpret_cast<GS::uchar_t*>(elem.mesh.head.elemID.elemIDStr),
		p.meshName.ToUStr().Get());

	// Выделяем memo
	ACAPI_DisposeElemMemoHdls(&memo);

	memo.coords    = reinterpret_cast<API_Coord**>  (BMAllocateHandle((nTotal + 1) * sizeof(API_Coord),   ALLOCATE_CLEAR, 0));
	memo.meshPolyZ = reinterpret_cast<double**>     (BMAllocateHandle((nTotal + 1) * sizeof(double),      ALLOCATE_CLEAR, 0));
	memo.pends     = reinterpret_cast<Int32**>      (BMAllocateHandle(2             * sizeof(Int32),       ALLOCATE_CLEAR, 0));

	if (memo.coords == nullptr || memo.meshPolyZ == nullptr || memo.pends == nullptr) {
		ACAPI_WriteReport("[TopoMesh] Ошибка: выделение памяти", false);
		ACAPI_DisposeElemMemoHdls(&memo);
		return false;
	}

	// Заполняем coords и meshPolyZ (1-based: индекс 0 не используется)

	// Углы контура (индексы 1..4)
	// BL, BR, TR, TL
	const double corners[4][2] = {
		{ minX, minY },
		{ maxX, minY },
		{ maxX, maxY },
		{ minX, maxY }
	};

	for (int i = 0; i < nCorners; ++i) {
		(*memo.coords)[i + 1].x    = corners[i][0];
		(*memo.coords)[i + 1].y    = corners[i][1];
		(*memo.meshPolyZ)[i + 1]   = minZ - storyElevM; // относительно reference plane
	}

	// Топо точки (индексы 5..nTotal)
	for (int i = 0; i < nTopoPoints; ++i) {
		const int idx = nCorners + i + 1;
		(*memo.coords)[idx].x    = points[i].x;
		(*memo.coords)[idx].y    = points[i].y;
		(*memo.meshPolyZ)[idx]   = points[i].z - storyElevM;
	}

	// pends: один контур — индексы [0, nTotal]
	// pends[0] = 0 (начало контура)
	// pends[1] = nTotal (конец контура, 1-based последняя точка контура = nCorners)
	// Контур задаётся только угловыми точками (1..4), остальные — внутренние точки
	(*memo.pends)[0] = 0;
	(*memo.pends)[1] = nCorners; // контур из 4 угловых точек

	// nSubPolys = 1
	elem.mesh.u.siz = nCorners;   // число вершин контура

	// Создаём элемент
	const GSErrCode err = ACAPI_Element_Create(&elem, &memo);
	ACAPI_DisposeElemMemoHdls(&memo);

	if (err != NoError) {
		ACAPI_WriteReport("[TopoMesh] Ошибка создания Mesh: %d", false, (int)err);
		return false;
	}

	ACAPI_WriteReport("[TopoMesh] Mesh создан успешно (%d точек)", false, nTopoPoints);
	return true;
}

// =============================================================================
// Получение высоты этажа
// =============================================================================

static double GetStoryElevationM(Int32 storyIdx)
{
	API_StoryInfo storyInfo = {};
	if (ACAPI_Environment(APIEnv_GetStorySettingsID, &storyInfo) != NoError)
		return 0.0;

	double elevation = 0.0;
	const short nStories = storyInfo.lastStory - storyInfo.firstStory + 1;

	for (short i = 0; i < nStories; ++i) {
		if (storyInfo.data[i].index == (short)storyIdx) {
			elevation = storyInfo.data[i].level; // уже в метрах
			break;
		}
	}

	BMpKill(reinterpret_cast<GSPtr*>(&storyInfo.data));
	return elevation;
}

} // anonymous namespace

// =============================================================================
// Публичный API
// =============================================================================

namespace TopoMeshHelper {

GS::UniString GetLayerListJson()
{
	GS::UniString json = "[";
	bool first = true;

	API_Attribute attrib = {};
	attrib.header.typeID = API_LayerID;

	Int32 idx = 0;
	short attrIdx = 1;

	while (true) {
		attrib.header.index = attrIdx;
		if (ACAPI_Attribute_Get(&attrib) != NoError) break;

		if (!first) json += ",";
		first = false;

		// Экранируем имя слоя
		GS::UniString name = GS::UniString(attrib.layer.head.name);
		// Простое экранирование кавычек
		GS::UniString escaped;
		for (Int32 ci = 0; ci < (Int32)name.GetLength(); ++ci) {
			if (name[ci] == '"') escaped += "\\\"";
			else escaped += name[ci];
		}

		json += GS::UniString::Printf("{\"name\":\"%s\",\"index\":%d}",
			escaped.ToCStr().Get(), (int)idx);

		++idx;
		++attrIdx;
		if (attrIdx > 1000) break;
	}

	json += "]";
	return json;
}

GS::UniString GetStoryListJson()
{
	GS::UniString json = "[";

	API_StoryInfo storyInfo = {};
	if (ACAPI_Environment(APIEnv_GetStorySettingsID, &storyInfo) != NoError)
		return "[]";

	const short nStories = storyInfo.lastStory - storyInfo.firstStory + 1;
	bool first = true;

	for (short i = 0; i < nStories; ++i) {
		if (!first) json += ",";
		first = false;

		GS::UniString name = GS::UniString(storyInfo.data[i].uName);
		GS::UniString escaped;
		for (Int32 ci = 0; ci < (Int32)name.GetLength(); ++ci) {
			if (name[ci] == '"') escaped += "\\\"";
			else escaped += name[ci];
		}

		json += GS::UniString::Printf("{\"name\":\"%s\",\"index\":%d}",
			escaped.ToCStr().Get(), (int)storyInfo.data[i].index);
	}

	BMpKill(reinterpret_cast<GSPtr*>(&storyInfo.data));
	json += "]";
	return json;
}

GS::UniString GetSampleElevationText(Int32 layerIdx)
{
	const API_AttributeIndex layerAttrIdx = GetLayerAttributeIndex(layerIdx);

	GS::Array<API_Guid> textList;
	ACAPI_Element_GetElemList(API_TextID, &textList);

	for (const API_Guid& guid : textList) {
		API_Element elem = {};
		elem.header.guid = guid;
		if (ACAPI_Element_Get(&elem) != NoError) continue;
		if (elem.header.layer != layerAttrIdx) continue;

		API_ElementMemo memo = {};
		if (ACAPI_Element_GetMemo(guid, &memo, APIMemoMask_TextContent) != NoError) continue;

		GS::UniString result;
		if (memo.textContent != nullptr && *memo.textContent != nullptr)
			result = GS::UniString(*memo.textContent);

		ACAPI_DisposeElemMemoHdls(&memo);

		if (!result.IsEmpty())
			return result;
	}

	return GS::UniString("(текстов на слое не найдено)");
}

bool CreateTopoMesh(const GS::UniString& jsonPayload)
{
	// 1. Парсим параметры
	ParseParams params = {};
	if (!ParseParams(jsonPayload, params)) return false;

	// 2. Индекс слоя → API_AttributeIndex
	const API_AttributeIndex layerAttrIdx = GetLayerAttributeIndex(params.layerIdx);

	// 3. Собираем элементы со слоя
	std::vector<ArcPoint>  arcs;
	std::vector<TextItem>  texts;
	CollectElementsOnLayer(layerAttrIdx, arcs, texts);

	ACAPI_WriteReport("[TopoMesh] Найдено дуг: %d, текстов: %d", false,
		(int)arcs.size(), (int)texts.size());

	if (arcs.empty()) {
		ACAPI_WriteReport("[TopoMesh] Ошибка: нет Arc/Circle на слое", false);
		return false;
	}
	if (texts.empty()) {
		ACAPI_WriteReport("[TopoMesh] Ошибка: нет текстов на слое", false);
		return false;
	}

	// 4. Сопоставляем Arc ↔ Text
	std::vector<TopoPoint> topoPoints = MatchPointsWithElevations(
		arcs, texts,
		params.radiusMm,
		params.separator,
		params.mFactor,
		params.mmFactor
	);

	ACAPI_WriteReport("[TopoMesh] Сопоставлено точек: %d", false, (int)topoPoints.size());

	// 5. Высота этажа
	const double storyElevM = GetStoryElevationM(params.storyIdx);

	// 6. Создаём Mesh
	ACAPI_CallUndoableCommand("Create Topo Mesh", [&]() -> GSErrCode {
		return BuildMesh(topoPoints, params, storyElevM) ? NoError : Error;
	});

	return true;
}

} // namespace TopoMeshHelper
