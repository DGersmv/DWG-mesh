#pragma once

#include "UniString.hpp"

namespace TopoMeshHelper {

// Возвращает JSON строку: [{"name":"Слой 1","index":1}, ...]
GS::UniString GetLayerListJson();

// Возвращает JSON строку: [{"name":"0. Этаж","index":0}, ...]
GS::UniString GetStoryListJson();

// Возвращает первый текст с указанного слоя (для preview парсинга)
GS::UniString GetSampleElevationText(Int32 layerIdx);

// Создаёт Mesh из DWG топо данных
// jsonPayload: {
//   "layerIdx":     int,      // слой с точками (Arc/Circle + Text)
//   "radius":       double,   // радиус поиска текста (мм)
//   "separator":    string,   // "." или ","
//   "mFactor":      int,      // множитель метров (обычно 1)
//   "mmFactor":     int,      // множитель мм (обычно 1000)
//   "storyIdx":     int,      // индекс этажа (reference plane)
//   "bboxOffset":   double,   // отступ bounding box (мм)
//   "meshName":     string,   // имя элемента
//   "meshLayer":    int       // индекс слоя для Mesh
// }
bool CreateTopoMesh(const GS::UniString& jsonPayload);

} // namespace TopoMeshHelper
