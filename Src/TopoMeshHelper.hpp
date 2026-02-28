#pragma once

#include "UniString.hpp"
#include "Array.hpp"
#include "Pair.hpp"

namespace TopoMeshHelper {

// Возвращает список слоёв: [("Layer name", index), ...]
void GetLayerList (GS::Array<GS::Pair<GS::UniString, Int32>>& out);

// Возвращает список этажей: [("Story name", index), ...]
void GetStoryList (GS::Array<GS::Pair<GS::UniString, Int32>>& out);

// (можешь оставить, если уже используешь где-то)
GS::UniString GetLayerListJson ();
GS::UniString GetStoryListJson ();

GS::UniString GetSampleElevationText (Int32 layerIdx);

bool CreateTopoMesh (const GS::UniString& jsonPayload);

} // namespace TopoMeshHelper