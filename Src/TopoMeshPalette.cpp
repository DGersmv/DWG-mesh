#include "TopoMeshPalette.hpp"
#include "TopoMeshHelper.hpp"

#include "APIEnvir.h"
#include "ACAPinc.h"
#include "DGBrowser.hpp"

#include "Array.hpp"
#include "Pair.hpp"

#include <cstdio>

// =============================================================================
// Загрузка HTML из ресурса DATA 100
// =============================================================================

static GS::UniString LoadTopoMeshHtml ()
{
	GS::UniString html;

	GSHandle data = RSLoadResource ('DATA', ACAPI_GetOwnResModule (), TopoMeshHtmlResId);
	if (data != nullptr) {
		const GSSize size = BMhGetSize (data);
		html.Append (*data, size);
		BMhKill (&data);
	} else {
		html =
			"<html><body style='font-family:Arial;color:#900;padding:12px;'>"
			"Не удалось загрузить HTML палитры TopoMesh."
			"</body></html>";
	}

	return html;
}

// =============================================================================
// Утилиты для извлечения параметров из JS::Base
// =============================================================================

static GS::UniString GetStringFromJs (GS::Ref<JS::Base> p)
{
	if (p == nullptr)
		return GS::EmptyUniString;

	if (GS::Ref<JS::Value> v = GS::DynamicCast<JS::Value> (p)) {
		if (v->GetType () == JS::Value::STRING)
			return v->GetString ();
	}

	return GS::EmptyUniString;
}

static Int32 GetIntFromJs (GS::Ref<JS::Base> p, Int32 def = 0)
{
	if (p == nullptr)
		return def;

	if (GS::Ref<JS::Value> v = GS::DynamicCast<JS::Value> (p)) {
		const auto t = v->GetType ();

		if (t == JS::Value::INTEGER)
			return static_cast<Int32> (v->GetInteger ());

		if (t == JS::Value::DOUBLE)
			return static_cast<Int32> (v->GetDouble ());

		if (t == JS::Value::STRING) {
			Int32 out = def;
			std::sscanf (v->GetString ().ToCStr ().Get (), "%d", &out);
			return out;
		}
	}

	return def;
}

// =============================================================================
// Регистрация JS объекта ACAPI в браузере
// =============================================================================

static void RegisterTopoMeshJSObject (DG::Browser& browser)
{
	JS::Object* jsACAPI = new JS::Object ("ACAPI");

	// -------------------------------------------------------------------------
	// ACAPI.GetLayerList() -> JS Array: [ [name, layerIndex], ... ]
	// (layerIndex = реальный индекс слоя Archicad)
	// -------------------------------------------------------------------------
	jsACAPI->AddItem (new JS::Function ("GetLayerList",
		[] (GS::Ref<JS::Base>) -> GS::Ref<JS::Base> {

			GS::Array<GS::Pair<GS::UniString, Int32>> layers;
			TopoMeshHelper::GetLayerList (layers);

			JS::Array* arr = new JS::Array ();
			for (const auto& it : layers) {
				JS::Array* row = new JS::Array ();
				row->AddItem (new JS::Value (it.first));                       // name
				row->AddItem (new JS::Value (static_cast<double> (it.second))); // layerIndex
				arr->AddItem (row);
			}

			return arr;
		}));

	// -------------------------------------------------------------------------
	// ACAPI.GetStoryList() -> JS Array: [ [name, storyIndex], ... ]
	// -------------------------------------------------------------------------
	jsACAPI->AddItem (new JS::Function ("GetStoryList",
		[] (GS::Ref<JS::Base>) -> GS::Ref<JS::Base> {

			GS::Array<GS::Pair<GS::UniString, Int32>> stories;
			TopoMeshHelper::GetStoryList (stories);

			JS::Array* arr = new JS::Array ();
			for (const auto& it : stories) {
				JS::Array* row = new JS::Array ();
				row->AddItem (new JS::Value (it.first));                        // name
				row->AddItem (new JS::Value (static_cast<double> (it.second)));  // storyIndex
				arr->AddItem (row);
			}

			return arr;
		}));

	// -------------------------------------------------------------------------
	// ACAPI.GetSampleElevationText(layerIdx) -> string
	// layerIdx = индекс слоя (то, что придёт из select.value)
	// -------------------------------------------------------------------------
	jsACAPI->AddItem (new JS::Function ("GetSampleElevationText",
		[] (GS::Ref<JS::Base> param) -> GS::Ref<JS::Base> {
			const Int32 layerIdx = GetIntFromJs (param, 0);
			const GS::UniString sample = TopoMeshHelper::GetSampleElevationText (layerIdx);
			return new JS::Value (sample);
		}));

	// -------------------------------------------------------------------------
	// ACAPI.CreateTopoMesh(jsonPayload) -> bool
	// -------------------------------------------------------------------------
	jsACAPI->AddItem (new JS::Function ("CreateTopoMesh",
		[] (GS::Ref<JS::Base> param) -> GS::Ref<JS::Base> {
			const GS::UniString payload = GetStringFromJs (param);
			const bool ok = TopoMeshHelper::CreateTopoMesh (payload);
			return new JS::Value (ok);
		}));

	browser.RegisterAsynchJSObject (jsACAPI);
}

// =============================================================================
// Palette callback
// =============================================================================

static GSErrCode TopoMeshPaletteCallback (Int32 /*refCon*/, API_PaletteMessageID messageID, GS::IntPtr param)
{
	switch (messageID) {
	case APIPalMsg_OpenPalette:
		if (!TopoMeshPalette::HasInstance ())
			TopoMeshPalette::CreateInstance ();
		TopoMeshPalette::GetInstance ().Show ();
		break;

	case APIPalMsg_ClosePalette:
		if (TopoMeshPalette::HasInstance ())
			TopoMeshPalette::GetInstance ().Hide ();
		break;

	case APIPalMsg_HidePalette_Begin:
		if (TopoMeshPalette::HasInstance () && TopoMeshPalette::GetInstance ().IsVisible ())
			TopoMeshPalette::GetInstance ().Hide ();
		break;

	case APIPalMsg_HidePalette_End:
		if (TopoMeshPalette::HasInstance () && !TopoMeshPalette::GetInstance ().IsVisible ())
			TopoMeshPalette::GetInstance ().Show ();
		break;

	case APIPalMsg_DisableItems_Begin:
		if (TopoMeshPalette::HasInstance () && TopoMeshPalette::GetInstance ().IsVisible ())
			TopoMeshPalette::GetInstance ().DisableItems ();
		break;

	case APIPalMsg_DisableItems_End:
		if (TopoMeshPalette::HasInstance () && TopoMeshPalette::GetInstance ().IsVisible ())
			TopoMeshPalette::GetInstance ().EnableItems ();
		break;

	case APIPalMsg_IsPaletteVisible:
		if (param != 0) {
			*reinterpret_cast<bool*> (param) =
				TopoMeshPalette::HasInstance () && TopoMeshPalette::GetInstance ().IsVisible ();
		}
		break;

	default:
		break;
	}

	return NoError;
}

// =============================================================================
// Static members
// =============================================================================

GS::Ref<TopoMeshPalette> TopoMeshPalette::s_instance (nullptr);
const GS::Guid TopoMeshPalette::s_guid ("{b7e2a941-3f84-4d1c-a927-65f80c312eb4}");

// =============================================================================
// Constructor / Destructor
// =============================================================================

TopoMeshPalette::TopoMeshPalette () :
	DG::Palette (ACAPI_GetOwnResModule (), TopoMeshPaletteResId, ACAPI_GetOwnResModule (), s_guid)
{
	m_browserCtrl = new DG::Browser (GetReference (), TopoMeshBrowserCtrlId);

	Attach (*this);
	BeginEventProcessing ();
	Init ();
}

TopoMeshPalette::~TopoMeshPalette ()
{
	EndEventProcessing ();

	delete m_browserCtrl;
	m_browserCtrl = nullptr;
}

void TopoMeshPalette::Init ()
{
	// Register the ACAPI bridge before the HTML loads.  The browser may run
	// the page's <script> immediately when LoadHTML is called, and the
	// JavaScript code expects window.ACAPI to exist in its load handler.  If
	// we load HTML first, the script can execute before registration, causing
	// the "ACAPI недоступен" error even though we add the object moments
	// later.  Swapping the order fixes the race.
	if (m_browserCtrl != nullptr)
		RegisterTopoMeshJSObject (*m_browserCtrl);

	LoadHtml ();
}

void TopoMeshPalette::LoadHtml ()
{
	if (m_browserCtrl == nullptr)
		return;

	m_browserCtrl->LoadHTML (LoadTopoMeshHtml ());
}

// =============================================================================
// Static methods
// =============================================================================

bool TopoMeshPalette::HasInstance ()
{
	return s_instance != nullptr;
}

void TopoMeshPalette::CreateInstance ()
{
	DBASSERT (!HasInstance ());

	s_instance = new TopoMeshPalette ();
	ACAPI_KeepInMemory (true);
}

TopoMeshPalette& TopoMeshPalette::GetInstance ()
{
	DBASSERT (HasInstance ());
	return *s_instance;
}

void TopoMeshPalette::DestroyInstance ()
{
	s_instance = nullptr;
}

void TopoMeshPalette::ShowPalette ()
{
	if (!HasInstance ())
		CreateInstance ();

	GetInstance ().Show ();
}

void TopoMeshPalette::HidePalette ()
{
	if (!HasInstance ())
		return;

	GetInstance ().Hide ();
}

GSErrCode TopoMeshPalette::RegisterPaletteControlCallBack ()
{
	return ACAPI_RegisterModelessWindow (
		GS::CalculateHashValue (s_guid),
		TopoMeshPaletteCallback,
		API_PalEnabled_FloorPlan		 |
		API_PalEnabled_Section			 |
		API_PalEnabled_Elevation		 |
		API_PalEnabled_InteriorElevation |
		API_PalEnabled_3D				 |
		API_PalEnabled_Detail			 |
		API_PalEnabled_Worksheet		 |
		API_PalEnabled_Layout			 |
		API_PalEnabled_DocumentFrom3D,
		GSGuid2APIGuid (s_guid)
	);
}

// =============================================================================
// Event handlers
// =============================================================================

void TopoMeshPalette::PanelResized (const DG::PanelResizeEvent& ev)
{
	BeginMoveResizeItems ();

	if (m_browserCtrl != nullptr)
		m_browserCtrl->Resize (ev.GetHorizontalChange (), ev.GetVerticalChange ());

	EndMoveResizeItems ();
}

void TopoMeshPalette::PanelCloseRequested (const DG::PanelCloseRequestEvent&, bool* accepted)
{
	HidePalette ();
	if (accepted != nullptr)
		*accepted = true;
}