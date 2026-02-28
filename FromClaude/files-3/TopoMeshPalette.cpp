#include "TopoMeshPalette.hpp"
#include "TopoMeshHelper.hpp"

#include "APIEnvir.h"
#include "ACAPinc.h"
#include "DGBrowser.hpp"

// -----------------------------------------------------------------------------
// Загрузка HTML из ресурса
// -----------------------------------------------------------------------------

static GS::UniString LoadTopoMeshHtml()
{
	GS::UniString html;
	GSHandle data = RSLoadResource('DATA', ACAPI_GetOwnResModule(), TopoMeshHtmlResId);
	if (data != nullptr) {
		const GSSize size = BMhGetSize(data);
		html.Append(*data, size);
		BMhKill(&data);
	} else {
		html = "<html><body style='font-family:Arial;color:#900;padding:12px;'>"
			"Не удалось загрузить HTML палитры TopoMesh."
			"</body></html>";
	}
	return html;
}

// -----------------------------------------------------------------------------
// JavaScript API — мост между HTML и C++
// -----------------------------------------------------------------------------

class TopoMeshJSHandler : public DG::JSResultHandler
{
public:
	TopoMeshJSHandler(DG::Browser& browser) : m_browser(browser) {}

	void HandleJSResult(const GS::UniString& funcName, const GS::UniString& param, DG::JSResult* result) override
	{
		if (funcName == "GetLayerList") {
			const GS::UniString json = TopoMeshHelper::GetLayerListJson();
			if (result) result->SetResult(json);
		}
		else if (funcName == "GetStoryList") {
			const GS::UniString json = TopoMeshHelper::GetStoryListJson();
			if (result) result->SetResult(json);
		}
		else if (funcName == "GetSampleElevationText") {
			const int layerIdx = param.ToInt();
			const GS::UniString sample = TopoMeshHelper::GetSampleElevationText(layerIdx);
			if (result) result->SetResult(sample);
		}
		else if (funcName == "CreateTopoMesh") {
			const bool ok = TopoMeshHelper::CreateTopoMesh(param);
			if (result) result->SetResult(ok ? "true" : "false");
		}
	}

private:
	DG::Browser& m_browser;
};

// -----------------------------------------------------------------------------
// Palette callback
// -----------------------------------------------------------------------------

static GSErrCode TopoMeshPaletteCallback(Int32 /*refCon*/, API_PaletteMessageID messageID, GS::IntPtr param)
{
	switch (messageID) {
	case APIPalMsg_OpenPalette:
		if (!TopoMeshPalette::HasInstance()) TopoMeshPalette::CreateInstance();
		TopoMeshPalette::GetInstance().Show();
		break;
	case APIPalMsg_ClosePalette:
		if (TopoMeshPalette::HasInstance()) TopoMeshPalette::GetInstance().Hide();
		break;
	case APIPalMsg_HidePalette_Begin:
		if (TopoMeshPalette::HasInstance() && TopoMeshPalette::GetInstance().IsVisible())
			TopoMeshPalette::GetInstance().Hide();
		break;
	case APIPalMsg_HidePalette_End:
		if (TopoMeshPalette::HasInstance() && !TopoMeshPalette::GetInstance().IsVisible())
			TopoMeshPalette::GetInstance().Show();
		break;
	case APIPalMsg_DisableItems_Begin:
		if (TopoMeshPalette::HasInstance() && TopoMeshPalette::GetInstance().IsVisible())
			TopoMeshPalette::GetInstance().DisableItems();
		break;
	case APIPalMsg_DisableItems_End:
		if (TopoMeshPalette::HasInstance() && TopoMeshPalette::GetInstance().IsVisible())
			TopoMeshPalette::GetInstance().EnableItems();
		break;
	case APIPalMsg_IsPaletteVisible:
		if (param != 0)
			*reinterpret_cast<bool*>(param) = TopoMeshPalette::HasInstance() && TopoMeshPalette::GetInstance().IsVisible();
		break;
	default:
		break;
	}
	return NoError;
}

// -----------------------------------------------------------------------------
// Static members
// -----------------------------------------------------------------------------

GS::Ref<TopoMeshPalette> TopoMeshPalette::s_instance(nullptr);
const GS::Guid TopoMeshPalette::s_guid("{a4f3c821-7d52-4e9b-b3f1-92c6d087e3a1}");

// -----------------------------------------------------------------------------
// Constructor / Destructor
// -----------------------------------------------------------------------------

TopoMeshPalette::TopoMeshPalette()
	: DG::Palette(ACAPI_GetOwnResModule(), TopoMeshPaletteResId, ACAPI_GetOwnResModule(), s_guid)
{
	m_browserCtrl = new DG::Browser(GetReference(), TopoMeshBrowserCtrlId);
	Attach(*this);
	BeginEventProcessing();
	Init();
}

TopoMeshPalette::~TopoMeshPalette()
{
	EndEventProcessing();
	delete m_browserCtrl;
	m_browserCtrl = nullptr;
}

void TopoMeshPalette::Init()
{
	LoadHtml();
	if (m_browserCtrl != nullptr) {
		static TopoMeshJSHandler handler(*m_browserCtrl);
		m_browserCtrl->SetJSResultHandler(&handler);
	}
}

void TopoMeshPalette::LoadHtml()
{
	if (m_browserCtrl == nullptr)
		return;
	const GS::UniString html = LoadTopoMeshHtml();
	m_browserCtrl->LoadHTML(html);
}

// -----------------------------------------------------------------------------
// Static methods
// -----------------------------------------------------------------------------

bool TopoMeshPalette::HasInstance()
{
	return s_instance != nullptr;
}

void TopoMeshPalette::CreateInstance()
{
	DBASSERT(!HasInstance());
	s_instance = new TopoMeshPalette();
	ACAPI_KeepInMemory(true);
}

TopoMeshPalette& TopoMeshPalette::GetInstance()
{
	DBASSERT(HasInstance());
	return *s_instance;
}

void TopoMeshPalette::DestroyInstance()
{
	s_instance = nullptr;
}

void TopoMeshPalette::ShowPalette()
{
	if (!HasInstance()) CreateInstance();
	GetInstance().Show();
}

void TopoMeshPalette::HidePalette()
{
	if (!HasInstance()) return;
	GetInstance().Hide();
}

GSErrCode TopoMeshPalette::RegisterPaletteControlCallBack()
{
	return ACAPI_RegisterModelessWindow(
		GS::CalculateHashValue(s_guid),
		TopoMeshPaletteCallback,
		API_PalEnabled_FloorPlan |
		API_PalEnabled_Section |
		API_PalEnabled_Elevation |
		API_PalEnabled_InteriorElevation |
		API_PalEnabled_3D |
		API_PalEnabled_Detail |
		API_PalEnabled_Worksheet |
		API_PalEnabled_Layout |
		API_PalEnabled_DocumentFrom3D,
		GSGuid2APIGuid(s_guid)
	);
}

void TopoMeshPalette::PanelResized(const DG::PanelResizeEvent& ev)
{
	BeginMoveResizeItems();
	if (m_browserCtrl != nullptr)
		m_browserCtrl->Resize(ev.GetHorizontalChange(), ev.GetVerticalChange());
	EndMoveResizeItems();
}

void TopoMeshPalette::PanelCloseRequested(const DG::PanelCloseRequestEvent&, bool* accepted)
{
	HidePalette();
	if (accepted != nullptr)
		*accepted = true;
}
