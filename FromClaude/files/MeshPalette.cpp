#include "MeshPalette.hpp"

#include "APIEnvir.h"
#include "ACAPinc.h"

#include "DGBrowser.hpp"
#include "BrowserRepl.hpp"

static GS::UniString LoadMeshHtml()
{
	GS::UniString html;
	GSHandle data = RSLoadResource('DATA', ACAPI_GetOwnResModule(), MeshHtmlResId);
	if (data != nullptr) {
		const GSSize size = BMhGetSize(data);
		html.Append(*data, size);
		BMhKill(&data);
	} else {
		html = "<html><body style='font-family:Arial;color:#900;padding:12px;'>"
			"Не удалось загрузить HTML ресурса сетки."
			"</body></html>";
	}
	return html;
}

static GSErrCode MeshPaletteCallback(Int32 /*refCon*/, API_PaletteMessageID messageID, GS::IntPtr param)
{
	switch (messageID) {
	case APIPalMsg_OpenPalette:
		if (!MeshPalette::HasInstance()) MeshPalette::CreateInstance();
		MeshPalette::GetInstance().Show();
		break;

	case APIPalMsg_ClosePalette:
		if (MeshPalette::HasInstance()) MeshPalette::GetInstance().Hide();
		break;

	case APIPalMsg_HidePalette_Begin:
		if (MeshPalette::HasInstance() && MeshPalette::GetInstance().IsVisible())
			MeshPalette::GetInstance().Hide();
		break;

	case APIPalMsg_HidePalette_End:
		if (MeshPalette::HasInstance() && !MeshPalette::GetInstance().IsVisible())
			MeshPalette::GetInstance().Show();
		break;

	case APIPalMsg_DisableItems_Begin:
		if (MeshPalette::HasInstance() && MeshPalette::GetInstance().IsVisible())
			MeshPalette::GetInstance().DisableItems();
		break;

	case APIPalMsg_DisableItems_End:
		if (MeshPalette::HasInstance() && MeshPalette::GetInstance().IsVisible())
			MeshPalette::GetInstance().EnableItems();
		break;

	case APIPalMsg_IsPaletteVisible:
		if (param != 0)
			*reinterpret_cast<bool*>(param) = MeshPalette::HasInstance() && MeshPalette::GetInstance().IsVisible();
		break;

	default:
		break;
	}
	return NoError;
}

GS::Ref<MeshPalette> MeshPalette::s_instance(nullptr);
const GS::Guid MeshPalette::s_guid("{3b2ad979-26c9-4b81-842f-58c8829ae8e5}");

MeshPalette::MeshPalette()
	: DG::Palette(ACAPI_GetOwnResModule(), MeshPaletteResId, ACAPI_GetOwnResModule(), s_guid)
{
	m_browserCtrl = new DG::Browser(GetReference(), MeshBrowserCtrlId);
	Attach(*this);
	BeginEventProcessing();
	Init();
}

MeshPalette::~MeshPalette()
{
	EndEventProcessing();
	delete m_browserCtrl;
	m_browserCtrl = nullptr;
}

void MeshPalette::Init()
{
	LoadHtml();
	if (m_browserCtrl != nullptr)
		BrowserRepl::RegisterACAPIJavaScriptObject(*m_browserCtrl);
}

void MeshPalette::LoadHtml()
{
	if (m_browserCtrl == nullptr)
		return;

	const GS::UniString html = LoadMeshHtml();
	m_browserCtrl->LoadHTML(html);
}

bool MeshPalette::HasInstance()
{
	return s_instance != nullptr;
}

void MeshPalette::CreateInstance()
{
	DBASSERT(!HasInstance());
	s_instance = new MeshPalette();
	ACAPI_KeepInMemory(true);
}

MeshPalette& MeshPalette::GetInstance()
{
	DBASSERT(HasInstance());
	return *s_instance;
}

void MeshPalette::DestroyInstance()
{
	s_instance = nullptr;
}

void MeshPalette::ShowPalette()
{
	if (!HasInstance())
		CreateInstance();

	GetInstance().Show();
}

void MeshPalette::HidePalette()
{
	if (!HasInstance())
		return;

	GetInstance().Hide();
}

GSErrCode MeshPalette::RegisterPaletteControlCallBack()
{
	return ACAPI_RegisterModelessWindow(
		GS::CalculateHashValue(s_guid),
		MeshPaletteCallback,
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

void MeshPalette::PanelResized(const DG::PanelResizeEvent& ev)
{
	BeginMoveResizeItems();
	if (m_browserCtrl != nullptr)
		m_browserCtrl->Resize(ev.GetHorizontalChange(), ev.GetVerticalChange());
	EndMoveResizeItems();
}

void MeshPalette::PanelCloseRequested(const DG::PanelCloseRequestEvent&, bool* accepted)
{
	HidePalette();
	if (accepted != nullptr)
		*accepted = true;
}






