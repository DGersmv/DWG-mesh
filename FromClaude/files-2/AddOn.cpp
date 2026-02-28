// *****************************************************************************
// DWG Topo Mesh Add-On
// *****************************************************************************

#include "APIEnvir.h"
#include "ACAPinc.h"
#include "APICommon.h"
#include "ResourceIDs.hpp"
#include "TopoMeshPalette.hpp"

// -----------------------------------------------------------------------------
// MenuCommandHandler
// -----------------------------------------------------------------------------

GSErrCode MenuCommandHandler (const API_MenuParams* menuParams)
{
	switch (menuParams->menuItemRef.menuResID) {
		case TopoMeshMenuResId:
			switch (menuParams->menuItemRef.itemIndex) {
				case 1:  // "Топо Mesh из DWG"
					TopoMeshPalette::ShowPalette();
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	return NoError;
}

// -----------------------------------------------------------------------------
// CheckEnvironment
// -----------------------------------------------------------------------------

API_AddonType CheckEnvironment (API_EnvirParams* envir)
{
	RSGetIndString (&envir->addOnInfo.name,        32000, 1, ACAPI_GetOwnResModule());
	RSGetIndString (&envir->addOnInfo.description, 32000, 2, ACAPI_GetOwnResModule());
	return APIAddon_Preload;
}

// -----------------------------------------------------------------------------
// RegisterInterface
// -----------------------------------------------------------------------------

GSErrCode RegisterInterface (void)
{
	return ACAPI_MenuItem_RegisterMenu (TopoMeshMenuResId, 0, MenuCode_UserDef, MenuFlag_Default);
}

// -----------------------------------------------------------------------------
// Initialize
// -----------------------------------------------------------------------------

GSErrCode Initialize ()
{
	GSErrCode err = ACAPI_MenuItem_InstallMenuHandler (TopoMeshMenuResId, MenuCommandHandler);
	if (err != NoError)
		return err;

	err = TopoMeshPalette::RegisterPaletteControlCallBack();
	return err;
}

// -----------------------------------------------------------------------------
// FreeData
// -----------------------------------------------------------------------------

GSErrCode FreeData (void)
{
	return NoError;
}
