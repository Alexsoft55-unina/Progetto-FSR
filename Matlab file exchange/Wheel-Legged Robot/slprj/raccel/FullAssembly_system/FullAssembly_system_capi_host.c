#include "FullAssembly_system_capi_host.h"
static FullAssembly_system_host_DataMapInfo_T root;
static int initialized = 0;
__declspec( dllexport ) rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        FullAssembly_system_host_InitializeDataMapInfo(&(root), "FullAssembly_system");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction(){return(getRootMappingInfo());}
