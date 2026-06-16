#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "pm_default_allocator.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ssc_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "FullAssembly_system_fad601a9_49_geometries.h"
PmfMessageId FullAssembly_system_fad601a9_49_checkDynamics ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const double *
input , const double * inputDot , const double * inputDdot , const double *
discreteState , const int * modeVector , double * errorResult ,
NeuDiagnosticManager * neDiagMgr ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) input ; ( void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void ) modeVector ; ( void ) neDiagMgr ; errorResult [ 0 ] = 0.0 ; return NULL ; }
