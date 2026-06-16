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
const NeZCData * FullAssembly_system_fad601a9_49_ZCData = NULL ; PmfMessageId
FullAssembly_system_fad601a9_49_computeAsmModeVector ( const double * input ,
const double * inputDot , const double * inputDdot , int * modeVector ,
double * errorResult , NeuDiagnosticManager * neDiagMgr ) { ( void ) input ;
( void ) inputDot ; ( void ) inputDdot ; ( void ) modeVector ; ( void )
neDiagMgr ; errorResult [ 0 ] = 0.0 ; return NULL ; } PmfMessageId
FullAssembly_system_fad601a9_49_computeSimModeVector ( const double * input ,
const double * inputDot , const double * inputDdot , int * modeVector ,
double * errorResult , NeuDiagnosticManager * neDiagMgr ) { ( void ) input ;
( void ) inputDot ; ( void ) inputDdot ; ( void ) modeVector ; ( void )
neDiagMgr ; errorResult [ 0 ] = 0.0 ; return NULL ; } PmfMessageId
FullAssembly_system_fad601a9_49_computeZeroCrossings ( const
RuntimeDerivedValuesBundle * rtdv , const double * solverStateVector , const
double * input , const double * inputDot , const double * inputDdot , const
double * discreteStateVector , double * zeroCrossingsVector , double *
errorResult , NeuDiagnosticManager * neDiagMgr ) { const double * rtdvd =
rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) solverStateVector ; ( void ) input ; ( void ) inputDot ; ( void ) inputDdot ; ( void ) discreteStateVector ; ( void ) zeroCrossingsVector ; ( void ) neDiagMgr ; errorResult [ 0 ] = 0.0 ; return NULL ; }
