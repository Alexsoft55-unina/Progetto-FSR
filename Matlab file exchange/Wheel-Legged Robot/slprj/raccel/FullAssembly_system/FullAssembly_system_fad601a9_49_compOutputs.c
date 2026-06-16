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
PmfMessageId FullAssembly_system_fad601a9_49_compOutputs ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * input , const double * inputDot , const double *
inputDdot , const double * discreteState , double * output ,
NeuDiagnosticManager * neDiagMgr ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 39 ] ;
( void ) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) input ; ( void
) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void ) neDiagMgr
; xx [ 0 ] = 0.3044173159292037 ; xx [ 1 ] = 0.6382241751629747 ; xx [ 2 ] =
xx [ 0 ] ; xx [ 3 ] = xx [ 1 ] ; xx [ 4 ] = - xx [ 0 ] ; xx [ 5 ] = - xx [ 1
] ; xx [ 0 ] = 3.509618054673539e-3 ; xx [ 1 ] = 0.9999938412715902 ; xx [ 6
] = xx [ 0 ] * state [ 4 ] - xx [ 1 ] * state [ 3 ] ; xx [ 7 ] = - ( xx [ 0 ]
* state [ 3 ] + xx [ 1 ] * state [ 4 ] ) ; xx [ 8 ] = - ( xx [ 0 ] * state [
6 ] + xx [ 1 ] * state [ 5 ] ) ; xx [ 9 ] = - ( xx [ 1 ] * state [ 6 ] - xx [
0 ] * state [ 5 ] ) ; pm_math_Quaternion_compose_ra ( xx + 2 , xx + 6 , xx +
10 ) ; xx [ 2 ] = 0.9942537100240552 ; xx [ 3 ] = 0.1070493325745648 ; xx [ 4
] = 2.129171098288088e-5 ; xx [ 5 ] = 6.738364049671423e-6 ; xx [ 6 ] = 0.5 ;
xx [ 7 ] = xx [ 6 ] * state [ 13 ] ; xx [ 8 ] = 4.076899370023929e-5 ; xx [ 9
] = sin ( xx [ 7 ] ) ; xx [ 14 ] = 0.2197214676051225 ; xx [ 15 ] =
0.9755626453546385 ; xx [ 16 ] = cos ( xx [ 7 ] ) ; xx [ 17 ] = xx [ 8 ] * xx
[ 9 ] ; xx [ 18 ] = - ( xx [ 14 ] * xx [ 9 ] ) ; xx [ 19 ] = - ( xx [ 15 ] *
xx [ 9 ] ) ; pm_math_Quaternion_compose_ra ( xx + 2 , xx + 16 , xx + 20 ) ;
pm_math_Quaternion_compose_ra ( xx + 10 , xx + 20 , xx + 2 ) ; xx [ 7 ] =
0.08064329797072847 ; xx [ 9 ] = 7.019192879778114e-3 ; xx [ 10 ] = xx [ 6 ]
* state [ 31 ] ; xx [ 11 ] = sin ( xx [ 10 ] ) ; xx [ 12 ] = xx [ 9 ] * xx [
11 ] ; xx [ 13 ] = 0.7024996659833728 ; xx [ 16 ] = 0.9999753651622205 ; xx [
17 ] = xx [ 16 ] * xx [ 11 ] ; xx [ 11 ] = 0.7030386702748058 ; xx [ 18 ] =
cos ( xx [ 10 ] ) ; xx [ 10 ] = 0.07568028728728934 ; xx [ 24 ] = xx [ 7 ] *
xx [ 12 ] + xx [ 13 ] * xx [ 17 ] - xx [ 11 ] * xx [ 18 ] ; xx [ 25 ] = xx [
10 ] * xx [ 18 ] - xx [ 7 ] * xx [ 17 ] + xx [ 13 ] * xx [ 12 ] ; xx [ 26 ] =
xx [ 11 ] * xx [ 12 ] + xx [ 7 ] * xx [ 18 ] + xx [ 10 ] * xx [ 17 ] ; xx [
27 ] = xx [ 11 ] * xx [ 17 ] + xx [ 13 ] * xx [ 18 ] - xx [ 10 ] * xx [ 12 ]
; pm_math_Quaternion_compose_ra ( xx + 2 , xx + 24 , xx + 10 ) ; xx [ 2 ] = -
0.4966591793468652 ; xx [ 3 ] = 0.5033240014880911 ; xx [ 4 ] = -
0.5035731579861172 ; xx [ 5 ] = - 0.4963956926207194 ; xx [ 7 ] = xx [ 6 ] *
state [ 35 ] ; xx [ 6 ] = 0.999976722339301 ; xx [ 17 ] = sin ( xx [ 7 ] ) ;
xx [ 18 ] = 1.427996225678019e-5 ; xx [ 19 ] = 6.823091354501987e-3 ; xx [ 28
] = cos ( xx [ 7 ] ) ; xx [ 29 ] = xx [ 6 ] * xx [ 17 ] ; xx [ 30 ] = xx [ 18
] * xx [ 17 ] ; xx [ 31 ] = xx [ 19 ] * xx [ 17 ] ;
pm_math_Quaternion_compose_ra ( xx + 2 , xx + 28 , xx + 32 ) ;
pm_math_Quaternion_compose_ra ( xx + 10 , xx + 32 , xx + 2 ) ; xx [ 10 ] = -
0.9999941477112748 ; xx [ 11 ] = 2.563451802845173e-4 ; xx [ 12 ] =
3.411567248852681e-3 ; xx [ 13 ] = - 6.265478974447751e-6 ;
pm_math_Quaternion_compose_ra ( xx + 2 , xx + 10 , xx + 28 ) ; xx [ 2 ] = xx
[ 0 ] * state [ 11 ] ; xx [ 3 ] = xx [ 0 ] * state [ 12 ] ; xx [ 4 ] = 2.0 ;
xx [ 36 ] = state [ 10 ] ; xx [ 37 ] = state [ 11 ] - ( xx [ 0 ] * xx [ 2 ] -
xx [ 1 ] * xx [ 3 ] ) * xx [ 4 ] ; xx [ 38 ] = state [ 12 ] - xx [ 4 ] * ( xx
[ 1 ] * xx [ 2 ] + xx [ 0 ] * xx [ 3 ] ) ; pm_math_Quaternion_inverseXform_ra
( xx + 20 , xx + 36 , xx + 0 ) ; xx [ 3 ] = xx [ 0 ] + xx [ 8 ] * state [ 14
] ; xx [ 4 ] = xx [ 1 ] - xx [ 14 ] * state [ 14 ] ; xx [ 5 ] = xx [ 2 ] - xx
[ 15 ] * state [ 14 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 24 , xx + 3
, xx + 0 ) ; xx [ 3 ] = xx [ 0 ] ; xx [ 4 ] = xx [ 1 ] - xx [ 9 ] * state [
33 ] ; xx [ 5 ] = xx [ 2 ] - xx [ 16 ] * state [ 33 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 32 , xx + 3 , xx + 0 ) ; xx [ 3 ] =
xx [ 0 ] + xx [ 6 ] * state [ 36 ] ; xx [ 4 ] = xx [ 1 ] + xx [ 18 ] * state
[ 36 ] ; xx [ 5 ] = xx [ 2 ] + xx [ 19 ] * state [ 36 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 10 , xx + 3 , xx + 0 ) ;
pm_math_Quaternion_xform_ra ( xx + 28 , xx + 0 , xx + 3 ) ; output [ 0 ] =
state [ 63 ] ; output [ 1 ] = state [ 93 ] ; output [ 2 ] = state [ 35 ] ;
output [ 3 ] = state [ 13 ] ; output [ 4 ] = xx [ 3 ] ; return NULL ; }
