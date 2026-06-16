#include "ne_std.h"
#include "pm_default_allocator.h"
#include "ssc_dae.h"
#include "sm_ssci_NeDaePrivateData.h"
NeDae * sm_ssci_constructDae ( NeDaePrivateData * smData ) ; void
FullAssembly_system_fad601a9_49_NeDaePrivateData_create ( NeDaePrivateData *
smData ) ; void FullAssembly_system_fad601a9_49_dae ( NeDae * * dae , const
NeModelParameters * modelParams , const NeSolverParameters * solverParams ) {
PmAllocator * alloc = pm_default_allocator ( ) ; NeDaePrivateData * smData =
( NeDaePrivateData * ) alloc -> mCallocFcn ( alloc , sizeof ( NeDaePrivateData
) , 1 ) ; ( void ) modelParams ; ( void ) solverParams ;
FullAssembly_system_fad601a9_49_NeDaePrivateData_create ( smData ) ; * dae =
sm_ssci_constructDae ( smData ) ; }
