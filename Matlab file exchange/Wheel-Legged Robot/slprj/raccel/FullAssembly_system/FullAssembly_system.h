#ifndef FullAssembly_system_h_
#define FullAssembly_system_h_
#ifndef FullAssembly_system_COMMON_INCLUDES_
#define FullAssembly_system_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "dt_info.h"
#include "ext_work.h"
#include "nesl_rtw.h"
#include "FullAssembly_system_fad601a9_1_gateway.h"
#endif
#include "FullAssembly_system_types.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#include "rtGetInf.h"
#define MODEL_NAME FullAssembly_system
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (70) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (236)   
#elif NCSTATES != 236
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T mby2gg4jgt ; real_T difagtosdt ; real_T os0ebvnexy ;
real_T gzkmrjqeel ; real_T djrrpvjwx5 ; real_T dwo1yb3eu3 ; real_T cerdp32tn4
; real_T hhzs1ndn4k ; real_T foqywexyz5 ; real_T medr5m5ecu ; real_T
lpppkq0mo3 ; real_T ds3xmzccwx ; real_T dp4ex0qtp1 ; real_T c2ihsbt0dj ;
real_T lxgozgn0yl ; real_T okw4xf2pdu ; real_T eiselq1fmm ; real_T go3oa1beoa
; real_T p4hgdattf2 ; real_T gmhbg2apiv ; real_T chisoov5k4 ; real_T
gs0sxl0fxr [ 219 ] ; real_T gkjugtc5ui [ 5 ] ; real_T armhtwdr0g ; real_T
emfbzb4tgv ; real_T h4sb2hdvqs ; real_T ffqrfknnoz ; real_T fyqu4fz511 ;
real_T mrf3exm2xd ; real_T eefa3vsr3u ; real_T hi3rqi5vyt ; real_T oieixnysf1
; real_T k2g2xqija3 ; real_T misakfid0j ; real_T gb4x0jfanf ; real_T
plhwhurr2l ; real_T fajvs1ugem ; real_T gzsfvwfyan ; real_T hir5pemzzc ;
real_T azhyg2i4qg ; real_T n3iwvgj15u ; real_T gq4e4lo25z ; real_T lj35kknfkm
; real_T je41jdmpg2 ; real_T lrgijcn3ft ; real_T cqo0fmq5gh ; real_T
htbabqfbrr ; real_T a1lmlx15bv ; real_T hzfqkzjqra ; real_T kceny33cji ;
real_T ez0zrbtqsy ; real_T l125fp4ibr ; real_T erhydliqya ; real_T jalptojx2m
; real_T bdilohfvdz ; real_T k42gmmqwyh ; real_T jxfwvbegog ; real_T
owj31uuwct ; real_T cb4gonmskq ; real_T nvwjv0tfyk ; real_T p1if2uccjq ;
real_T ntjvp1exme ; real_T fnp1kweqe2 ; real_T k5kn2vruzi [ 4 ] ; real_T
a0k5xyl25h [ 4 ] ; real_T ppu3j4e43j [ 4 ] ; real_T mngdq0v4mv [ 4 ] ; real_T
h0nwpxh25t [ 4 ] ; real_T h3tfpg3121 [ 4 ] ; real_T kobb0qacwc ; } B ;
typedef struct { real_T ef0tjfqxq2 [ 2 ] ; real_T gsh0bhwzck ; real_T
f1tzv3yf52 ; real_T o4fsuetuds [ 2 ] ; real_T ahtltjzp4f [ 2 ] ; real_T
l4plqwgfvd [ 2 ] ; real_T ldzxxir0ou ; real_T ngjaietdzr ; real_T abeyvkp1wu
; real_T pn4f1iubcb ; real_T nydhqhpgj2 ; real_T i4igjngorc ; struct { void *
LoggedData [ 2 ] ; } lsl5oegdau ; void * kke31lspbs ; void * adc1dhx1xv ;
void * hwyzdtvule ; void * fylriinzod ; void * bdfbjodek0 ; void * ipybrgxmsj
; void * gf3dofpjfq ; void * fijtoipbrp ; void * lazkvdq4xy ; void *
gajsjtep3g ; struct { void * LoggedData [ 2 ] ; } fodkyk4o3c ; struct { void
* LoggedData ; } b2qcbxfqhv ; struct { void * LoggedData ; } j40xjawgtd ;
struct { void * LoggedData [ 2 ] ; } bsizlxhy4g ; struct { void * LoggedData
; } if00eqe43y ; struct { void * LoggedData ; } m5agigng2p ; void *
nvkibpxgyb ; void * f02lveu5sb ; void * g5ujc20rct ; void * pvocqr40ve ; void
* okvpaob3h5 ; int_T epi4mp50fu ; int_T ojhughkrf1 ; int_T etq5dvu3fk ; int_T
d5wsirya45 ; int_T akbpgtx52o ; int_T gh4k1smub5 ; int_T k1k2vpdtfo ; uint8_T
jbiypwbkax ; uint8_T ef4vp4u1ze ; uint8_T gagbx1bw4u ; uint8_T cdpmqpsy10 ;
boolean_T ptfxlysanb ; boolean_T cp44xoo0qb ; } DW ; typedef struct { real_T
bgrtwrx4hx ; real_T e4dlx03sxo ; real_T h2rt105hjp ; real_T dcut2ftvji ;
real_T lvr5gioqil ; real_T kbshnqchrr [ 219 ] ; real_T f4tk55yhcc ; real_T
dfu4ibp04g ; real_T ac4fvpgi4l ; real_T pteymlppyj ; real_T hp4xrbjgrr ;
real_T cnlehkbxn1 ; real_T ldru5gthfz ; real_T f2hypuphw5 ; real_T gwjbrtbmwu
[ 2 ] ; real_T asaix2dvam [ 2 ] ; } X ; typedef struct { real_T bgrtwrx4hx ;
real_T e4dlx03sxo ; real_T h2rt105hjp ; real_T dcut2ftvji ; real_T lvr5gioqil
; real_T kbshnqchrr [ 219 ] ; real_T f4tk55yhcc ; real_T dfu4ibp04g ; real_T
ac4fvpgi4l ; real_T pteymlppyj ; real_T hp4xrbjgrr ; real_T cnlehkbxn1 ;
real_T ldru5gthfz ; real_T f2hypuphw5 ; real_T gwjbrtbmwu [ 2 ] ; real_T
asaix2dvam [ 2 ] ; } XDot ; typedef struct { boolean_T bgrtwrx4hx ; boolean_T
e4dlx03sxo ; boolean_T h2rt105hjp ; boolean_T dcut2ftvji ; boolean_T
lvr5gioqil ; boolean_T kbshnqchrr [ 219 ] ; boolean_T f4tk55yhcc ; boolean_T
dfu4ibp04g ; boolean_T ac4fvpgi4l ; boolean_T pteymlppyj ; boolean_T
hp4xrbjgrr ; boolean_T cnlehkbxn1 ; boolean_T ldru5gthfz ; boolean_T
f2hypuphw5 ; boolean_T gwjbrtbmwu [ 2 ] ; boolean_T asaix2dvam [ 2 ] ; } XDis
; typedef struct { real_T bgrtwrx4hx ; real_T e4dlx03sxo ; real_T h2rt105hjp
; real_T dcut2ftvji ; real_T lvr5gioqil ; real_T kbshnqchrr [ 219 ] ; real_T
f4tk55yhcc ; real_T dfu4ibp04g ; real_T ac4fvpgi4l ; real_T pteymlppyj ;
real_T hp4xrbjgrr ; real_T cnlehkbxn1 ; real_T ldru5gthfz ; real_T f2hypuphw5
; real_T gwjbrtbmwu [ 2 ] ; real_T asaix2dvam [ 2 ] ; } CStateAbsTol ;
typedef struct { real_T bgrtwrx4hx ; real_T e4dlx03sxo ; real_T h2rt105hjp ;
real_T dcut2ftvji ; real_T lvr5gioqil ; real_T kbshnqchrr [ 219 ] ; real_T
f4tk55yhcc ; real_T dfu4ibp04g ; real_T ac4fvpgi4l ; real_T pteymlppyj ;
real_T hp4xrbjgrr ; real_T cnlehkbxn1 ; real_T ldru5gthfz ; real_T f2hypuphw5
; real_T gwjbrtbmwu [ 2 ] ; real_T asaix2dvam [ 2 ] ; } CXPtMin ; typedef
struct { real_T bgrtwrx4hx ; real_T e4dlx03sxo ; real_T h2rt105hjp ; real_T
dcut2ftvji ; real_T lvr5gioqil ; real_T kbshnqchrr [ 219 ] ; real_T
f4tk55yhcc ; real_T dfu4ibp04g ; real_T ac4fvpgi4l ; real_T pteymlppyj ;
real_T hp4xrbjgrr ; real_T cnlehkbxn1 ; real_T ldru5gthfz ; real_T f2hypuphw5
; real_T gwjbrtbmwu [ 2 ] ; real_T asaix2dvam [ 2 ] ; } CXPtMax ; typedef
struct { real_T anzbvibkwh ; real_T falza2lokh ; real_T ej2pasazsi ; real_T
fduahvae5o ; real_T itzqsvq0tl ; } ZCV ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T
PIDController1_D ; real_T PIDController_D ; real_T PIDController_D_pgipdvl2t0
; real_T PIDController_D_cmpawloux5 ; real_T PIDController_D_bbj5cvi5gf ;
real_T PIDController_D_mgncsilnse ; real_T PIDController_I ; real_T
PIDController1_I ; real_T PIDController_I_f1l2iwf344 ; real_T
PIDController_I_hz4g5v32qr ; real_T PIDController_I_ijsojcoxkl ; real_T
PIDController_I_ay5xqbhbz1 ; real_T PIDController1_InitialConditionForFilter
; real_T PIDController_InitialConditionForFilter ; real_T
PIDController_InitialConditionForFilter_bjzek0osia ; real_T
PIDController_InitialConditionForFilter_aks1ksah31 ; real_T
PIDController_InitialConditionForFilter_bojlusop1m ; real_T
PIDController_InitialConditionForFilter_nwa4cn4h0f ; real_T
PIDController1_InitialConditionForIntegrator ; real_T
PIDController_InitialConditionForIntegrator ; real_T
PIDController_InitialConditionForIntegrator_iwhwqfir5i ; real_T
PIDController_InitialConditionForIntegrator_cmdkqikqzm ; real_T
PIDController_InitialConditionForIntegrator_n5qjw4tqvb ; real_T
PIDController_InitialConditionForIntegrator_e3x5mj5zgc ; real_T
PIDController1_N ; real_T PIDController_N ; real_T PIDController_N_crc4kkzmcw
; real_T PIDController_N_du3ycvz5e5 ; real_T PIDController_N_fdn2lygism ;
real_T PIDController_N_o5l1fyg2f1 ; real_T PIDController1_P ; real_T
PIDController_P ; real_T PIDController_P_crkbcfm51g ; real_T
PIDController_P_eek150r3nk ; real_T PIDController_P_cqjd4trxgu ; real_T
PIDController_P_mcr05tm55t ; real_T DesiredAngleSetpoint_Time ; real_T
DesiredAngleSetpoint_Y0 ; real_T DesiredAngleSetpoint_YFinal ; real_T
Integrator_IC ; real_T Gain_Gain ; real_T Step_Time ; real_T Step_Y0 ; real_T
Step_YFinal ; real_T Step_Time_ccmcys4jti ; real_T Step_Y0_kuxnkwajmq ;
real_T Step_YFinal_mqgxnbnv5v ; real_T Step_Time_leph3y11xg ; real_T
Step_Y0_lect43vy12 ; real_T Step_YFinal_comjertxag ; real_T
Step_Time_mcpuvzrbmb ; real_T Step_Y0_bp5qi5uzqe ; real_T
Step_YFinal_gjnpk0skzo ; real_T DEG_Value ; real_T Gain1_Gain ; } ; extern
const char_T * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ;
extern DW rtDW ; extern P rtP ; extern mxArray *
mr_FullAssembly_system_GetDWork ( ) ; extern void
mr_FullAssembly_system_SetDWork ( const mxArray * ssDW ) ; extern mxArray *
mr_FullAssembly_system_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * FullAssembly_system_GetCAPIStaticMap ( void
) ; extern SimStruct * const rtS ; extern DataMapInfo * rt_dataMapInfoPtr ;
extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs ( int_T
tid ) ; void MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#endif
