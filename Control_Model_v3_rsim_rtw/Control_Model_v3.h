/*
 * Control_Model_v3.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Control_Model_v3".
 *
 * Model version              : 1.16
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Mon Jun 04 21:06:25 2018
 *
 * Target selection: rsim.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Control_Model_v3_h_
#define RTW_HEADER_Control_Model_v3_h_
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef Control_Model_v3_COMMON_INCLUDES_
# define Control_Model_v3_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rsim.h"
#include "rt_logging.h"
#include "dt_info.h"
#endif                                 /* Control_Model_v3_COMMON_INCLUDES_ */

#include "Control_Model_v3_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#define MODEL_NAME                     Control_Model_v3
#define NSAMPLE_TIMES                  (2)                       /* Number of sample times */
#define NINPUTS                        (0)                       /* Number of model inputs */
#define NOUTPUTS                       (0)                       /* Number of model outputs */
#define NBLOCKIO                       (36)                      /* Number of data output port signals */
#define NUM_ZC_EVENTS                  (0)                       /* Number of zero-crossing events */
#ifndef NCSTATES
# define NCSTATES                      (9)                       /* Number of continuous states */
#elif NCSTATES != 9
# error Invalid specification of NCSTATES defined in compiler command
#endif

#ifndef rtmGetDataMapInfo
# define rtmGetDataMapInfo(rtm)        (NULL)
#endif

#ifndef rtmSetDataMapInfo
# define rtmSetDataMapInfo(rtm, val)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T Add13;                        /* '<Root>/Add13' */
  real_T Add12;                        /* '<Root>/Add12' */
  real_T Add11;                        /* '<Root>/Add11' */
  real_T Add14;                        /* '<Root>/Add14' */
  real_T Gain21;                       /* '<Root>/Gain21' */
  real_T TrigonometricFunction;        /* '<Root>/Trigonometric Function' */
  real_T Integrator7;                  /* '<Root>/Integrator7' */
  real_T Derivative2;                  /* '<Root>/Derivative2' */
  real_T Integrator8;                  /* '<Root>/Integrator8' */
  real_T Derivative3;                  /* '<Root>/Derivative3' */
  real_T TrigonometricFunction2;       /* '<Root>/Trigonometric Function2' */
  real_T Add8;                         /* '<Root>/Add8' */
  real_T Gain18;                       /* '<Root>/Gain18' */
  real_T Gain8;                        /* '<Root>/Gain8' */
  real_T Gain15;                       /* '<Root>/Gain15' */
  real_T Add17;                        /* '<Root>/Add17' */
  real_T Add15;                        /* '<Root>/Add15' */
  real_T TrigonometricFunction1;       /* '<Root>/Trigonometric Function1' */
  real_T Integrator4;                  /* '<Root>/Integrator4' */
  real_T Derivative;                   /* '<Root>/Derivative' */
  real_T MathFunction2;                /* '<Root>/Math Function2' */
  real_T TrigonometricFunction3;       /* '<Root>/Trigonometric Function3' */
  real_T Add3;                         /* '<Root>/Add3' */
  real_T Add6;                         /* '<Root>/Add6' */
  real_T Add18;                        /* '<Root>/Add18' */
  real_T Add16;                        /* '<Root>/Add16' */
  real_T Add19;                        /* '<Root>/Add19' */
  real_T Add5;                         /* '<Root>/Add5' */
  real_T Add2;                         /* '<Root>/Add2' */
  real_T Gain11;                       /* '<Root>/Gain11' */
  real_T Add4;                         /* '<Root>/Add4' */
  real_T Add1;                         /* '<Root>/Add1' */
  real_T Add;                          /* '<Root>/Add' */
  real_T x_a;                          /* '<Root>/MATLAB Function' */
  real_T y_a;                          /* '<Root>/MATLAB Function' */
  real_T z_a;                          /* '<Root>/MATLAB Function' */
} B;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T TimeStampA;                   /* '<Root>/Derivative2' */
  real_T LastUAtTimeA;                 /* '<Root>/Derivative2' */
  real_T TimeStampB;                   /* '<Root>/Derivative2' */
  real_T LastUAtTimeB;                 /* '<Root>/Derivative2' */
  real_T TimeStampA_b;                 /* '<Root>/Derivative3' */
  real_T LastUAtTimeA_k;               /* '<Root>/Derivative3' */
  real_T TimeStampB_d;                 /* '<Root>/Derivative3' */
  real_T LastUAtTimeB_n;               /* '<Root>/Derivative3' */
  real_T TimeStampA_a;                 /* '<Root>/Derivative1' */
  real_T LastUAtTimeA_i;               /* '<Root>/Derivative1' */
  real_T TimeStampB_dw;                /* '<Root>/Derivative1' */
  real_T LastUAtTimeB_g;               /* '<Root>/Derivative1' */
  real_T TimeStampA_f;                 /* '<Root>/Derivative' */
  real_T LastUAtTimeA_l;               /* '<Root>/Derivative' */
  real_T TimeStampB_dj;                /* '<Root>/Derivative' */
  real_T LastUAtTimeB_l;               /* '<Root>/Derivative' */
  struct {
    void *LoggedData;
  } ActualPosition_PWORK;              /* '<Root>/Actual Position' */

  struct {
    void *LoggedData;
  } CoordinateError_PWORK;             /* '<Root>/Coordinate Error' */

  struct {
    void *LoggedData;
  } CoordinateError1_PWORK;            /* '<Root>/Coordinate Error1' */

  struct {
    void *LoggedData;
  } q_1correction_PWORK;               /* '<Root>/q_1 correction' */

  struct {
    void *LoggedData;
  } q_1correction1_PWORK;              /* '<Root>/q_1 correction1' */

  struct {
    void *LoggedData;
  } q_1correction2_PWORK;              /* '<Root>/q_1 correction2' */

  struct {
    void *LoggedData;
  } q_1correction3_PWORK;              /* '<Root>/q_1 correction3' */

  struct {
    void *LoggedData;
  } q_1correction4_PWORK;              /* '<Root>/q_1 correction4' */

  struct {
    void *LoggedData;
  } q_1correction5_PWORK;              /* '<Root>/q_1 correction5' */

  struct {
    void *LoggedData;
  } q_2correction1_PWORK;              /* '<Root>/q_2 correction1' */

  struct {
    void *LoggedData;
  } q_3correction_PWORK;               /* '<Root>/q_3 correction' */

  struct {
    void *LoggedData;
  } q_4correction_PWORK;               /* '<Root>/q_4 correction' */

  struct {
    void *LoggedData;
  } q_5correction1_PWORK;              /* '<Root>/q_5 correction1' */

  struct {
    void *LoggedData;
  } q_6correction2_PWORK;              /* '<Root>/q_6 correction2' */
} DW;

/* Continuous states (auto storage) */
typedef struct {
  real_T Integrator5_CSTATE;           /* '<Root>/Integrator5' */
  real_T Integrator7_CSTATE;           /* '<Root>/Integrator7' */
  real_T Integrator8_CSTATE;           /* '<Root>/Integrator8' */
  real_T Integrator4_CSTATE;           /* '<Root>/Integrator4' */
  real_T Integrator6_CSTATE;           /* '<Root>/Integrator6' */
  real_T Integrator3_CSTATE;           /* '<Root>/Integrator3' */
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
} X;

/* State derivatives (auto storage) */
typedef struct {
  real_T Integrator5_CSTATE;           /* '<Root>/Integrator5' */
  real_T Integrator7_CSTATE;           /* '<Root>/Integrator7' */
  real_T Integrator8_CSTATE;           /* '<Root>/Integrator8' */
  real_T Integrator4_CSTATE;           /* '<Root>/Integrator4' */
  real_T Integrator6_CSTATE;           /* '<Root>/Integrator6' */
  real_T Integrator3_CSTATE;           /* '<Root>/Integrator3' */
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T Integrator5_CSTATE;        /* '<Root>/Integrator5' */
  boolean_T Integrator7_CSTATE;        /* '<Root>/Integrator7' */
  boolean_T Integrator8_CSTATE;        /* '<Root>/Integrator8' */
  boolean_T Integrator4_CSTATE;        /* '<Root>/Integrator4' */
  boolean_T Integrator6_CSTATE;        /* '<Root>/Integrator6' */
  boolean_T Integrator3_CSTATE;        /* '<Root>/Integrator3' */
  boolean_T Integrator_CSTATE;         /* '<Root>/Integrator' */
  boolean_T Integrator1_CSTATE;        /* '<Root>/Integrator1' */
  boolean_T Integrator2_CSTATE;        /* '<Root>/Integrator2' */
} XDis;

/* Parameters (auto storage) */
struct P_ {
  real_T Constant5_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant5'
                                        */
  real_T Constant1_Value;              /* Expression: 410
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Step_Time;                    /* Expression: 1
                                        * Referenced by: '<Root>/Step'
                                        */
  real_T Step_Y0;                      /* Expression: 0
                                        * Referenced by: '<Root>/Step'
                                        */
  real_T Step_YFinal;                  /* Expression: -60
                                        * Referenced by: '<Root>/Step'
                                        */
  real_T Constant3_Value;              /* Expression: 239.914064606
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T Step1_Time;                   /* Expression: 1
                                        * Referenced by: '<Root>/Step1'
                                        */
  real_T Step1_Y0;                     /* Expression: 0
                                        * Referenced by: '<Root>/Step1'
                                        */
  real_T Step1_YFinal;                 /* Expression: -50
                                        * Referenced by: '<Root>/Step1'
                                        */
  real_T Step2_Time;                   /* Expression: 1
                                        * Referenced by: '<Root>/Step2'
                                        */
  real_T Step2_Y0;                     /* Expression: 0
                                        * Referenced by: '<Root>/Step2'
                                        */
  real_T Step2_YFinal;                 /* Expression: 0
                                        * Referenced by: '<Root>/Step2'
                                        */
  real_T Step3_Time;                   /* Expression: 1
                                        * Referenced by: '<Root>/Step3'
                                        */
  real_T Step3_Y0;                     /* Expression: 0
                                        * Referenced by: '<Root>/Step3'
                                        */
  real_T Step3_YFinal;                 /* Expression: 100
                                        * Referenced by: '<Root>/Step3'
                                        */
  real_T Constant4_Value;              /* Expression: 5.657518
                                        * Referenced by: '<Root>/Constant4'
                                        */
  real_T Integrator5_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator5'
                                        */
  real_T Gain21_Gain;                  /* Expression: 0.8
                                        * Referenced by: '<Root>/Gain21'
                                        */
  real_T Integrator7_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator7'
                                        */
  real_T Integrator8_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator8'
                                        */
  real_T Gain18_Gain;                  /* Expression: 1
                                        * Referenced by: '<Root>/Gain18'
                                        */
  real_T Gain8_Gain;                   /* Expression: 0.002
                                        * Referenced by: '<Root>/Gain8'
                                        */
  real_T Gain15_Gain;                  /* Expression: 0
                                        * Referenced by: '<Root>/Gain15'
                                        */
  real_T Constant2_Value;              /* Expression: 2*pi
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T Constant6_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T Gain9_Gain;                   /* Expression: 0
                                        * Referenced by: '<Root>/Gain9'
                                        */
  real_T Integrator4_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator4'
                                        */
  real_T Gain13_Gain;                  /* Expression: 1
                                        * Referenced by: '<Root>/Gain13'
                                        */
  real_T Gain7_Gain;                   /* Expression: 0.9
                                        * Referenced by: '<Root>/Gain7'
                                        */
  real_T Integrator6_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator6'
                                        */
  real_T Gain17_Gain;                  /* Expression: 0.1
                                        * Referenced by: '<Root>/Gain17'
                                        */
  real_T Constant7_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant7'
                                        */
  real_T Gain6_Gain;                   /* Expression: 0.7
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real_T Integrator3_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator3'
                                        */
  real_T Gain16_Gain;                  /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain16'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<Root>/Integrator'
                                        */
  real_T Gain_Gain;                    /* Expression: 0.02
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Gain12_Gain;                  /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain12'
                                        */
  real_T Gain14_Gain;                  /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain14'
                                        */
  real_T Gain5_Gain;                   /* Expression: 0.1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T Integrator1_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator1'
                                        */
  real_T Gain1_Gain;                   /* Expression: 0
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Gain10_Gain;                  /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain10'
                                        */
  real_T Gain11_Gain;                  /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain11'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T Integrator2_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Integrator2'
                                        */
  real_T Gain2_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T Gain3_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<Root>/Gain3'
                                        */
};

extern P rtP;                          /* parameters */

/* External data declarations for dependent source files */
extern const char *RT_MEMORY_ALLOCATION_ERROR;
extern B rtB;                          /* block i/o */
extern X rtX;                          /* states (continuous) */
extern DW rtDW;                        /* states (dwork) */

/* Simulation Structure */
extern SimStruct *const rtS;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Control_Model_v3'
 * '<S1>'   : 'Control_Model_v3/MATLAB Function'
 * '<S2>'   : 'Control_Model_v3/MATLAB Function1'
 */

/* user code (bottom of header file) */
extern const int_T gblNumToFiles;
extern const int_T gblNumFrFiles;
extern const int_T gblNumFrWksBlocks;
extern rtInportTUtable *gblInportTUtables;
extern const char *gblInportFileName;
extern const int_T gblNumRootInportBlks;
extern const int_T gblNumModelInputs;
extern const int_T gblInportDataTypeIdx[];
extern const int_T gblInportDims[];
extern const int_T gblInportComplex[];
extern const int_T gblInportInterpoFlag[];
extern const int_T gblInportContinuous[];

#endif                                 /* RTW_HEADER_Control_Model_v3_h_ */
