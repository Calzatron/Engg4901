/*
 * Control_Model_v3.c
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

#include <math.h>
#include "Control_Model_v3.h"
#include "Control_Model_v3_private.h"
#include "Control_Model_v3_dt.h"

/* user code (top of parameter file) */
const int_T gblNumToFiles = 0;
const int_T gblNumFrFiles = 0;
const int_T gblNumFrWksBlocks = 0;
const char *gblSlvrJacPatternFileName =
  "Control_Model_v3_rsim_rtw\\Control_Model_v3_Jpattern.mat";

/* Root inports information  */
const int_T gblNumRootInportBlks = 0;
const int_T gblNumModelInputs = 0;
extern rtInportTUtable *gblInportTUtables;
extern const char *gblInportFileName;
const int_T gblInportDataTypeIdx[] = { -1 };

const int_T gblInportDims[] = { -1 } ;

const int_T gblInportComplex[] = { -1 };

const int_T gblInportInterpoFlag[] = { -1 };

const int_T gblInportContinuous[] = { -1 };

#include "simstruc.h"
#include "fixedpoint.h"

/* Block signals (auto storage) */
B rtB;

/* Continuous states */
X rtX;

/* Block states (auto storage) */
DW rtDW;

/* Parent Simstruct */
static SimStruct model_S;
SimStruct *const rtS = &model_S;
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  if (u1 == 0.0) {
    y = u0;
  } else if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf
                (u1))))) {
    y = (rtNaN);
  } else {
    tmp = u0 / u1;
    if (u1 <= floor(u1)) {
      y = u0 - floor(tmp) * u1;
    } else if (fabs(tmp - rt_roundd_snf(tmp)) <= DBL_EPSILON * fabs(tmp)) {
      y = 0.0;
    } else {
      y = (tmp - floor(tmp)) * u1;
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T a;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

/* Initial conditions for root system: '<Root>' */
void MdlInitialize(void)
{
  /* InitializeConditions for Integrator: '<Root>/Integrator5' */
  rtX.Integrator5_CSTATE = rtP.Integrator5_IC;

  /* InitializeConditions for Integrator: '<Root>/Integrator7' */
  rtX.Integrator7_CSTATE = rtP.Integrator7_IC;

  /* InitializeConditions for Derivative: '<Root>/Derivative2' */
  rtDW.TimeStampA = (rtInf);
  rtDW.TimeStampB = (rtInf);

  /* InitializeConditions for Integrator: '<Root>/Integrator8' */
  rtX.Integrator8_CSTATE = rtP.Integrator8_IC;

  /* InitializeConditions for Derivative: '<Root>/Derivative3' */
  rtDW.TimeStampA_b = (rtInf);
  rtDW.TimeStampB_d = (rtInf);

  /* InitializeConditions for Derivative: '<Root>/Derivative1' */
  rtDW.TimeStampA_a = (rtInf);
  rtDW.TimeStampB_dw = (rtInf);

  /* InitializeConditions for Integrator: '<Root>/Integrator4' */
  rtX.Integrator4_CSTATE = rtP.Integrator4_IC;

  /* InitializeConditions for Derivative: '<Root>/Derivative' */
  rtDW.TimeStampA_f = (rtInf);
  rtDW.TimeStampB_dj = (rtInf);

  /* InitializeConditions for Integrator: '<Root>/Integrator6' */
  rtX.Integrator6_CSTATE = rtP.Integrator6_IC;

  /* InitializeConditions for Integrator: '<Root>/Integrator3' */
  rtX.Integrator3_CSTATE = rtP.Integrator3_IC;

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  rtX.Integrator_CSTATE = rtP.Integrator_IC;

  /* InitializeConditions for Integrator: '<Root>/Integrator1' */
  rtX.Integrator1_CSTATE = rtP.Integrator1_IC;

  /* InitializeConditions for Integrator: '<Root>/Integrator2' */
  rtX.Integrator2_CSTATE = rtP.Integrator2_IC;
}

/* Start for root system: '<Root>' */
void MdlStart(void)
{
  MdlInitialize();
}

/* Outputs for root system: '<Root>' */
void MdlOutputs(int_T tid)
{
  real_T currentTime;
  real_T *lastU;
  real_T rtb_Integrator2;

  /* Step: '<Root>/Step' */
  if (ssGetTaskTime(rtS,0) < rtP.Step_Time) {
    rtb_Integrator2 = rtP.Step_Y0;
  } else {
    rtb_Integrator2 = rtP.Step_YFinal;
  }

  /* End of Step: '<Root>/Step' */

  /* Sum: '<Root>/Add13' incorporates:
   *  Constant: '<Root>/Constant1'
   */
  rtB.Add13 = rtP.Constant1_Value + rtb_Integrator2;

  /* Step: '<Root>/Step1' */
  if (ssGetTaskTime(rtS,0) < rtP.Step1_Time) {
    rtb_Integrator2 = rtP.Step1_Y0;
  } else {
    rtb_Integrator2 = rtP.Step1_YFinal;
  }

  /* End of Step: '<Root>/Step1' */

  /* Sum: '<Root>/Add12' incorporates:
   *  Constant: '<Root>/Constant3'
   */
  rtB.Add12 = rtP.Constant3_Value + rtb_Integrator2;

  /* Step: '<Root>/Step3' */
  currentTime = ssGetTaskTime(rtS,0);

  /* Step: '<Root>/Step2' */
  if (ssGetTaskTime(rtS,0) < rtP.Step2_Time) {
    rtb_Integrator2 = rtP.Step2_Y0;
  } else {
    rtb_Integrator2 = rtP.Step2_YFinal;
  }

  /* End of Step: '<Root>/Step2' */

  /* Step: '<Root>/Step3' */
  if (currentTime < rtP.Step3_Time) {
    currentTime = rtP.Step3_Y0;
  } else {
    currentTime = rtP.Step3_YFinal;
  }

  /* Sum: '<Root>/Add11' incorporates:
   *  Constant: '<Root>/Constant4'
   */
  rtB.Add11 = (rtb_Integrator2 + currentTime) + rtP.Constant4_Value;

  /* MATLAB Function: '<Root>/MATLAB Function1' */
  /* MATLAB Function 'MATLAB Function1': '<S2>:1' */
  /* 275.5; */
  /* '<S2>:1:11' */
  /* '<S2>:1:12' */
  /*  q_3 + */
  /* q_3__ = 1 * acos( (px^2 + (pz-d1)^2 - d2^2 - d3^2) / (2*d2*d3) );   */
  /* cq_2 = (px*(d2+d3*cos(q_3__)) + d3*sin(q_3__)*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3__)); */
  /* sq_2 = (-px*d3*sin(q_3__) + (d2+d3*cos(q_3__))*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3__)); */
  /* q_2__ = (pi/2) - atan2(sq_2,cq_2); */
  /* q_3__*180/pi */
  /* q_2__*180/pi */
  /*  q_3 - */
  /* '<S2>:1:25' */
  currentTime = acos(((((rtB.Add11 - 197.13) * (rtB.Add11 - 197.13) + rtB.Add13 *
                        rtB.Add13) - 168100.0) - 42973.290000000008) / 169986.0);

  /* Sum: '<Root>/Add14' incorporates:
   *  Constant: '<Root>/Constant5'
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  /* '<S2>:1:27' */
  /* '<S2>:1:28' */
  /* '<S2>:1:29' */
  /*  q_1 offset */
  /* '<S2>:1:32' */
  rtB.Add14 = rtP.Constant5_Value - (rt_atan2d_snf(rtB.Add12, rtB.Add13) - asin
    (9.8 / sqrt(rtB.Add13 * rtB.Add13 + rtB.Add12 * rtB.Add12)));

  /* Gain: '<Root>/Gain21' incorporates:
   *  Integrator: '<Root>/Integrator5'
   */
  rtB.Gain21 = rtP.Gain21_Gain * rtX.Integrator5_CSTATE;

  /* Trigonometry: '<Root>/Trigonometric Function' */
  rtB.TrigonometricFunction = rt_atan2d_snf(rtB.Add12, rtB.Add13);

  /* Integrator: '<Root>/Integrator7' */
  rtB.Integrator7 = rtX.Integrator7_CSTATE;

  /* Derivative: '<Root>/Derivative2' */
  if ((rtDW.TimeStampA >= ssGetT(rtS)) && (rtDW.TimeStampB >= ssGetT(rtS))) {
    rtB.Derivative2 = 0.0;
  } else {
    rtb_Integrator2 = rtDW.TimeStampA;
    lastU = &rtDW.LastUAtTimeA;
    if (rtDW.TimeStampA < rtDW.TimeStampB) {
      if (rtDW.TimeStampB < ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB;
        lastU = &rtDW.LastUAtTimeB;
      }
    } else {
      if (rtDW.TimeStampA >= ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB;
        lastU = &rtDW.LastUAtTimeB;
      }
    }

    rtB.Derivative2 = (rtB.Integrator7 - *lastU) / (ssGetT(rtS) -
      rtb_Integrator2);
  }

  /* End of Derivative: '<Root>/Derivative2' */

  /* Integrator: '<Root>/Integrator8' */
  rtB.Integrator8 = rtX.Integrator8_CSTATE;

  /* Derivative: '<Root>/Derivative3' */
  if ((rtDW.TimeStampA_b >= ssGetT(rtS)) && (rtDW.TimeStampB_d >= ssGetT(rtS)))
  {
    rtB.Derivative3 = 0.0;
  } else {
    rtb_Integrator2 = rtDW.TimeStampA_b;
    lastU = &rtDW.LastUAtTimeA_k;
    if (rtDW.TimeStampA_b < rtDW.TimeStampB_d) {
      if (rtDW.TimeStampB_d < ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB_d;
        lastU = &rtDW.LastUAtTimeB_n;
      }
    } else {
      if (rtDW.TimeStampA_b >= ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB_d;
        lastU = &rtDW.LastUAtTimeB_n;
      }
    }

    rtB.Derivative3 = (rtB.Integrator8 - *lastU) / (ssGetT(rtS) -
      rtb_Integrator2);
  }

  /* End of Derivative: '<Root>/Derivative3' */

  /* Trigonometry: '<Root>/Trigonometric Function2' */
  rtB.TrigonometricFunction2 = rt_atan2d_snf(rtB.Derivative2, rtB.Derivative3);

  /* Sum: '<Root>/Add8' */
  rtB.Add8 = rtB.TrigonometricFunction - rtB.TrigonometricFunction2;

  /* Gain: '<Root>/Gain18' */
  rtB.Gain18 = rtP.Gain18_Gain * rtB.Add8;

  /* Gain: '<Root>/Gain8' */
  rtB.Gain8 = rtP.Gain8_Gain * rtB.Gain18;

  /* Derivative: '<Root>/Derivative1' */
  if ((rtDW.TimeStampA_a >= ssGetT(rtS)) && (rtDW.TimeStampB_dw >= ssGetT(rtS)))
  {
    rtb_Integrator2 = 0.0;
  } else {
    rtb_Integrator2 = rtDW.TimeStampA_a;
    lastU = &rtDW.LastUAtTimeA_i;
    if (rtDW.TimeStampA_a < rtDW.TimeStampB_dw) {
      if (rtDW.TimeStampB_dw < ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB_dw;
        lastU = &rtDW.LastUAtTimeB_g;
      }
    } else {
      if (rtDW.TimeStampA_a >= ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB_dw;
        lastU = &rtDW.LastUAtTimeB_g;
      }
    }

    rtb_Integrator2 = (rtB.Gain18 - *lastU) / (ssGetT(rtS) - rtb_Integrator2);
  }

  /* End of Derivative: '<Root>/Derivative1' */

  /* Gain: '<Root>/Gain15' */
  rtB.Gain15 = rtP.Gain15_Gain * rtb_Integrator2;

  /* Sum: '<Root>/Add17' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Math: '<Root>/Math Function1'
   *  Sum: '<Root>/Add7'
   */
  rtB.Add17 = rt_modd_snf((rtB.Gain21 + rtB.Gain8) + rtB.Gain15,
    rtP.Constant2_Value) + rtB.Add14;

  /* Sum: '<Root>/Add15' incorporates:
   *  Constant: '<Root>/Constant6'
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  rtB.Add15 = (1.5707963267948966 - rt_atan2d_snf(((207.3 * cos(currentTime) +
    410.0) * (rtB.Add11 - 197.13) + -rtB.Add13 * 207.3 * sin(currentTime)) /
    (169986.0 * cos(currentTime) + 211073.29), ((207.3 * cos(currentTime) +
    410.0) * rtB.Add13 + 207.3 * sin(currentTime) * (rtB.Add11 - 197.13)) /
    (169986.0 * cos(currentTime) + 211073.29))) + rtP.Constant6_Value;

  /* Trigonometry: '<Root>/Trigonometric Function1' incorporates:
   *  Math: '<Root>/Math Function'
   */
  rtB.TrigonometricFunction1 = rt_atan2d_snf(rtB.Add11, rt_hypotd_snf(rtB.Add13,
    rtB.Add12));

  /* Integrator: '<Root>/Integrator4' */
  rtB.Integrator4 = rtX.Integrator4_CSTATE;

  /* Derivative: '<Root>/Derivative' */
  if ((rtDW.TimeStampA_f >= ssGetT(rtS)) && (rtDW.TimeStampB_dj >= ssGetT(rtS)))
  {
    rtB.Derivative = 0.0;
  } else {
    rtb_Integrator2 = rtDW.TimeStampA_f;
    lastU = &rtDW.LastUAtTimeA_l;
    if (rtDW.TimeStampA_f < rtDW.TimeStampB_dj) {
      if (rtDW.TimeStampB_dj < ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB_dj;
        lastU = &rtDW.LastUAtTimeB_l;
      }
    } else {
      if (rtDW.TimeStampA_f >= ssGetT(rtS)) {
        rtb_Integrator2 = rtDW.TimeStampB_dj;
        lastU = &rtDW.LastUAtTimeB_l;
      }
    }

    rtB.Derivative = (rtB.Integrator4 - *lastU) / (ssGetT(rtS) - rtb_Integrator2);
  }

  /* End of Derivative: '<Root>/Derivative' */

  /* Math: '<Root>/Math Function2' */
  rtB.MathFunction2 = rt_hypotd_snf(rtB.Derivative2, rtB.Derivative3);

  /* Trigonometry: '<Root>/Trigonometric Function3' */
  rtB.TrigonometricFunction3 = rt_atan2d_snf(rtB.Derivative, rtB.MathFunction2);

  /* Sum: '<Root>/Add3' */
  rtB.Add3 = rtB.TrigonometricFunction1 - rtB.TrigonometricFunction3;

  /* Sum: '<Root>/Add6' incorporates:
   *  Gain: '<Root>/Gain13'
   *  Gain: '<Root>/Gain9'
   */
  rtB.Add6 = rtP.Gain9_Gain * rtB.Add8 + rtP.Gain13_Gain * rtB.Add3;

  /* Sum: '<Root>/Add18' incorporates:
   *  Gain: '<Root>/Gain17'
   *  Gain: '<Root>/Gain7'
   *  Integrator: '<Root>/Integrator6'
   *  Sum: '<Root>/Add10'
   */
  rtB.Add18 = (rtP.Gain7_Gain * rtB.Add6 + rtP.Gain17_Gain *
               rtX.Integrator6_CSTATE) + rtB.Add15;

  /* Sum: '<Root>/Add16' incorporates:
   *  Constant: '<Root>/Constant7'
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  rtB.Add16 = currentTime + rtP.Constant7_Value;

  /* Sum: '<Root>/Add19' incorporates:
   *  Gain: '<Root>/Gain16'
   *  Gain: '<Root>/Gain6'
   *  Integrator: '<Root>/Integrator3'
   *  Sum: '<Root>/Add9'
   */
  rtB.Add19 = (rtP.Gain6_Gain * rtB.Add6 + rtP.Gain16_Gain *
               rtX.Integrator3_CSTATE) + rtB.Add16;

  /* Sum: '<Root>/Add5' incorporates:
   *  Gain: '<Root>/Gain12'
   *  Gain: '<Root>/Gain14'
   */
  rtB.Add5 = rtP.Gain12_Gain * rtB.Add8 + rtP.Gain14_Gain * rtB.Add3;

  /* Sum: '<Root>/Add2' incorporates:
   *  Gain: '<Root>/Gain'
   *  Gain: '<Root>/Gain5'
   *  Integrator: '<Root>/Integrator'
   */
  rtB.Add2 = rtP.Gain_Gain * rtX.Integrator_CSTATE + rtP.Gain5_Gain * rtB.Add5;

  /* Gain: '<Root>/Gain11' */
  rtB.Gain11 = rtP.Gain11_Gain * rtB.Add3;

  /* Sum: '<Root>/Add4' incorporates:
   *  Gain: '<Root>/Gain10'
   */
  rtB.Add4 = rtP.Gain10_Gain * rtB.Add8 + rtB.Gain11;

  /* Sum: '<Root>/Add1' incorporates:
   *  Gain: '<Root>/Gain1'
   *  Gain: '<Root>/Gain4'
   *  Integrator: '<Root>/Integrator1'
   */
  rtB.Add1 = rtP.Gain1_Gain * rtX.Integrator1_CSTATE + rtP.Gain4_Gain * rtB.Add4;

  /* Sum: '<Root>/Add' incorporates:
   *  Gain: '<Root>/Gain2'
   *  Gain: '<Root>/Gain3'
   *  Integrator: '<Root>/Integrator2'
   */
  rtB.Add = rtP.Gain2_Gain * rtX.Integrator2_CSTATE + rtP.Gain3_Gain *
    rtB.Gain11;

  /* MATLAB Function: '<Root>/MATLAB Function' */
  /* MATLAB Function 'MATLAB Function': '<S1>:1' */
  /* pi-(q_1); */
  /* -pi/2 + (q_2+pi); */
  /* pi/2 + (pi+q_3); */
  /* q_4; */
  /* -pi + q_5; */
  /* pi + q_6; */
  /* A_new = vpa(subs(A, [q1,q2,q3,q4,q5,q6], [ pi-(q_1), -pi/2 + (q_2+pi), pi/2 + (pi+q_3), q_4, -pi + q_5, pi])); */
  /* '<S1>:1:14' */
  rtB.x_a = (((((sin(rtB.Add17) * sin(rtB.Add2) - (cos(rtB.Add17) * cos
    (rtB.Add18) * cos(rtB.Add19) + cos(rtB.Add17) * sin(rtB.Add18) * sin
    (rtB.Add19)) * cos(rtB.Add2)) * (175.61406460551018 * sin(rtB.Add1)) +
                ((410.0 * cos(rtB.Add17) * cos(rtB.Add18) + 9.8 * sin(rtB.Add17))
                 - 161.90703230275506 * cos(rtB.Add2) * sin(rtB.Add17))) - (cos
    (rtB.Add17) * cos(rtB.Add18) * cos(rtB.Add19) + cos(rtB.Add17) * sin
    (rtB.Add18) * sin(rtB.Add19)) * (161.90703230275506 * sin(rtB.Add2))) -
              ((((cos(rtB.Add17) * cos(rtB.Add18) * cos(rtB.Add19) + cos
                  (rtB.Add17) * sin(rtB.Add18) * sin(rtB.Add19)) * (0.5 * sin
    (rtB.Add2)) + 0.5 * cos(rtB.Add2) * sin(rtB.Add17)) - 0.8660254037844386 *
                cos(rtB.Add17) * cos(rtB.Add18) * sin(rtB.Add19)) +
               0.8660254037844386 * cos(rtB.Add17) * cos(rtB.Add19) * sin
               (rtB.Add18)) * (175.61406460551018 * cos(rtB.Add1))) -
             343.55872363064032 * cos(rtB.Add17) * cos(rtB.Add18) * sin
             (rtB.Add19)) + 343.55872363064032 * cos(rtB.Add17) * cos(rtB.Add19)
    * sin(rtB.Add18);

  /* '<S1>:1:15' */
  rtB.y_a = (((((((0.5 * cos(rtB.Add17) * cos(rtB.Add2) - (sin(rtB.Add17) * sin
    (rtB.Add18) * sin(rtB.Add19) + cos(rtB.Add18) * cos(rtB.Add19) * sin
    (rtB.Add17)) * (0.5 * sin(rtB.Add2))) + 0.8660254037844386 * cos(rtB.Add18) *
                  sin(rtB.Add17) * sin(rtB.Add19)) - 0.8660254037844386 * cos
                 (rtB.Add19) * sin(rtB.Add17) * sin(rtB.Add18)) *
                (175.61406460551018 * cos(rtB.Add1)) + ((161.90703230275506 *
    cos(rtB.Add17) * cos(rtB.Add2) - 9.8 * cos(rtB.Add17)) + 410.0 * cos
    (rtB.Add18) * sin(rtB.Add17))) - ((sin(rtB.Add17) * sin(rtB.Add18) * sin
    (rtB.Add19) + cos(rtB.Add18) * cos(rtB.Add19) * sin(rtB.Add17)) * cos
    (rtB.Add2) + cos(rtB.Add17) * sin(rtB.Add2)) * (175.61406460551018 * sin
    (rtB.Add1))) - (sin(rtB.Add17) * sin(rtB.Add18) * sin(rtB.Add19) + cos
                    (rtB.Add18) * cos(rtB.Add19) * sin(rtB.Add17)) *
              (161.90703230275506 * sin(rtB.Add2))) - 343.55872363064032 * cos
             (rtB.Add18) * sin(rtB.Add17) * sin(rtB.Add19)) + 343.55872363064032
    * cos(rtB.Add19) * sin(rtB.Add17) * sin(rtB.Add18);

  /* '<S1>:1:16' */
  rtB.z_a = (((((0.8660254037844386 * cos(rtB.Add18) * cos(rtB.Add19) +
                 0.8660254037844386 * sin(rtB.Add18) * sin(rtB.Add19)) + (cos
    (rtB.Add18) * sin(rtB.Add19) - cos(rtB.Add19) * sin(rtB.Add18)) * (0.5 * sin
    (rtB.Add2))) * (175.61406460551018 * cos(rtB.Add1)) + ((410.0 * sin
    (rtB.Add18) - 343.55872363064032 * cos(rtB.Add18) * cos(rtB.Add19)) -
    343.55872363064032 * sin(rtB.Add18) * sin(rtB.Add19))) + (cos(rtB.Add18) *
    sin(rtB.Add19) - cos(rtB.Add19) * sin(rtB.Add18)) * (161.90703230275506 *
    sin(rtB.Add2))) + 175.61406460551018 * cos(rtB.Add2) * sin(rtB.Add1) * (cos
              (rtB.Add18) * sin(rtB.Add19) - cos(rtB.Add19) * sin(rtB.Add18))) +
    197.13;

  /* y = u; */
  if (ssIsSampleHit(rtS, 1, 0)) {
  }

  UNUSED_PARAMETER(tid);
}

/* Update for root system: '<Root>' */
void MdlUpdate(int_T tid)
{
  real_T *lastU;

  /* Update for Derivative: '<Root>/Derivative2' */
  if (rtDW.TimeStampA == (rtInf)) {
    rtDW.TimeStampA = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA;
  } else if (rtDW.TimeStampB == (rtInf)) {
    rtDW.TimeStampB = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB;
  } else if (rtDW.TimeStampA < rtDW.TimeStampB) {
    rtDW.TimeStampA = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA;
  } else {
    rtDW.TimeStampB = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB;
  }

  *lastU = rtB.Integrator7;

  /* End of Update for Derivative: '<Root>/Derivative2' */

  /* Update for Derivative: '<Root>/Derivative3' */
  if (rtDW.TimeStampA_b == (rtInf)) {
    rtDW.TimeStampA_b = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA_k;
  } else if (rtDW.TimeStampB_d == (rtInf)) {
    rtDW.TimeStampB_d = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB_n;
  } else if (rtDW.TimeStampA_b < rtDW.TimeStampB_d) {
    rtDW.TimeStampA_b = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA_k;
  } else {
    rtDW.TimeStampB_d = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB_n;
  }

  *lastU = rtB.Integrator8;

  /* End of Update for Derivative: '<Root>/Derivative3' */

  /* Update for Derivative: '<Root>/Derivative1' */
  if (rtDW.TimeStampA_a == (rtInf)) {
    rtDW.TimeStampA_a = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA_i;
  } else if (rtDW.TimeStampB_dw == (rtInf)) {
    rtDW.TimeStampB_dw = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB_g;
  } else if (rtDW.TimeStampA_a < rtDW.TimeStampB_dw) {
    rtDW.TimeStampA_a = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA_i;
  } else {
    rtDW.TimeStampB_dw = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB_g;
  }

  *lastU = rtB.Gain18;

  /* End of Update for Derivative: '<Root>/Derivative1' */

  /* Update for Derivative: '<Root>/Derivative' */
  if (rtDW.TimeStampA_f == (rtInf)) {
    rtDW.TimeStampA_f = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA_l;
  } else if (rtDW.TimeStampB_dj == (rtInf)) {
    rtDW.TimeStampB_dj = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB_l;
  } else if (rtDW.TimeStampA_f < rtDW.TimeStampB_dj) {
    rtDW.TimeStampA_f = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeA_l;
  } else {
    rtDW.TimeStampB_dj = ssGetT(rtS);
    lastU = &rtDW.LastUAtTimeB_l;
  }

  *lastU = rtB.Integrator4;

  /* End of Update for Derivative: '<Root>/Derivative' */
  UNUSED_PARAMETER(tid);
}

/* Derivatives for root system: '<Root>' */
void MdlDerivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) ssGetdX(rtS));

  /* Derivatives for Integrator: '<Root>/Integrator5' */
  _rtXdot->Integrator5_CSTATE = rtB.Gain18;

  /* Derivatives for Integrator: '<Root>/Integrator7' */
  _rtXdot->Integrator7_CSTATE = rtB.y_a;

  /* Derivatives for Integrator: '<Root>/Integrator8' */
  _rtXdot->Integrator8_CSTATE = rtB.x_a;

  /* Derivatives for Integrator: '<Root>/Integrator4' */
  _rtXdot->Integrator4_CSTATE = rtB.z_a;

  /* Derivatives for Integrator: '<Root>/Integrator6' */
  _rtXdot->Integrator6_CSTATE = rtB.Add6;

  /* Derivatives for Integrator: '<Root>/Integrator3' */
  _rtXdot->Integrator3_CSTATE = rtB.Add6;

  /* Derivatives for Integrator: '<Root>/Integrator' */
  _rtXdot->Integrator_CSTATE = rtB.Add5;

  /* Derivatives for Integrator: '<Root>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = rtB.Add4;

  /* Derivatives for Integrator: '<Root>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = rtB.Gain11;
}

/* Projection for root system: '<Root>' */
void MdlProjection(void)
{
}

/* Termination for root system: '<Root>' */
void MdlTerminate(void)
{
}

/* Function to initialize sizes */
void MdlInitializeSizes(void)
{
  ssSetNumContStates(rtS, 9);          /* Number of continuous states */
  ssSetNumPeriodicContStates(rtS, 0);  /* Number of periodic continuous states */
  ssSetNumY(rtS, 0);                   /* Number of model outputs */
  ssSetNumU(rtS, 0);                   /* Number of model inputs */
  ssSetDirectFeedThrough(rtS, 0);      /* The model is not direct feedthrough */
  ssSetNumSampleTimes(rtS, 2);         /* Number of sample times */
  ssSetNumBlocks(rtS, 89);             /* Number of blocks */
  ssSetNumBlockIO(rtS, 36);            /* Number of block outputs */
  ssSetNumBlockParams(rtS, 48);        /* Sum of parameter "widths" */
}

/* Function to initialize sample times. */
void MdlInitializeSampleTimes(void)
{
  /* task periods */
  ssSetSampleTime(rtS, 0, 0.0);
  ssSetSampleTime(rtS, 1, 0.002);

  /* task offsets */
  ssSetOffsetTime(rtS, 0, 0.0);
  ssSetOffsetTime(rtS, 1, 0.0);
}

/* Function to register the model */
SimStruct * Control_Model_v3(void)
{
  static struct _ssMdlInfo mdlInfo;
  (void) memset((char *)rtS, 0,
                sizeof(SimStruct));
  (void) memset((char *)&mdlInfo, 0,
                sizeof(struct _ssMdlInfo));
  ssSetMdlInfoPtr(rtS, &mdlInfo);

  /* timing info */
  {
    static time_T mdlPeriod[NSAMPLE_TIMES];
    static time_T mdlOffset[NSAMPLE_TIMES];
    static time_T mdlTaskTimes[NSAMPLE_TIMES];
    static int_T mdlTsMap[NSAMPLE_TIMES];
    static int_T mdlSampleHits[NSAMPLE_TIMES];

    {
      int_T i;
      for (i = 0; i < NSAMPLE_TIMES; i++) {
        mdlPeriod[i] = 0.0;
        mdlOffset[i] = 0.0;
        mdlTaskTimes[i] = 0.0;
        mdlTsMap[i] = i;
        mdlSampleHits[i] = 1;
      }
    }

    ssSetSampleTimePtr(rtS, &mdlPeriod[0]);
    ssSetOffsetTimePtr(rtS, &mdlOffset[0]);
    ssSetSampleTimeTaskIDPtr(rtS, &mdlTsMap[0]);
    ssSetTPtr(rtS, &mdlTaskTimes[0]);
    ssSetSampleHitPtr(rtS, &mdlSampleHits[0]);
  }

  ssSetSolverMode(rtS, SOLVER_MODE_SINGLETASKING);

  /*
   * initialize model vectors and cache them in SimStruct
   */

  /* block I/O */
  {
    ssSetBlockIO(rtS, ((void *) &rtB));
    (void) memset(((void *) &rtB), 0,
                  sizeof(B));
  }

  /* parameters */
  ssSetDefaultParam(rtS, (real_T *) &rtP);

  /* states (continuous)*/
  {
    real_T *x = (real_T *) &rtX;
    ssSetContStates(rtS, x);
    (void) memset((void *)x, 0,
                  sizeof(X));
  }

  /* states (dwork) */
  {
    void *dwork = (void *) &rtDW;
    ssSetRootDWork(rtS, dwork);
    (void) memset(dwork, 0,
                  sizeof(DW));
  }

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    ssSetModelMappingInfo(rtS, &dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* Model specific registration */
  ssSetRootSS(rtS, rtS);
  ssSetVersion(rtS, SIMSTRUCT_VERSION_LEVEL2);
  ssSetModelName(rtS, "Control_Model_v3");
  ssSetPath(rtS, "Control_Model_v3");
  ssSetTStart(rtS, 0.0);
  ssSetTFinal(rtS, 120.0);
  ssSetStepSize(rtS, 0.002);
  ssSetFixedStepSize(rtS, 0.002);

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    ssSetRTWLogInfo(rtS, &rt_DataLoggingInfo);
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogXSignalPtrs(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogT(ssGetRTWLogInfo(rtS), "tout");
    rtliSetLogX(ssGetRTWLogInfo(rtS), "");
    rtliSetLogXFinal(ssGetRTWLogInfo(rtS), "");
    rtliSetLogVarNameModifier(ssGetRTWLogInfo(rtS), "rt_");
    rtliSetLogFormat(ssGetRTWLogInfo(rtS), 0);
    rtliSetLogMaxRows(ssGetRTWLogInfo(rtS), 1000);
    rtliSetLogDecimation(ssGetRTWLogInfo(rtS), 1);
    rtliSetLogY(ssGetRTWLogInfo(rtS), "");
    rtliSetLogYSignalInfo(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogYSignalPtrs(ssGetRTWLogInfo(rtS), (NULL));
  }

  {
    static struct _ssStatesInfo2 statesInfo2;
    ssSetStatesInfo2(rtS, &statesInfo2);
  }

  {
    static ssPeriodicStatesInfo periodicStatesInfo;
    ssSetPeriodicStatesInfo(rtS, &periodicStatesInfo);
  }

  ssSetChecksumVal(rtS, 0, 137488856U);
  ssSetChecksumVal(rtS, 1, 52474277U);
  ssSetChecksumVal(rtS, 2, 1027164879U);
  ssSetChecksumVal(rtS, 3, 2566039862U);
  return rtS;
}
