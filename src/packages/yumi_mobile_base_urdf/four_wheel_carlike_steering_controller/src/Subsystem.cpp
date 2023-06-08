//
// File: Subsystem.cpp
//
// Code generated for Simulink model 'Subsystem'.
//
// Model version                  : 1.332
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Tue Sep 29 10:37:32 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "four_wheel_carlike_steering_controller/Subsystem.h"
#define NumBitsPerChar                 8U

extern "C" {
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"
  extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static void rt_InitInfAndNaN(size_t realSize);
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
  typedef struct {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  } BigEndianIEEEDouble;

  typedef struct {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  } LittleEndianIEEEDouble;

  typedef struct {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  } IEEESingle;
}                                      // extern "C"

extern "C" {
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}
  extern "C"
{
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"

extern "C" {
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T nan = 0.0;
    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0 } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}
  extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  static void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result = (boolean_T) 0;
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}

extern "C" {
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T inf = 0.0;
    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T minf = 0.0;
    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}
// Function for MATLAB Function: '<S1>/MATLAB Function5'
  real_T SubsystemModelClass::inv(real_T x)
{
  return 1.0 / x;
}

// Model step function
void SubsystemModelClass::step()
{
  real_T Accx;
  real_T Accy;
  real_T omegaDotDotN;
  real_T Zetadoti[3];
  real_T Zetadotdoti[3];
  real_T Betaidot[4];
  real_T F_hat[12];
  real32_T ZetaDot_hat[3];
  real_T b_F_hat[12];
  real_T A[9];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  real_T x_dot;
  real_T y_dot;
  real_T theta_dot;
  real_T rtb_beta[4];
  real_T rtb_Phi_dotN[4];
  real_T rtb_Memory[4];
  real_T A_tmp[12];
  real32_T ZetaDot_hat_tmp;
  real32_T ZetaDot_hat_tmp_0;
  real32_T ZetaDot_hat_tmp_1[9];
  real_T rtb_Memory_l;
  real_T rtb_beta_o;
  real_T F1i_idx_0;
  real_T F1i_idx_1;
  real_T F1i_idx_2;
  real_T F2i_idx_0;
  real_T F2i_idx_2;
  static const real_T b[4] = { 0.263, 0.263, -0.263, -0.263 };

  static const real_T c[4] = { 0.263, -0.263, -0.263, 0.263 };

  static const real_T b_y[9] = { 1.0E-8, 0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0,
    1.0E-8 };

  static const int8_T b_0[3] = { 0, 0, 1 };

  // Outputs for Atomic SubSystem: '<Root>/Subsystem'
  // Outputs for Atomic SubSystem: '<S1>/Inverse Kinematics'
  // Memory: '<S2>/Memory'
  rtb_Memory_l = rtDW.Memory_PreviousInput[0];

  // MATLAB Function: '<S2>/computeBeta' incorporates:
  //   Inport: '<Root>/Ref'

  x_dot = 0.371938166904124 * rtU.Ref[2] * 0.70710678118654746 + -rtU.Ref[0];
  if ((std::abs(x_dot) < 2.2204460492503131E-16) && (std::abs(-rtU.Ref[1] -
        0.371938166904124 * rtU.Ref[2] * 0.70710678118654757) <
       2.2204460492503131E-16)) {
    rtb_beta_o = rtb_Memory_l;
  } else {
    rtb_beta_o = std::atan((-rtU.Ref[1] - 0.371938166904124 * rtU.Ref[2] *
      0.70710678118654757) / x_dot);
    Accx = std::abs(rtb_beta_o - rtb_Memory_l);
    if (Accx > 4.71238898038469) {
      if (rtb_Memory_l > 0.0) {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = -rtb_Memory_l * rtb_beta_o + 6.2831853071795862;
      } else {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = rtb_Memory_l * rtb_beta_o - 6.2831853071795862;
      }
    } else {
      if (Accx > 1.5707963267948966) {
        if (rtb_Memory_l > 0.0) {
          rtb_beta_o += 3.1415926535897931;
        } else {
          rtb_beta_o -= 3.1415926535897931;
        }
      }
    }
  }

  // Update for Memory: '<S2>/Memory'
  rtDW.Memory_PreviousInput[0] = rtb_beta_o;

  // End of Outputs for SubSystem: '<S1>/Inverse Kinematics'

  // MATLAB Function: '<S1>/MATLAB Function'
  rtb_Phi_dotN[0] = 0.0;

  // Outputs for Atomic SubSystem: '<S1>/Inverse Kinematics'
  // Memory: '<S2>/Memory'
  rtb_Memory[0] = 0.0;

  // MATLAB Function: '<S2>/computeBeta'
  rtb_beta[0] = rtb_beta_o;

  // Memory: '<S2>/Memory'
  rtb_Memory_l = rtDW.Memory_PreviousInput[1];

  // MATLAB Function: '<S2>/computeBeta' incorporates:
  //   Inport: '<Root>/Ref'

  x_dot = 0.371938166904124 * rtU.Ref[2] * -0.70710678118654768;
  y_dot = x_dot + -rtU.Ref[0];
  if ((std::abs(y_dot) < 2.2204460492503131E-16) && (std::abs(-rtU.Ref[1] -
        0.371938166904124 * rtU.Ref[2] * 0.70710678118654735) <
       2.2204460492503131E-16)) {
    rtb_beta_o = rtb_Memory_l;
  } else {
    rtb_beta_o = std::atan((-rtU.Ref[1] - 0.371938166904124 * rtU.Ref[2] *
      0.70710678118654735) / y_dot);
    Accx = std::abs(rtb_beta_o - rtb_Memory_l);
    if (Accx > 4.71238898038469) {
      if (rtb_Memory_l > 0.0) {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = -rtb_Memory_l * rtb_beta_o + 6.2831853071795862;
      } else {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = rtb_Memory_l * rtb_beta_o - 6.2831853071795862;
      }
    } else {
      if (Accx > 1.5707963267948966) {
        if (rtb_Memory_l > 0.0) {
          rtb_beta_o += 3.1415926535897931;
        } else {
          rtb_beta_o -= 3.1415926535897931;
        }
      }
    }
  }

  // Update for Memory: '<S2>/Memory'
  rtDW.Memory_PreviousInput[1] = rtb_beta_o;

  // End of Outputs for SubSystem: '<S1>/Inverse Kinematics'

  // MATLAB Function: '<S1>/MATLAB Function'
  rtb_Phi_dotN[1] = 0.0;

  // Outputs for Atomic SubSystem: '<S1>/Inverse Kinematics'
  // Memory: '<S2>/Memory'
  rtb_Memory[1] = 0.0;

  // MATLAB Function: '<S2>/computeBeta'
  rtb_beta[1] = rtb_beta_o;

  // Memory: '<S2>/Memory'
  rtb_Memory_l = rtDW.Memory_PreviousInput[2];

  // MATLAB Function: '<S2>/computeBeta' incorporates:
  //   Inport: '<Root>/Ref'

  y_dot = 0.371938166904124 * rtU.Ref[2] * -0.70710678118654746;
  rtb_beta_o = y_dot + -rtU.Ref[0];
  if ((std::abs(rtb_beta_o) < 2.2204460492503131E-16) && (std::abs(-rtU.Ref[1] -
        x_dot) < 2.2204460492503131E-16)) {
    rtb_beta_o = rtb_Memory_l;
  } else {
    rtb_beta_o = std::atan((-rtU.Ref[1] - 0.371938166904124 * rtU.Ref[2] *
      -0.70710678118654768) / rtb_beta_o);
    Accx = std::abs(rtb_beta_o - rtb_Memory_l);
    if (Accx > 4.71238898038469) {
      if (rtb_Memory_l > 0.0) {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = -rtb_Memory_l * rtb_beta_o + 6.2831853071795862;
      } else {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = rtb_Memory_l * rtb_beta_o - 6.2831853071795862;
      }
    } else {
      if (Accx > 1.5707963267948966) {
        if (rtb_Memory_l > 0.0) {
          rtb_beta_o += 3.1415926535897931;
        } else {
          rtb_beta_o -= 3.1415926535897931;
        }
      }
    }
  }

  // Update for Memory: '<S2>/Memory'
  rtDW.Memory_PreviousInput[2] = rtb_beta_o;

  // End of Outputs for SubSystem: '<S1>/Inverse Kinematics'

  // MATLAB Function: '<S1>/MATLAB Function'
  rtb_Phi_dotN[2] = 0.0;

  // Outputs for Atomic SubSystem: '<S1>/Inverse Kinematics'
  // Memory: '<S2>/Memory'
  rtb_Memory[2] = 0.0;

  // MATLAB Function: '<S2>/computeBeta'
  rtb_beta[2] = rtb_beta_o;

  // Memory: '<S2>/Memory'
  rtb_Memory_l = rtDW.Memory_PreviousInput[3];

  // MATLAB Function: '<S2>/computeBeta' incorporates:
  //   Inport: '<Root>/Ref'

  x_dot = 0.371938166904124 * rtU.Ref[2] * 0.70710678118654757 + -rtU.Ref[0];
  if ((std::abs(x_dot) < 2.2204460492503131E-16) && (std::abs(-rtU.Ref[1] -
        y_dot) < 2.2204460492503131E-16)) {
    rtb_beta_o = rtb_Memory_l;
  } else {
    rtb_beta_o = std::atan((-rtU.Ref[1] - 0.371938166904124 * rtU.Ref[2] *
      -0.70710678118654746) / x_dot);
    Accx = std::abs(rtb_beta_o - rtb_Memory_l);
    if (Accx > 4.71238898038469) {
      if (rtb_Memory_l > 0.0) {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = -rtb_Memory_l * rtb_beta_o + 6.2831853071795862;
      } else {
        if (rtb_beta_o < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (rtb_beta_o > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (rtb_beta_o == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_beta_o = rtb_Memory_l * rtb_beta_o - 6.2831853071795862;
      }
    } else {
      if (Accx > 1.5707963267948966) {
        if (rtb_Memory_l > 0.0) {
          rtb_beta_o += 3.1415926535897931;
        } else {
          rtb_beta_o -= 3.1415926535897931;
        }
      }
    }
  }

  // Update for Memory: '<S2>/Memory'
  rtDW.Memory_PreviousInput[3] = rtb_beta_o;

  // End of Outputs for SubSystem: '<S1>/Inverse Kinematics'

  // MATLAB Function: '<S1>/MATLAB Function'
  rtb_Phi_dotN[3] = 0.0;

  // Outputs for Atomic SubSystem: '<S1>/Inverse Kinematics'
  // Memory: '<S2>/Memory'
  rtb_Memory[3] = 0.0;

  // MATLAB Function: '<S2>/computeBeta'
  rtb_beta[3] = rtb_beta_o;

  // End of Outputs for SubSystem: '<S1>/Inverse Kinematics'

  // MATLAB Function: '<S1>/MATLAB Function' incorporates:
  //   Inport: '<Root>/Ref'
  //   Inport: '<Root>/reset_sig'
  //   Inport: '<Root>/sampleTime'

  x_dot = rtU.Ref[0];
  y_dot = rtU.Ref[1];
  theta_dot = rtU.Ref[2];
  Accx = (rtU.Ref[0] - rtDW.VxO) / rtU.sampleTime;
  Accy = (rtU.Ref[1] - rtDW.VyO) / rtU.sampleTime;
  omegaDotDotN = (rtU.Ref[2] - rtDW.omegaDotO) / rtU.sampleTime;
  rtDW.VxO = rtU.Ref[0];
  rtDW.VyO = rtU.Ref[1];
  rtDW.omegaDotO = rtU.Ref[2];
  Zetadoti[0] = rtU.Ref[0];
  Zetadoti[1] = rtU.Ref[1];
  Zetadoti[2] = rtU.Ref[2];
  Zetadotdoti[0] = Accx;
  Zetadotdoti[1] = Accy;
  if ((!rtIsNaN(rtU.Ref[0])) && (!rtIsNaN(rtU.Ref[1])) && (!rtIsNaN(rtU.Ref[2]))
      && (!rtIsNaN(Accx)) && (!rtIsNaN(Accy)) && (!rtIsNaN(omegaDotDotN))) {
    if ((rtU.Ref[0] != 0.0) || (rtU.Ref[1] != 0.0) || (rtU.Ref[2] != 0.0)) {
      for (r1 = 0; r1 < 4; r1++) {
        Accx = b[r1] * theta_dot + y_dot;
        Accy = x_dot - c[r1] * theta_dot;
        if (Accy < 0.0) {
          rtb_Memory_l = -1.0;
        } else if (Accy > 0.0) {
          rtb_Memory_l = 1.0;
        } else if (Accy == 0.0) {
          rtb_Memory_l = 0.0;
        } else {
          rtb_Memory_l = (rtNaN);
        }

        rtb_Memory_l = std::atan(Accx / (1.0E-9 * rtb_Memory_l + Accy));
        F1i_idx_2 = Accy * Accy + Accx * Accx;
        F1i_idx_0 = -Accx / F1i_idx_2;
        F1i_idx_1 = Accy / F1i_idx_2;
        F1i_idx_2 = (b[r1] * x_dot + c[r1] * y_dot) / F1i_idx_2;
        Accx = std::cos(rtb_Memory_l);
        Accy = std::sin(rtb_Memory_l);
        F2i_idx_0 = Accx;
        F2i_idx_2 = (0.0 - c[r1] * Accx) + b[r1] * Accy;
        Accx = std::abs(rtb_Memory_l - rtDW.previousBeta[r1]);
        if (Accx > 4.71238898038469) {
          if (rtDW.previousBeta[r1] > 0.0) {
            if (rtb_Memory_l < 0.0) {
              Accx = -1.0;
            } else if (rtb_Memory_l > 0.0) {
              Accx = 1.0;
            } else if (rtb_Memory_l == 0.0) {
              Accx = 0.0;
            } else {
              Accx = (rtNaN);
            }

            rtb_Memory_l = -Accx * rtb_Memory_l + 6.2831853071795862;
            Betaidot[r1] = (F1i_idx_0 * Zetadotdoti[0] + F1i_idx_1 *
                            Zetadotdoti[1]) + F1i_idx_2 * omegaDotDotN;
            rtb_Phi_dotN[r1] = ((11.904761904761903 * F2i_idx_0 * Zetadoti[0] +
                                 11.904761904761903 * Accy * Zetadoti[1]) +
                                11.904761904761903 * F2i_idx_2 * Zetadoti[2]) +
              ((0.0 * F1i_idx_0 * Zetadotdoti[0] + 0.0 * F1i_idx_1 *
                Zetadotdoti[1]) + 0.0 * F1i_idx_2 * omegaDotDotN);
          } else {
            if (rtb_Memory_l < 0.0) {
              Accx = -1.0;
            } else if (rtb_Memory_l > 0.0) {
              Accx = 1.0;
            } else if (rtb_Memory_l == 0.0) {
              Accx = 0.0;
            } else {
              Accx = (rtNaN);
            }

            rtb_Memory_l = Accx * rtb_Memory_l - 6.2831853071795862;
            Betaidot[r1] = (F1i_idx_0 * Zetadotdoti[0] + F1i_idx_1 *
                            Zetadotdoti[1]) + F1i_idx_2 * omegaDotDotN;
            rtb_Phi_dotN[r1] = ((11.904761904761903 * F2i_idx_0 * Zetadoti[0] +
                                 11.904761904761903 * Accy * Zetadoti[1]) +
                                11.904761904761903 * F2i_idx_2 * Zetadoti[2]) +
              ((0.0 * F1i_idx_0 * Zetadotdoti[0] + 0.0 * F1i_idx_1 *
                Zetadotdoti[1]) + 0.0 * F1i_idx_2 * omegaDotDotN);
          }
        } else if (Accx > 1.5707963267948966) {
          if (rtDW.previousBeta[r1] > 0.0) {
            rtb_Memory_l += 3.1415926535897931;
            Betaidot[r1] = (F1i_idx_0 * Zetadotdoti[0] + F1i_idx_1 *
                            Zetadotdoti[1]) + F1i_idx_2 * omegaDotDotN;
            rtb_Phi_dotN[r1] = ((-11.904761904761903 * F2i_idx_0 * Zetadoti[0] +
                                 -11.904761904761903 * Accy * Zetadoti[1]) +
                                -11.904761904761903 * F2i_idx_2 * Zetadoti[2]) +
              ((0.0 * F1i_idx_0 * Zetadotdoti[0] + 0.0 * F1i_idx_1 *
                Zetadotdoti[1]) + 0.0 * F1i_idx_2 * omegaDotDotN);
          } else {
            rtb_Memory_l -= 3.1415926535897931;
            Betaidot[r1] = (F1i_idx_0 * Zetadotdoti[0] + F1i_idx_1 *
                            Zetadotdoti[1]) + F1i_idx_2 * omegaDotDotN;
            rtb_Phi_dotN[r1] = ((-11.904761904761903 * F2i_idx_0 * Zetadoti[0] +
                                 -11.904761904761903 * Accy * Zetadoti[1]) +
                                -11.904761904761903 * F2i_idx_2 * Zetadoti[2]) +
              ((0.0 * F1i_idx_0 * Zetadotdoti[0] + 0.0 * F1i_idx_1 *
                Zetadotdoti[1]) + 0.0 * F1i_idx_2 * omegaDotDotN);
          }
        } else {
          Betaidot[r1] = (F1i_idx_0 * Zetadotdoti[0] + F1i_idx_1 * Zetadotdoti[1])
            + F1i_idx_2 * omegaDotDotN;
          rtb_Phi_dotN[r1] = ((11.904761904761903 * F2i_idx_0 * Zetadoti[0] +
                               11.904761904761903 * Accy * Zetadoti[1]) +
                              11.904761904761903 * F2i_idx_2 * Zetadoti[2]) +
            ((0.0 * F1i_idx_0 * Zetadotdoti[0] + 0.0 * F1i_idx_1 * Zetadotdoti[1])
             + 0.0 * F1i_idx_2 * omegaDotDotN);
        }

        rtb_Memory[r1] = rtb_Memory_l;
      }

      if ((!rtIsNaN(rtb_Memory[0])) && (!rtIsNaN(Betaidot[0])) && (!rtIsNaN
           (rtb_Phi_dotN[0])) && (!rtIsNaN(rtb_Memory[1])) && (!rtIsNaN
           (Betaidot[1])) && (!rtIsNaN(rtb_Phi_dotN[1])) && (!rtIsNaN
           (rtb_Memory[2])) && (!rtIsNaN(Betaidot[2])) && (!rtIsNaN
           (rtb_Phi_dotN[2])) && (!rtIsNaN(rtb_Memory[3])) && (!rtIsNaN
           (Betaidot[3])) && (!rtIsNaN(rtb_Phi_dotN[3]))) {
        rtDW.previousBeta[0] = rtb_Memory[0];
        rtDW.previousBeta[1] = rtb_Memory[1];
        rtDW.previousBeta[2] = rtb_Memory[2];
        rtDW.previousBeta[3] = rtb_Memory[3];
      }
    } else {
      if ((rtU.Ref[0] == 0.0) && (rtU.Ref[1] == 0.0) && (rtU.Ref[2] == 0.0)) {
        rtb_Memory[0] = rtDW.previousBeta[0];
        rtb_Memory[1] = rtDW.previousBeta[1];
        rtb_Memory[2] = rtDW.previousBeta[2];
        rtb_Memory[3] = rtDW.previousBeta[3];
      }
    }
  }

  if (rtU.reset_sig == 1.0) {
    rtb_Memory[0] = 0.0;
    rtb_Memory[1] = 0.0;
    rtb_Memory[2] = 0.0;
    rtb_Memory[3] = 0.0;
  }

  // MATLAB Function: '<S1>/MATLAB Function5' incorporates:
  //   Inport: '<Root>/MBeta'

  rtDW.ss1.z = rtU.MBeta[0];
  if (rtIsNaN(rtDW.ss1.x)) {
    rtDW.ss1.x = inv(rtDW.ss1.H) * rtDW.ss1.z;
    x_dot = inv(rtDW.ss1.H);
    rtDW.ss1.P = x_dot * rtDW.ss1.R * x_dot;
  } else {
    rtDW.ss1.x = rtDW.ss1.A * rtDW.ss1.x + rtDW.ss1.B * rtDW.ss1.u;
    rtDW.ss1.P = rtDW.ss1.A * rtDW.ss1.P * rtDW.ss1.A + rtDW.ss1.Q;
    rtb_Memory_l = rtDW.ss1.H * rtDW.ss1.P;
    x_dot = inv(rtb_Memory_l * rtDW.ss1.H + rtDW.ss1.R) * rtb_Memory_l;
    rtDW.ss1.x += (rtDW.ss1.z - rtDW.ss1.H * rtDW.ss1.x) * x_dot;
    rtDW.ss1.P -= x_dot * rtDW.ss1.H * rtDW.ss1.P;
  }

  rtDW.ss2.z = rtU.MBeta[1];
  if (rtIsNaN(rtDW.ss2.x)) {
    rtDW.ss2.x = inv(rtDW.ss2.H) * rtDW.ss2.z;
    x_dot = inv(rtDW.ss2.H);
    rtDW.ss2.P = x_dot * rtDW.ss2.R * x_dot;
  } else {
    rtDW.ss2.x = rtDW.ss2.A * rtDW.ss2.x + rtDW.ss2.B * rtDW.ss2.u;
    rtDW.ss2.P = rtDW.ss2.A * rtDW.ss2.P * rtDW.ss2.A + rtDW.ss2.Q;
    rtb_Memory_l = rtDW.ss2.H * rtDW.ss2.P;
    x_dot = inv(rtb_Memory_l * rtDW.ss2.H + rtDW.ss2.R) * rtb_Memory_l;
    rtDW.ss2.x += (rtDW.ss2.z - rtDW.ss2.H * rtDW.ss2.x) * x_dot;
    rtDW.ss2.P -= x_dot * rtDW.ss2.H * rtDW.ss2.P;
  }

  rtDW.ss3.z = rtU.MBeta[2];
  if (rtIsNaN(rtDW.ss3.x)) {
    rtDW.ss3.x = inv(rtDW.ss3.H) * rtDW.ss3.z;
    x_dot = inv(rtDW.ss3.H);
    rtDW.ss3.P = x_dot * rtDW.ss3.R * x_dot;
  } else {
    rtDW.ss3.x = rtDW.ss3.A * rtDW.ss3.x + rtDW.ss3.B * rtDW.ss3.u;
    rtDW.ss3.P = rtDW.ss3.A * rtDW.ss3.P * rtDW.ss3.A + rtDW.ss3.Q;
    rtb_Memory_l = rtDW.ss3.H * rtDW.ss3.P;
    x_dot = inv(rtb_Memory_l * rtDW.ss3.H + rtDW.ss3.R) * rtb_Memory_l;
    rtDW.ss3.x += (rtDW.ss3.z - rtDW.ss3.H * rtDW.ss3.x) * x_dot;
    rtDW.ss3.P -= x_dot * rtDW.ss3.H * rtDW.ss3.P;
  }

  rtDW.ss4.z = rtU.MBeta[3];
  if (rtIsNaN(rtDW.ss4.x)) {
    rtDW.ss4.x = inv(rtDW.ss4.H) * rtDW.ss4.z;
    x_dot = inv(rtDW.ss4.H);
    rtDW.ss4.P = x_dot * rtDW.ss4.R * x_dot;
  } else {
    rtDW.ss4.x = rtDW.ss4.A * rtDW.ss4.x + rtDW.ss4.B * rtDW.ss4.u;
    rtDW.ss4.P = rtDW.ss4.A * rtDW.ss4.P * rtDW.ss4.A + rtDW.ss4.Q;
    rtb_Memory_l = rtDW.ss4.H * rtDW.ss4.P;
    x_dot = inv(rtb_Memory_l * rtDW.ss4.H + rtDW.ss4.R) * rtb_Memory_l;
    rtDW.ss4.x += (rtDW.ss4.z - rtDW.ss4.H * rtDW.ss4.x) * x_dot;
    rtDW.ss4.P -= x_dot * rtDW.ss4.H * rtDW.ss4.P;
  }

  // MATLAB Function: '<S1>/MATLAB Function6' incorporates:
  //   Inport: '<Root>/phidot'

  rtDW.s1.z = rtU.phidot[0];
  if (rtIsNaN(rtDW.s1.x)) {
    rtDW.s1.x = inv(rtDW.s1.H) * rtDW.s1.z;
    x_dot = inv(rtDW.s1.H);
    rtDW.s1.P = x_dot * rtDW.s1.R * x_dot;
  } else {
    rtDW.s1.x = rtDW.s1.A * rtDW.s1.x + rtDW.s1.B * rtDW.s1.u;
    rtDW.s1.P = rtDW.s1.A * rtDW.s1.P * rtDW.s1.A + rtDW.s1.Q;
    rtb_Memory_l = rtDW.s1.H * rtDW.s1.P;
    x_dot = inv(rtb_Memory_l * rtDW.s1.H + rtDW.s1.R) * rtb_Memory_l;
    rtDW.s1.x += (rtDW.s1.z - rtDW.s1.H * rtDW.s1.x) * x_dot;
    rtDW.s1.P -= x_dot * rtDW.s1.H * rtDW.s1.P;
  }

  rtDW.s2.z = rtU.phidot[1];
  if (rtIsNaN(rtDW.s2.x)) {
    rtDW.s2.x = inv(rtDW.s2.H) * rtDW.s2.z;
    x_dot = inv(rtDW.s2.H);
    rtDW.s2.P = x_dot * rtDW.s2.R * x_dot;
  } else {
    rtDW.s2.x = rtDW.s2.A * rtDW.s2.x + rtDW.s2.B * rtDW.s2.u;
    rtDW.s2.P = rtDW.s2.A * rtDW.s2.P * rtDW.s2.A + rtDW.s2.Q;
    rtb_Memory_l = rtDW.s2.H * rtDW.s2.P;
    x_dot = inv(rtb_Memory_l * rtDW.s2.H + rtDW.s2.R) * rtb_Memory_l;
    rtDW.s2.x += (rtDW.s2.z - rtDW.s2.H * rtDW.s2.x) * x_dot;
    rtDW.s2.P -= x_dot * rtDW.s2.H * rtDW.s2.P;
  }

  rtDW.s3.z = rtU.phidot[2];
  if (rtIsNaN(rtDW.s3.x)) {
    rtDW.s3.x = inv(rtDW.s3.H) * rtDW.s3.z;
    x_dot = inv(rtDW.s3.H);
    rtDW.s3.P = x_dot * rtDW.s3.R * x_dot;
  } else {
    rtDW.s3.x = rtDW.s3.A * rtDW.s3.x + rtDW.s3.B * rtDW.s3.u;
    rtDW.s3.P = rtDW.s3.A * rtDW.s3.P * rtDW.s3.A + rtDW.s3.Q;
    rtb_Memory_l = rtDW.s3.H * rtDW.s3.P;
    x_dot = inv(rtb_Memory_l * rtDW.s3.H + rtDW.s3.R) * rtb_Memory_l;
    rtDW.s3.x += (rtDW.s3.z - rtDW.s3.H * rtDW.s3.x) * x_dot;
    rtDW.s3.P -= x_dot * rtDW.s3.H * rtDW.s3.P;
  }

  rtDW.s4.z = rtU.phidot[3];
  if (rtIsNaN(rtDW.s4.x)) {
    rtDW.s4.x = inv(rtDW.s4.H) * rtDW.s4.z;
    x_dot = inv(rtDW.s4.H);
    rtDW.s4.P = x_dot * rtDW.s4.R * x_dot;
  } else {
    rtDW.s4.x = rtDW.s4.A * rtDW.s4.x + rtDW.s4.B * rtDW.s4.u;
    rtDW.s4.P = rtDW.s4.A * rtDW.s4.P * rtDW.s4.A + rtDW.s4.Q;
    rtb_Memory_l = rtDW.s4.H * rtDW.s4.P;
    x_dot = inv(rtb_Memory_l * rtDW.s4.H + rtDW.s4.R) * rtb_Memory_l;
    rtDW.s4.x += (rtDW.s4.z - rtDW.s4.H * rtDW.s4.x) * x_dot;
    rtDW.s4.P -= x_dot * rtDW.s4.H * rtDW.s4.P;
  }

  // MATLAB Function: '<S1>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/sampleTime'
  //   MATLAB Function: '<S1>/MATLAB Function5'
  //   MATLAB Function: '<S1>/MATLAB Function6'

  x_dot = std::cos(rtDW.ss1.x);
  y_dot = std::sin(rtDW.ss1.x);
  F_hat[0] = x_dot;
  F_hat[1] = y_dot;
  F_hat[2] = -0.263 * x_dot + 0.263 * y_dot;
  x_dot = std::cos(rtDW.ss2.x);
  y_dot = std::sin(rtDW.ss2.x);
  F_hat[3] = x_dot;
  F_hat[4] = y_dot;
  F_hat[5] = 0.263 * x_dot + 0.263 * y_dot;
  x_dot = std::cos(rtDW.ss3.x);
  y_dot = std::sin(rtDW.ss3.x);
  F_hat[6] = x_dot;
  F_hat[7] = y_dot;
  F_hat[8] = 0.263 * x_dot + -0.263 * y_dot;
  x_dot = std::cos(rtDW.ss4.x);
  y_dot = std::sin(rtDW.ss4.x);
  F_hat[9] = x_dot;
  F_hat[10] = y_dot;
  F_hat[11] = -0.263 * x_dot + -0.263 * y_dot;
  for (r1 = 0; r1 < 3; r1++) {
    r2 = r1 << 2;
    b_F_hat[r2] = F_hat[r1];
    b_F_hat[r2 + 1] = F_hat[r1 + 3];
    b_F_hat[r2 + 2] = F_hat[r1 + 6];
    b_F_hat[r2 + 3] = F_hat[r1 + 9];
  }

  for (r1 = 0; r1 < 4; r1++) {
    A_tmp[3 * r1] = b_F_hat[r1];
    A_tmp[3 * r1 + 1] = b_F_hat[r1 + 4];
    A_tmp[3 * r1 + 2] = b_F_hat[r1 + 8];
  }

  for (r1 = 0; r1 < 3; r1++) {
    for (r2 = 0; r2 < 3; r2++) {
      r3 = r1 << 2;
      rtemp = 3 * r1 + r2;
      A[rtemp] = (((b_F_hat[r3 + 1] * A_tmp[r2 + 3] + b_F_hat[r3] * A_tmp[r2]) +
                   b_F_hat[r3 + 2] * A_tmp[r2 + 6]) + b_F_hat[r3 + 3] * A_tmp[r2
                  + 9]) + b_y[rtemp];
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  x_dot = std::abs(A[0]);
  y_dot = std::abs(A[1]);
  if (y_dot > x_dot) {
    x_dot = y_dot;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(A[2]) > x_dot) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] /= A[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  F_hat[1] = A_tmp[r2] - A_tmp[r1] * A[r2];
  rtb_Memory_l = A[r3 + 3];
  F_hat[2] = (A_tmp[r3] - A_tmp[r1] * A[r3]) - rtb_Memory_l * F_hat[1];
  Accx = A[r3 + 6];
  F_hat[2] /= Accx;
  x_dot = A[r1 + 6];
  F_hat[0] = A_tmp[r1] - x_dot * F_hat[2];
  y_dot = A[r2 + 6];
  F_hat[1] -= y_dot * F_hat[2];
  theta_dot = A[r2 + 3];
  F_hat[1] /= theta_dot;
  Accy = A[r1 + 3];
  F_hat[0] -= Accy * F_hat[1];
  F_hat[0] /= A[r1];
  omegaDotDotN = A_tmp[r1 + 3];
  F_hat[4] = A_tmp[r2 + 3] - omegaDotDotN * A[r2];
  F_hat[5] = (A_tmp[r3 + 3] - omegaDotDotN * A[r3]) - rtb_Memory_l * F_hat[4];
  F_hat[5] /= Accx;
  F_hat[3] = omegaDotDotN - x_dot * F_hat[5];
  F_hat[4] -= y_dot * F_hat[5];
  F_hat[4] /= theta_dot;
  F_hat[3] -= Accy * F_hat[4];
  F_hat[3] /= A[r1];
  omegaDotDotN = A_tmp[r1 + 6];
  F_hat[7] = A_tmp[r2 + 6] - omegaDotDotN * A[r2];
  F_hat[8] = (A_tmp[r3 + 6] - omegaDotDotN * A[r3]) - rtb_Memory_l * F_hat[7];
  F_hat[8] /= Accx;
  F_hat[6] = omegaDotDotN - x_dot * F_hat[8];
  F_hat[7] -= y_dot * F_hat[8];
  F_hat[7] /= theta_dot;
  F_hat[6] -= Accy * F_hat[7];
  F_hat[6] /= A[r1];
  omegaDotDotN = A_tmp[r1 + 9];
  F_hat[10] = A_tmp[r2 + 9] - omegaDotDotN * A[r2];
  F_hat[11] = (A_tmp[r3 + 9] - omegaDotDotN * A[r3]) - rtb_Memory_l * F_hat[10];
  F_hat[11] /= Accx;
  F_hat[9] = omegaDotDotN - x_dot * F_hat[11];
  F_hat[10] -= y_dot * F_hat[11];
  F_hat[10] /= theta_dot;
  F_hat[9] -= Accy * F_hat[10];
  F_hat[9] /= A[r1];
  ZetaDot_hat_tmp = std::sin(rtDW.Zeta_hatN_f[2]);
  ZetaDot_hat_tmp_0 = std::cos(rtDW.Zeta_hatN_f[2]);
  ZetaDot_hat_tmp_1[0] = ZetaDot_hat_tmp_0;
  ZetaDot_hat_tmp_1[3] = -ZetaDot_hat_tmp;
  ZetaDot_hat_tmp_1[6] = 0.0F;
  ZetaDot_hat_tmp_1[1] = ZetaDot_hat_tmp;
  ZetaDot_hat_tmp_1[4] = ZetaDot_hat_tmp_0;
  ZetaDot_hat_tmp_1[7] = 0.0F;
  for (r1 = 0; r1 < 3; r1++) {
    rtb_Memory_l = F_hat[r1 + 9] * 0.084 * rtDW.s4.x + (F_hat[r1 + 6] * 0.084 *
      rtDW.s3.x + (F_hat[r1 + 3] * 0.084 * rtDW.s2.x + 0.084 * F_hat[r1] *
                   rtDW.s1.x));
    ZetaDot_hat_tmp_1[3 * r1 + 2] = b_0[r1];
    Zetadoti[r1] = rtb_Memory_l;
  }

  for (r1 = 0; r1 < 3; r1++) {
    ZetaDot_hat[r1] = ZetaDot_hat_tmp_1[r1 + 6] * static_cast<real32_T>
      (Zetadoti[2]) + (ZetaDot_hat_tmp_1[r1 + 3] * static_cast<real32_T>
                       (Zetadoti[1]) + ZetaDot_hat_tmp_1[r1] *
                       static_cast<real32_T>(Zetadoti[0]));
  }

  rtDW.posex_c += ZetaDot_hat[0] * static_cast<real32_T>(rtU.sampleTime);
  rtDW.posey_l += ZetaDot_hat[1] * static_cast<real32_T>(rtU.sampleTime);
  rtDW.theta_e += ZetaDot_hat[2] * static_cast<real32_T>(rtU.sampleTime);
  rtDW.Zeta_hatN_f[0] = rtDW.posex_c;
  rtDW.Zeta_hatN_f[1] = rtDW.posey_l;
  rtDW.Zeta_hatN_f[2] = rtDW.theta_e;

  // MATLAB Function: '<S1>/MATLAB Function2' incorporates:
  //   Inport: '<Root>/sampleTime'

  x_dot = std::cos(rtb_Memory[0]);
  y_dot = std::sin(rtb_Memory[0]);
  F_hat[0] = x_dot;
  F_hat[1] = y_dot;
  F_hat[2] = -0.263 * x_dot + 0.263 * y_dot;
  x_dot = std::cos(rtb_Memory[1]);
  y_dot = std::sin(rtb_Memory[1]);
  F_hat[3] = x_dot;
  F_hat[4] = y_dot;
  F_hat[5] = 0.263 * x_dot + 0.263 * y_dot;
  x_dot = std::cos(rtb_Memory[2]);
  y_dot = std::sin(rtb_Memory[2]);
  F_hat[6] = x_dot;
  F_hat[7] = y_dot;
  F_hat[8] = 0.263 * x_dot + -0.263 * y_dot;
  x_dot = std::cos(rtb_Memory[3]);
  y_dot = std::sin(rtb_Memory[3]);
  F_hat[9] = x_dot;
  F_hat[10] = y_dot;
  F_hat[11] = -0.263 * x_dot + -0.263 * y_dot;
  for (r1 = 0; r1 < 3; r1++) {
    r2 = r1 << 2;
    b_F_hat[r2] = F_hat[r1];
    b_F_hat[r2 + 1] = F_hat[r1 + 3];
    b_F_hat[r2 + 2] = F_hat[r1 + 6];
    b_F_hat[r2 + 3] = F_hat[r1 + 9];
  }

  for (r1 = 0; r1 < 4; r1++) {
    A_tmp[3 * r1] = b_F_hat[r1];
    A_tmp[3 * r1 + 1] = b_F_hat[r1 + 4];
    A_tmp[3 * r1 + 2] = b_F_hat[r1 + 8];
  }

  for (r1 = 0; r1 < 3; r1++) {
    for (r2 = 0; r2 < 3; r2++) {
      r3 = r1 << 2;
      rtemp = 3 * r1 + r2;
      A[rtemp] = (((b_F_hat[r3 + 1] * A_tmp[r2 + 3] + b_F_hat[r3] * A_tmp[r2]) +
                   b_F_hat[r3 + 2] * A_tmp[r2 + 6]) + b_F_hat[r3 + 3] * A_tmp[r2
                  + 9]) + b_y[rtemp];
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  x_dot = std::abs(A[0]);
  y_dot = std::abs(A[1]);
  if (y_dot > x_dot) {
    x_dot = y_dot;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(A[2]) > x_dot) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] /= A[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  F_hat[1] = A_tmp[r2] - A_tmp[r1] * A[r2];
  rtb_Memory_l = A[r3 + 3];
  F_hat[2] = (A_tmp[r3] - A_tmp[r1] * A[r3]) - rtb_Memory_l * F_hat[1];
  Accx = A[r3 + 6];
  F_hat[2] /= Accx;
  x_dot = A[r1 + 6];
  F_hat[0] = A_tmp[r1] - x_dot * F_hat[2];
  y_dot = A[r2 + 6];
  F_hat[1] -= y_dot * F_hat[2];
  theta_dot = A[r2 + 3];
  F_hat[1] /= theta_dot;
  Accy = A[r1 + 3];
  F_hat[0] -= Accy * F_hat[1];
  F_hat[0] /= A[r1];
  omegaDotDotN = A_tmp[r1 + 3];
  F_hat[4] = A_tmp[r2 + 3] - omegaDotDotN * A[r2];
  F_hat[5] = (A_tmp[r3 + 3] - omegaDotDotN * A[r3]) - rtb_Memory_l * F_hat[4];
  F_hat[5] /= Accx;
  F_hat[3] = omegaDotDotN - x_dot * F_hat[5];
  F_hat[4] -= y_dot * F_hat[5];
  F_hat[4] /= theta_dot;
  F_hat[3] -= Accy * F_hat[4];
  F_hat[3] /= A[r1];
  omegaDotDotN = A_tmp[r1 + 6];
  F_hat[7] = A_tmp[r2 + 6] - omegaDotDotN * A[r2];
  F_hat[8] = (A_tmp[r3 + 6] - omegaDotDotN * A[r3]) - rtb_Memory_l * F_hat[7];
  F_hat[8] /= Accx;
  F_hat[6] = omegaDotDotN - x_dot * F_hat[8];
  F_hat[7] -= y_dot * F_hat[8];
  F_hat[7] /= theta_dot;
  F_hat[6] -= Accy * F_hat[7];
  F_hat[6] /= A[r1];
  omegaDotDotN = A_tmp[r1 + 9];
  F_hat[10] = A_tmp[r2 + 9] - omegaDotDotN * A[r2];
  F_hat[11] = (A_tmp[r3 + 9] - omegaDotDotN * A[r3]) - rtb_Memory_l * F_hat[10];
  F_hat[11] /= Accx;
  F_hat[9] = omegaDotDotN - x_dot * F_hat[11];
  F_hat[10] -= y_dot * F_hat[11];
  F_hat[10] /= theta_dot;
  F_hat[9] -= Accy * F_hat[10];
  F_hat[9] /= A[r1];
  ZetaDot_hat_tmp = std::sin(rtDW.Zeta_hatN[2]);
  ZetaDot_hat_tmp_0 = std::cos(rtDW.Zeta_hatN[2]);
  ZetaDot_hat_tmp_1[0] = ZetaDot_hat_tmp_0;
  ZetaDot_hat_tmp_1[3] = -ZetaDot_hat_tmp;
  ZetaDot_hat_tmp_1[6] = 0.0F;
  ZetaDot_hat_tmp_1[1] = ZetaDot_hat_tmp;
  ZetaDot_hat_tmp_1[4] = ZetaDot_hat_tmp_0;
  ZetaDot_hat_tmp_1[7] = 0.0F;
  for (r1 = 0; r1 < 3; r1++) {
    rtb_Memory_l = F_hat[r1 + 9] * 0.084 * rtb_Phi_dotN[3] + (F_hat[r1 + 6] *
      0.084 * rtb_Phi_dotN[2] + (F_hat[r1 + 3] * 0.084 * rtb_Phi_dotN[1] + 0.084
      * F_hat[r1] * rtb_Phi_dotN[0]));
    ZetaDot_hat_tmp_1[3 * r1 + 2] = b_0[r1];
    Zetadotdoti[r1] = rtb_Memory_l;
  }

  for (r1 = 0; r1 < 3; r1++) {
    ZetaDot_hat[r1] = ZetaDot_hat_tmp_1[r1 + 6] * static_cast<real32_T>
      (Zetadotdoti[2]) + (ZetaDot_hat_tmp_1[r1 + 3] * static_cast<real32_T>
                          (Zetadotdoti[1]) + ZetaDot_hat_tmp_1[r1] *
                          static_cast<real32_T>(Zetadotdoti[0]));
  }

  rtDW.posex += ZetaDot_hat[0] * static_cast<real32_T>(rtU.sampleTime);
  rtDW.posey += ZetaDot_hat[1] * static_cast<real32_T>(rtU.sampleTime);
  rtDW.theta += ZetaDot_hat[2] * static_cast<real32_T>(rtU.sampleTime);
  rtDW.Zeta_hatN[0] = rtDW.posex;
  rtDW.Zeta_hatN[1] = rtDW.posey;
  rtDW.Zeta_hatN[2] = rtDW.theta;

  // Outport: '<Root>/Vel' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function1'

  rtY.Vel[0] = Zetadoti[0];
  rtY.Vel[1] = Zetadoti[1];
  rtY.Vel[2] = Zetadoti[2];

  // Outport: '<Root>/Pose' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function1'

  rtY.Pose[0] = rtDW.posex_c;
  rtY.Pose[1] = rtDW.posey_l;
  rtY.Pose[2] = rtDW.theta_e;

  // Outport: '<Root>/VelN' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function2'

  rtY.VelN[0] = Zetadotdoti[0];
  rtY.VelN[1] = Zetadotdoti[1];
  rtY.VelN[2] = Zetadotdoti[2];

  // Outport: '<Root>/PoseN' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function2'

  rtY.PoseN[0] = rtDW.posex;
  rtY.PoseN[1] = rtDW.posey;
  rtY.PoseN[2] = rtDW.theta;

  // Outputs for Atomic SubSystem: '<S1>/Inverse Kinematics'
  // MATLAB Function: '<S2>/computeBeta'
  F_hat[0] = std::cos(rtb_beta[0]);
  F_hat[4] = std::sin(rtb_beta[0]);
  F_hat[8] = std::sin(rtb_beta[0] - 0.78539816339744828) * 0.371938166904124;
  F_hat[1] = std::cos(rtb_beta[1]);
  F_hat[5] = std::sin(rtb_beta[1]);
  F_hat[9] = std::sin(rtb_beta[1] - 5.497787143782138) * 0.371938166904124;
  F_hat[2] = std::cos(rtb_beta[2]);
  F_hat[6] = std::sin(rtb_beta[2]);
  F_hat[10] = std::sin(rtb_beta[2] - 3.9269908169872414) * 0.371938166904124;
  F_hat[3] = std::cos(rtb_beta_o);
  F_hat[7] = std::sin(rtb_beta_o);
  F_hat[11] = std::sin(rtb_beta_o - 2.3561944901923448) * 0.371938166904124;

  // End of Outputs for SubSystem: '<S1>/Inverse Kinematics'
  // End of Outputs for SubSystem: '<Root>/Subsystem'
  for (r1 = 0; r1 < 4; r1++) {
    // Outport: '<Root>/BetaJ '
    rtY.BetaJ[r1] = rtb_beta[r1];

    // Outport: '<Root>/Phidot'
    rtY.Phidot[r1] = rtb_Phi_dotN[r1];

    // Outputs for Atomic SubSystem: '<Root>/Subsystem'
    // Outputs for Atomic SubSystem: '<S1>/Inverse Kinematics'
    // Outport: '<Root>/PhidotJ' incorporates:
    //   Inport: '<Root>/Ref'
    //   MATLAB Function: '<S2>/computeBeta'

    rtY.PhidotJ[r1] = ((F_hat[r1 + 4] * rtU.Ref[1] + F_hat[r1] * rtU.Ref[0]) +
                       F_hat[r1 + 8] * rtU.Ref[2]) / 0.084;

    // End of Outputs for SubSystem: '<S1>/Inverse Kinematics'
    // End of Outputs for SubSystem: '<Root>/Subsystem'

    // Outport: '<Root>/Beta'
    rtY.Beta[r1] = rtb_Memory[r1];
  }
}

// Model initialize function
void SubsystemModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // SystemInitialize for Atomic SubSystem: '<Root>/Subsystem'
  // SystemInitialize for MATLAB Function: '<S1>/MATLAB Function5'
  rtDW.ss1.x = 0.0;
  rtDW.ss1.P = 1.0;
  rtDW.ss1.A = 1.0;
  rtDW.ss1.Q = 4.0;
  rtDW.ss1.H = 1.0;
  rtDW.ss1.R = 100.0;
  rtDW.ss1.B = 0.0;
  rtDW.ss1.u = 0.0;
  rtDW.ss2.x = 0.0;
  rtDW.ss2.P = 1.0;
  rtDW.ss2.A = 1.0;
  rtDW.ss2.Q = 4.0;
  rtDW.ss2.H = 1.0;
  rtDW.ss2.R = 100.0;
  rtDW.ss2.B = 0.0;
  rtDW.ss2.u = 0.0;
  rtDW.ss3.x = 0.0;
  rtDW.ss3.P = 1.0;
  rtDW.ss3.A = 1.0;
  rtDW.ss3.Q = 4.0;
  rtDW.ss3.H = 1.0;
  rtDW.ss3.R = 100.0;
  rtDW.ss3.B = 0.0;
  rtDW.ss3.u = 0.0;
  rtDW.ss4.x = 0.0;
  rtDW.ss4.P = 1.0;
  rtDW.ss4.A = 1.0;
  rtDW.ss4.Q = 4.0;
  rtDW.ss4.H = 1.0;
  rtDW.ss4.R = 100.0;
  rtDW.ss4.B = 0.0;
  rtDW.ss4.u = 0.0;

  // SystemInitialize for MATLAB Function: '<S1>/MATLAB Function6'
  rtDW.s1.x = 0.0;
  rtDW.s1.P = 1.0;
  rtDW.s1.A = 1.0;
  rtDW.s1.Q = 4.0;
  rtDW.s1.H = 1.0;
  rtDW.s1.R = 100.0;
  rtDW.s1.B = 0.0;
  rtDW.s1.u = 0.0;
  rtDW.s2.x = 0.0;
  rtDW.s2.P = 1.0;
  rtDW.s2.A = 1.0;
  rtDW.s2.Q = 4.0;
  rtDW.s2.H = 1.0;
  rtDW.s2.R = 100.0;
  rtDW.s2.B = 0.0;
  rtDW.s2.u = 0.0;
  rtDW.s3.x = 0.0;
  rtDW.s3.P = 1.0;
  rtDW.s3.A = 1.0;
  rtDW.s3.Q = 4.0;
  rtDW.s3.H = 1.0;
  rtDW.s3.R = 100.0;
  rtDW.s3.B = 0.0;
  rtDW.s3.u = 0.0;
  rtDW.s4.x = 0.0;
  rtDW.s4.P = 1.0;
  rtDW.s4.A = 1.0;
  rtDW.s4.Q = 4.0;
  rtDW.s4.H = 1.0;
  rtDW.s4.R = 100.0;
  rtDW.s4.B = 0.0;
  rtDW.s4.u = 0.0;

  // End of SystemInitialize for SubSystem: '<Root>/Subsystem'
}

// Constructor
SubsystemModelClass::SubsystemModelClass() : rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
SubsystemModelClass::~SubsystemModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL * SubsystemModelClass::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
