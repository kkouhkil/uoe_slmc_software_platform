//
// File: Subsystem.h
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
#ifndef RTW_HEADER_Subsystem_h_
#define RTW_HEADER_Subsystem_h_
#include <stddef.h>
#include "rtwtypes.h"
#include <cmath>
#ifndef Subsystem_COMMON_INCLUDES_
# define Subsystem_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // Subsystem_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Custom Type definition for MATLAB Function: '<S1>/MATLAB Function6'
#ifndef struct_tag_s3sjEYxYkUW3rOZt9knTmLB
#define struct_tag_s3sjEYxYkUW3rOZt9knTmLB

struct tag_s3sjEYxYkUW3rOZt9knTmLB
{
  real_T x;
  real_T P;
  real_T A;
  real_T Q;
  real_T H;
  real_T R;
  real_T B;
  real_T u;
  real_T z;
};

#endif                                 //struct_tag_s3sjEYxYkUW3rOZt9knTmLB

#ifndef typedef_s3sjEYxYkUW3rOZt9knTmLB
#define typedef_s3sjEYxYkUW3rOZt9knTmLB

typedef struct tag_s3sjEYxYkUW3rOZt9knTmLB s3sjEYxYkUW3rOZt9knTmLB;

#endif                                 //typedef_s3sjEYxYkUW3rOZt9knTmLB

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  s3sjEYxYkUW3rOZt9knTmLB ss1;         // '<S1>/MATLAB Function5'
  s3sjEYxYkUW3rOZt9knTmLB ss2;         // '<S1>/MATLAB Function5'
  s3sjEYxYkUW3rOZt9knTmLB ss3;         // '<S1>/MATLAB Function5'
  s3sjEYxYkUW3rOZt9knTmLB ss4;         // '<S1>/MATLAB Function5'
  s3sjEYxYkUW3rOZt9knTmLB s1;          // '<S1>/MATLAB Function6'
  s3sjEYxYkUW3rOZt9knTmLB s2;          // '<S1>/MATLAB Function6'
  s3sjEYxYkUW3rOZt9knTmLB s3;          // '<S1>/MATLAB Function6'
  s3sjEYxYkUW3rOZt9knTmLB s4;          // '<S1>/MATLAB Function6'
  real_T previousBeta[4];              // '<S1>/MATLAB Function'
  real_T Memory_PreviousInput[4];      // '<S2>/Memory'
  real_T VxO;                          // '<S1>/MATLAB Function'
  real_T VyO;                          // '<S1>/MATLAB Function'
  real_T omegaDotO;                    // '<S1>/MATLAB Function'
  real32_T Zeta_hatN[3];               // '<S1>/MATLAB Function2'
  real32_T Zeta_hatN_f[3];             // '<S1>/MATLAB Function1'
  real32_T posex;                      // '<S1>/MATLAB Function2'
  real32_T posey;                      // '<S1>/MATLAB Function2'
  real32_T theta;                      // '<S1>/MATLAB Function2'
  real32_T posex_c;                    // '<S1>/MATLAB Function1'
  real32_T posey_l;                    // '<S1>/MATLAB Function1'
  real32_T theta_e;                    // '<S1>/MATLAB Function1'
} DW;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T Ref[3];                       // '<Root>/Ref'
  real_T MBeta[4];                     // '<Root>/MBeta'
  real_T phidot[4];                    // '<Root>/phidot'
  real_T sampleTime;                   // '<Root>/sampleTime'
  real_T reset_sig;                    // '<Root>/reset_sig'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T BetaJ[4];                     // '<Root>/BetaJ '
  real_T Phidot[4];                    // '<Root>/Phidot'
  real_T Vel[3];                       // '<Root>/Vel'
  real32_T Pose[3];                    // '<Root>/Pose'
  real_T VelN[3];                      // '<Root>/VelN'
  real32_T PoseN[3];                   // '<Root>/PoseN'
  real_T PhidotJ[4];                   // '<Root>/PhidotJ'
  real_T Beta[4];                      // '<Root>/Beta'
} ExtY;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T * volatile errorStatus;
};

// Class declaration for model Subsystem
class SubsystemModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  SubsystemModelClass();

  // Destructor
  ~SubsystemModelClass();

  // Real-Time Model get method
  RT_MODEL * getRTM();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;

  // Real-Time Model
  RT_MODEL rtM;

  // private member function(s) for subsystem '<Root>'
  real_T inv(real_T x);
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('kinematic_model_ros/Subsystem')    - opens subsystem kinematic_model_ros/Subsystem
//  hilite_system('kinematic_model_ros/Subsystem/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'kinematic_model_ros'
//  '<S1>'   : 'kinematic_model_ros/Subsystem'
//  '<S2>'   : 'kinematic_model_ros/Subsystem/Inverse Kinematics'
//  '<S3>'   : 'kinematic_model_ros/Subsystem/MATLAB Function'
//  '<S4>'   : 'kinematic_model_ros/Subsystem/MATLAB Function1'
//  '<S5>'   : 'kinematic_model_ros/Subsystem/MATLAB Function2'
//  '<S6>'   : 'kinematic_model_ros/Subsystem/MATLAB Function5'
//  '<S7>'   : 'kinematic_model_ros/Subsystem/MATLAB Function6'
//  '<S8>'   : 'kinematic_model_ros/Subsystem/Inverse Kinematics/computeBeta'

#endif                                 // RTW_HEADER_Subsystem_h_

//
// File trailer for generated code.
//
// [EOF]
//
