/*
 * File: roll_loop.h
 *
 * Code generated for Simulink model 'roll_loop'.
 *
 * Model version                  : 1.38
 * Simulink Coder version         : 8.4 (R2013a) 13-Feb-2013
 * TLC version                    : 8.4 (Jan 18 2013)
 * C/C++ source code generated on : Sat Sep 17 18:31:27 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->ST10/Super10
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#ifndef RTW_HEADER_roll_loop_h_
#define RTW_HEADER_roll_loop_h_
#include "rtwtypes.h"
#ifndef roll_loop_COMMON_INCLUDES_
# define roll_loop_COMMON_INCLUDES_
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#endif                                 /* roll_loop_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_roll_loop_T RT_MODEL_roll_loop_T;

#ifndef SS_LONG
#define SS_LONG                        14
#endif

#ifndef SS_ULONG
#define SS_ULONG                       15
#endif

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T roll_setpoint;              /* '<Root>/roll_setpoint' */
  real32_T roll;                       /* '<Root>/roll' */
  real32_T roll_rate;                  /* '<Root>/roll_rate' */
} ExtU_roll_loop_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real32_T cmd;                        /* '<Root>/cmd' */
} ExtY_roll_loop_T;

/* Parameters (auto storage) */
struct P_roll_loop_T_ {
  real32_T P1_Gain;                    /* Computed Parameter: P1_Gain
                                        * Referenced by: '<Root>/P1'
                                        */
  real32_T D1_Gain;                    /* Computed Parameter: D1_Gain
                                        * Referenced by: '<Root>/D1'
                                        */
  real32_T Bound_UpperSat;             /* Computed Parameter: Bound_UpperSat
                                        * Referenced by: '<Root>/Bound'
                                        */
  real32_T Bound_LowerSat;             /* Computed Parameter: Bound_LowerSat
                                        * Referenced by: '<Root>/Bound'
                                        */
};

/* Parameters (auto storage) */
typedef struct P_roll_loop_T_ P_roll_loop_T;

/* Real-time Model Data Structure */
struct tag_RTM_roll_loop_T {
  const char_T *errorStatus;
};

/* Block parameters (auto storage) */
extern P_roll_loop_T roll_loop_P;

/* External inputs (root inport signals with auto storage) */
extern ExtU_roll_loop_T roll_loop_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_roll_loop_T roll_loop_Y;

/* Model entry point functions */
void roll_loop_initialize(void);
void roll_loop_step(void);

/* Real-time Model object */
extern RT_MODEL_roll_loop_T *const roll_loop_M;

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
 * '<Root>' : 'roll_loop'
 */
#endif                                 /* RTW_HEADER_roll_loop_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
