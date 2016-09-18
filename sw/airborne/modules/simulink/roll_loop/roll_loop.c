/*
 * File: roll_loop.c
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

#include "roll_loop.h"

/* Block parameters (auto storage) */
P_roll_loop_T roll_loop_P = {
  9000.0F,                             /* Computed Parameter: P1_Gain
                                        * Referenced by: '<Root>/P1'
                                        */
  1600.0F,                             /* Computed Parameter: D1_Gain
                                        * Referenced by: '<Root>/D1'
                                        */
  9600.0F,                             /* Computed Parameter: Bound_UpperSat
                                        * Referenced by: '<Root>/Bound'
                                        */
  -9600.0F                             /* Computed Parameter: Bound_LowerSat
                                        * Referenced by: '<Root>/Bound'
                                        */
};

/* External inputs (root inport signals with auto storage) */
ExtU_roll_loop_T roll_loop_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_roll_loop_T roll_loop_Y;

/* Real-time model */
RT_MODEL_roll_loop_T roll_loop_M_;
RT_MODEL_roll_loop_T *const roll_loop_M = &roll_loop_M_;

/* Model step function */
void roll_loop_step(void)
{
  real32_T u;

  /* Sum: '<Root>/Sum6' incorporates:
   *  Gain: '<Root>/D1'
   *  Gain: '<Root>/P1'
   *  Inport: '<Root>/roll'
   *  Inport: '<Root>/roll_rate'
   *  Inport: '<Root>/roll_setpoint'
   *  Sum: '<Root>/Sum5'
   */
  u = (roll_loop_U.roll - roll_loop_U.roll_setpoint) * roll_loop_P.P1_Gain +
    roll_loop_P.D1_Gain * roll_loop_U.roll_rate;

  /* Saturate: '<Root>/Bound' */
  if (u >= roll_loop_P.Bound_UpperSat) {
    /* Outport: '<Root>/cmd' */
    roll_loop_Y.cmd = roll_loop_P.Bound_UpperSat;
  } else if (u <= roll_loop_P.Bound_LowerSat) {
    /* Outport: '<Root>/cmd' */
    roll_loop_Y.cmd = roll_loop_P.Bound_LowerSat;
  } else {
    /* Outport: '<Root>/cmd' */
    roll_loop_Y.cmd = u;
  }

  /* End of Saturate: '<Root>/Bound' */
}

/* Model initialize function */
void roll_loop_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(roll_loop_M, (NULL));

  /* external inputs */
  (void) memset((void *)&roll_loop_U, 0,
                sizeof(ExtU_roll_loop_T));

  /* external outputs */
  roll_loop_Y.cmd = 0.0F;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
