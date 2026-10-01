/**
  * @file    pid.c
  * @author  Leonardo Costamagna
  * @brief   Industrial PID Controller module implementation.
  */

#include "pid.h"
#include <stddef.h>

/* Core PID Operations */

int32_t pid_init(pid_controller_t *pid,
                 float Kp,
                 float Ki,
                 float Kd,
                 float N,
                 float dt,
                 float out_min,
                 float out_max,
                 bool enable_windup,
                 float setpoint_rate)
{
  if (pid == NULL)
  {
    return -1;
  }

  pid->Kp                 = Kp;
  pid->Ki                 = Ki;
  pid->Kd                 = Kd;
  pid->N                  = N;
  pid->dt                 = dt;
  pid->out_min            = out_min;
  pid->out_max            = out_max;
  pid->anti_windup_enable = enable_windup;
  pid->setpoint_rate      = setpoint_rate;

  pid->setpoint_target  = 0.0f;
  pid->setpoint_current = 0.0f;

  return pid_reset(pid);
}

int32_t pid_reset(pid_controller_t *pid)
{
  if (pid == NULL)
  {
    return -1;
  }

  pid->integral         = 0.0f;
  pid->last_measurement = 0.0f;
  pid->derivative_filt  = 0.0f;

  return 0;
}

float pid_compute(pid_controller_t *pid, float setpoint, float measurement)
{
  float max_step;
  float error_sp;
  float error;
  float proportional;
  float integral_unclamped;
  float integral_term;
  float derivative_raw;
  float derivative_term;
  float output;
  bool saturated;

  if (pid == NULL)
  {
    return 0.0f;
  }

  /* 0. INTELLIGENT SETPOINT RAMP MANAGEMENT */
  /* Detect if the user changed the target setpoint */
  if (setpoint != pid->setpoint_target)
  {
    pid->setpoint_target = setpoint;

    /* If the ramp rate is 0 or negative, bypass the ramp and update directly */
    if (pid->setpoint_rate <= 0.0f)
    {
      pid->setpoint_current = setpoint;
    }
  }

  /* Advance the ramp ONLY if the current setpoint has not yet reached the target */
  if (pid->setpoint_current != pid->setpoint_target)
  {
    max_step = pid->setpoint_rate * pid->dt;
    error_sp = pid->setpoint_target - pid->setpoint_current;

    if (error_sp > max_step)
    {
      pid->setpoint_current += max_step;
    }
    else if (error_sp < -max_step)
    {
      pid->setpoint_current -= max_step;
    }
    else
    {
      /* Fully stabilized at the target value */
      pid->setpoint_current = pid->setpoint_target;
    }
  }

  /* 1. Calculate current error using the smoothed/ramped setpoint */
  error = pid->setpoint_current - measurement;

  /* 2. Proportional Term */
  proportional = pid->Kp * error;

  /* 3. Integral Term (Forward rectangular method) */
  integral_unclamped = pid->integral + (error * pid->dt);
  integral_term      = pid->Ki * integral_unclamped;

  /* 4. Derivative Term with low-pass filter and "Derivative Kick" mitigation */
  derivative_raw = -(measurement - pid->last_measurement) / pid->dt;

  /* Discrete low-pass filter: y[k] = y[k-1] + dt * N * (x[k] - y[k-1]) */
  pid->derivative_filt += pid->dt * pid->N * (derivative_raw - pid->derivative_filt);
  derivative_term       = pid->Kd * pid->derivative_filt;

  /* 5. Potential output (unclamped) */
  output = proportional + integral_term + derivative_term;

  /* 6. Output saturation protection */
  saturated = false;
  if (output > pid->out_max)
  {
    output    = pid->out_max;
    saturated = true;
  }
  else if (output < pid->out_min)
  {
    output    = pid->out_min;
    saturated = true;
  }

  /* Anti-Windup via Dynamic Clamping */
  if (pid->anti_windup_enable && saturated)
  {
    /* Block integration if the current error pushes the output further beyond limits */
    if (((error > 0.0f) && (output >= pid->out_max)) ||
        ((error < 0.0f) && (output <= pid->out_min)))
    {
      /* Discard the new calculation, keep the previous integral in the output term */
      integral_term = pid->Ki * pid->integral;
    }
    else
    {
      pid->integral = integral_unclamped;
    }
  }
  else
  {
    pid->integral = integral_unclamped;
  }

  /* Save measurement history for the next iteration */
  pid->last_measurement = measurement;

  return output;
}