/**
  * @file    pid.h
  * @author  Leonardo Costamagna
  * @brief   Header file containing API declarations, data types,
  *          and context structures for the Industrial PID Controller module.
  */

#ifndef PID_H
#define PID_H

/* Ensures C linkage compatibility when compiled with C++, preventing name mangling. */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/**
  * @defgroup PID_Driver Industrial PID Controller Module
  * @{
  */

/**
  * @defgroup PID_Types Data Types & Structures
  * @brief Core structures and state definitions for the PID controller.
  * @{
  */

/**
  * @brief PID Controller Configuration and State Context Structure
  */
typedef struct
{
  float Kp;                   /**< Proportional gain */
  float Ki;                   /**< Integral gain */
  float Kd;                   /**< Derivative gain */
  float N;                    /**< Derivative filter coefficient (typically between 5 and 20) */
  float dt;                   /**< Sampling period in seconds */
  float out_min;              /**< Minimum output saturation limit */
  float out_max;              /**< Maximum output saturation limit */
  float integral;             /**< Accumulated integral state */
  float last_measurement;     /**< Previous measurement for derivative kick mitigation */
  float derivative_filt;      /**< Low-pass filtered derivative state */
  bool  anti_windup_enable;   /**< Flag to enable/disable dynamic clamping anti-windup */
  float setpoint_target;      /**< Desired final setpoint (user input) */
  float setpoint_current;     /**< Current active setpoint (ramping value) */
  float setpoint_rate;        /**< Maximum rate of change (units/second). 0.0f to disable ramp */
} pid_controller_t;

/**
  * @} PID_Types
  */

/**
  * @defgroup PID_Core_API High-Level Application Interface
  * @brief Primary API function declarations for PID initialization, reset, and computation.
  * @{
  */

/**
  * @brief  Initializes the PID controller instance with tuning parameters and limits.
  * @param  pid            Pointer to the PID controller instance.
  * @param  Kp             Proportional gain.
  * @param  Ki             Integral gain.
  * @param  Kd             Derivative gain.
  * @param  N              Derivative filter coefficient.
  * @param  dt             Sampling period in seconds.
  * @param  out_min        Minimum output saturation limit.
  * @param  out_max        Maximum output saturation limit.
  * @param  enable_windup  Enable/disable anti-windup protection.
  * @param  setpoint_rate  Maximum setpoint rate of change (units/s).
  * @return Execution status (0: Success, <0: Invalid parameters).
  */
int32_t pid_init(pid_controller_t *pid,
                 float Kp,
                 float Ki,
                 float Kd,
                 float N,
                 float dt,
                 float out_min,
                 float out_max,
                 bool enable_windup,
                 float setpoint_rate);

/**
  * @brief  Resets the PID controller internal states (integral and filters).
  * @param  pid Pointer to the PID controller instance.
  * @return Execution status (0: Success, <0: Invalid parameters).
  */
int32_t pid_reset(pid_controller_t *pid);

/**
  * @brief  Computes the PID control output based on setpoint and process measurement.
  * @param  pid         Pointer to the PID controller instance.
  * @param  setpoint    Target setpoint value.
  * @param  measurement Current process variable measurement.
  * @return Calculated control output value (returns 0.0f if instance is NULL).
  */
float pid_compute(pid_controller_t *pid, float setpoint, float measurement);

/**
  * @} PID_Core_API
  */

/**
  * @} PID_Driver
  */

#ifdef __cplusplus
}
#endif

#endif /* PID_H */