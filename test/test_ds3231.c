/**
 * @file test_pid.c
 * @brief Unit tests for the Industrial PID Controller using Ceedling and Unity.
 */

#include "unity.h"
#include "pid.h"

static pid_controller_t pid;

void setUp(void) {
    /* Standard default initialization for individual test isolation */
    pid_init(&pid, 2.0f, 0.5f, 0.1f, 10.0f, 0.01f, -100.0f, 100.0f, true, 0.0f);
}

void tearDown(void) {
}

/* ============================================================================
   1. DEFENSIVE PROGRAMMING TESTS (NULL Pointers)
   ============================================================================ */

void test_pid_null_context_guard(void) {
    /* Verification of defensive guards against NULL instance pointers */
    TEST_ASSERT_EQUAL_INT32(-1, pid_init(NULL, 1.0f, 0.0f, 0.0f, 10.0f, 0.1f, -10.0f, 10.0f, true, 0.0f));
    TEST_ASSERT_EQUAL_INT32(-1, pid_reset(NULL));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_compute(NULL, 10.0f, 5.0f));
}

/* ============================================================================
   2. INITIALIZATION AND RESET OPERATIONS
   ============================================================================ */

void test_pid_init_and_reset_state(void) {
    /* Verify initialization sets parameters and resets internal states correctly */
    TEST_ASSERT_EQUAL_INT32(0, pid_init(&pid, 1.5f, 0.2f, 0.05f, 8.0f, 0.05f, -50.0f, 50.0f, true, 2.0f));
    
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.5f, pid.Kp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, pid.Ki);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, pid.Kd);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 8.0f, pid.N);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, pid.dt);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -50.0f, pid.out_min);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 50.0f, pid.out_max);
    TEST_ASSERT_TRUE(pid.anti_windup_enable);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.0f, pid.setpoint_rate);

    /* Verify states start clean */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.integral);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.last_measurement);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.derivative_filt);

    /* Simulate state accumulation and verify reset functionality */
    pid.integral = 15.5f;
    pid.last_measurement = 25.0f;
    pid.derivative_filt = -2.1f;

    TEST_ASSERT_EQUAL_INT32(0, pid_reset(&pid));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.integral);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.last_measurement);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.derivative_filt);
}

/* ============================================================================
   3. INDIVIDUAL P, I, D TERM COMPUTATIONS
   ============================================================================ */

void test_pid_pure_proportional_response(void) {
    /* Pure P-controller setup: Kp = 2.5, Ki = 0, Kd = 0 */
    pid_init(&pid, 2.5f, 0.0f, 0.0f, 10.0f, 0.1f, -100.0f, 100.0f, false, 0.0f);

    /* Setpoint = 10, Measurement = 4 -> Error = 6 -> Output = 2.5 * 6 = 15.0 */
    float output = pid_compute(&pid, 10.0f, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, output);
}

void test_pid_pure_integral_accumulation(void) {
    /* Pure I-controller setup: Kp = 0, Ki = 2.0, Kd = 0, dt = 0.1 */
    pid_init(&pid, 0.0f, 2.0f, 0.0f, 10.0f, 0.1f, -100.0f, 100.0f, false, 0.0f);

    /* Iteration 1: Error = 5.0 -> integral_unclamped = 0 + (5 * 0.1) = 0.5 -> Output = 2.0 * 0.5 = 1.0 */
    float out1 = pid_compute(&pid, 10.0f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, out1);

    /* Iteration 2: Error = 5.0 -> integral_unclamped = 0.5 + (5 * 0.1) = 1.0 -> Output = 2.0 * 1.0 = 2.0 */
    float out2 = pid_compute(&pid, 10.0f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out2);
}

void test_pid_derivative_kick_mitigation_and_filtering(void) {
    /* Pure D-controller setup: Kp = 0, Ki = 0, Kd = 1.0, N = 10, dt = 0.1 */
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 10.0f, 0.1f, -100.0f, 100.0f, false, 0.0f);

    /* Iteration 1: Step in measurement from 0.0 to 2.0
       derivative_raw = -(2.0 - 0.0) / 0.1 = -20.0
       derivative_filt = 0 + 0.1 * 10.0 * (-20.0 - 0) = -20.0
       Output = 1.0 * -20.0 = -20.0 */
    float out1 = pid_compute(&pid, 10.0f, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f, out1);

    /* Iteration 2: Measurement remains constant at 2.0
       derivative_raw = -(2.0 - 2.0) / 0.1 = 0.0
       derivative_filt = -20.0 + 0.1 * 10.0 * (0.0 - (-20.0)) = 0.0
       Output = 1.0 * 0.0 = 0.0 */
    float out2 = pid_compute(&pid, 10.0f, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out2);
}

/* ============================================================================
   4. SETPOINT RAMP MANAGEMENT
   ============================================================================ */

void test_pid_setpoint_ramping(void) {
    /* Setup with setpoint_rate = 10.0 units/s, dt = 0.1s -> max_step = 1.0 unit per step */
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 10.0f, 0.1f, -100.0f, 100.0f, false, 10.0f);

    /* Step target setpoint to 5.0 */
    /* Step 1: current setpoint ramps from 0.0 to 1.0 -> Error = 1.0 - 0.0 = 1.0 */
    float out1 = pid_compute(&pid, 5.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, out1);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pid.setpoint_current);

    /* Step 2: current setpoint ramps from 1.0 to 2.0 -> Error = 2.0 - 0.0 = 2.0 */
    float out2 = pid_compute(&pid, 5.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out2);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, pid.setpoint_current);
}

void test_pid_setpoint_ramp_disabled_when_rate_zero_or_negative(void) {
    /* Setup with setpoint_rate = 0.0 (disabled) */
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 10.0f, 0.1f, -100.0f, 100.0f, false, 0.0f);

    /* Target setpoint immediately updates current active setpoint */
    pid_compute(&pid, 50.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, pid.setpoint_current);
}

/* ============================================================================
   5. SATURATION AND ANTI-WINDUP PROTECTION
   ============================================================================ */

void test_pid_output_saturation_limits(void) {
    /* Limits: [-10.0, 10.0] */
    pid_init(&pid, 5.0f, 0.0f, 0.0f, 10.0f, 0.1f, -10.0f, 10.0f, false, 0.0f);

    /* Unclamped output would be 5.0 * 10.0 = 50.0, must saturate at 10.0 */
    float out_high = pid_compute(&pid, 10.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out_high);

    /* Unclamped output would be 5.0 * -10.0 = -50.0, must saturate at -10.0 */
    float out_low = pid_compute(&pid, -10.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, out_low);
}

void test_pid_anti_windup_dynamic_clamping(void) {
    /* Kp = 1.0, Ki = 1.0, limits = [0.0, 10.0], Anti-Windup enabled */
    pid_init(&pid, 1.0f, 1.0f, 0.0f, 10.0f, 1.0f, 0.0f, 10.0f, true, 0.0f);

    /* Step 1: Force output into high saturation (Error = 20.0) */
    /* P = 20, I_unclamped = 20, Output unclamped = 40 -> Saturates to 10.0 */
    pid_compute(&pid, 20.0f, 0.0f);

    /* Since output saturated at max and error > 0, anti-windup prevents integral accumulation */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.integral);

    /* Step 2: Negative error to drive output out of saturation (Error = -5.0) */
    /* P = -5, I_unclamped = -5. Output = -10 -> Saturates to 0.0 (min limit) */
    pid_compute(&pid, -5.0f, 0.0f);

    /* Output saturated at min and error < 0, anti-windup prevents further negative accumulation */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.integral);
}