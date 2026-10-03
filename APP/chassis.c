#include "chassis.h"

#include "app_config.h"
#include "emm_v5.h"
#include "rk_link.h"
#include "rk_protocol.h"

#include "cmsis_os.h"
#include "main.h"

static const uint8_t motor_addresses[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_MOTOR_ADDR_0, CHASSIS_MOTOR_ADDR_1,
    CHASSIS_MOTOR_ADDR_2, CHASSIS_MOTOR_ADDR_3
};

static volatile int16_t target_rpm[CHASSIS_MOTOR_COUNT];
static int16_t applied_rpm[CHASSIS_MOTOR_COUNT];
static uint8_t stop_required;
static uint8_t safe_stopped;
static uint8_t motor_ready;

static int16_t round_rpm(float value)
{
    if (value >= 0.0f)
        return (int16_t)(value + 0.5f);
    return (int16_t)(value - 0.5f);
}

static int16_t apply_direction_sign(int16_t rpm, int sign)
{
    return sign < 0 ? (int16_t)-rpm : rpm;
}

void chassis_calculate_wheel_rpm(float vx_mm_s, float vy_mm_s,
                                 float yaw_deg_s, int16_t rpm[4])
{
#if CHASSIS_ENABLE_BODY_KINEMATICS
    const float pi_over_180 = 0.01745329252f;
    const float two_pi = 6.28318530718f;
    const float wheel_circumference = two_pi * CHASSIS_WHEEL_RADIUS_MM;
    const float yaw_mm_s = yaw_deg_s * pi_over_180 *
                           (CHASSIS_HALF_LENGTH_MM + CHASSIS_HALF_WIDTH_MM);
    const float scale = 60.0f / wheel_circumference;
    float raw[CHASSIS_MOTOR_COUNT];
    float max_abs = 0.0f;
    float limit_scale = 1.0f;
    uint32_t i;

    if (rpm == NULL)
        return;

    /* X-layout mecanum inverse kinematics: FL, FR, RR, RL. */
    raw[CHASSIS_WHEEL_FL] = (vx_mm_s - vy_mm_s - yaw_mm_s) * scale;
    raw[CHASSIS_WHEEL_FR] = (vx_mm_s + vy_mm_s + yaw_mm_s) * scale;
    raw[CHASSIS_WHEEL_RR] = (vx_mm_s - vy_mm_s + yaw_mm_s) * scale;
    raw[CHASSIS_WHEEL_RL] = (vx_mm_s + vy_mm_s - yaw_mm_s) * scale;

    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        float magnitude = raw[i] < 0.0f ? -raw[i] : raw[i];
        if (magnitude > max_abs)
            max_abs = magnitude;
    }
    if (max_abs > (float)CHASSIS_SAFE_MAX_RPM && max_abs > 0.0f)
        limit_scale = (float)CHASSIS_SAFE_MAX_RPM / max_abs;

    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
        raw[i] *= limit_scale;

    rpm[CHASSIS_WHEEL_FL] = apply_direction_sign(
        round_rpm(raw[CHASSIS_WHEEL_FL]), CHASSIS_DIR_SIGN_0);
    rpm[CHASSIS_WHEEL_FR] = apply_direction_sign(
        round_rpm(raw[CHASSIS_WHEEL_FR]), CHASSIS_DIR_SIGN_1);
    rpm[CHASSIS_WHEEL_RR] = apply_direction_sign(
        round_rpm(raw[CHASSIS_WHEEL_RR]), CHASSIS_DIR_SIGN_2);
    rpm[CHASSIS_WHEEL_RL] = apply_direction_sign(
        round_rpm(raw[CHASSIS_WHEEL_RL]), CHASSIS_DIR_SIGN_3);
#else
    (void)vx_mm_s;
    (void)vy_mm_s;
    (void)yaw_deg_s;
    if (rpm != NULL) {
        rpm[0] = 0;
        rpm[1] = 0;
        rpm[2] = 0;
        rpm[3] = 0;
    }
#endif
}

void chassis_set_wheel_rpm(const int16_t rpm[4])
{
    uint32_t i;
    if (rpm == NULL)
        return;
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
        target_rpm[i] = rpm[i];
    stop_required = 0U;
}

void chassis_set_body_velocity(float vx_mm_s, float vy_mm_s,
                               float yaw_deg_s)
{
    int16_t rpm[CHASSIS_MOTOR_COUNT];
    chassis_calculate_wheel_rpm(vx_mm_s, vy_mm_s, yaw_deg_s, rpm);
    chassis_set_wheel_rpm(rpm);
}

void chassis_request_stop(void)
{
    uint32_t i;
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
        target_rpm[i] = 0;
    stop_required = 1U;
}

static uint8_t send_stop_all(void)
{
    uint32_t i;
    uint8_t all_queued = 1U;
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        if (emm_v5_stop(motor_addresses[i], 0U) != HAL_OK)
            all_queued = 0U;
    }
    if (all_queued != 0U) {
        safe_stopped = 1U;
        stop_required = 0U;
        for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
            applied_rpm[i] = 0;
    } else {
        safe_stopped = 0U;
    }
    return all_queued;
}

static uint8_t send_targets(void)
{
    uint32_t i;
    uint8_t all_queued = 1U;
    uint8_t any_target = 0U;
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        int16_t rpm = target_rpm[i];
        if (rpm != 0)
            any_target = 1U;
        if (rpm == applied_rpm[i])
            continue;
        if (emm_v5_set_velocity(motor_addresses[i], rpm,
                                CHASSIS_MOTOR_ACCEL, 0U) == HAL_OK) {
            applied_rpm[i] = rpm;
        } else {
            all_queued = 0U;
        }
    }
    if (all_queued != 0U)
        safe_stopped = any_target == 0U ? 1U : 0U;
    return all_queued;
}

void chassis_init(void)
{
    uint32_t i;
    motor_ready = emm_v5_init() == HAL_OK ? 1U : 0U;
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        target_rpm[i] = 0;
        applied_rpm[i] = 0;
    }
    stop_required = 1U;
    /* Force one stop pass even when initialization failed part-way through;
     * another controller may have left an axis enabled before this boot. */
    safe_stopped = 0U;
}

uint8_t chassis_is_safe_stopped(void)
{
    return safe_stopped;
}

uint8_t chassis_motor_is_ready(void)
{
    return motor_ready;
}

void chassis_get_applied_wheel_rpm(int16_t rpm[4])
{
    uint32_t i;
    if (rpm == NULL)
        return;
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
        rpm[i] = applied_rpm[i];
}

void chassis_get_actual_wheel_rpm_x10(int16_t rpm_x10[4],
                                      uint16_t *valid_mask)
{
    emm_v5_get_actual_rpm_x10(rpm_x10, valid_mask);
}

void chassis_task(void *argument)
{
    uint32_t last_status_ms = 0U;
    (void)argument;

    /* Motor initialization uses timed inter-frame gaps.  Run it from the
     * task context after the scheduler has started so the HAL tick remains
     * available while the motor bus is being initialized. */
    chassis_init();

    for (;;) {
        uint32_t now = HAL_GetTick();
        rk_velocity_command_t command;
        uint8_t link_alive;
        uint8_t command_enabled = 0U;
        uint8_t emergency_stop = 0U;
        emm_v5_stats_t transport_stats;

        link_alive = rk_link_is_alive(now);
        if (rk_link_get_command(&command, now) != 0U) {
            command_enabled = command.enabled;
            emergency_stop = command.emergency_stop;
            if (command_enabled != 0U && emergency_stop == 0U &&
                motor_ready != 0U) {
                chassis_set_body_velocity((float)command.vx_mm_s,
                                          (float)command.vy_mm_s,
                                          (float)command.yaw_mdeg_s / 1000.0f);
            } else {
                chassis_request_stop();
            }
        } else {
            chassis_request_stop();
        }

        emm_v5_get_stats(&transport_stats);
        if (transport_stats.faulted != 0U ||
            transport_stats.motion_blocked != 0U)
            motor_ready = 0U;

        if (stop_required != 0U) {
            if (safe_stopped == 0U || applied_rpm[0] != 0 ||
                applied_rpm[1] != 0 || applied_rpm[2] != 0 ||
                applied_rpm[3] != 0)
                (void)send_stop_all();
        } else if (link_alive != 0U && motor_ready != 0U) {
            if (send_targets() == 0U)
                chassis_request_stop();
        } else {
            chassis_request_stop();
        }

        if ((uint32_t)(now - last_status_ms) >= RK_STATUS_PERIOD_MS) {
            rk_chassis_status_t status = {0};
            status.uptime_ms = now;
            chassis_get_applied_wheel_rpm(status.wheel_rpm);
            /* Report the speed command and the motor's measured speed as
             * separate fields.  During acceleration these values are
             * expected to differ; the feedback validity mask prevents the
             * host from treating an unanswered query as a real zero. */
            emm_v5_get_actual_rpm_x10(status.actual_rpm_x10,
                                      &status.actual_valid_mask);
            if (link_alive != 0U)
                status.flags |= RK_STATUS_FLAG_LINK_ALIVE;
            if (command_enabled != 0U)
                status.flags |= RK_STATUS_FLAG_ENABLED;
            if (emergency_stop != 0U)
                status.flags |= RK_STATUS_FLAG_EMERGENCY_STOP;
            if (safe_stopped != 0U)
                status.flags |= RK_STATUS_FLAG_SAFE_STOPPED;
            if (motor_ready != 0U)
                status.flags |= RK_STATUS_FLAG_MOTOR_READY;
            rk_link_send_status(&status);
            last_status_ms = now;
        }
        osDelay(20U);
    }
}
