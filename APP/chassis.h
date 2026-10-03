#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* Wheel order used by the lower controller: front-left, front-right,
 * rear-right, rear-left. Body coordinates are x forward, y left and
 * positive yaw counter-clockwise. */
#define CHASSIS_WHEEL_FL 0U
#define CHASSIS_WHEEL_FR 1U
#define CHASSIS_WHEEL_RR 2U
#define CHASSIS_WHEEL_RL 3U

/* Initialize the four X_V2 drives and enter the safe stopped state. This is
 * called once after the EMM transport has been created. */
void chassis_init(void);

/* FreeRTOS task: consume RK3588S velocity commands and drive the four wheels. */
void chassis_task(void *argument);

/* Convert body velocity to wheel RPM. The result is scaled as a group so that
 * all four wheels retain their requested ratio when the safety limit applies. */
void chassis_calculate_wheel_rpm(float vx_mm_s, float vy_mm_s,
                                 float yaw_deg_s, int16_t rpm[4]);

/* Direct wheel target API for a higher-level controller or a bench test. */
void chassis_set_wheel_rpm(const int16_t rpm[4]);

/* Body velocity API: vx/vy in mm/s, yaw in deg/s. */
void chassis_set_body_velocity(float vx_mm_s, float vy_mm_s,
                               float yaw_deg_s);

/* Request a safe stop. The task sends the actual stop frames. */
void chassis_request_stop(void);

uint8_t chassis_is_safe_stopped(void);
uint8_t chassis_motor_is_ready(void);

/* Returns the last wheel RPM command accepted by the transport. */
void chassis_get_applied_wheel_rpm(int16_t rpm[4]);

/* Returns X firmware real-speed feedback in signed 0.1 RPM units. A bit in
 * valid_mask is set only after a syntactically valid response was received. */
void chassis_get_actual_wheel_rpm_x10(int16_t rpm_x10[4], uint16_t *valid_mask);

#endif /* CHASSIS_H */
