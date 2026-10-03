#ifndef EMM_V5_H
#define EMM_V5_H

#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef emm_v5_transport_init(void);
/* transport_init creates the sole USART3 TX task; do not create it twice. */
void emm_v5_task(void *argument);
void emm_v5_uart_tx_complete(void);
void emm_v5_uart_error(void);
void emm_v5_uart_rx_complete(void);
void emm_v5_uart_rx_error(void);

HAL_StatusTypeDef emm_v5_init(void);
/* Waits for UART transmission, not for motor feedback. Task context only
 * once the scheduler is running. No motion command is sent. */
HAL_StatusTypeDef emm_v5_init_motor(uint8_t address);
/* The following command APIs return HAL_OK when queued (DMA mode).
 * Their callers must not access huart3 TX directly. */
HAL_StatusTypeDef emm_v5_set_ctrl_mode(uint8_t address, uint8_t mode, uint8_t save_to_flash);
HAL_StatusTypeDef emm_v5_enable(uint8_t address, uint8_t enable, uint8_t synchronized);
HAL_StatusTypeDef emm_v5_set_velocity(uint8_t address, int16_t rpm, uint16_t acceleration, uint8_t synchronized);
/* Official X_V2 FD trajectory-position command. angle_tenths is signed 0.1°. */
HAL_StatusTypeDef emm_v5_set_position(uint8_t address, int32_t angle_tenths,
                                      uint16_t rpm, uint16_t acceleration,
                                      uint16_t deceleration, uint8_t reference,
                                      uint8_t synchronized);
HAL_StatusTypeDef emm_v5_stop(uint8_t address, uint8_t synchronized);
HAL_StatusTypeDef emm_v5_synchronize(void);

/* X firmware S_VEL feedback. Values are signed 0.1 RPM units. The validity
 * mask has one bit per chassis motor address/order. */
void emm_v5_get_actual_rpm_x10(int16_t rpm_x10[4], uint16_t *valid_mask);

typedef struct {
    uint32_t tx_completed;
    uint32_t tx_failed;
    uint32_t queue_full;
    uint32_t stale_motion;
    uint32_t pending;
    uint8_t faulted;
    uint8_t motion_blocked;
} emm_v5_stats_t;
void emm_v5_get_stats(emm_v5_stats_t *stats);

#endif
