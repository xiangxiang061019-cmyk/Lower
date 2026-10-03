#include "emm_v5.h"
#include "app_config.h"
#include "cmsis_os2.h"
#include "usart.h"
#include <string.h>

#define EMM_V5_MAX_FRAME_SIZE 20U
#define EMM_V5_TX_DONE_FLAG (1U << 0)
#define EMM_V5_RX_RING_SIZE 64U

static volatile uint32_t tx_completed;
static volatile uint32_t tx_failed;
static volatile uint32_t queue_full;
static volatile uint32_t stale_motion;
static volatile uint8_t transport_fault;
static volatile uint8_t motion_blocked;
static volatile int16_t actual_rpm_x10[CHASSIS_MOTOR_COUNT];
static volatile uint16_t actual_valid_mask;
static volatile uint8_t feedback_enabled;

#if EMM_V5_USE_DMA
typedef struct {
    uint8_t data[EMM_V5_MAX_FRAME_SIZE];
    uint16_t length;
    uint8_t synchronous;
    uint32_t queued_ms;
} emm_v5_tx_item_t;

static osMessageQueueId_t tx_queue;
static osThreadId_t tx_task_id;
static osMutexId_t sync_mutex;
static osSemaphoreId_t sync_done;
static HAL_StatusTypeDef sync_result;
static volatile uint8_t tx_error;
static volatile uint8_t tx_active;
static uint8_t rx_byte;
static uint8_t rx_ring[EMM_V5_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_started;
static uint8_t feedback_index;
static uint32_t next_feedback_ms;
/* Dedicated storage remains valid even if a DMA abort itself fails. */
static uint8_t dma_frame[EMM_V5_MAX_FRAME_SIZE];
static const osThreadAttr_t tx_task_attributes = {
    .name = "emmV5Task",
    .stack_size = 1024U,
    .priority = osPriorityAboveNormal
};
#endif

static const uint8_t feedback_addresses[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_MOTOR_ADDR_0, CHASSIS_MOTOR_ADDR_1,
    CHASSIS_MOTOR_ADDR_2, CHASSIS_MOTOR_ADDR_3
};

static uint32_t ticks_from_ms(uint32_t ms)
{
    uint32_t ticks = (ms * osKernelGetTickFreq() + 999U) / 1000U;
    return ticks != 0U ? ticks : 1U;
}

static void frame_gap(void)
{
    if (osKernelGetState() == osKernelRunning)
        (void)osDelay(ticks_from_ms(EMM_V5_FRAME_GAP_MS));
    else
        HAL_Delay(EMM_V5_FRAME_GAP_MS);
}

#if EMM_V5_USE_DMA
static uint8_t rx_ring_pop(uint8_t *value)
{
    uint16_t tail;
    if (value == NULL || rx_tail == rx_head)
        return 0U;
    tail = rx_tail;
    *value = rx_ring[tail];
    rx_tail = (uint16_t)((tail + 1U) % EMM_V5_RX_RING_SIZE);
    return 1U;
}

static void rx_ring_flush(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rx_tail = rx_head;
    if (primask == 0U)
        __enable_irq();
}

static uint8_t read_feedback_velocity(uint8_t address, int16_t *rpm_x10)
{
    const uint8_t query[] = {address, 0x35U, 0x6BU};
    uint8_t response[6];
    uint8_t used = 0U;
    uint8_t byte;
    uint32_t flags;
    uint32_t deadline;
    HAL_StatusTypeDef status;

    if (rpm_x10 == NULL || address == 0U || transport_fault != 0U)
        return 0U;

    /* Each query is serialized on the shared bus. Discard bytes left by a
     * timed-out previous query before accepting this address's response. */
    rx_ring_flush();
    memcpy(dma_frame, query, sizeof(query));
    tx_error = 0U;
    (void)osThreadFlagsClear(EMM_V5_TX_DONE_FLAG);
    tx_active = 1U;
    status = HAL_UART_Transmit_DMA(&huart3, dma_frame, (uint16_t)sizeof(query));
    if (status == HAL_OK) {
        flags = osThreadFlagsWait(EMM_V5_TX_DONE_FLAG, osFlagsWaitAny,
                                  ticks_from_ms(EMM_V5_TX_TIMEOUT_MS));
        if ((flags & osFlagsError) != 0U ||
            (flags & EMM_V5_TX_DONE_FLAG) == 0U || tx_error != 0U)
            status = HAL_ERROR;
    }
    if (status != HAL_OK) {
        if (HAL_UART_AbortTransmit(&huart3) != HAL_OK)
            transport_fault = 1U;
        (void)osThreadFlagsClear(EMM_V5_TX_DONE_FLAG);
        tx_active = 0U;
        return 0U;
    }
    tx_active = 0U;
    frame_gap();

    deadline = HAL_GetTick() + EMM_V5_FEEDBACK_TIMEOUT_MS;
    while ((int32_t)(deadline - HAL_GetTick()) > 0) {
        while (rx_ring_pop(&byte) != 0U) {
            if (used == 0U) {
                if (byte == address)
                    response[used++] = byte;
            } else if (used == 1U) {
                if (byte == 0x35U)
                    response[used++] = byte;
                else
                    used = byte == address ? 1U : 0U;
            } else {
                response[used++] = byte;
                if (used == sizeof(response)) {
                    uint16_t magnitude;
                    if (response[5] == 0x6BU &&
                        (response[2] == 0U || response[2] == 1U)) {
                        magnitude = (uint16_t)(((uint16_t)response[3] << 8U) |
                                               response[4]);
                        *rpm_x10 = response[2] != 0U ?
                            (int16_t)-(int32_t)magnitude : (int16_t)magnitude;
                        return 1U;
                    }
                    used = response[0] == address ? 1U : 0U;
                }
            }
        }
        (void)osDelay(1U);
    }
    return 0U;
}

static void service_feedback(void)
{
    uint8_t index = feedback_index;
    int16_t value;

    if (feedback_enabled == 0U || motion_blocked != 0U ||
        transport_fault != 0U)
        return;
    if (read_feedback_velocity(feedback_addresses[index], &value) != 0U) {
        actual_rpm_x10[index] = value;
        actual_valid_mask |= (uint16_t)(1U << index);
    } else {
        actual_valid_mask &= (uint16_t)~(1U << index);
    }
    feedback_index = (uint8_t)((index + 1U) % CHASSIS_MOTOR_COUNT);
}
#endif

static HAL_StatusTypeDef send_frame_blocking(const uint8_t *frame, uint16_t length)
{
    HAL_StatusTypeDef status;
    if (frame == NULL || length == 0U || length > EMM_V5_MAX_FRAME_SIZE)
        return HAL_ERROR;
    status = HAL_UART_Transmit(&huart3, (uint8_t *)frame, length, EMM_V5_TX_TIMEOUT_MS);
    if (status == HAL_OK)
        ++tx_completed;
    else
        ++tx_failed;
    return status;
}

static HAL_StatusTypeDef send_frame_async(const uint8_t *frame, uint16_t length)
{
#if EMM_V5_USE_DMA
    emm_v5_tx_item_t item;
    if (__get_IPSR() != 0U || osKernelGetState() != osKernelRunning ||
        tx_queue == NULL || tx_task_id == NULL || transport_fault != 0U ||
        frame == NULL || length == 0U || length > EMM_V5_MAX_FRAME_SIZE)
        return HAL_ERROR;
    if (motion_blocked != 0U && (frame[1] == 0xF6U || frame[1] == 0xFDU))
        return HAL_ERROR;
    memset(&item, 0, sizeof(item));
    memcpy(item.data, frame, length);
    item.length = length;
    item.queued_ms = HAL_GetTick();
    if (osMessageQueuePut(tx_queue, &item, 0U, 0U) == osOK)
        return HAL_OK;
    ++queue_full;
    return HAL_BUSY;
#else
    return send_frame_blocking(frame, length);
#endif
}

static HAL_StatusTypeDef send_frame_sync(const uint8_t *frame, uint16_t length)
{
#if EMM_V5_USE_DMA
    emm_v5_tx_item_t item;
    HAL_StatusTypeDef result;
    osKernelState_t state;
    if (__get_IPSR() != 0U)
        return HAL_ERROR;
    state = osKernelGetState();
    if (state == osKernelInactive || state == osKernelReady)
        return send_frame_blocking(frame, length);
    if (state != osKernelRunning || tx_queue == NULL || tx_task_id == NULL ||
        transport_fault != 0U || osThreadGetId() == tx_task_id ||
        frame == NULL || length == 0U || length > EMM_V5_MAX_FRAME_SIZE)
        return HAL_ERROR;
    if (osMutexAcquire(sync_mutex, ticks_from_ms(EMM_V5_TX_TIMEOUT_MS)) != osOK)
        return HAL_BUSY;
    memset(&item, 0, sizeof(item));
    memcpy(item.data, frame, length);
    item.length = length;
    item.synchronous = 1U;
    item.queued_ms = HAL_GetTick();
    if (osMessageQueuePut(tx_queue, &item, 0U, ticks_from_ms(EMM_V5_TX_TIMEOUT_MS)) != osOK) {
        ++queue_full;
        (void)osMutexRelease(sync_mutex);
        return HAL_BUSY;
    }
    /* Once queued, await the worker's terminal result. The worker has a
     * bounded DMA timeout and always releases this dedicated semaphore.
     * No pointer to a caller's stack is stored in the queue. */
    (void)osSemaphoreAcquire(sync_done, osWaitForever);
    result = sync_result;
    (void)osMutexRelease(sync_mutex);
    return result;
#else
    return send_frame_blocking(frame, length);
#endif
}

HAL_StatusTypeDef emm_v5_transport_init(void)
{
    feedback_enabled = EMM_V5_FEEDBACK_ENABLE != 0U ? 1U : 0U;
    feedback_index = 0U;
    actual_valid_mask = 0U;
    memset((void *)actual_rpm_x10, 0, sizeof(actual_rpm_x10));
#if EMM_V5_USE_DMA
    if (tx_task_id != NULL)
        return HAL_OK;
    if (tx_queue == NULL)
        tx_queue = osMessageQueueNew(EMM_V5_TX_QUEUE_LENGTH, sizeof(emm_v5_tx_item_t), NULL);
    if (sync_mutex == NULL)
        sync_mutex = osMutexNew(NULL);
    if (sync_done == NULL)
        sync_done = osSemaphoreNew(1U, 0U, NULL);
    if (tx_queue == NULL || sync_mutex == NULL || sync_done == NULL)
        return HAL_ERROR;
    if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1U) != HAL_OK)
        return HAL_ERROR;
    rx_started = 1U;
    tx_task_id = osThreadNew(emm_v5_task, NULL, &tx_task_attributes);
    return tx_task_id != NULL ? HAL_OK : HAL_ERROR;
#else
    return HAL_OK;
#endif
}

void emm_v5_task(void *argument)
{
    (void)argument;
#if !EMM_V5_USE_DMA
    for (;;) {
        osDelay(osWaitForever);
    }
#else
    emm_v5_tx_item_t item;
    tx_task_id = osThreadGetId();
    for (;;) {
        HAL_StatusTypeDef status;
        uint32_t now = HAL_GetTick();
        if (feedback_enabled != 0U &&
            (int32_t)(now - next_feedback_ms) >= 0 &&
            osMessageQueueGetCount(tx_queue) == 0U) {
            service_feedback();
            next_feedback_ms = HAL_GetTick() + EMM_V5_FEEDBACK_PERIOD_MS;
            continue;
        }
        if (osMessageQueueGet(tx_queue, &item, NULL, ticks_from_ms(1U)) != osOK)
            continue;
        tx_active = 1U;
        status = HAL_ERROR;
        if (transport_fault == 0U) {
            if (motion_blocked != 0U && (item.data[1] == 0xF6U || item.data[1] == 0xFDU)) {
                status = HAL_ERROR;
            } else if (item.data[1] == 0xF6U &&
                (uint32_t)(HAL_GetTick() - item.queued_ms) > EMM_V5_MOTION_MAX_AGE_MS) {
                ++stale_motion;
                status = HAL_TIMEOUT;
            } else {
                uint32_t flags;
                memcpy(dma_frame, item.data, item.length);
                tx_error = 0U;
                (void)osThreadFlagsClear(EMM_V5_TX_DONE_FLAG);
                status = HAL_UART_Transmit_DMA(&huart3, dma_frame, item.length);
                if (status == HAL_OK) {
                    flags = osThreadFlagsWait(EMM_V5_TX_DONE_FLAG, osFlagsWaitAny,
                                              ticks_from_ms(EMM_V5_TX_TIMEOUT_MS));
                    if ((flags & osFlagsError) != 0U)
                        status = flags == osFlagsErrorTimeout ? HAL_TIMEOUT : HAL_ERROR;
                    else if ((flags & EMM_V5_TX_DONE_FLAG) == 0U || tx_error != 0U)
                        status = HAL_ERROR;
                }
                if (status != HAL_OK) {
                    /* Stop hardware before reusing its source buffer. If abort
                     * fails, retain that buffer forever and reject new work. */
                    if (HAL_UART_AbortTransmit(&huart3) != HAL_OK)
                        transport_fault = 1U;
                    (void)osThreadFlagsClear(EMM_V5_TX_DONE_FLAG);
                }
                frame_gap();
            }
        }
        if (status == HAL_OK)
            ++tx_completed;
        else {
            ++tx_failed;
            motion_blocked = 1U;
        }
        tx_active = 0U;
        if (item.synchronous != 0U) {
            sync_result = status;
            (void)osSemaphoreRelease(sync_done);
        }
    }
#endif
}

void emm_v5_uart_tx_complete(void)
{
#if EMM_V5_USE_DMA
    if (tx_task_id != NULL)
        (void)osThreadFlagsSet(tx_task_id, EMM_V5_TX_DONE_FLAG);
#endif
}

void emm_v5_uart_error(void)
{
#if EMM_V5_USE_DMA
    if (tx_active != 0U) {
        tx_error = 1U;
        if (tx_task_id != NULL)
            (void)osThreadFlagsSet(tx_task_id, EMM_V5_TX_DONE_FLAG);
    }
#endif
}

void emm_v5_uart_rx_complete(void)
{
#if EMM_V5_USE_DMA
    uint16_t next = (uint16_t)((rx_head + 1U) % EMM_V5_RX_RING_SIZE);
    if (next != rx_tail) {
        rx_ring[rx_head] = rx_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1U) != HAL_OK)
        rx_started = 0U;
#endif
}

void emm_v5_uart_rx_error(void)
{
#if EMM_V5_USE_DMA
    rx_started = 0U;
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1U) == HAL_OK)
        rx_started = 1U;
#endif
}

void emm_v5_get_actual_rpm_x10(int16_t rpm_x10[4], uint16_t *valid_mask)
{
    uint32_t i;
    uint32_t primask;
    if (rpm_x10 == NULL || valid_mask == NULL)
        return;
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
        rpm_x10[i] = actual_rpm_x10[i];
    *valid_mask = actual_valid_mask;
    if (primask == 0U)
        __enable_irq();
}

HAL_StatusTypeDef emm_v5_set_ctrl_mode(uint8_t address, uint8_t mode, uint8_t save_to_flash)
{
    const uint8_t frame[] = {address, 0x46U, 0x69U, save_to_flash, mode, 0x6BU};
    return send_frame_async(frame, (uint16_t)sizeof(frame));
}

HAL_StatusTypeDef emm_v5_enable(uint8_t address, uint8_t enable, uint8_t synchronized)
{
    const uint8_t frame[] = {address, 0xF3U, 0xABU, enable, synchronized, 0x6BU};
    return send_frame_async(frame, (uint16_t)sizeof(frame));
}

HAL_StatusTypeDef emm_v5_set_velocity(uint8_t address, int16_t rpm, uint16_t acceleration, uint8_t synchronized)
{
    uint8_t direction = rpm < 0 ? 1U : 0U;
    uint32_t magnitude = rpm < 0 ? (uint32_t)(-(int32_t)rpm) : (uint32_t)rpm;
#if EMM_V5_X_V2_PROTOCOL
    uint32_t speed_x10;
    uint16_t speed;
    uint16_t ramp = acceleration;
    if (rpm == 0)
        return emm_v5_stop(address, synchronized);
    speed_x10 = magnitude * 10U;
    if (speed_x10 > EMM_V5_X_V2_MAX_RPM * 10U)
        speed_x10 = EMM_V5_X_V2_MAX_RPM * 10U;
    speed = (uint16_t)speed_x10;
    {
        const uint8_t frame[] = {
            address, 0xF6U, direction,
            (uint8_t)(ramp >> 8U), (uint8_t)ramp,
            (uint8_t)(speed >> 8U), (uint8_t)speed,
            synchronized, 0x6BU
        };
        return send_frame_async(frame, (uint16_t)sizeof(frame));
    }
#elif CHASSIS_VELOCITY_MODE_POSITION
    uint8_t frame[13];
    uint16_t speed = (uint16_t)(magnitude > CHASSIS_MOTOR_MAX_RPM ? CHASSIS_MOTOR_MAX_RPM : magnitude);
    if (rpm == 0)
        return emm_v5_stop(address, synchronized);
    frame[0] = address; frame[1] = 0xFDU; frame[2] = direction;
    frame[3] = (uint8_t)(speed >> 8U); frame[4] = (uint8_t)speed; frame[5] = acceleration;
    frame[6] = 0U; frame[7] = 0U; frame[8] = 0xFFU; frame[9] = 0xFFU;
    frame[10] = 0U; frame[11] = synchronized; frame[12] = 0x6BU;
    return send_frame_async(frame, (uint16_t)sizeof(frame));
#else
    uint16_t speed = (uint16_t)(magnitude > CHASSIS_MOTOR_MAX_RPM ? CHASSIS_MOTOR_MAX_RPM : magnitude);
    const uint8_t frame[] = {address, 0xF6U, direction, (uint8_t)(speed >> 8U), (uint8_t)speed, acceleration, synchronized, 0x6BU};
    return send_frame_async(frame, (uint16_t)sizeof(frame));
#endif
}

HAL_StatusTypeDef emm_v5_set_position(uint8_t address, int32_t angle_tenths,
                                      uint16_t rpm, uint16_t acceleration,
                                      uint16_t deceleration, uint8_t reference,
                                      uint8_t synchronized)
{
#if EMM_V5_X_V2_PROTOCOL
    uint8_t direction = angle_tenths < 0 ? 1U : 0U;
    uint32_t magnitude = angle_tenths < 0 ? (uint32_t)(-(int64_t)angle_tenths) :
                                           (uint32_t)angle_tenths;
    uint32_t speed_x10 = (uint32_t)rpm * 10U;
    uint16_t speed;
    uint8_t frame[16];
    if (address == 0U || reference > 2U ||
        speed_x10 == 0U || speed_x10 > EMM_V5_X_V2_MAX_RPM * 10U)
        return HAL_ERROR;
    speed = (uint16_t)speed_x10;
    frame[0] = address; frame[1] = 0xFDU; frame[2] = direction;
    frame[3] = (uint8_t)(acceleration >> 8U); frame[4] = (uint8_t)acceleration;
    frame[5] = (uint8_t)(deceleration >> 8U); frame[6] = (uint8_t)deceleration;
    frame[7] = (uint8_t)(speed >> 8U); frame[8] = (uint8_t)speed;
    frame[9] = (uint8_t)(magnitude >> 24U); frame[10] = (uint8_t)(magnitude >> 16U);
    frame[11] = (uint8_t)(magnitude >> 8U); frame[12] = (uint8_t)magnitude;
    frame[13] = reference; frame[14] = synchronized; frame[15] = 0x6BU;
    return send_frame_async(frame, (uint16_t)sizeof(frame));
#else
    (void)address; (void)angle_tenths; (void)rpm; (void)acceleration;
    (void)deceleration; (void)reference; (void)synchronized;
    return HAL_ERROR;
#endif
}

HAL_StatusTypeDef emm_v5_stop(uint8_t address, uint8_t synchronized)
{
    const uint8_t frame[] = {address, 0xFEU, 0x98U, synchronized, 0x6BU};
    return send_frame_async(frame, (uint16_t)sizeof(frame));
}

HAL_StatusTypeDef emm_v5_synchronize(void)
{
    const uint8_t frame[] = {0x00U, 0xFFU, 0x66U, 0x6BU};
    return send_frame_async(frame, (uint16_t)sizeof(frame));
}

HAL_StatusTypeDef emm_v5_init(void)
{
    static const uint8_t addresses[CHASSIS_MOTOR_COUNT] = {CHASSIS_MOTOR_ADDR_0, CHASSIS_MOTOR_ADDR_1, CHASSIS_MOTOR_ADDR_2, CHASSIS_MOTOR_ADDR_3};
    uint32_t i;
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        if (emm_v5_init_motor(addresses[i]) != HAL_OK)
            return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef emm_v5_init_motor(uint8_t address)
{
    const uint8_t mode[] = {address, 0x46U, 0x69U, 0U, EMM_V5_X_V2_CTRL_MODE, 0x6BU};
    const uint8_t enable[] = {address, 0xF3U, 0xABU, 1U, 0U, 0x6BU};
    const uint8_t stop[] = {address, 0xFEU, 0x98U, 0U, 0x6BU};
    HAL_StatusTypeDef status;
    if (address == 0U || __get_IPSR() != 0U)
        return HAL_ERROR;
    status = send_frame_sync(stop, sizeof(stop));
    if (status != HAL_OK)
        return status;
    frame_gap();
    status = send_frame_sync(mode, sizeof(mode));
    if (status != HAL_OK)
        return status;
    frame_gap();
    status = send_frame_sync(enable, sizeof(enable));
    frame_gap();
    if (status == HAL_OK)
        motion_blocked = 0U;
    return status;
}

void emm_v5_get_stats(emm_v5_stats_t *stats)
{
    if (stats == NULL)
        return;
    stats->tx_completed = tx_completed;
    stats->tx_failed = tx_failed;
    stats->queue_full = queue_full;
    stats->stale_motion = stale_motion;
    stats->faulted = transport_fault;
    stats->motion_blocked = motion_blocked;
#if EMM_V5_USE_DMA
    stats->pending = tx_queue != NULL ? osMessageQueueGetCount(tx_queue) + tx_active : 0U;
#else
    stats->pending = 0U;
#endif
}
