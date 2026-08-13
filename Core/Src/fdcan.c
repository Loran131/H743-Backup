/**
 ******************************************************************************
 * @file    fdcan.c
 * @brief   FDCAN2 driver for PD42S1 motor CAN communication
 *          125 kbps, 29-bit extended ID, interrupt-driven RX with reassembly
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include <stdio.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ====================== GLOBAL VARIABLES ================================== */

FDCAN_HandleTypeDef hfdcan2;

/* RX reassembly buffer */
CanRxFrame_t g_can_rx_frame;
CanDiagnostics_t g_can_diagnostics;

static volatile uint8_t g_can_recovery_requested;
static volatile uint8_t g_can_bus_off_active;
static volatile uint8_t g_can_hard_fault_active;
static volatile uint32_t g_can_recovery_reason;
static volatile uint32_t g_can_recovery_request_tick;
static volatile uint32_t g_can_error_state_tick;
static uint32_t g_can_last_recovery_tick;

#define CAN_RECOVERY_REASON_BUS_OFF       (1UL << 0)
#define CAN_RECOVERY_REASON_RAM_ACCESS    (1UL << 1)
#define CAN_RECOVERY_REASON_RAM_WDG       (1UL << 2)
#define CAN_RECOVERY_REASON_HAL_STATE     (1UL << 3)

#define CAN_ERROR_NOTIFICATIONS (FDCAN_IT_BUS_OFF                 | \
                                 FDCAN_IT_ERROR_PASSIVE           | \
                                 FDCAN_IT_ERROR_WARNING           | \
                                 FDCAN_IT_ARB_PROTOCOL_ERROR      | \
                                 FDCAN_IT_DATA_PROTOCOL_ERROR     | \
                                 FDCAN_IT_RX_FIFO0_MESSAGE_LOST   | \
                                 FDCAN_IT_RAM_ACCESS_FAILURE      | \
                                 FDCAN_IT_RAM_WATCHDOG            | \
                                 FDCAN_IT_ERROR_LOGGING_OVERFLOW)

#define CAN_ACTIVE_NOTIFICATIONS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
                                  CAN_ERROR_NOTIFICATIONS)

static void can_request_recovery(uint32_t reason)
{
    if (!g_can_recovery_requested)
    {
        g_can_recovery_request_tick = HAL_GetTick();
    }
    g_can_recovery_reason |= reason;
    g_can_recovery_requested = 1U;
}

/* TX header (reused for all sends) */
static FDCAN_TxHeaderTypeDef g_tx_header;

static uint8_t can_read_fifo0_message(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t dlc;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                               &rx_header, rx_data) != HAL_OK)
    {
        return 0U;
    }

    dlc = (uint8_t)(rx_header.DataLength & 0x0FU);
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    g_can_diagnostics.rx_frame_count++;
    g_can_diagnostics.last_rx_id = rx_header.Identifier;
    g_can_diagnostics.last_rx_dlc = dlc;
    g_can_diagnostics.last_rx_id_type = rx_header.IdType;

    if (g_can_rx_frame.frame_done)
    {
        g_can_rx_frame.len = 0U;
        g_can_rx_frame.frame_done = 0U;
    }

    if ((g_can_rx_frame.len + dlc) <= CAN_RX_BUF_SIZE)
    {
        memcpy(&g_can_rx_frame.buf[g_can_rx_frame.len], rx_data, dlc);
        g_can_rx_frame.len += dlc;
    }
    else
    {
        g_can_rx_frame.len = 0U;
    }

    g_can_rx_frame.last_rx_tick = HAL_GetTick();
    return 1U;
}

/* ====================== INITIALIZATION ==================================== */

/**
 * @brief  FDCAN2 Initialization
 *         125 kbps, Classic CAN, internal loopback diagnostic mode
 */
void MX_FDCAN2_Init(void)
{

    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
#if CAN_INTERNAL_LOOPBACK_TEST
    hfdcan2.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
#else
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
#endif
    hfdcan2.Init.AutoRetransmission = ENABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;

    /* Nominal timing: 125 kbps @ 25 MHz HSE kernel clock
     * BRP=10, TSEG1=16, TSEG2=3, SJW=1
     */
    hfdcan2.Init.NominalPrescaler = CAN_NOMINAL_PRESCALER;
    hfdcan2.Init.NominalSyncJumpWidth = CAN_NOMINAL_SJW;
    hfdcan2.Init.NominalTimeSeg1 = CAN_NOMINAL_TIME_SEG1;
    hfdcan2.Init.NominalTimeSeg2 = CAN_NOMINAL_TIME_SEG2;

    /* Data timing (not used in Classic mode, set safe defaults) */
    hfdcan2.Init.DataPrescaler = 1;
    hfdcan2.Init.DataSyncJumpWidth = 1;
    hfdcan2.Init.DataTimeSeg1 = 1;
    hfdcan2.Init.DataTimeSeg2 = 1;

    /* Message RAM allocation:
     *   StdFilters=0, ExtFilters=1, RxFifo0=4, RxFifo1=0,
     *   TxEventFifo=0, TxBuffers=0, TxFifoQueue=4
     */
    hfdcan2.Init.MessageRAMOffset = 0;
    hfdcan2.Init.StdFiltersNbr = 0;
    hfdcan2.Init.ExtFiltersNbr = 1;
    hfdcan2.Init.RxFifo0ElmtsNbr = 4;
    hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxFifo1ElmtsNbr = 0;
    hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.TxEventsNbr = 0;
    hfdcan2.Init.TxBuffersNbr = 0;
    hfdcan2.Init.TxFifoQueueElmtsNbr = 4;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Match the factory example while diagnosing the motor's configured ID. */
    can_set_filter_mask(0U, 0U);

    /* Monitor reception and all fault states that can block communication. */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, CAN_ACTIVE_NOTIFICATIONS, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Clear RX buffer */
    memset(&g_can_rx_frame, 0, sizeof(g_can_rx_frame));
    memset(&g_can_diagnostics, 0, sizeof(g_can_diagnostics));
    g_can_recovery_requested = 0U;
    g_can_bus_off_active = 0U;
    g_can_hard_fault_active = 0U;
    g_can_recovery_reason = 0U;
    g_can_recovery_request_tick = 0U;
    g_can_error_state_tick = 0U;
    g_can_last_recovery_tick = 0U;

    /* USER CODE BEGIN FDCAN2_Init 2 */

    /* USER CODE END FDCAN2_Init 2 */
}

/* ====================== MSP INIT / DEINIT ================================= */

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (fdcanHandle->Instance == FDCAN2)
    {
        /* USER CODE BEGIN FDCAN2_MspInit 0 */

        /* USER CODE END FDCAN2_MspInit 0 */

        /* Keep fdcan_ker_ck within its rated range. The 25 MHz HSE also gives
         * an exact 125 kbit/s nominal rate with BRP=10 and 20 time quanta. */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /* Enable FDCAN clock */
        __HAL_RCC_FDCAN_CLK_ENABLE();

        /* GPIO clock enable */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* FDCAN2 GPIO: PB12=RX, PB13=TX (AF9) */
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Enable FDCAN2 interrupt */
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);

        /* USER CODE BEGIN FDCAN2_MspInit 1 */

        /* USER CODE END FDCAN2_MspInit 1 */
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (fdcanHandle->Instance == FDCAN2)
    {
        /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

        /* USER CODE END FDCAN2_MspDeInit 0 */

        /* Disable FDCAN2 interrupt */
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);

        /* Peripheral clock disable */
        __HAL_RCC_FDCAN_CLK_DISABLE();

        /* GPIO deinit */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13);

        /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

        /* USER CODE END FDCAN2_MspDeInit 1 */
    }
}

/* ====================== FILTER CONFIGURATION ============================== */

/**
 * @brief  Set extended ID filter to accept a specific CAN ID
 * @param  ext_id  29-bit extended CAN ID to accept
 */
void can_set_filter_id(uint32_t ext_id)
{
    can_set_filter_mask(ext_id, 0x1FFFFFFFU);
}

void can_set_filter_mask(uint32_t ext_id, uint32_t mask)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = ext_id;
    sFilterConfig.FilterID2 = mask;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* The factory example accepts all frames into FIFO0. */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ====================== CAN SEND FUNCTIONS ================================ */

/**
 * @brief  Send a single CAN message (≤8 bytes)
 * @param  ext_id  29-bit extended ID
 * @param  data    Payload buffer
 * @param  len     Payload length (1-8)
 * @retval 0=success, 1=error
 */
uint8_t can_send_msg(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    if ((len > 8U) || g_can_recovery_requested || g_can_bus_off_active)
        return 1;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        g_can_diagnostics.tx_enqueue_fail_count++;
        return 1;
    }

    /* Prepare TX header */
    g_tx_header.Identifier = ext_id;
    g_tx_header.IdType = FDCAN_EXTENDED_ID;
    g_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    /* This HAL expects the unshifted DLC value and shifts it when writing RAM. */
    g_tx_header.DataLength = len;
    g_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    g_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    g_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    g_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &g_tx_header,
                                       (uint8_t *)data) != HAL_OK)
    {
        g_can_diagnostics.tx_enqueue_fail_count++;
        if (HAL_FDCAN_GetState(&hfdcan2) != HAL_FDCAN_STATE_BUSY)
        {
            g_can_hard_fault_active = 1U;
            can_request_recovery(CAN_RECOVERY_REASON_HAL_STATE);
        }
        return 1;
    }

#if CAN_INTERNAL_LOOPBACK_TEST
    printf("[CAN TX] id=0x%08lX type=EXT len=%u data=",
           (unsigned long)ext_id, (unsigned int)len);
    for (uint8_t i = 0U; i < len; ++i)
    {
        printf("%02X%s", data[i], (i + 1U < len) ? " " : "\r\n");
    }
#endif

    return 0;
}

/**
 * @brief  Send a long message, fragmenting across multiple CAN frames
 * @param  ext_id  29-bit extended ID (same for all fragments)
 * @param  data    Full payload buffer
 * @param  len     Total payload length (can exceed 8 bytes)
 * @retval 0=success, 1=error
 *
 * Each fragment carries up to 8 bytes. The receiver must reassemble them
 * in order. Frame boundaries are detected by the 5ms idle timeout.
 */
uint8_t can_send_long_msg(uint32_t ext_id, const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;

    while (offset < len)
    {
        uint8_t chunk_len = (len - offset) > 8 ? 8 : (len - offset);

        if (can_send_msg(ext_id, &data[offset], chunk_len) != 0)
        {
            return 1;
        }

        offset += chunk_len;
    }

    return 0;
}

/* ====================== RX INTERRUPT CALLBACK ============================= */

/**
 * @brief  FDCAN RX FIFO0 callback — called by HAL from ISR
 *
 * Appends received CAN data bytes to the reassembly buffer.
 * Resets the idle timeout on each reception.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                    &rx_header, rx_data) != HAL_OK)
        {
            return;
        }

        /* Get actual data length */
        uint8_t dlc = (uint8_t)(rx_header.DataLength & 0x0FU);
        if (dlc > 8)
            dlc = 8;

        g_can_diagnostics.rx_frame_count++;
        g_can_diagnostics.last_rx_id = rx_header.Identifier;
        g_can_diagnostics.last_rx_dlc = dlc;
        g_can_diagnostics.last_rx_id_type = rx_header.IdType;

        /* If previous frame was already processed, start new */
        if (g_can_rx_frame.frame_done)
        {
            g_can_rx_frame.len = 0;
            g_can_rx_frame.frame_done = 0;
        }

        /* Append to reassembly buffer */
        if (g_can_rx_frame.len + dlc <= CAN_RX_BUF_SIZE)
        {
            memcpy(&g_can_rx_frame.buf[g_can_rx_frame.len], rx_data, dlc);
            g_can_rx_frame.len += dlc;
        }
        else
        {
            /* Buffer overflow — reset */
            g_can_rx_frame.len = 0;
        }

        /* Record reception time for idle timeout */
        g_can_rx_frame.last_rx_tick = HAL_GetTick();

        /* A long SMD response may already have more fragments queued. The
         * RX-new-message interrupt is edge-like here, so drain FIFO now. */
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
        {
            if (can_read_fifo0_message(hfdcan) == 0U) break;
        }
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != RESET)
    {
        g_can_diagnostics.rx_lost_count++;
        g_can_diagnostics.last_error_events = RxFifo0ITs;
        g_can_rx_frame.len = 0U;
        g_can_rx_frame.frame_done = 0U;
    }
}

/* ====================== RX TIMEOUT CHECK ================================== */

/**
 * @brief  Check for RX idle timeout (call in main loop)
 *
 * When no CAN message received for CAN_RX_TIMEOUT_MS,
 * marks the frame as complete.
 */
void can_rx_timeout_check(void)
{
    /* A later fragment can enter FIFO just after the RX callback exits
     * without producing another callback edge, so polling also drains it. */
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U)
    {
        if (!can_read_fifo0_message(&hfdcan2))
        {
            break;
        }
    }

    if (g_can_rx_frame.len > 0 && !g_can_rx_frame.frame_done)
    {
        uint32_t elapsed = HAL_GetTick() - g_can_rx_frame.last_rx_tick;

        if (elapsed >= CAN_RX_TIMEOUT_MS)
        {
            g_can_rx_frame.frame_done = 1;
        }
    }
}

static uint8_t can_restart_controller(void)
{
    uint32_t pending = hfdcan2.Instance->TXBRP;

    if (pending != 0U)
    {
        (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, pending);
    }

    if (HAL_FDCAN_Stop(&hfdcan2) != HAL_OK)
    {
        return 0U;
    }

    memset(&g_can_rx_frame, 0, sizeof(g_can_rx_frame));
    __HAL_FDCAN_CLEAR_FLAG(&hfdcan2, CAN_ACTIVE_NOTIFICATIONS);

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan2, CAN_ACTIVE_NOTIFICATIONS, 0) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void can_recovery_poll(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};
    FDCAN_ErrorCountersTypeDef counters = {0};
    uint32_t now = HAL_GetTick();
    uint32_t pending;

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status) != HAL_OK)
    {
        g_can_hard_fault_active = 1U;
        can_request_recovery(CAN_RECOVERY_REASON_HAL_STATE);
    }
    else
    {
        g_can_bus_off_active = (uint8_t)status.BusOff;

        if (status.BusOff)
        {
            can_request_recovery(CAN_RECOVERY_REASON_BUS_OFF);
        }
        else if (status.ErrorPassive || status.Warning)
        {
            if (g_can_error_state_tick == 0U)
            {
                g_can_error_state_tick = now;
            }
            if ((now - g_can_error_state_tick) >= CAN_ERROR_STATE_GRACE_MS)
            {
                /* Error Warning/Passive can still communicate. Auto
                 * retransmission may leave an unacknowledged frame pending
                 * forever, so cancel only that request and keep FDCAN alive. */
                pending = hfdcan2.Instance->TXBRP;
                if (pending != 0U)
                {
                    (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, pending);
                    printf("[CAN] unacknowledged TX cancelled: passive=%lu warning=%lu TEC=%lu\r\n",
                           (unsigned long)status.ErrorPassive,
                           (unsigned long)status.Warning,
                           (unsigned long)((hfdcan2.Instance->ECR & FDCAN_ECR_TEC) >>
                                           FDCAN_ECR_TEC_Pos));
                }
                g_can_error_state_tick = now;
            }
        }
        else
        {
            g_can_error_state_tick = 0U;
        }
    }

    if (!g_can_recovery_requested)
    {
        return;
    }

    /* A full FIFO caused by an unacknowledged frame is recovered by aborting
     * the pending request. Warning/Passive are not controller lock states. */
    if (!g_can_bus_off_active && !g_can_hard_fault_active &&
        (status.ErrorPassive || status.Warning))
    {
        pending = hfdcan2.Instance->TXBRP;
        if (pending != 0U)
        {
            (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, pending);
        }
        g_can_recovery_requested = 0U;
        g_can_recovery_request_tick = 0U;
        return;
    }

    /* A momentarily full TX FIFO is not a controller fault. If it drains and
     * the protocol state is healthy, resume without restarting FDCAN. */
    if (!g_can_bus_off_active && !g_can_hard_fault_active &&
        !status.ErrorPassive && !status.Warning &&
        (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0U))
    {
        g_can_recovery_requested = 0U;
        g_can_recovery_request_tick = 0U;
        return;
    }

    if ((now - g_can_recovery_request_tick) < CAN_BUS_OFF_RECOVERY_DELAY_MS)
    {
        return;
    }
    if ((g_can_last_recovery_tick != 0U) &&
        ((now - g_can_last_recovery_tick) < CAN_RECOVERY_RETRY_MS))
    {
        return;
    }

    (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &counters);
    printf("[CAN] recovery: reason=0x%02lX HAL[state=%u err=0x%08lX] "
           "busoff=%u passive=%u warning=%u TEC=%lu REC=%lu\r\n",
           (unsigned long)g_can_recovery_reason,
           (unsigned int)HAL_FDCAN_GetState(&hfdcan2),
           (unsigned long)HAL_FDCAN_GetError(&hfdcan2),
           (unsigned int)g_can_bus_off_active,
           (unsigned int)status.ErrorPassive,
           (unsigned int)status.Warning,
           (unsigned long)counters.TxErrorCnt,
           (unsigned long)counters.RxErrorCnt);

    g_can_last_recovery_tick = now;
    if (can_restart_controller())
    {
        g_can_diagnostics.recovery_count++;
        g_can_recovery_requested = 0U;
        g_can_bus_off_active = 0U;
        g_can_hard_fault_active = 0U;
        g_can_recovery_reason = 0U;
        g_can_recovery_request_tick = 0U;
        g_can_error_state_tick = 0U;
        printf("[CAN] recovery complete\r\n");
    }
    else
    {
        g_can_diagnostics.recovery_fail_count++;
        printf("[CAN] recovery failed, retry scheduled\r\n");
    }
}

void can_print_status(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};
    FDCAN_ErrorCountersTypeDef counters = {0};

#if CAN_INTERNAL_LOOPBACK_TEST
    printf("[CAN] mode=INTERNAL_LOOPBACK CCCR=0x%08lX TEST=0x%08lX IR=0x%08lX IE=0x%08lX RXF0S=0x%08lX\r\n",
           (unsigned long)hfdcan2.Instance->CCCR,
           (unsigned long)hfdcan2.Instance->TEST,
           (unsigned long)hfdcan2.Instance->IR,
           (unsigned long)hfdcan2.Instance->IE,
           (unsigned long)hfdcan2.Instance->RXF0S);
#else
    printf("[CAN] mode=NORMAL\r\n");
#endif

    printf("[CAN] kernel clock=%lu Hz nominal=125000 bit/s\r\n",
           (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN));

    if ((HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status) != HAL_OK) ||
        (HAL_FDCAN_GetErrorCounters(&hfdcan2, &counters) != HAL_OK))
    {
        printf("[CAN] status unavailable, HAL state=%u error=0x%08lX\r\n",
               (unsigned int)HAL_FDCAN_GetState(&hfdcan2),
               (unsigned long)HAL_FDCAN_GetError(&hfdcan2));
        return;
    }

    printf("[CAN] busoff=%lu passive=%lu warning=%lu TEC=%lu REC=%lu txfree=%lu\r\n",
           (unsigned long)status.BusOff,
           (unsigned long)status.ErrorPassive,
           (unsigned long)status.Warning,
           (unsigned long)counters.TxErrorCnt,
           (unsigned long)counters.RxErrorCnt,
           (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2));
    printf("[CAN] proto: activity=0x%02lX LEC=%lu DLEC=%lu PB12_RX=%u PSR=0x%08lX CCCR=0x%08lX\r\n",
           (unsigned long)status.Activity,
           (unsigned long)status.LastErrorCode,
           (unsigned long)status.DataLastErrorCode,
           (unsigned int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12),
           (unsigned long)hfdcan2.Instance->PSR,
           (unsigned long)hfdcan2.Instance->CCCR);
    printf("[CAN] tx: TXFQS=0x%08lX TXBRP=0x%08lX TXBTO=0x%08lX TXBCF=0x%08lX\r\n",
           (unsigned long)hfdcan2.Instance->TXFQS,
           (unsigned long)hfdcan2.Instance->TXBRP,
           (unsigned long)hfdcan2.Instance->TXBTO,
           (unsigned long)hfdcan2.Instance->TXBCF);
    printf("[CAN] events: BO=%lu EP=%lu EW=%lu protocol=%lu rx_lost=%lu tx_fail=%lu\r\n",
           (unsigned long)g_can_diagnostics.bus_off_count,
           (unsigned long)g_can_diagnostics.error_passive_count,
           (unsigned long)g_can_diagnostics.error_warning_count,
           (unsigned long)g_can_diagnostics.protocol_error_count,
           (unsigned long)g_can_diagnostics.rx_lost_count,
           (unsigned long)g_can_diagnostics.tx_enqueue_fail_count);
    printf("[CAN] rx: count=%lu last_id=0x%08lX type=%s dlc=%lu\r\n",
           (unsigned long)g_can_diagnostics.rx_frame_count,
           (unsigned long)g_can_diagnostics.last_rx_id,
           (g_can_diagnostics.last_rx_id_type == FDCAN_EXTENDED_ID) ? "EXT" : "STD",
           (unsigned long)g_can_diagnostics.last_rx_dlc);
    printf("[CAN] recovery: ok=%lu failed=%lu pending=%u reason=0x%02lX last=0x%08lX\r\n",
           (unsigned long)g_can_diagnostics.recovery_count,
           (unsigned long)g_can_diagnostics.recovery_fail_count,
           (unsigned int)g_can_recovery_requested,
           (unsigned long)g_can_recovery_reason,
           (unsigned long)g_can_diagnostics.last_error_events);
}

/* ====================== ERROR CALLBACKS =================================== */

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t error = HAL_FDCAN_GetError(hfdcan);
    g_can_diagnostics.last_error_events = error;

    if ((error & (HAL_FDCAN_ERROR_PROTOCOL_ARBT | HAL_FDCAN_ERROR_PROTOCOL_DATA)) != 0U)
    {
        g_can_diagnostics.protocol_error_count++;
    }

    if ((error & HAL_FDCAN_ERROR_FIFO_FULL) != 0U)
    {
        /* Backpressure is not a controller fault. The caller observes a busy
         * return and can retry the command later. */
        g_can_diagnostics.tx_enqueue_fail_count++;
    }

    if ((error & HAL_FDCAN_ERROR_RAM_ACCESS) != 0U)
    {
        g_can_hard_fault_active = 1U;
        can_request_recovery(CAN_RECOVERY_REASON_RAM_ACCESS);
    }
    if ((error & HAL_FDCAN_ERROR_RAM_WDG) != 0U)
    {
        g_can_hard_fault_active = 1U;
        can_request_recovery(CAN_RECOVERY_REASON_RAM_WDG);
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)hfdcan;
    g_can_diagnostics.last_error_events = ErrorStatusITs;

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
        g_can_diagnostics.bus_off_count++;
        g_can_bus_off_active = 1U;
        can_request_recovery(CAN_RECOVERY_REASON_BUS_OFF);
    }
    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U)
    {
        g_can_diagnostics.error_passive_count++;
        if (g_can_error_state_tick == 0U) g_can_error_state_tick = HAL_GetTick();
    }
    if ((ErrorStatusITs & FDCAN_IT_ERROR_WARNING) != 0U)
    {
        g_can_diagnostics.error_warning_count++;
        if (g_can_error_state_tick == 0U) g_can_error_state_tick = HAL_GetTick();
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
