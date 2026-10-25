#include "hc05.h"

UART_HandleTypeDef huart2;

/* Dedicated buffer for interrupt-based packet transmission */
static uint8_t s_hc05_tx_buf[5];

/**
 * @brief  UART MSP Initialization
 *         Configures GPIO pins and NVIC for USART2.
 * @param  huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART2) {
        /* Enable peripheral clocks */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};

        /*
         * USART2 GPIO Configuration:
         * PA2 -> USART2_TX (Connect to HC-05 RXD): Alternate Function Push-Pull
         * PA3 -> USART2_RX (Connect to HC-05 TXD): Input Floating
         */
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 interrupt configuration */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

/**
 * @brief  UART MSP De-Initialization
 * @param  huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}

/**
 * @brief  Initialise USART2 for HC-05 module (9600 baud, 8N1, no flow control).
 * @retval 0 on success, negative error code on failure.
 */
int hc05_init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief  Send 5-byte airmouse packet over USART2 using interrupt-based transmit.
 *         If previous transmission is still busy, skip packet to never block.
 * @param  dx: Horizontal mouse movement (-127 to 127)
 * @param  dy: Vertical mouse movement (-127 to 127)
 * @param  buttons: Button bitmask (bit0 = left, bit1 = right)
 * @retval 0 on success, -1 if busy (skipped), -2 if transmit error
 */
int hc05_send_mouse(int8_t dx, int8_t dy, uint8_t buttons) {
    /* If previous transmit is still in progress, skip packet */
    if (huart2.gState != HAL_UART_STATE_READY) {
        return -1;
    }

    s_hc05_tx_buf[0] = 0xAA;
    s_hc05_tx_buf[1] = (uint8_t)dx;
    s_hc05_tx_buf[2] = (uint8_t)dy;
    s_hc05_tx_buf[3] = buttons;
    s_hc05_tx_buf[4] = (uint8_t)(s_hc05_tx_buf[0] ^ s_hc05_tx_buf[1] ^ s_hc05_tx_buf[2] ^ s_hc05_tx_buf[3]);

    if (HAL_UART_Transmit_IT(&huart2, s_hc05_tx_buf, 5) != HAL_OK) {
        return -2;
    }
    return 0;
}

/**
 * @brief  USART2 interrupt request handler
 */
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}
