#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "main.h"

void MX_UART1_MspInit(void);
void MX_I2C1_MspInit(void);
void MX_SPI1_MspInit(void);
void MX_USART1_UART_Init(void);
void MX_I2C1_Init(void);
void MX_SPI1_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* PERIPHERALS_H */
