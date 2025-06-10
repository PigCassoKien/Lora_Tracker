#include "main.h"
#include "system_config.h"
#include "peripherals.h"
#include "sx1278.h"
#include "test_cases.h"

// Định nghĩa các biến toàn cục
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_UART1_MspInit();
  MX_I2C1_MspInit();
  MX_SPI1_MspInit();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  sx1278_init(&hspi1);
  initializeTestCases();

  for (int i = 0; i < NUM_TEST_CASES; i++) {
    sendTestCase(i);
    HAL_Delay(1000); // Wait 1 second between transmissions
  }

  while (1) {
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  }
}
