#include "sx1278.h"

void sx1278_init(SPI_HandleTypeDef *hspi) {
  // Reset module
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(10);

  // LoRa mode, sleep
  uint8_t data[] = {SX1278_REG_OP_MODE, 0x80 | 0x01};
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, data, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Frequency: 433 MHz
  uint64_t frf = ((uint64_t)433000000 << 19) / 32000000;
  uint8_t frf_msb = (frf >> 16) & 0xFF;
  uint8_t frf_mid = (frf >> 8) & 0xFF;
  uint8_t frf_lsb = frf & 0xFF;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_FR_MSB, frf_msb}, 2, 100);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_FR_MSB+1, frf_mid}, 2, 100);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_FR_MSB+2, frf_lsb}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Power: Max
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_PA_CONFIG, 0x7F}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Spreading Factor: SF12, CR4/5
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_MODEM_CONFIG_2, 0xC0}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Preamble length: 8 symbols
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_PREAMBLE_MSB, 0x00}, 2, 100);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_PREAMBLE_LSB, 0x08}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Sync Word: 0x12
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_SYNC_WORD, 0x12}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void sx1278_send(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len) {
  // Standby mode
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_OP_MODE, 0x81}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Configure FIFO
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_FIFO_ADDR_PTR, 0x00}, 2, 100);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_FIFO_TX_BASE_ADDR, 0x00}, 2, 100);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_PAYLOAD_LENGTH, len}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Write data to FIFO
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  for (int i = 0; i < len; i++) {
    HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_FIFO, data[i]}, 2, 100);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // TX mode
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_OP_MODE, 0x83}, 2, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
