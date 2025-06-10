#ifndef SX1278_H
#define SX1278_H

#include "main.h"

#define SX1278_REG_OP_MODE 0x01
#define SX1278_REG_FR_MSB 0x06
#define SX1278_REG_PA_CONFIG 0x09
#define SX1278_REG_FIFO 0x00
#define SX1278_REG_FIFO_ADDR_PTR 0x0D
#define SX1278_REG_FIFO_TX_BASE_ADDR 0x0E
#define SX1278_REG_PAYLOAD_LENGTH 0x22
#define SX1278_REG_MODEM_CONFIG_2 0x1D
#define SX1278_REG_PREAMBLE_MSB 0x20
#define SX1278_REG_PREAMBLE_LSB 0x21
#define SX1278_REG_SYNC_WORD 0x39

void sx1278_init(SPI_HandleTypeDef *hspi);
void sx1278_send(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len);

#endif /* SX1278_H */
