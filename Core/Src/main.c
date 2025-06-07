#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_spi.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* Private variables */
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

#define ADXL345_ADDR 0x53<<1 // Địa chỉ I2C của ADXL345
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

// Định nghĩa 4 điểm tọa độ của hàng rào ảo (hình chữ nhật)
float rectLat1 = 21.123000; // Góc 1: vĩ độ
float rectLon1 = 105.654000; // Góc 1: kinh độ
float rectLat2 = 21.123000; // Góc 2: vĩ độ
float rectLon2 = 105.655000; // Góc 2: kinh độ
float rectLat3 = 21.124000; // Góc 3: vĩ độ
float rectLon3 = 105.655000; // Góc 3: kinh độ
float rectLat4 = 21.124000; // Góc 4: vĩ độ
float rectLon4 = 105.654000; // Góc 4: kinh độ

uint8_t gpsBuffer[100];
uint8_t gpsIndex = 0;
uint8_t byte;
uint8_t isOutsideFence = 0; // Trạng thái: 0 = trong hàng rào, 1 = ngoài hàng rào
uint32_t lastSendTime = 0; // Thời gian gửi lần cuối (ms)
#define SEND_INTERVAL 30000 // 30 giây

/* Dataframe structure */
typedef struct {
    uint8_t dev_id;
    uint32_t timestamp;
    float latitude;
    float longitude;
    uint8_t checksum;
} DataFrame;

/* Kiểm tra tọa độ có nằm trong hình chữ nhật không */
int isInsideRectangle(float lat, float lon) {
    float minLat = fmin(fmin(rectLat1, rectLat2), fmin(rectLat3, rectLat4));
    float maxLat = fmax(fmax(rectLat1, rectLat2), fmax(rectLat3, rectLat4));
    float minLon = fmin(fmin(rectLon1, rectLon2), fmin(rectLon3, rectLon4));
    float maxLon = fmax(fmax(rectLon1, rectLon2), fmax(rectLon3, rectLon4));

    return (lat >= minLat && lat <= maxLat && lon >= minLon && lon <= maxLon);
}

/* Parse NMEA GPGGA sentence */
int parseGPGGA(char *sentence, float *lat, float *lon, uint32_t *timestamp) {
  char *token = strtok(sentence, ",");
  int field = 0;
  char latDir, lonDir;
  float latDeg, lonDeg;
  char timeStr[10];

  while (token != NULL) {
    if (field == 1) strncpy(timeStr, token, 10); // UTC Time
    if (field == 2) latDeg = atof(token); // Latitude
    if (field == 3) latDir = token[0];   // N/S
    if (field == 4) lonDeg = atof(token); // Longitude
    if (field == 5) lonDir = token[0];   // E/W
    token = strtok(NULL, ",");
    field++;
  }

  if (field < 6) return 0; // Invalid data

  *lat = (int)(latDeg / 100) + (latDeg - (int)(latDeg / 100) * 100) / 60.0;
  if (latDir == 'S') *lat = -(*lat);
  *lon = (int)(lonDeg / 100) + (lonDeg - (int)(lonDeg / 100) * 100) / 60.0;
  if (lonDir == 'W') *lon = -(*lon);

  // Convert UTC time to UNIX timestamp (simplified, assumes date is current)
  int hours = atoi(timeStr) / 10000;
  int minutes = (atoi(timeStr) % 10000) / 100;
  int seconds = atoi(timeStr) % 100;
  *timestamp = hours * 3600 + minutes * 60 + seconds + 1717587397; // Offset from a known date
  return 1;
}

/* Initialize ADXL345 */
// void adxl345_init(I2C_HandleTypeDef *hi2c) {
//   uint8_t data = 0x08; // Bật chế độ đo
//   HAL_I2C_Mem_Write(hi2c, ADXL345_ADDR, 0x2D, 1, &data, 1, 100); // POWER_CTL
// }

/* Read ADXL345 acceleration */
// float adxl345_read(I2C_HandleTypeDef *hi2c) {
//   uint8_t data[6];
//   HAL_I2C_Mem_Read(hi2c, ADXL345_ADDR, 0x32, 1, data, 6, 100); // DATAX0
//   int16_t x = (int16_t)((data[1] << 8) | data[0]);
//   int16_t y = (int16_t)((data[3] << 8) | data[2]);
//   int16_t z = (int16_t)((data[5] << 8) | data[4]);
//   return sqrt(x*x + y*y + z*z) * 0.0039; // Chuyển sang g
// }

/* Initialize SX1278 (LoRa) */
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
  HAL_SPI_Transmit(hspi, (uint8_t[]){SX1278_REG_MODEM_CONFIG_2, 0xC0}, 2, 100); // SF12, CR=4/5
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

/* Send data via LoRa */
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

/* System Clock Configuration */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/* GPIO Initialization */
void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* DIO0 (PB0) as interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* RST (PB1) as output */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* NSS (PA4) as output */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* UART1 MSP Initialization */
void MX_UART1_MspInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* UART1 RX Pin (PA10) */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* UART1 Interrupt Init */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* I2C1 MSP Initialization */
void MX_I2C1_MspInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* I2C1 SCL (PB6) and SDA (PB7) */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* SPI1 MSP Initialization */
void MX_SPI1_MspInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  /* SPI1 SCK (PA5), MISO (PA6), MOSI (PA7), NSS (PA4) */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* UART1 Initialization */
void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart1);
}

/* I2C1 Initialization */
void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  HAL_I2C_Init(&hi2c1);
}

/* SPI1 Initialization */
void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  HAL_SPI_Init(&hspi1);
}

/* EXTI callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_0) {
    // Xử lý ngắt LoRa nếu cần
  }
}

/* Send DataFrame */
void sendDataFrame(float lat, float lon, uint32_t timestamp) {
    DataFrame df;
    df.dev_id = 0x02; // Example device ID
    df.timestamp = timestamp;
    df.latitude = lat;
    df.longitude = lon;
    df.checksum = 0;
    // Calculate checksum (XOR of first 13 bytes)
    for (int i = 0; i < 13; i++) {
        df.checksum ^= ((uint8_t*)&df)[i];
    }
    sx1278_send(&hspi1, (uint8_t*)&df, sizeof(df));
}

/* UART Receive Callback */
/* UART Receive Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (gpsIndex < sizeof(gpsBuffer) - 1) {
      gpsBuffer[gpsIndex++] = byte;
      if (byte == '\n' && gpsIndex > 0 && strstr((char*)gpsBuffer, "$GPGGA")) {
        gpsBuffer[gpsIndex] = '\0';
        float lat, lon;
        uint32_t timestamp;
        if (parseGPGGA((char*)gpsBuffer, &lat, &lon, &timestamp)) {
          if (!isInsideRectangle(lat, lon)) {
            // Con bò ra ngoài hàng rào
            if (!isOutsideFence) {
              // Lần đầu ra ngoài, gửi ngay lập tức
              sendDataFrame(lat, lon, timestamp);
              isOutsideFence = 1;
              lastSendTime = HAL_GetTick();
            } else if (HAL_GetTick() - lastSendTime >= SEND_INTERVAL) {
              // Gửi lại sau mỗi 30 giây nếu vẫn ở ngoài
              sendDataFrame(lat, lon, timestamp);
              lastSendTime = HAL_GetTick();
            }
          } else {
            // Con bò trong hàng rào
            isOutsideFence = 0;
          }
        }
        gpsIndex = 0;
      }
    } else {
      gpsIndex = 0; // Reset nếu buffer đầy
    }
    HAL_UART_Receive_IT(&huart1, &byte, 1); // Kích hoạt lại nhận dữ liệu
  }
}

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

  // adxl345_init(&hi2c1); // Khởi tạo ADXL345
  sx1278_init(&hspi1);

//  uint8_t byte;
  HAL_UART_Receive_IT(&huart1, &byte, 1); // Bắt đầu nhận dữ liệu không blocking

  while (1) {
    // Đọc gia tốc
    // float accel = adxl345_read(&hi2c1);
    // if (accel > 0.5) { // Ngưỡng gia tốc
    //   // Xử lý GPS đã được thực hiện trong callback
    // }

    // Sleep mode
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  }
}
