#include "gps.h"
#include "sx1278.h"

uint8_t gpsBuffer[100];
uint8_t gpsIndex = 0;
uint8_t byte;
uint8_t isOutsideFence = 0;
uint32_t lastSendTime = 0;
#define SEND_INTERVAL 30000

float rectLat1 = 21.123000;
float rectLon1 = 105.654000;
float rectLat2 = 21.123000;
float rectLon2 = 105.655000;
float rectLat3 = 21.124000;
float rectLon3 = 105.655000;
float rectLat4 = 21.124000;
float rectLon4 = 105.654000;

int isInsideRectangle(float lat, float lon) {
    float minLat = fmin(fmin(rectLat1, rectLat2), fmin(rectLat3, rectLat4));
    float maxLat = fmax(fmax(rectLat1, rectLat2), fmax(rectLat3, rectLat4));
    float minLon = fmin(fmin(rectLon1, rectLon2), fmin(rectLon3, rectLon4));
    float maxLon = fmax(fmax(rectLon1, rectLon2), fmax(rectLon3, rectLon4));

    return (lat >= minLat && lat <= maxLat && lon >= minLon && lon <= maxLon);
}

int parseGPGGA(char *sentence, float *lat, float *lon, uint32_t *timestamp) {
  char *token = strtok(sentence, ",");
  int field = 0;
  char latDir, lonDir;
  float latDeg, lonDeg;
  char timeStr[10];

  while (token != NULL) {
    if (field == 1) strncpy(timeStr, token, 10);
    if (field == 2) latDeg = atof(token);
    if (field == 3) latDir = token[0];
    if (field == 4) lonDeg = atof(token);
    if (field == 5) lonDir = token[0];
    token = strtok(NULL, ",");
    field++;
  }

  if (field < 6) return 0;

  *lat = (int)(latDeg / 100) + (latDeg - (int)(latDeg / 100) * 100) / 60.0;
  if (latDir == 'S') *lat = -(*lat);
  *lon = (int)(lonDeg / 100) + (lonDeg - (int)(lonDeg / 100) * 100) / 60.0;
  if (lonDir == 'W') *lon = -(*lon);

  int hours = atoi(timeStr) / 10000;
  int minutes = (atoi(timeStr) % 10000) / 100;
  int seconds = atoi(timeStr) % 100;
  *timestamp = hours * 3600 + minutes * 60 + seconds + 1717587397;
  return 1;
}

void sendDataFrame(float lat, float lon, uint32_t timestamp) {
    DataFrame df;
    df.dev_id = 0x02;
    df.timestamp = timestamp;
    df.latitude = lat;
    df.longitude = lon;
    df.checksum = 0;
    for (int i = 0; i < 13; i++) {
        df.checksum ^= ((uint8_t*)&df)[i];
    }
    sx1278_send(&hspi1, (uint8_t*)&df, sizeof(df));
}

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
            if (!isOutsideFence) {
              sendDataFrame(lat, lon, timestamp);
              isOutsideFence = 1;
              lastSendTime = HAL_GetTick();
            } else if (HAL_GetTick() - lastSendTime >= SEND_INTERVAL) {
              sendDataFrame(lat, lon, timestamp);
              lastSendTime = HAL_GetTick();
            }
          } else {
            isOutsideFence = 0;
          }
        }
        gpsIndex = 0;
      }
    } else {
      gpsIndex = 0;
    }
    HAL_UART_Receive_IT(&huart1, &byte, 1);
  }
}
