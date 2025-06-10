#ifndef GPS_H
#define GPS_H

#include "main.h"

#define SEND_INTERVAL 30000

typedef struct {
    uint8_t dev_id;
    uint32_t timestamp;
    float latitude;
    float longitude;
    uint8_t checksum;
} DataFrame;

extern uint8_t gpsBuffer[100];
extern uint8_t gpsIndex;
extern uint8_t byte;
extern uint8_t isOutsideFence;
extern uint32_t lastSendTime;

int isInsideRectangle(float lat, float lon);
int parseGPGGA(char *sentence, float *lat, float *lon, uint32_t *timestamp);
void sendDataFrame(float lat, float lon, uint32_t timestamp);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* GPS_H */
