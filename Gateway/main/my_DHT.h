#ifndef MY_DHT_H
#define MY_DHT_H

#include <stdint.h>
#include <stdbool.h>

#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

extern float humidity;
extern float temperature;

float getHumidity();
float getTemperature();
void errorHandler(int response);
int getSignalLevel(int usTimeOut, bool state);
float convertUint8ToFloat(uint8_t highByte, uint8_t lowByte);
int readDHT();
void DHT_task();

#endif /* MY_DHT_H */