#ifndef MQ136_H
#define MQ136_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"

#include "rom/ets_sys.h"

#define MQ136_PIN ADC1_CHANNEL_4  // GPIO32 (ADC1_CHANNEL_4) được kết nối với cảm biến MQ136

extern const char *TAG_MQ136;

// Hàm tính toán Rs cho MQ136
float calculateRs_MQ136(int sensorValue);

// Hàm tính toán nồng độ SO2 cho MQ136
float getSO2Concentration(float Rs);

#endif