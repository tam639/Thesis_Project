#ifndef MQ131_H
#define MQ131_H

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

extern const char *TAG_MQ131;

#define MQ131_PIN ADC1_CHANNEL_5  // GPIO33 (ADC1_CHANNEL_5) được kết nối với cảm biến MQ131

// Hàm tính toán Rs cho MQ131
float calculateRs_MQ131(int sensorValue);

// Hàm tính toán nồng độ O3 cho MQ131
float getO3Concentration(float Rs);

#endif