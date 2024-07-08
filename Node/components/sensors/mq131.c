#include "mq131.h"

const float R0_MQ131 = 10000.0;

const char *TAG_MQ131 = "MQ131";

// Hàm tính toán Rs cho MQ131
float calculateRs_MQ131(int sensorValue) {
    float Vrl = (float)(sensorValue / 4095.0) * 3.3; // ESP32 có ADC 12-bit
    float Rs = ((3.3 - Vrl) / Vrl) * R0_MQ131;
    return Rs;
}

// Hàm tính toán nồng độ O3 cho MQ131
float getO3Concentration(float Rs) {
    // Ví dụ: Giả định rằng chúng ta có một phép xấp xỉ tuyến tính cho tỷ lệ Rs/R0 để ánh xạ nồng độ O3
    float ratio = Rs / R0_MQ131;
    float o3Concentration = pow(10, ((log10(ratio) - 0.2) / -0.3)); // Công thức chuyển đổi, điều chỉnh dựa trên datasheet
    return o3Concentration;
}