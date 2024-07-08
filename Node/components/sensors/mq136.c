#include "mq136.h"

const float R0_MQ136 = 10000.0;

const float Vc = 5.0;
const float RL = 10.0;

const char *TAG_MQ136 = "MQ136";

// Hàm tính toán Rs cho MQ136
float calculateRs_MQ136(int sensorValue) {
    float Vrl = (float)sensorValue / 4095.0 * Vc; // ESP32 có ADC 12-bit
    float Rs = (Vc - Vrl) / Vrl * RL;
    return Rs;
}

// Hàm tính toán nồng độ SO2 cho MQ136
float getSO2Concentration(float Rs) {
    // Ví dụ: Giả định rằng chúng ta có một phép xấp xỉ tuyến tính cho tỷ lệ Rs/RL để ánh xạ nồng độ SO2
    float ratio = Rs / RL;
    float so2Concentration = ratio * 50; // Công thức chuyển đổi, điều chỉnh dựa trên dữ liệu hiệu chuẩn thực tế
    return so2Concentration;
}