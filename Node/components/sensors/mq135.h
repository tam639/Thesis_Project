// #ifndef MQ135_H
// #define MQ135_H

// #include "driver/adc.h"
// #include "esp_system.h"
// #include "esp_adc_cal.h"

// /// Parameters for calculating ppm of CO2 from sensor resistance
// #define PARA 116.6020682
// #define PARB 2.769034857

// /// Parameters to model temperature and humidity dependence
// #define CORA .00035
// #define CORB .02718
// #define CORC 1.39538
// #define CORD .0018
// #define CORE -.003333333
// #define CORF -.001923077
// #define CORG 1.130128205

// /// Atmospheric CO2 level for calibration purposes
// #define ATMOCO2 415.58 // Global CO2 Aug 2022

// #define DEFAULT_VREF    1100
// #define NO_OF_SAMPLES   64

// #define RZERO                       76.63
// #define RLOAD                       10.0

// #define MQ135_PIN                   32

// typedef struct {
//     uint8_t pin;
//     float rload; // The load resistance on the board in kOhm
//     float rzero; // Calibration resistance at atmospheric CO2 level
// } MQ135_t;

// void MQ135_init(MQ135_t *sensor, uint8_t pin, float rzero, float rload);
// float MQ135_getCorrectionFactor(float t, float h);
// float MQ135_getResistance(uint8_t pin, float rload, uint32_t val);
// float MQ135_getCorrectedResistance(uint8_t pin, float rload, uint32_t val, float t, float h);
// float MQ135_getPPM(uint8_t pin, float rzero, float rload, uint32_t val);
// float MQ135_getCorrectedPPM(uint8_t pin, float rzero, float rload, uint32_t val, float t, float h);
// float MQ135_getRZero(uint8_t pin, float rload, uint32_t val);
// float MQ135_getCorrectedRZero(uint8_t pin, float rload, uint32_t val, float t, float h);

// #endif