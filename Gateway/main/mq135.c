#include "MQ135.h"
#include <math.h>

void MQ135_init(MQ135_t *sensor, uint8_t pin, float rzero, float rload) {
    sensor->pin = pin;
    sensor->rzero = rzero;
    sensor->rload = rload;
}

float MQ135_getCorrectionFactor(float t, float h) {
    if(t < 20) {
        return CORA * t * t - CORB * t + CORC - (h - 33.)*CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

float MQ135_getResistance(uint8_t pin, float rload, uint32_t val) {
    return ((1023./(float)val) - 1.)*rload;
}

float MQ135_getCorrectedResistance(uint8_t pin, float rload, uint32_t val, float t, float h) {
    return MQ135_getResistance(pin, rload, val)/MQ135_getCorrectionFactor(t, h);
}

float MQ135_getPPM(uint8_t pin, float rzero, float rload, uint32_t val) {
    return PARA * pow((MQ135_getResistance(pin, rload, val)/rzero), -PARB);
}

float MQ135_getCorrectedPPM(uint8_t pin, float rzero, float rload, uint32_t val, float t, float h) {
    return PARA * pow((MQ135_getCorrectedResistance(pin, rload, val, t, h)/rzero), -PARB);
}

float MQ135_getRZero(uint8_t pin, float rload, uint32_t val) {
    return MQ135_getResistance(pin, rload, val) * pow((ATMOCO2/PARA), (1./PARB));
}

float MQ135_getCorrectedRZero(uint8_t pin, float rload, uint32_t val, float t, float h) {
    return MQ135_getCorrectedResistance(pin, rload, val, t, h) * pow((ATMOCO2/PARA), (1./PARB));
}