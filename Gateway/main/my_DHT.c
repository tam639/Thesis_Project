#include "my_DHT.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

static const char* TAG = "DHT";

#define MAXdhtData 5 // to complete 40 = 5*8 Bits

int DHTgpio = 27;				// default DHT pin = 4
float humidity = 0.;
float temperature = 0.;

// == get temp & hum =============================================
float getHumidity() { return humidity; }
float getTemperature() { return temperature; }

// == error handler ===============================================
void errorHandler(int response) {
    switch(response) {
        case DHT_TIMEOUT_ERROR :
            ESP_LOGE(TAG, "Sensor Timeout\n");
            break;
        case DHT_CHECKSUM_ERROR:
            ESP_LOGE(TAG, "CheckSum error\n");
            break;
        case DHT_OK:
            break;
        default :
            ESP_LOGE(TAG, "Unknown error\n");
    }
}

// Measures how long a signal remains at a level for a given period of time
int getSignalLevel(int usTimeOut, bool state ) {
    int uSec = 0;
    while(gpio_get_level(DHTgpio) == state) {
        if( uSec > usTimeOut )
            return -1;
        ++uSec;
        ets_delay_us(1);		// uSec delay
    }
    return uSec;
}

// Convert 2 uint8 strings to float type
float convertUint8ToFloat(uint8_t highByte, uint8_t lowByte) {
    uint16_t combinedValue = (highByte << 8) | lowByte;
    return (float)combinedValue;
}

int readDHT() {
    int uSec = 0;

    uint8_t dhtData[MAXdhtData];
    uint8_t byteInx = 0;
    uint8_t bitInx = 7;

    // set all elements in the data array of the DHT to the value 0, preparing for storing new data
    for (int k = 0; k < MAXdhtData; k++)
        dhtData[k] = 0;

    // == MCU sends out start signal to DHT sensor ===========
    gpio_set_direction(DHTgpio, GPIO_MODE_OUTPUT);
    // MCU pulls down voltage for at least 18 ms to let DHT detect the signal => choose 3000ms for a smooth and nice wake up 
    gpio_set_level(DHTgpio, 0);
    ets_delay_us(3000);
    // MCU pulls up voltage and waits for DHT response (20 - 40us) => choose 30us
    gpio_set_level(DHTgpio, 1);
    ets_delay_us(30);

     // == switch from output to input mode (to receive data from DHT) ===========
    gpio_set_direction(DHTgpio, GPIO_MODE_INPUT);

    // == DHT sends out response signal & keeps it for 80us ====
    uSec = getSignalLevel(80, 0);
    if(uSec < 0) return DHT_TIMEOUT_ERROR;
    // -- DHT pulls up voltage and keeps it for 80us ------------------------
    uSec = getSignalLevel(80, 1);
    if(uSec < 0) return DHT_TIMEOUT_ERROR;

    /* Start data transmission*/
	// == No errors, read the 40 data bits ================
    for(int k = 0; k < 40; k++)
    {
        // -- starts to transmit 1 bit data (50us)
        uSec = getSignalLevel(50, 0);
        if(uSec < 0) return DHT_TIMEOUT_ERROR;

        // -- check to see if after >70us receive data 
        uSec = getSignalLevel(70 , 1);
        if(uSec < 0) return DHT_TIMEOUT_ERROR;
        else
        {
            // add the current read to the output data
            // because all dhtData array where set to 0 at the start, only look for "1" (>28us us)
            // state: 26-28us voltage-length means data '0'; 70us voltage-length means 1 bit data '1'
            if (uSec > 28)
            {
                dhtData[byteInx] |= (1 << bitInx);
            }
            // index to next byte
            if (bitInx == 0) { bitInx = 7; ++byteInx; }
            else bitInx--;
        }
    }

    // == get humidity from Data[0] and Data[1] ==========================
    humidity = convertUint8ToFloat(dhtData[0], dhtData[1]) / 10;
    // == get temp from Data[2] and Data[3]
    temperature = convertUint8ToFloat(dhtData[2], dhtData[3]) / 10;

    if( dhtData[2] & 0x80 ) // negative temp, brrr it's freezing
        temperature *= -1;

    // == verify if checksum is ok ===========================================
	// Checksum is the sum of Data 8 bits masked out 0xFF => make sure checksum in range 0-255
    if (dhtData[4] == ((dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]) & 0xFF))
        return DHT_OK;
    else
        return DHT_CHECKSUM_ERROR;
}