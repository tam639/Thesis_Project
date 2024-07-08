#ifndef UART_H
#define UART_H

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

#define SOFTUART_MAX_UARTS 6
#define SOFTUART_MAX_RX_BUFF 64 //!< Must be power of two: 2, 4, 8, 16 etc.

#define debug(fmt, ...)

#define LENGTH 256
#define BUF_SIZE (1024)

#define TXD_PIN (GPIO_NUM_26)
#define RXD_PIN (GPIO_NUM_25)
#define M0_PIN  (GPIO_NUM_4)
#define M1_PIN  (GPIO_NUM_5)

#define UART UART_NUM_1

#define GPS_UART_NUM UART_NUM_2
#define GPS_UART_TX_PIN 17
#define GPS_UART_RX_PIN 16

#define LENGTH 256

#define TX_BUF_SIZE 128

static const char *TAG_TX = "Transmit UART";

extern int num;
extern uint8_t data[LENGTH];



typedef struct
{
    char receive_buffer[SOFTUART_MAX_RX_BUFF];
    uint8_t receive_buffer_tail;
    uint8_t receive_buffer_head;
    uint8_t buffer_overflow;
} softuart_buffer_t;

typedef struct
{
    uint32_t rx_pin, tx_pin;
    uint32_t baudrate;
    volatile softuart_buffer_t buffer;
    uint16_t bit_time;
    bool invert;
} softuart_t;


extern softuart_t uarts[SOFTUART_MAX_UARTS];

extern softuart_t *uart;

static void tx_task1(void *arg);

static void rx_task1(void *arg);

inline static int8_t find_uart_by_rx(uint8_t rx_pin);

// GPIO interrupt handler
static void handle_rx(void *arg);

static bool check_uart_no(uint8_t uart_no);

static bool check_uart_enabled(uint8_t uart_no);

///////////////////////////////////////////////////////////////////////////////
/// Public
///////////////////////////////////////////////////////////////////////////////

bool softuart_close(uint8_t uart_no);

bool softuart_open(uint8_t uart_no, uint32_t baudrate, uint32_t rx_pin, uint32_t tx_pin, bool invert);

bool softuart_put(uint8_t uart_no, char c);

bool softuart_puts(uint8_t uart_no, const char *s);

bool softuart_putn(uint8_t uart_no, const uint8_t* s, size_t n);

bool softuart_available(uint8_t uart_no);

uint8_t softuart_read(uint8_t uart_no);

void init_uart(void);

// Hàm gửi dữ liệu qua UART
void uart_send_data(const unsigned char *data, size_t len);



#endif