#include "uart.h"

softuart_t uarts[SOFTUART_MAX_UARTS] = { { 0 } };

softuart_t *uart = uarts + 3;

int num = 0;
uint8_t data[LENGTH];

const uart_config_t uart_config_lora = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

const uart_config_t uart_config_gps = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

const uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};


static void tx_task1(void *arg)
{
	char* Txdata = (char*) malloc(30);
    while (1)
    {
    	sprintf (Txdata, "Send to 1: %d\r\n", num++);
    	uart_write_bytes(UART_NUM_1, Txdata, strlen(Txdata));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    free (Txdata);
}

static void rx_task1(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(256+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, 256, 500 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = '\0';
            ESP_LOGI(RX_TASK_TAG, "Received from 1: Read %d bytes: '%s'", rxBytes, data);
        }
    }
    free(data);
}

inline static int8_t find_uart_by_rx(uint8_t rx_pin)
{
    for (uint8_t i = 0; i < SOFTUART_MAX_UARTS; i++)
        if (uarts[i].baudrate && uarts[i].rx_pin == rx_pin) return i;

    return -1;
}

// GPIO interrupt handler
static void handle_rx(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    // find uart
    int8_t uart_no = find_uart_by_rx(gpio_num);
    if (uart_no < 0) return;

    softuart_t *uart = uarts + uart_no;

    // Disable interrupt
    gpio_set_intr_type(gpio_num, GPIO_INTR_DISABLE);
    // gpio_isr_handler_remove(gpio_num);

    // Wait till start bit is half over so we can sample the next one in the center
    ets_delay_us(uart->bit_time / 2);

    // Now sample bits
    uint8_t d = 0;
    uint32_t start_time = 0x7FFFFFFF & esp_timer_get_time();

    for (uint8_t i = 0; i < 8; i++)
    {
        while ((0x7FFFFFFF & esp_timer_get_time()) < (start_time + (uart->bit_time * (i + 1))))
        {
            // If system timer overflow, escape from while loop
            if ((0x7FFFFFFF & esp_timer_get_time()) < start_time)
                break;
        }
        // Shift d to the right
        d >>= 1;

        // Read bit
        if (gpio_get_level(uart->rx_pin) ^ uart->invert)
        {
            // If high, set msb of 8bit to 1
            d |= 0x80;
        }
    }

    // Store byte in buffer
    // If buffer full, set the overflow flag and return
    uint8_t next = (uart->buffer.receive_buffer_tail + 1) % SOFTUART_MAX_RX_BUFF;
    if (next != uart->buffer.receive_buffer_head)
    {
        // save new data in buffer: tail points to where byte goes
        uart->buffer.receive_buffer[uart->buffer.receive_buffer_tail] = d; // save new byte
        uart->buffer.receive_buffer_tail = next;
    }
    else
    {
        uart->buffer.buffer_overflow = 1;
    }

    // Wait for stop bit
    ets_delay_us(uart->bit_time);

    // Done, reenable interrupt
    gpio_set_intr_type(uart->rx_pin, uart->invert ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE);
    // gpio_isr_handler_add(uart->rx_pin, handle_rx, (void *)(uart->rx_pin));
}

static bool check_uart_no(uint8_t uart_no)
{
    if (uart_no >= SOFTUART_MAX_UARTS)
    {
        printf("Invalid uart number %d, %d max", uart_no, SOFTUART_MAX_UARTS);
        return false;
    }

    return true;
}

static bool check_uart_enabled(uint8_t uart_no)
{
    if (!uarts[uart_no].baudrate)
    {
        printf("Uart %d is disabled", uart_no);
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
/// Public
///////////////////////////////////////////////////////////////////////////////

bool softuart_close(uint8_t uart_no)
{
    if (!check_uart_no(uart_no)) return false;
    softuart_t *uart = uarts + uart_no;

    if (!uart->baudrate) return true;

    // Remove interrupt
    gpio_set_intr_type(uart->rx_pin, GPIO_INTR_DISABLE);
    // Mark as unused
    uart->baudrate = 0;
    gpio_uninstall_isr_service();

    return true;
}

bool softuart_open(uint8_t uart_no, uint32_t baudrate, uint32_t rx_pin, uint32_t tx_pin, bool invert)
{
    // do some checks
    if (!check_uart_no(uart_no)) return false;
    if (baudrate == 0)
    {
        printf("Invalid baudrate");
        return false;
    }
    for (uint8_t i = 0; i < SOFTUART_MAX_UARTS; i++)
        if (uarts[i].baudrate && i != uart_no
            && (uarts[i].rx_pin == rx_pin || uarts[i].tx_pin == tx_pin || uarts[i].rx_pin == tx_pin || uarts[i].tx_pin == rx_pin))
        {
            printf("Cannot share pins between uarts");
            return false;
        }

    softuart_close(uart_no);

    softuart_t *uart = uarts + uart_no;

    uart->baudrate = baudrate;
    uart->rx_pin = rx_pin;
    uart->tx_pin = tx_pin;
    uart->invert = invert;

    // Calculate bit_time
    uart->bit_time = (1000000 / baudrate);
    if (((100000000 / baudrate) - (100 * uart->bit_time)) > 50) uart->bit_time++;

    // Setup Rx
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL<<rx_pin;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // gpio_enable(rx_pin, GPIO_MODE_INPUT);
    // gpio_pullup_en(rx_pin);

    // Setup Tx
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL<<tx_pin;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // gpio_enable(tx_pin, GPIO_MODE_OUTPUT);
    // gpio_pullup_en(tx_pin);
    gpio_set_level(tx_pin, !uart->invert);

    //install gpio isr service
    gpio_install_isr_service(0);
    // Setup the interrupt handler to get the start bit
    gpio_set_intr_type(rx_pin, uart->invert ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(rx_pin, handle_rx, (void *)rx_pin);

    ets_delay_us(1000); // TODO: not sure if it really needed

    return true;
}

bool softuart_put(uint8_t uart_no, char c)
{
    if (!check_uart_no(uart_no)) return false;
    if (!check_uart_enabled(uart_no)) return false;
    softuart_t *uart = uarts + uart_no;

    uint32_t start_time = 0x7FFFFFFF & esp_timer_get_time();
    gpio_set_level(uart->tx_pin, uart->invert);

    for (uint8_t i = 0; i <= 8; i++)
    {
        while ((0x7FFFFFFF & esp_timer_get_time()) < (start_time + (uart->bit_time * (i + 1))))
        {
            if ((0x7FFFFFFF & esp_timer_get_time()) < start_time)
                break;
        }
        gpio_set_level(uart->tx_pin, c & ((!uart->invert) << i));
    }

    while ((0x7FFFFFFF & esp_timer_get_time()) < (start_time + (uart->bit_time * 9)))
    {
        if ((0x7FFFFFFF & esp_timer_get_time()) < start_time)
            break;
    }
    gpio_set_level(uart->tx_pin, !uart->invert);
    ets_delay_us(uart->bit_time * 6);

    return true;
}

bool softuart_puts(uint8_t uart_no, const char *s)
{
    while (*s)
    {
        if (!softuart_put(uart_no, *s++))
            return false;
    }

    return true;
}

bool softuart_putn(uint8_t uart_no, const uint8_t* s, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (!softuart_put(uart_no, s[i])) {
            return false;
        }
    }

    return true;
}

bool softuart_available(uint8_t uart_no)
{
    if (!check_uart_no(uart_no)) return false;
    if (!check_uart_enabled(uart_no)) return false;
    softuart_t *uart = uarts + uart_no;

    return (uart->buffer.receive_buffer_tail + SOFTUART_MAX_RX_BUFF - uart->buffer.receive_buffer_head) % SOFTUART_MAX_RX_BUFF;
}

uint8_t softuart_read(uint8_t uart_no)
{
    if (!check_uart_no(uart_no)) return 0;
    if (!check_uart_enabled(uart_no)) return 0;
    softuart_t *uart = uarts + uart_no;

    // Empty buffer?
    if (uart->buffer.receive_buffer_head == uart->buffer.receive_buffer_tail) return 0;

    // Read from "head"
    uint8_t d = uart->buffer.receive_buffer[uart->buffer.receive_buffer_head]; // grab next byte
    uart->buffer.receive_buffer_head = (uart->buffer.receive_buffer_head + 1) % SOFTUART_MAX_RX_BUFF;
    return d;
}

void init_uart(void)
{
    uart_driver_install(UART, LENGTH * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config_lora);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_param_config(GPS_UART_NUM, &uart_config_gps);
    uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// Hàm gửi dữ liệu qua UART
void uart_send_data(const unsigned char *data, size_t len) {
    uart_write_bytes(UART_NUM_1, (const char *)data, len);
}