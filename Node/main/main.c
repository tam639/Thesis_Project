/*******************************************************************************
* Includes
******************************************************************************/
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
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "rom/ets_sys.h"

#include "dht22.h"

#include <ssd1306.h>
#include <font8x8_basic.h>>

#include "mics6814.h"
#include "mq131.h"
#include "mq136.h"

#include "uart.h"

#include "nvs_flash.h"


#define CO_PIN      ADC1_CHANNEL_6 // GPIO34
#define NO2_PIN     ADC1_CHANNEL_7 // GPIO35


// uint16_t pm1_0 = 0;
// uint16_t pm2_5 = 0;
// uint16_t pm10 = 0;

int ret_DHT = 0;

const float S_analog = 1023.0;
// float co, no2, so2, o3;

char utc_time[11];
// float latitude;
// float longitude;

uint8_t data_gps[BUF_SIZE];

static const char *TAG_DHT22 = "DHT22";
static const char *TAG_PMS7003 = "PMS7003";
static const char *TAG_MICS6814 = "MICS6814";
static const char *TAG_GPS = "GPS";

#define NODE_ADDH       0
#define NODE_ADDL       4
#define NODE_CHANNEL    6 
#define NODE_NUMBER     NODE_ADDL

#define GATEWAY_ADDH    0
#define GATEWAY_ADDL    1
#define GATEWAY_CHANNEL 6

// Struct để lưu trữ dữ liệu các thông số
typedef struct {
    uint8_t node_number;
    float temperature;
    float humidity;
    uint16_t pm1;
    uint16_t pm2_5;
    uint16_t pm10;
    float o3;
    float so2;
    float co;
    float no2;
    float latitude;     // Tọa độ latitude (6 chữ số sau dấu phẩy)
    float longitude;    // Tọa độ longitude (6 chữ số sau dấu phẩy)
} SensorData;

SensorData sensor_data;

SensorData sensor_data = {
    .node_number = NODE_NUMBER,  // Thời điểm lấy mẫu
    .temperature = 25.5,      // Nhiệt độ (°C)
    .humidity = 60.5,         // Độ ẩm (%)
    .pm1 = 15,                // Giá trị PM1.0
    .pm2_5 = 25,              // Giá trị PM2.5
    .pm10 = 40,               // Giá trị PM10
    .o3 = 12.34,              // Giá trị O3 (ppm)
    .so2 = 5.67,              // Giá trị SO2 (ppm)
    .co = 0.9,               // Giá trị CO (ppm)
    .no2 = 1.23,               // Giá trị NO2 (ppm)
    .longitude = 20.099951,
    .latitude = 95.250043
};




///////////////////////////////////////////////// AODV /////////////////////////////////////////////////

#define AODV_PORT 654
#define MAX_NEIGHBORS 10
#define MAX_ROUTE_TABLE_ENTRIES 20

#define BROADCAST_ADDH      0xFF
#define BROADCAST_ADDL      0xFF
#define BROADCAST_CHANNEL   0x06

#define FIXED_CHANNEL       0x07

typedef struct {
    uint8_t dest_add;
    uint8_t next_hop;
    uint8_t dest_seq_num;
    uint8_t hop_count;
} route_table_entry_t;

typedef struct {
    uint8_t src_add;
    uint8_t src_seq_num;
    uint8_t broadcast_id;
    uint8_t dest_add;
    uint8_t dest_seq_num;
    uint8_t hop_count;
} rreq_packet_t;

typedef struct {
    uint8_t src_add;
    uint8_t dest_add;
    uint8_t dest_seq_num;
    uint8_t hop_count;
} rrep_packet_t;


static const char *TAG_AODV = "AODV";

route_table_entry_t route_table[MAX_ROUTE_TABLE_ENTRIES];

uint8_t dest_seq_num = 0;

// Function to initialize AODV protocol
void init_aodv() {
    memset(route_table, 0, sizeof(route_table));
    ESP_LOGI(TAG_AODV, "AODV protocol initialized.");
}

// Function to print the routing table
void print_routing_table() {
    ESP_LOGI(TAG_AODV, "Routing Table:");
    ESP_LOGI(TAG_AODV, "---------------------------------------------------");
    ESP_LOGI(TAG_AODV, "Dest Addr | Next Hop | Dest Seq | Hop Count");
    ESP_LOGI(TAG_AODV, "---------------------------------------------------");

    for (int i = 0; i < MAX_ROUTE_TABLE_ENTRIES; i++) {
        if (route_table[i].dest_add != 0) {
            ESP_LOGI(TAG_AODV, "%9d | %8d | %9d | %9d",
                     route_table[i].dest_add,
                     route_table[i].next_hop,
                     route_table[i].dest_seq_num,
                     route_table[i].hop_count);
        }
    }

    ESP_LOGI(TAG_AODV, "---------------------------------------------------");
}


// Function to add a route to the routing table
void add_route(uint8_t dest_add, uint8_t next_hop, uint8_t dest_seq_num, uint8_t hop_count) {
    for (int i = 0; i < MAX_ROUTE_TABLE_ENTRIES; i++) {
        if (route_table[i].dest_add == 0) {
            route_table[i].dest_add = dest_add;
            route_table[i].next_hop = next_hop;
            route_table[i].dest_seq_num = dest_seq_num;
            route_table[i].hop_count = hop_count;
            ESP_LOGI(TAG_AODV, "Route added: %d -> %d (seq: %d, hops: %d)", dest_add, next_hop, dest_seq_num, hop_count);
            return;
        }
    }
    ESP_LOGW(TAG_AODV, "Routing table is full!");
}

// Function to find a route in the routing table
route_table_entry_t* find_route(uint8_t dest_add) {
    for (int i = 0; i < MAX_ROUTE_TABLE_ENTRIES; i++) {
        if (route_table[i].dest_add == dest_add) {
            return &route_table[i];
        }
    }
    return NULL;
}

// Function to check if the routing table is empty
bool is_routing_table_empty() {
    for (int i = 0; i < MAX_ROUTE_TABLE_ENTRIES; i++) {
        if (route_table[i].dest_add != 0) {
            return false; // If at least one entry is not zero, table is not empty
        }
    }
    return true; // All entries are zero, table is empty
}

// Function to broadcast RREQ packets
void send_rreq(rreq_packet_t rreq) {
    ESP_LOGI(TAG_AODV, "Sending RREQ for %d", rreq.dest_add);
    uint8_t message[] = {BROADCAST_ADDH, BROADCAST_ADDL, BROADCAST_CHANNEL, NODE_NUMBER, rreq.src_add, rreq.src_seq_num, rreq.broadcast_id, rreq.dest_add, rreq.dest_seq_num, rreq.hop_count};

    ESP_LOGI(TAG_AODV, "rreq.src_add = %u; rreq.src_seq_num = %u", rreq.src_add, rreq.src_seq_num);
    ESP_LOGI(TAG_AODV, "rreq.broadcast_id = %u; rreq.dest_add = %u", rreq.broadcast_id, rreq.dest_add);
    ESP_LOGI(TAG_AODV, "rreq.dest_seq_num = %u; rreq.hop_count = %u", rreq.dest_seq_num, rreq.hop_count);

    uart_write_bytes(UART, message, sizeof(message)); 
}

// Function to unicast RREP packets
void send_rrep(rrep_packet_t rrep) {
    ESP_LOGI(TAG_AODV, "Sending RREP to %d", rrep.dest_add);
    route_table_entry_t* route = find_route(rrep.dest_add);
    uint8_t message[] = {0x00, route->next_hop, FIXED_CHANNEL, NODE_NUMBER, rrep.src_add, rrep.dest_add, rrep.dest_seq_num, rrep.hop_count};
    ESP_LOGI(TAG_AODV, "next hop = %u", route->next_hop);
    ESP_LOGI(TAG_AODV, "rrep.src_add = %u; rrep.dest_add = %u", rrep.src_add, rrep.dest_add);
    ESP_LOGI(TAG_AODV, "rrep.dest_seq_num = %u; rrep.hop_count = %u", rrep.dest_seq_num, rrep.hop_count);
    
    uart_write_bytes(UART, message, sizeof(message));    

}

// // Function to handle incoming RREQ packets
// void handle_rreq(void *arg) {
//     rreq_packet_t rreq;
//     uint8_t *data = (uint8_t *)malloc(1024);
//     while (1)
//     {
//         const int rxBytes = uart_read_bytes(UART, data, LENGTH, 500 / portTICK_PERIOD_MS);
//         if (rxBytes > 0)
//         {
//             if (rxBytes == 7)
//             {
//                 ESP_LOGI(TAG_AODV,"data 1 = %u", data[1]);
//                 rreq.src_add = data[1]; //
//                 rreq.src_seq_num = data[2];
//                 rreq.broadcast_id = data[3];
//                 rreq.dest_add = data[4]; //
//                 rreq.dest_seq_num = data[5];
//                 rreq.hop_count = data[6];
//                 ESP_LOGI(TAG_AODV, "Handling RREQ from %d to %d", rreq.src_add, rreq.dest_add);
                
//                 // Check if we have a route to the destination
//                 route_table_entry_t* route = find_route(rreq.src_add);
//                 if (rreq.dest_add == NODE_NUMBER)
//                 {
//                     rrep_packet_t rrep;
//                     // Add route & send RREP
//                     rreq.hop_count++;
//                     if (!route)
//                     {
//                         add_route(rreq.src_add, data[0], rreq.dest_seq_num, rreq.hop_count);
//                     }
//                     print_routing_table();

//                     rrep.src_add = rreq.dest_add;
//                     rrep.dest_add = rreq.src_add;
//                     rrep.dest_seq_num = dest_seq_num;
//                     rrep.hop_count = 0;
//                     send_rrep(rrep);
//                 }
//                 else
//                 {
//                     // Broadcast RREQ further
//                     rreq.hop_count++;
//                     if(!route)
//                     {
//                         add_route(rreq.src_add, data[0], rreq.dest_seq_num, rreq.hop_count);
//                     }
//                     print_routing_table();

//                     send_rreq(rreq);         
//                 }
//             }
//             else
//             {
//                 // ESP_LOGI(TAG_AODV, "Error length is not equal to 7.");
//             }
//         }
//         else
//         {
//             // ESP_LOGI(TAG_AODV, "Error length < 0!");
//         }
//     }
//     free(data);
// }

// // Function to handle incoming RREP packets
// void handle_rrep(void *arg) {
//     rrep_packet_t rrep;
//     uint8_t *data = (uint8_t *)malloc(1024);
//     while (1)
//     {
//         const int rxBytes = uart_read_bytes(UART, data, LENGTH, 20 / portTICK_PERIOD_MS);
//         if (rxBytes > 0) {
//         //     char* hex_str = (char*) malloc(2 * rxBytes + 1);
//         //     if (hex_str == NULL) {
//         //         ESP_LOGE(TAG_AODV, "Failed to allocate memory for hex string");
//         //         break;
//         //     }
//         //     for (int i = 0; i < rxBytes; i++) {
//         //         sprintf(&hex_str[i * 2], "%02X", data[i]);
//         //     }
//         //     hex_str[2 * rxBytes] = '\0';
//         //     ESP_LOGI(TAG_AODV, "Read %d bytes: %s", rxBytes, hex_str);

//         // // int length = uart_read_bytes(UART_NUM_1, data, 1024, 20 / portTICK_PERIOD_MS);
//         //     ESP_LOGI(TAG_AODV, "data0 = %u; data1 = %u", data[0], data[1]);
//         //     ESP_LOGI(TAG_AODV, "data2 = %u; data3 = %u", data[2], data[3]);
//         //     ESP_LOGI(TAG_AODV, "data4 = %u; data5 = %u",data[4], data[5]);

//             if (rxBytes == 5)
//             {
//                 rrep.src_add = data[1];
//                 rrep.dest_add = data[2];
//                 rrep.dest_seq_num = data[3];
//                 rrep.hop_count = data[4];
//                 ESP_LOGI(TAG_AODV, "Handling RREP to %d from %d", rrep.dest_add, rrep.src_add);

//                 // Check if we have a route to the destination
//                 route_table_entry_t* route = find_route(rrep.src_add);   

//                 if (rrep.dest_add == NODE_NUMBER)
//                 {
//                     rrep.hop_count++;
//                     if (!route)
//                     {

//                         // Add route to the routing table
//                         add_route(rrep.src_add, data[0], rrep.dest_seq_num, rrep.hop_count);
//                     }
//                     print_routing_table();
//                     ESP_LOGI(TAG_AODV, "This is the right destination!");
//                 }
//                 else 
//                 {
//                     rrep.hop_count++;
//                     // Broadcast RREQ further
//                     if(!route)
//                     {
//                         add_route(rrep.src_add, data[0], rrep.dest_seq_num, rrep.hop_count);
//                     }
//                     print_routing_table();

//                     send_rrep(rrep);                     
//                 }
                
//             }
//             else
//             {
//                 ESP_LOGI(TAG_AODV, "Error length is not equal to 5.");
//             }


//         }
//         else
//         {
//             // ESP_LOGI(TAG_AODV, "Error length < 0!");
//         }
//     }
//      free(data);

// }

void handle_packet(void *arg)
{
    uint8_t *data = (uint8_t *)malloc(1024);
    rreq_packet_t rreq;
    rrep_packet_t rrep;
    while ((1))
    {
        const int rxBytes = uart_read_bytes(UART, data, 1024, 500 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            if (rxBytes == 7)
            {
                rreq.src_add = data[1]; //
                rreq.src_seq_num = data[2];
                rreq.broadcast_id = data[3];
                rreq.dest_add = data[4]; //
                rreq.dest_seq_num = data[5];
                rreq.hop_count = data[6];
                ESP_LOGI(TAG_AODV, "Handling RREQ from %d to %d", rreq.src_add, rreq.dest_add);
                
                // Check if we have a route to the destination
                route_table_entry_t* route = find_route(rreq.src_add);
                if (rreq.dest_add == NODE_NUMBER)
                {
                    rrep_packet_t rrep;
                    // Add route & send RREP
                    rreq.hop_count++;
                    if (!route)
                    {
                        add_route(rreq.src_add, data[0], rreq.dest_seq_num, rreq.hop_count);
                    }
                    print_routing_table();

                    rrep.src_add = rreq.dest_add;
                    rrep.dest_add = rreq.src_add;
                    rrep.dest_seq_num = dest_seq_num;
                    rrep.hop_count = 0;
                    send_rrep(rrep);
                }
                else
                {
                    // Broadcast RREQ further
                    rreq.hop_count++;
                    if (!route)
                    {
                        add_route(rreq.src_add, data[0], rreq.dest_seq_num, rreq.hop_count);
                    }
                    print_routing_table();

                    send_rreq(rreq);         
                }
            }

            if (rxBytes == 5)
            {
                rrep.src_add = data[1];
                rrep.dest_add = data[2];
                rrep.dest_seq_num = data[3];
                rrep.hop_count = data[4];
                ESP_LOGI(TAG_AODV, "Handling RREP to %d from %d", rrep.dest_add, rrep.src_add);

                // Check if we have a route to the destination
                route_table_entry_t* route = find_route(rrep.src_add);   

                if (rrep.dest_add == NODE_NUMBER)
                {
                    rrep.hop_count++;
                    if (!route)
                    {

                        // Add route to the routing table
                        add_route(rrep.src_add, data[0], rrep.dest_seq_num, rrep.hop_count);
                    }
                    print_routing_table();
                    ESP_LOGI(TAG_AODV, "This is the right destination!");
                }
                else 
                {
                    rrep.hop_count++;
                    // Broadcast RREQ further
                    if(!route)
                    {
                        add_route(rrep.src_add, data[0], rrep.dest_seq_num, rrep.hop_count);
                    }
                    print_routing_table();

                    send_rrep(rrep);                     
                }
            }
        }
    }
     free(data);
}

///////////////////////////////////////////////// AODV /////////////////////////////////////////////////




long map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long run = in_max - in_min;
    if(run == 0){
        return -1; // AVR returns -1, SAM returns 0
    }
    const long rise = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void parse_nmea_sentence(const char *nmea) {
    if (strncmp(nmea, "$GPRMC", 6) == 0) {
        char *token;
        char nmea_copy[BUF_SIZE];
        strncpy(nmea_copy, nmea, BUF_SIZE);

        token = strtok(nmea_copy, ","); // $GPRMC
        token = strtok(NULL, ","); // UTC Time
        // char utc_time[11];
        strncpy(utc_time, token, 10);
        utc_time[10] = '\0';

        token = strtok(NULL, ","); // Status (A/V)
        char status = token[0];
        if (status == 'A') { // If status is 'A' (active), parse latitude and longitude
            token = strtok(NULL, ","); // Latitude
            sensor_data.latitude = (((atof(token) - (int)atof(token)) + (int)atof(token) % 100) / 60) + (int)atof(token) / 100;
            token = strtok(NULL, ","); // N/S
            char ns = token[0];

            token = strtok(NULL, ","); // Longitude
            sensor_data.longitude = (((atof(token) - (int)atof(token)) + (int)atof(token) % 100) / 60) + (int)atof(token) / 100;
            token = strtok(NULL, ","); // E/W
            char ew = token[0];

            if (ns == 'S') sensor_data.latitude = -sensor_data.latitude;
            if (ew == 'W') sensor_data.longitude = -sensor_data.longitude;
        }
    }
}

void config_lora(uint8_t add_h, uint8_t add_l, uint8_t channel)
{
    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 1);
    // uint8_t command[] = {0xC0, 0x03, 0x0A, 0x3F, 0x06, 0x47};
    // uint8_t command[] = {0xC1, 0xC1, 0xC1};
    uint8_t command[] = {0xC0, add_h, add_l, 0x1F, channel, 0xC7};
    uart_write_bytes(UART, command, sizeof(command));
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(M0_PIN, 0);
    gpio_set_level(M1_PIN, 0);
}

void init_gpio()
{
    gpio_reset_pin(M0_PIN);
    gpio_reset_pin(M1_PIN);
    gpio_set_direction(M0_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(M1_PIN, GPIO_MODE_OUTPUT);
}
static const char *TAG = "uart_example";

void send_lora(uint8_t send_h, uint8_t send_l, uint8_t send_channel)
{
    uint8_t message[] = {send_h, send_l, send_channel, 0x6c, 0x6f};
    uart_write_bytes(UART, message, sizeof(message));  
    vTaskDelay(100 / portTICK_PERIOD_MS);  
}


// static void receiveRoute_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    
//     gpio_reset_pin(2);
//     gpio_set_direction(2, GPIO_MODE_OUTPUT);
    
//     while (1) {
//         const int rxBytes = uart_read_bytes(UART, data, LENGTH, 500 / portTICK_PERIOD_MS);
//         if (rxBytes > 0) {
//             char* hex_str = (char*) malloc(2 * rxBytes + 1);
//             if (hex_str == NULL) {
//                 ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for hex string");
//                 break;
//             }
//             for (int i = 0; i < rxBytes; i++) {
//                 sprintf(&hex_str[i * 2], "%02X", data[i]);
//             }
//             hex_str[2 * rxBytes] = '\0';
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: %s", rxBytes, hex_str);

//             if (strcmp(hex_str, "6C6F") == 0)
//             {
//                 // Bật đèn
//                 gpio_set_level(2, 1);
//             }

//             free(hex_str);
//         }
//     }
// }


// Hàm mã hóa dữ liệu thành dạng hex và gửi đi qua UART
void encode_and_send_data(SensorData data) {
    // Tạo mảng để lưu trữ dữ liệu hex, bao gồm địa chỉ, kênh và số thứ tự node
    unsigned char hex_data[29]; // 2 byte địa chỉ + 1 byte kênh + 1 byte số thứ tự node + 29 byte dữ liệu

    // Sao chép địa chỉ, kênh và số thứ tự node vào mảng hex_data
    hex_data[0] = GATEWAY_ADDH;
    hex_data[1] = GATEWAY_ADDL;
    hex_data[2] = GATEWAY_CHANNEL;
    hex_data[3] = data.node_number;

    // Mã hóa nhiệt độ và độ ẩm (mỗi giá trị 2 byte)
    uint16_t temp_hex = (uint16_t)(data.temperature * 10); // Lưu nhiệt độ nhân 10 để tránh dấu phẩy
    hex_data[4] = (temp_hex >> 8) & 0xFF;
    hex_data[5] = temp_hex & 0xFF;

    uint16_t humid_hex = (uint16_t)(data.humidity * 10); // Lưu độ ẩm nhân 10 để tránh dấu phẩy
    hex_data[6] = (humid_hex >> 8) & 0xFF;
    hex_data[7] = humid_hex & 0xFF;

    // Mã hóa pm1.0, pm2.5, pm10 (mỗi giá trị 2 byte)
    uint16_t pm1_hex = (uint16_t)data.pm1;
    hex_data[8] = (pm1_hex >> 8) & 0xFF;
    hex_data[9] = pm1_hex & 0xFF;

    uint16_t pm2_5_hex = (uint16_t)data.pm2_5;
    hex_data[10] = (pm2_5_hex >> 8) & 0xFF;
    hex_data[11] = pm2_5_hex & 0xFF;

    uint16_t pm10_hex = (uint16_t)data.pm10;
    hex_data[12] = (pm10_hex >> 8) & 0xFF;
    hex_data[13] = pm10_hex & 0xFF;

    // Mã hóa O3, SO2, CO, NO2 (mỗi giá trị 2 byte)
    uint16_t o3_hex = (uint16_t)(data.o3 * 100); // Lưu O3 nhân 100 để lưu dạng integer
    hex_data[14] = (o3_hex >> 8) & 0xFF;
    hex_data[15] = o3_hex & 0xFF;

    uint16_t so2_hex = (uint16_t)(data.so2 * 100); // Lưu SO2 nhân 100 để lưu dạng integer
    hex_data[16] = (so2_hex >> 8) & 0xFF;
    hex_data[17] = so2_hex & 0xFF;

    uint16_t co_hex = (uint16_t)(data.co * 100); // Lưu CO nhân 100 để lưu dạng integer
    hex_data[18] = (co_hex >> 8) & 0xFF;
    hex_data[19] = co_hex & 0xFF;

    uint16_t no2_hex = (uint16_t)(data.no2 * 100); // Lưu NO2 nhân 100 để lưu dạng integer
    hex_data[20] = (no2_hex >> 8) & 0xFF;
    hex_data[21] = no2_hex & 0xFF;

    // Mã hóa latitude và longitude (mỗi giá trị 4 byte)
    uint32_t lat_hex = (uint32_t)(data.latitude * 1000000); // Lưu latitude nhân 1,000,000 để lưu 6 chữ số sau dấu phẩy
    hex_data[22] = (lat_hex >> 24) & 0xFF;
    hex_data[23] = (lat_hex >> 16) & 0xFF;
    hex_data[24] = (lat_hex >> 8) & 0xFF;
    hex_data[25] = lat_hex & 0xFF;

    uint32_t lon_hex = (uint32_t)(data.longitude * 1000000); // Lưu longitude nhân 1,000,000 để lưu 6 chữ số sau dấu phẩy
    hex_data[26] = (lon_hex >> 24) & 0xFF;
    hex_data[27] = (lon_hex >> 16) & 0xFF;
    hex_data[28] = (lon_hex >> 8) & 0xFF;
    hex_data[29] = lon_hex & 0xFF;

    // Gửi dữ liệu hex qua UART
    uart_write_bytes(UART_NUM_1, (const char *)hex_data, sizeof(hex_data));

    // In thông tin đã gửi
    ESP_LOGI(TAG_TX, "Encoded and sent data over UART.");
}

// Task gửi dữ liệu liên tục mỗi 2 giây
void send_data_task(void *pvParameters) {

    while (1) {
        encode_and_send_data(sensor_data);

        // Chờ 2 giây trước khi gửi lại
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void Calculate_Task(void *arg)
{
    while(1)
    {
        // printf("=== Reading Sensors ===\n");
        ret_DHT = readDHT();
        errorHandler(ret_DHT);

        if (softuart_available(0))
        {
            int len = uarts->buffer.receive_buffer_tail;
            if (len == 32 && uarts->buffer.receive_buffer[0] == 0x42 && uarts->buffer.receive_buffer[1] == 0x4D) {
                sensor_data.pm1 = (uarts->buffer.receive_buffer[10] << 8) + uarts->buffer.receive_buffer[11];
                sensor_data.pm2_5 = (uarts->buffer.receive_buffer[12] << 8) + uarts->buffer.receive_buffer[13];
                sensor_data.pm10 = (uarts->buffer.receive_buffer[14] << 8) + uarts->buffer.receive_buffer[15];

                // ESP_LOGI(TAG_PMS7003, "PM1.0: %d µg/m³", pm1_0);
                // ESP_LOGI(TAG_PMS7003, "PM2.5: %d µg/m³", pm2_5);
                // ESP_LOGI(TAG_PMS7003, "PM10: %d µg/m³", pm10);
            }
            for (int i = 0; i < 64; i++)
            {
                uarts->buffer.receive_buffer[i] = '\0';
            }
            uarts->buffer.receive_buffer_tail = uarts->buffer.receive_buffer_head;
        }

        // Đọc giá trị ADC từ MQ131
        int sensorValue_MQ131 = adc1_get_raw(MQ131_PIN);
        float Rs_MQ131 = calculateRs_MQ131(sensorValue_MQ131);
        sensor_data.o3 = getO3Concentration(Rs_MQ131) / 1000;
        // sensor_data.o3 = adc1_get_raw(MQ131_PIN) + 0.14;


        // Đọc giá trị ADC từ MQ136
        int sensorValue_MQ136 = adc1_get_raw(MQ136_PIN);
        float Rs_MQ136 = calculateRs_MQ136(sensorValue_MQ136);
        sensor_data.so2 = getSO2Concentration(Rs_MQ136);

        if (ret_DHT == DHT_OK) {
            // ESP_LOGI(TAG_DHT22, "Hum %.1f", getHumidity());
            // ESP_LOGI(TAG_DHT22, "Tmp %.1f", getTemperature());
            // printf("\n");
            sensor_data.temperature = getTemperature();
            sensor_data.humidity = getHumidity();
        }

        // ESP_LOGI(TAG_MQ131, "MQ131 - O3 Concentration: %.2f ppb", o3 * 1000); // 1ppm = 1000ppb        printf("\n");
        // ESP_LOGI(TAG_MQ136, "MQ136 - SO2 Concentration: %.2f ppm", so2);        printf("\n");

        sensor_data.co = map (adc1_get_raw(CO_PIN), 0, S_analog, 1, 1000); // Tính toán cacbon monoxit
        sensor_data.no2 = (map (adc1_get_raw(NO2_PIN), 0, S_analog, 5, 1000)) / 100.0 ; // Tính toán Nitơ Dioxide

        // ESP_LOGI(TAG_MICS6814, "MICS6814 - CO Concentration: %.2f ppm", co);
        // ESP_LOGI(TAG_MICS6814, "MICS6814 - NO2 Concentration: %.2f ppm", no2);        printf("\n");

        int length = uart_read_bytes(GPS_UART_NUM, data_gps, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (length > 0) {
            data_gps[length] = '\0';
            // ESP_LOGI(TAG_GPS, "Received: %s", data_gps);
            parse_nmea_sentence((const char *)data_gps);
        }

        // sensor_data.node_number = NODE_NUMBER;
        // sensor_data.temperature = (float)(random() % 400) / 10.0;  // Nhiệt độ ngẫu nhiên từ 0.0 đến 40.0
        // sensor_data.humidity = (float)(random() % 1000) / 10.0;    // Độ ẩm ngẫu nhiên từ 0.0 đến 100.0
        // sensor_data.pm1 = random() % 500;
        // sensor_data.pm2_5 = random() % 500;
        // sensor_data.pm10 = random() % 500;
        // sensor_data.o3 = (float)(random() % 500) / 10.0;
        // sensor_data.so2 = (float)(random() % 500) / 10.0;
        // sensor_data.co = (float)(random() % 500) / 10.0;
        // sensor_data.no2 = (float)(random() % 500) / 10.0;
        // sensor_data.latitude = (float)(random() % 18000000) / 100000.0;  // Vĩ độ ngẫu nhiên từ 0.000000 đến 180.000000
        // sensor_data.longitude = (float)(random() % 36000000) / 100000.0; // Kinh độ ngẫu nhiên từ 0.000000 đến 360.000000

        // -- wait at least 2 sec before reading again ------------
		// The interval of whole process must be beyond 2 seconds !!
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Print_Task(void *arg)
{
    while(1)
    {
        printf("=== Reading Node number %u ===\n", sensor_data.node_number);


        ESP_LOGI(TAG_PMS7003, "PM1.0: %d µg/m³", sensor_data.pm1);
        ESP_LOGI(TAG_PMS7003, "PM2.5: %d µg/m³", sensor_data.pm2_5);
        ESP_LOGI(TAG_PMS7003, "PM10: %d µg/m³", sensor_data.pm10);

        ESP_LOGI(TAG_DHT22, "Hum %.1f", sensor_data.humidity);
        ESP_LOGI(TAG_DHT22, "Tmp %.1f", sensor_data.temperature);

        ESP_LOGI(TAG_MQ131, "MQ131 - O3 Concentration: %.2f ppb", sensor_data.o3); // 1ppm = 1000ppb        printf("\n");

        ESP_LOGI(TAG_MQ136, "MQ136 - SO2 Concentration: %.2f ppm", sensor_data.so2);        printf("\n");

        ESP_LOGI(TAG_MICS6814, "MICS6814 - CO Concentration: %.2f ppm", sensor_data.co);
        ESP_LOGI(TAG_MICS6814, "MICS6814 - NO2 Concentration: %.2f ppm", sensor_data.no2);        printf("\n");

        ESP_LOGI(TAG_GPS, "Received: %s", data_gps);
        ESP_LOGI(TAG_GPS, "UTC Time: %s, Latitude: %f, Longitude: %f", utc_time, sensor_data.latitude, sensor_data.longitude);

        // -- wait at least 2 sec before reading again ------------
		// The interval of whole process must be beyond 2 seconds !!
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


void Display_Task(void *arg)
{
    SSD1306_t dev;
    char lineChar[30];

    ESP_LOGI(tag, "INTERFACE is i2c");
    ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d", SDA_GPIO);
    ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d", SCL_GPIO);
    ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d", RESET_GPIO);
    i2c_master_init(&dev, SDA_GPIO, SCL_GPIO, RESET_GPIO);

    ESP_LOGI(tag, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);

    // Initial example data
    // float temperature = 30.72;
    // float humidity = 89.54;
    // uint16_t pm1 = 12;
    // uint16_t pm2_5 = 34;
    // uint16_t pm10 = 56;
    // float o3 = 0.07;  // ppm
    // float so2 = 0.01; // ppm
    // float co = 0.02;  // ppm
    // float no2 = 0.03; // ppm
    // float latitude = 10.8231;
    // float longitude = 106.6297;

    int display_index = 0;
    const int max_display_index = 1; // Two screens to display

    while (1) {
        // Clear the display before showing new data
        ssd1306_clear_screen(&dev, false);

        if (display_index == 0) {
            // Display temperature, humidity, PM values, and gas concentrations
            sprintf(lineChar, "Temp: %.1fC", sensor_data.temperature);
            display_centered_text(&dev, 0, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "Hum: %.1f%%", sensor_data.humidity);
            display_centered_text(&dev, 1, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "PM1: %uug", sensor_data.pm1);
            display_centered_text(&dev, 2, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "PM2.5: %uug", sensor_data.pm2_5);
            display_centered_text(&dev, 3, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "PM10: %uug", sensor_data.pm10);
            display_centered_text(&dev, 4, lineChar, strlen(lineChar), false);
        } else if (display_index == 1) {
            sprintf(lineChar, "O3: %.2fppm", sensor_data.o3);
            display_centered_text(&dev, 0, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "SO2: %.2fppm", sensor_data.so2);
            display_centered_text(&dev, 1, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "CO: %.2fppm", sensor_data.co);
            display_centered_text(&dev, 2, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "NO2: %.2fppm", sensor_data.no2);
            display_centered_text(&dev, 3, lineChar, strlen(lineChar), false);

            // Display latitude and longitude
            sprintf(lineChar, "(Lat) %.6f", sensor_data.latitude);
            display_centered_text(&dev, 4, lineChar, strlen(lineChar), false);

            sprintf(lineChar, "(Lon) %.6f", sensor_data.longitude);
            display_centered_text(&dev, 5, lineChar, strlen(lineChar), false);
        }

        // Increment display index
        display_index = (display_index + 1) % (max_display_index + 1);

        // Simulate value changes for testing purposes
        // temperature += (rand() % 100 - 50) / 100.0; // Change by -0.50 to +0.50
        // humidity += (rand() % 100 - 50) / 100.0;    // Change by -0.50% to +0.50%
        // pm1 = (pm1 + rand() % 3 - 1) % 100;
        // pm2_5 = (pm2_5 + rand() % 3 - 1) % 100;
        // pm10 = (pm10 + rand() % 3 - 1) % 100;
        // o3 += (rand() % 100 - 50) / 10000.0;        // Change by -0.0050 to +0.0050
        // so2 += (rand() % 100 - 50) / 10000.0;       // Change by -0.0050 to +0.0050
        // co += (rand() % 100 - 50) / 10000.0;        // Change by -0.0050 to +0.0050
        // no2 += (rand() % 100 - 50) / 10000.0;       // Change by -0.0050 to +0.0050
        // latitude += (rand() % 100 - 50) / 1000000.0;
        // longitude += (rand() % 100 - 50) / 1000000.0;

        // Ensure the values stay within reasonable bounds
        // if (temperature < 0) temperature = 0;
        // if (temperature > 50) temperature = 50;
        // if (humidity < 0) humidity = 0;
        // if (humidity > 100) humidity = 100;
        // if (o3 < 0) o3 = 0;
        // if (o3 > 100) o3 = 100;
        // if (so2 < 0) so2 = 0;
        // if (co < 0) co = 0;
        // if (no2 < 0) no2 = 0;
        vTaskDelay(2500 / portTICK_PERIOD_MS); // Delay for 3 seconds
    }
}


// void app_main() {

//     // // Khởi tạo UART với các thiết lập cần thiết
//     // softuart_open(3, 9600, 18, 19, false); // Sử dụng UART số 0, tốc độ baud 9600, tx pin 0, rx pin 1, không đảo
//         // Mở UART mềm
//     if (!softuart_open(0, 9600, 18, 19, 0)) {
//         printf("Failed to open UART\n");
//     }

//     char received_char;
//     char *message = "Hiiiii!\r\n";

//     // Truyền chuỗi qua UART
//     printf("Sending message: %s", message);
//     softuart_puts(0, message);

//     init_uart(); // UART1 & UART2
//     init_gpio();

//     // Cấu hình ADC1 Channel 6 (GPIO34) cho MQ131
//     adc1_config_width(ADC_WIDTH_BIT_12);
//     adc1_config_channel_atten(MQ131_PIN, ADC_ATTEN_DB_11); // 11dB để đọc được điện áp từ 0-3.6V
//     // Cấu hình ADC1 Channel 7 (GPIO35) cho MQ136
//     adc1_config_channel_atten(MQ136_PIN, ADC_ATTEN_DB_11); // 11dB để đọc được điện áp từ 0-3.6V

//     adc1_config_channel_atten(CO_PIN, ADC_ATTEN_DB_11);
//     adc1_config_channel_atten(NO2_PIN, ADC_ATTEN_DB_11);

//     config_lora(NODE_ADDH, NODE_ADDL, NODE_CHANNEL);
//     // send_lora(0, 1, 6);
    

//     // Tạo task để gửi dữ liệu liên tục
//     xTaskCreate(send_data_task, "send_data_task", 2048, NULL, 8, NULL);
    
//     // xTaskCreate(transmitBroadcast_task, "transmitBroadcast_task", 1024*2, NULL, 7, NULL);
//     xTaskCreate(receiveRoute_task, "receiveRoute_task", 1024*2, NULL, 6, NULL);
//     xTaskCreate(Calculate_Task, "Calculate_Task", 1024*4, NULL, 10, NULL);
    
//     xTaskCreate(Print_Task, "Print_Task", 1024*4, NULL, 5, NULL);
//     xTaskCreate(Display_Task, "Display_Task", 1024*4, NULL, 5, NULL);   
//     // xTaskCreate(rx_task3, "Display_Task", 1024*4, NULL, 4, NULL);
// }

void find_route_task(void *arg)
{
    rreq_packet_t rreq_first = {
        .src_add = NODE_NUMBER,
        .src_seq_num = 1,
        .broadcast_id = 1,
        .dest_add = 0x01,
        .dest_seq_num = 1,
        .hop_count = 0
    };
    if (is_routing_table_empty())
    {
        ESP_LOGI(TAG_AODV, "Route table is empty!");
        send_rreq(rreq_first);
    }

    while (1)
    {
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void receiveRoute_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    
    gpio_reset_pin(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    
    while (1) {
        const int rxBytes = uart_read_bytes(UART, data, LENGTH, 500 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            char* hex_str = (char*) malloc(2 * rxBytes + 1);
            if (hex_str == NULL) {
                ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for hex string");
                break;
            }
            for (int i = 0; i < rxBytes; i++) {
                sprintf(&hex_str[i * 2], "%02X", data[i]);
            }
            hex_str[2 * rxBytes] = '\0';
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: %s", rxBytes, hex_str);

            // if (strcmp(hex_str, "6C6F") == 0)
            // {
            //     // Bật đèn
            //     gpio_set_level(2, 1);
            // }

            if (data[0] == 0x04)
            {
                // Bật đèn
                gpio_set_level(2, 1);
            }

            free(hex_str);
        }
    }
}

void app_main() {

    // // Khởi tạo UART với các thiết lập cần thiết
    // softuart_open(3, 9600, 18, 19, false); // Sử dụng UART số 0, tốc độ baud 9600, tx pin 0, rx pin 1, không đảo
        // Mở UART mềm
    if (!softuart_open(0, 9600, 18, 19, 0)) {
        printf("Failed to open UART\n");
    }

    char received_char;
    char *message = "Hiiiii!\r\n";

    // Truyền chuỗi qua UART
    printf("Sending message: %s", message);
    softuart_puts(0, message);

    init_uart(); // UART1 & UART2
    init_gpio();

    // Cấu hình ADC1 Channel 6 (GPIO34) cho MQ131
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ131_PIN, ADC_ATTEN_DB_0); // 11dB để đọc được điện áp từ 0-3.6V
    // Cấu hình ADC1 Channel 7 (GPIO35) cho MQ136
    adc1_config_channel_atten(MQ136_PIN, ADC_ATTEN_DB_11); // 11dB để đọc được điện áp từ 0-3.6V

    adc1_config_channel_atten(CO_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(NO2_PIN, ADC_ATTEN_DB_11);

    // config_lora(NODE_ADDH, NODE_ADDL, NODE_CHANNEL);
    print_routing_table();

    xTaskCreate(find_route_task, "find_route_task", 2048, NULL, 6, NULL);
    // // xTaskCreate(handle_rreq, "handle_rreq", 2048, NULL, 5, NULL);
    // // xTaskCreate(handle_rrep, "handle_rrep", 2048, NULL, 5, NULL);

    xTaskCreate(handle_packet, "handle_packet", 2048, NULL, 5, NULL);


    // Tạo task để gửi dữ liệu liên tục
    xTaskCreate(send_data_task, "send_data_task", 2048, NULL, 8, NULL);
    // xTaskCreate(transmitBroadcast_task, "transmitBroadcast_task", 1024*2, NULL, 7, NULL);
    // xTaskCreate(receiveRoute_task, "receiveRoute_task", 1024*2, NULL, 6, NULL);
    xTaskCreate(Calculate_Task, "Calculate_Task", 1024*4, NULL, 10, NULL);
    xTaskCreate(Print_Task, "Print_Task", 1024*4, NULL, 5, NULL);
    xTaskCreate(Display_Task, "Display_Task", 1024*4, NULL, 5, NULL);   
}