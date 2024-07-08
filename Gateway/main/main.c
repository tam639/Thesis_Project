#include <stdint.h>
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <stdio.h>
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"

#include "rom/ets_sys.h"
//// NEW
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "mqtt_client.h"

#include "driver/gpio.h"



#define WIFI_SSID       "Thanh Tam"
#define WIFI_PASS       "06111976"
#define MAXIMUN_ENTRY   10

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1

static const char *TAG_WIFI = "wifi station";
static const char *TAG_MQTT = "MQTT";

static int s_retry_num = 0;

typedef void (*mqtt_handle_t) (char *data, int len);


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

QueueHandle_t sensorDataQueue;

// Định nghĩa các chân ADC cho các cảm biến
#define MQ131_PIN ADC1_CHANNEL_5  // GPIO33 (ADC1_CHANNEL_5) được kết nối với cảm biến MQ131
#define MQ136_PIN ADC1_CHANNEL_4  // GPIO32 (ADC1_CHANNEL_4) được kết nối với cảm biến MQ136

#define CO_PIN      ADC1_CHANNEL_6 // GPIO34
#define NO2_PIN     ADC1_CHANNEL_7 // GPIO35

#define GATEWAY_ADDH    0
#define GATEWAY_ADDL    1
#define GATEWAY_CHANNEL 6
#define NODE_NUMBER     GATEWAY_ADDL

// Điện trở cơ sở trong không khí sạch (Ohms)
const float R0_MQ131 = 10000.0;
const float R0_MQ136 = 10000.0;

// Điện áp mạch và điện trở tải cho MQ136
const float Vc = 5.0;
const float RL = 10.0;

// uint16_t pm1_0 = 0;
// uint16_t pm2_5 = 0;
// uint16_t pm10 = 0;

const float S_analog = 1023.0;
// float co, no2;

static const char *TAG_DHT22 = "DHT22";
static const char *TAG_PMS7003 = "PMS7003";
static const char *TAG_MQ131 = "MQ131";
static const char *TAG_MQ136 = "MQ136";
static const char *TAG_MICS6814 = "MICS6814";
static const char *TAG_GPS = "GPS";

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


static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
   if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
   {
      esp_wifi_connect();
   }
   else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
   {
      if (s_retry_num < MAXIMUN_ENTRY)
      {
         esp_wifi_connect();
         s_retry_num++;
         ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
      }
      else
      {
         xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      }
      ESP_LOGI(TAG_WIFI, "connect to the AP fail");
   }
   else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
   {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(TAG_WIFI, "got IP:" IPSTR, IP2STR(&event->ip_info.ip));
      s_retry_num = 0;
      xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
   }
}

void Init_WiFi(void)
{
   s_wifi_event_group = xEventGroupCreate();

   ESP_ERROR_CHECK(esp_netif_init());

   ESP_ERROR_CHECK(esp_event_loop_create_default());
   esp_netif_create_default_wifi_sta();

   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_ERROR_CHECK(esp_wifi_init(&cfg));

   esp_event_handler_instance_t instance_any_id;
   esp_event_handler_instance_t instance_got_ip;
   ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
   ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));
   wifi_config_t wifi_config = {
      .sta = {
         .ssid = WIFI_SSID,
         .password = WIFI_PASS,
         .threshold.authmode = WIFI_AUTH_WPA2_PSK,

         .pmf_cfg = {
            .capable = true,
            .required = false
         },
      },
   };
   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
   ESP_ERROR_CHECK(esp_wifi_start());

   ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");

   EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

   if (bits & WIFI_CONNECTED_BIT)
   {
      ESP_LOGI(TAG_WIFI, "connected to AP SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
   }
   else if (bits & WIFI_FAIL_BIT)
   {
      ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
   }
   else
   {
      ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
   }

   ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
   ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
   vEventGroupDelete(s_wifi_event_group);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "esp/msg", "data_3", 0, 1, 0);
        ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "esp/msg", 1);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);

      //   msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
      //   ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      //   msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
      //   ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
      //   msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
      //   ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            // log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            // log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            // log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}


void Init_MQTT(void)
{
   esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = "mqtt://broker.hivemq.com:1883",
      // .session.keepalive = 60,
   };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

///////////////////////////////////////////////// AODV /////////////////////////////////////////////////

#define AODV_PORT 654
#define MAX_NEIGHBORS 10
#define MAX_ROUTE_TABLE_ENTRIES 20

#define BROADCAST_ADDH      0xFF
#define BROADCAST_ADDL      0xFF
#define BROADCAST_CHANNEL   0x06

#define FIXED_CHANNEL       0x08

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

uint8_t dest_seq_num = 0x09;

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
void decode_received_data(uint8_t *data, size_t length);

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
                    if(!route)
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

            if (rxBytes > 10)
            {
                decode_received_data(data, rxBytes);
                        // Send sensor data to the queue
                if (xQueueSend(sensorDataQueue, &sensor_data, portMAX_DELAY) == pdPASS)
                {
                    printf("\nAvailable after enqueue: %d \n\n", uxQueueSpacesAvailable(sensorDataQueue));
                }
            }
        }
    }
     free(data);
}
///////////////////////////////////////////////// AODV /////////////////////////////////////////////////

void init_uart(void)
{
    uart_driver_install(UART, LENGTH * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config_lora);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_param_config(GPS_UART_NUM, &uart_config_gps);
    uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void init_gpio(void)
{
    gpio_reset_pin(M0_PIN);
    gpio_reset_pin(M1_PIN);
    gpio_set_direction(M0_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(M1_PIN, GPIO_MODE_OUTPUT);
}

void init_queue(void)
{
   sensorDataQueue = xQueueCreate(20, sizeof(SensorData));

   if (sensorDataQueue == NULL)
   {
      printf("\nError!\n");
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(M0_PIN, 0);
    gpio_set_level(M1_PIN, 0);
}

void send_lora(uint8_t send_h, uint8_t send_l, uint8_t send_channel)
{
    uint8_t message[] = {send_h, send_l, send_channel, 0x6c, 0x6f};
    uart_write_bytes(UART, message, sizeof(message));  
    vTaskDelay(2000 / portTICK_PERIOD_MS);  
}

static void transmitBroadcast_task(void *arg)
{
    uint8_t message[] = {0x00, 0x04, BROADCAST_CHANNEL,GATEWAY_ADDL, 0x01, 0x04, 0x12, 0x00};
    while (1)
    {
        uart_write_bytes(UART, message, sizeof(message));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
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

//             // if (strcmp(hex_str, "6C6F") == 0)
//             // {
//             //     // Bật đèn
//             //     gpio_set_level(2, 1);
//             // }

//             if (data[0] == 0x04)
//             {
//                 // Bật đèn
//                 gpio_set_level(2, 1);
//             }

//             free(hex_str);
//         }
//     }
// }

#define UART_NUM UART_NUM_1
#define RX_BUF_SIZE 1024

static const char *TAG = "uart_example";

// Hàm giải mã dữ liệu nhận được từ UART
void decode_received_data(uint8_t *data, size_t length) {
    // Kiểm tra độ dài dữ liệu
    if (length < 26) {
        ESP_LOGE(TAG, "Received data length is too short");
        return;
    }

    // Giải mã số thứ tự node (1 byte)
    sensor_data.node_number = data[0];

    // Giải mã nhiệt độ và độ ẩm (mỗi giá trị 2 byte)
    uint16_t temp_hex = ((uint16_t)data[1] << 8) | data[2];
    sensor_data.temperature = (float)temp_hex / 10.0f;

    uint16_t humid_hex = ((uint16_t)data[3] << 8) | data[4];
    sensor_data.humidity = (float)humid_hex / 10.0f;

    // Giải mã pm1.0, pm2.5, pm10 (mỗi giá trị 2 byte)
    sensor_data.pm1 = ((uint16_t)data[5] << 8) | data[6];
    sensor_data.pm2_5 = ((uint16_t)data[7] << 8) | data[8];
    sensor_data.pm10 = ((uint16_t)data[9] << 8) | data[10];

    // Giải mã O3, SO2, CO, NO2 (mỗi giá trị 2 byte)
    uint16_t o3_hex = ((uint16_t)data[11] << 8) | data[12];
    sensor_data.o3 = (float)o3_hex / 100.0;

    uint16_t so2_hex = ((uint16_t)data[13] << 8) | data[14];
    sensor_data.so2 = (float)so2_hex / 100.0;

    uint16_t co_hex = ((uint16_t)data[15] << 8) | data[16];
    sensor_data.co = (float)co_hex / 100.0;

    uint16_t no2_hex = ((uint16_t)data[17] << 8) | data[18];
    sensor_data.no2 = (float)no2_hex / 100.0;

    // Giải mã latitude và longitude (mỗi giá trị 4 byte)
    uint32_t lat_hex = ((uint32_t)data[19] << 24) |
                       ((uint32_t)data[20] << 16) |
                       ((uint32_t)data[21] << 8) |
                       (uint32_t)data[22];
    sensor_data.latitude = (float)lat_hex / 1000000.0f;

    uint32_t lon_hex = ((uint32_t)data[23] << 24) |
                       ((uint32_t)data[24] << 16) |
                       ((uint32_t)data[25] << 8) |
                       (uint32_t)data[26];
    sensor_data.longitude = (float)lon_hex / 1000000.0f;
}

void print_value(void)
{
    // In các giá trị đã giải mã
    ESP_LOGI(TAG_AODV, "Node Number: %u", sensor_data.node_number);
    ESP_LOGI(TAG_AODV, "Temperature: %.1f°C, Humidity: %.1f%%", sensor_data.temperature, sensor_data.humidity);
    ESP_LOGI(TAG_AODV, "PM1.0: %u, PM2.5: %u, PM10: %u", sensor_data.pm1, sensor_data.pm2_5, sensor_data.pm10);
    ESP_LOGI(TAG_AODV, "O3: %.2f ppm, SO2: %.2f ppm", sensor_data.o3, sensor_data.so2);
    ESP_LOGI(TAG_AODV, "CO: %.2f ppm, NO2: %.2f ppm", sensor_data.co, sensor_data.no2);
    ESP_LOGI(TAG_AODV, "Latitude: %.6f, Longitude: %.6f", sensor_data.latitude, sensor_data.longitude);
}


// Task để đọc dữ liệu từ UART
void uart_rx_task(void *pvParameters) {
    config_lora(GATEWAY_ADDH, GATEWAY_ADDL, GATEWAY_CHANNEL);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE);
    while (1) {
        // Đọc dữ liệu từ UART
        int length = uart_read_bytes(UART_NUM, data, RX_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if(length > 0)
        {
            decode_received_data(data, length);
                    // Send sensor data to the queue
            if (xQueueSend(sensorDataQueue, &sensor_data, portMAX_DELAY) == pdPASS)
            {
                printf("\nAvailable after enqueue: %d \n\n", uxQueueSpacesAvailable(sensorDataQueue));
            }
        }
    }
    free(data);
}

void mqtt_app_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.hivemq.com:1883",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    // Publish random SensorData every 10 seconds
    char payload[256];
    while (1) {
        SensorData data;
        data.node_number = 1;
        data.temperature = (float)(esp_random() % 400) / 10.0;  // Nhiệt độ ngẫu nhiên từ 0.0 đến 40.0
        data.humidity = (float)(esp_random() % 1000) / 10.0;    // Độ ẩm ngẫu nhiên từ 0.0 đến 100.0
        data.pm1 = esp_random() % 500;
        data.pm2_5 = esp_random() % 500;
        data.pm10 = esp_random() % 500;
        data.o3 = (float)(esp_random() % 500) / 10.0;
        data.so2 = (float)(esp_random() % 500) / 10.0;
        data.co = (float)(esp_random() % 500) / 10.0;
        data.no2 = (float)(esp_random() % 500) / 10.0;
        data.latitude = (float)(esp_random() % 18000000) / 100000.0;  // Vĩ độ ngẫu nhiên từ 0.000000 đến 180.000000
        data.longitude = (float)(esp_random() % 36000000) / 100000.0; // Kinh độ ngẫu nhiên từ 0.000000 đến 360.000000

        snprintf(payload, sizeof(payload), "{\"node_number\": %u, \"temperature\": %.1f, \"humidity\": %.1f, \"pm1\": %u, \"pm2_5\": %u, \"pm10\": %u, \"o3\": %.2f, \"so2\": %.2f, \"co\": %.2f, \"no2\": %.2f, \"latitude\": %.6f, \"longitude\": %.6f}",
                 data.node_number, data.temperature, data.humidity, data.pm1, data.pm2_5, data.pm10, data.o3, data.so2, data.co, data.no2, data.latitude, data.longitude);
        esp_mqtt_client_publish(client, "esp32/sensor_data", payload, 0, 1, 0);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void handle_queue(void *pvParameters)
{
    SensorData receivedData;
   esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameters;
   char data[] = "HELLO";
   char payload[256];
    while (true)
    {
        // SensorData data;
        // data.node_number = 1;
        // data.temperature = (float)(esp_random() % 400) / 10.0;  // Nhiệt độ ngẫu nhiên từ 0.0 đến 40.0
        // data.humidity = (float)(esp_random() % 1000) / 10.0;    // Độ ẩm ngẫu nhiên từ 0.0 đến 100.0
        // data.pm1 = esp_random() % 500;
        // data.pm2_5 = esp_random() % 500;
        // data.pm10 = esp_random() % 500;
        // data.o3 = (float)(esp_random() % 500) / 10.0;
        // data.so2 = (float)(esp_random() % 500) / 10.0;
        // data.co = (float)(esp_random() % 500) / 10.0;
        // data.no2 = (float)(esp_random() % 500) / 10.0;
        // data.latitude = (float)(esp_random() % 18000000) / 100000.0;  // Vĩ độ ngẫu nhiên từ 0.000000 đến 180.000000
        // data.longitude = (float)(esp_random() % 36000000) / 100000.0; // Kinh độ ngẫu nhiên từ 0.000000 đến 360.000000
        
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdPASS)
        {
            if (sensor_data.node_number == 2)
            {
                snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f, \"pm1\": %u, \"pm2_5\": %u, \"pm10\": %u, \"o3\": %.2f, \"so2\": %.2f, \"co\": %.2f, \"no2\": %.2f, \"latitude\": %.6f, \"longitude\": %.6f}",
                    sensor_data.temperature, sensor_data.humidity, sensor_data.pm1, sensor_data.pm2_5, sensor_data.pm10, sensor_data.o3, 
                    sensor_data.so2, sensor_data.co, sensor_data.no2, sensor_data.latitude, sensor_data.longitude);
                int msg_id = esp_mqtt_client_publish(client, "esp32/sensor_data2", payload, 0, 1, 0);
                ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id); 
            }

            if (sensor_data.node_number == 4)
            {
                snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f, \"pm1\": %u, \"pm2_5\": %u, \"pm10\": %u, \"o3\": %.2f, \"so2\": %.2f, \"co\": %.2f, \"no2\": %.2f, \"latitude\": %.6f, \"longitude\": %.6f}",
                    sensor_data.temperature, sensor_data.humidity, sensor_data.pm1, sensor_data.pm2_5, sensor_data.pm10, sensor_data.o3, 
                    sensor_data.so2, sensor_data.co, sensor_data.no2, sensor_data.latitude, sensor_data.longitude);
                int msg_id = esp_mqtt_client_publish(client, "esp32/sensor_data4", payload, 0, 1, 0);
                ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id); 
            }

            printf("\nAvailable after dequeue: %d \n\n", uxQueueSpacesAvailable(sensorDataQueue));
            print_value();
            // vTaskDelay(2000 / portTICK_PERIOD_MS); 
        }
        else 
        {
            printf("Error Transfer!\n");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main() {

   // Init NVS
   esp_err_t ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
   {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
   }

   ESP_ERROR_CHECK(ret);

    // char received_char;
    // char *message = "Hiiiii!\r\n";

    // // Truyền chuỗi qua UART
    // printf("Sending message: %s", message);
   ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
   Init_WiFi();
   // Init_MQTT();
   esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = "mqtt://broker.hivemq.com:1883",
      // .session.keepalive = 60,
   };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);


    init_uart(); // UART1 & UART2
    init_gpio();  
    init_queue();
    
    // // xTaskCreate(transmitBroadcast_task, "transmitBroadcast_task", 1024*2, NULL, 7, NULL);
    // // xTaskCreate(receiveRoute_task, "receiveRoute_task", 1024*2, NULL, 6, NULL);
    // // Tạo task để đọc dữ liệu từ UART
    // // xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 10, NULL);
    xTaskCreate(handle_queue, "handle_queue", 4096, client, 5, NULL);

    xTaskCreate(handle_packet, "handle_packet", 2048, NULL, 8, NULL);
    // // xTaskCreate(handle_rrep, "handle_rrep", 2048, NULL, 4, NULL);


//    xTaskCreate(Sensor_Task, "Sensor_Task", 2048, NULL, 1, NULL);
//    xTaskCreate(Transfer_Task, "Transfer_Task", 4096, client, 5, NULL);
}