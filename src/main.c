#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_netif.h"

// --- Your WiFi Details --- 
// #define YOUR_WIFI_SSID      "olhar logo"
// #define YOUR_WIFI_PASS      "abaixo !!!"
/* 
Criar um arquivo secrets.ini na raiz do projeto com o seguinte conteúdo:
[secrets]
WIFI_SSID = seu_ssid_aqui  (SEM ASPAS)
WIFI_PASS = sua_senha_aqui (SEM ASPAS)
*/

// --- Lidar UART Config ---
#define LIDAR_UART_NUM      UART_NUM_2
#define LIDAR_UART_BAUD     115200
#define LIDAR_TX_PIN        (GPIO_NUM_16)
#define LIDAR_M_CTR_PIN     (GPIO_NUM_17)

// --- TCP Server Config ---
#define TCP_PORT            8888
#define RX_BUF_SIZE         1024
#define UART_BUF_SIZE       (1024 * 2)

static const char *TAG = "lidar_bridge";
static int g_client_socket = -1;

// --- Task 1: WiFi Connection ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = YOUR_WIFI_SSID,
            .password = YOUR_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// --- Task 2: UART Bridge Logic ---

// Task to read from UART and write to TCP socket
static void uart_to_tcp_task(void *pvParameters) {
    static uint8_t data[UART_BUF_SIZE];
    while (true) {
        if (g_client_socket == -1) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for client
            continue;
        }

        // Read data from UART
        int len = uart_read_bytes(LIDAR_UART_NUM, data, (UART_BUF_SIZE - 1), pdMS_TO_TICKS(20));
        
        if (len > 0) {
            // Send data to TCP
            int written = send(g_client_socket, data, len, 0);
            if (written < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                g_client_socket = -1; // Flag connection as bad
            }
        }
    }
    vTaskDelete(NULL);
}

// Task to read from TCP socket and write to UART
static void tcp_to_uart_task(void *pvParameters) {
    static char rx_buffer[RX_BUF_SIZE];
    while (true) {
        if (g_client_socket == -1) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for client
            continue;
        }

        int len = recv(g_client_socket, rx_buffer, sizeof(rx_buffer) - 1, 0);

        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
            close(g_client_socket);
            g_client_socket = -1;
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
            close(g_client_socket);
            g_client_socket = -1;
        } else {
            ESP_LOGI(TAG, "TCP -> Descartado: Recebido %d bytes:", len);
            ESP_LOG_BUFFER_HEX(TAG, rx_buffer, len);
            // Discard data received from TCP
            // uart_write_bytes(LIDAR_UART_NUM, (const char*)rx_buffer, len);
        }
    }
    vTaskDelete(NULL);
}


// --- Task 3: TCP Server Task ---
static void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(TCP_PORT);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", TCP_PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (true) {
        ESP_LOGI(TAG, "Socket listening... waiting for client.");
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // We only accept one client at a time for this bridge
        if (g_client_socket != -1) {
            ESP_LOGW(TAG, "Got a new connection, but client already connected. Closing new one.");
            close(sock);
            continue;
        }

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Client connected from: %s", addr_str);
        g_client_socket = sock;

        ESP_LOGI(TAG, "Setting M_CTR Pin HIGH");
        gpio_set_level(LIDAR_M_CTR_PIN, 1);

        // Wait until the client disconnects
        while (g_client_socket != -1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(TAG, "Client disconnected.");
        ESP_LOGI(TAG, "Setting M_CTR Pin LOW");
        gpio_set_level(LIDAR_M_CTR_PIN, 0);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

// --- Main App ---
void app_main(void) {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = LIDAR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART_NUM, UART_PIN_NO_CHANGE, LIDAR_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART Configured. Baud: %d", LIDAR_UART_BAUD);

    // Configure M_CTR Pin as output
    ESP_LOGI(TAG, "Configuring M_CTR Pin (GPIO %d) as output", LIDAR_M_CTR_PIN);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;      // Sem interrupção  
    io_conf.mode = GPIO_MODE_OUTPUT;            // Modo de saída
    io_conf.pin_bit_mask = (1ULL << LIDAR_M_CTR_PIN); // Máscara de bits do pino
    io_conf.pull_down_en = 0;                   // Desabilitar pull-down
    io_conf.pull_up_en = 0;                     // Desabilitar pull-up
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(LIDAR_M_CTR_PIN, 0)); // Definir como LOW por padrão

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start WiFi
    wifi_init_sta();
    
    // Start the tasks
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(uart_to_tcp_task, "uart_to_tcp", 2048, NULL, 4, NULL);
    xTaskCreate(tcp_to_uart_task, "tcp_to_uart", 2048, NULL, 4, NULL);
}
