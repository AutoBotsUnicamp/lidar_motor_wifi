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
#include "driver/ledc.h"
#include <stdlib.h>
#include <stdio.h>

// --- Your WiFi Details --- 
// #define YOUR_WIFI_SSID      "olhar logo"
// #define YOUR_WIFI_PASS      "abaixo !!!"
/* 
Criar um arquivo secrets.ini na raiz do projeto com o seguinte conteúdo:
[secrets]
WIFI_SSID = seu_ssid_aqui  (SEM ASPAS)
WIFI_PASS = sua_senha_aqui (SEM ASPAS)
*/

// --- Lidar Config ---
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

// --- Motor Control Config ---
#define MOTOR_TCP_PORT      8889 // Nova porta para o servidor dos motores
#define MOTOR_RX_BUF_SIZE   128

// --- Pinos do Motor ---
#define MOTOR_L_FWD_PIN     (GPIO_NUM_25) // Motor Esquerdo - Frente
#define MOTOR_L_REV_PIN     (GPIO_NUM_26) // Motor Esquerdo - Ré
#define MOTOR_R_FWD_PIN     (GPIO_NUM_27) // Motor Direito - Frente
#define MOTOR_R_REV_PIN     (GPIO_NUM_33) // Motor Direito - Ré

// --- Configuração do LEDC (PWM) ---
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT // Resolução de 10 bits (0-1023)
#define LEDC_MAX_DUTY       (1023) // (2^10 - 1)
#define LEDC_FREQUENCY      (5000) // Frequência do PWM em Hz

// Canais do LEDC (um para cada pino)
#define LEDC_L_FWD_CHAN     LEDC_CHANNEL_0
#define LEDC_L_REV_CHAN     LEDC_CHANNEL_1
#define LEDC_R_FWD_CHAN     LEDC_CHANNEL_2
#define LEDC_R_REV_CHAN     LEDC_CHANNEL_3

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

/**
 * @brief Configura os 4 canais do LEDC (PWM) para os motores
 */
static void motor_pwm_init(void) {
    // Configuração do Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configuração dos Canais
    ledc_channel_config_t ledc_channel[4] = {
        { // Motor Esquerdo - Frente
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_L_FWD_CHAN,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_L_FWD_PIN,
            .duty           = 0, // Inicia desligado
            .hpoint         = 0
        },
        { // Motor Esquerdo - Ré
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_L_REV_CHAN,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_L_REV_PIN,
            .duty           = 0,
            .hpoint         = 0
        },
        { // Motor Direito - Frente
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_R_FWD_CHAN,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_R_FWD_PIN,
            .duty           = 0,
            .hpoint         = 0
        },
        { // Motor Direito - Ré
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_R_REV_CHAN,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_R_REV_PIN,
            .duty           = 0,
            .hpoint         = 0
        }
    };

    // Aplica a configuração para os 4 canais
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
    }
    ESP_LOGI(TAG, "Controle de Motor (LEDC/PWM) inicializado.");
}

/**
 * @brief Define a velocidade de um motor
 * @param motor_idx 0 para motor esquerdo, 1 para motor direito
 * @param speed Velocidade de -1000 a 1000
 */
static void set_motor_speed(int motor_idx, int speed) {
    // Garante que a velocidade esteja no range -1000 a 1000
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    // Mapeia a velocidade (0-1000) para o duty cycle (0-1023)
    uint32_t duty = (uint32_t)((abs(speed) / 1000.0) * LEDC_MAX_DUTY);

    if (motor_idx == 0) { // Motor Esquerdo
        if (speed > 0) { // Frente
            ledc_set_duty(LEDC_MODE, LEDC_L_FWD_CHAN, duty);
            ledc_set_duty(LEDC_MODE, LEDC_L_REV_CHAN, 0);
        } else if (speed < 0) { // Ré
            ledc_set_duty(LEDC_MODE, LEDC_L_FWD_CHAN, 0);
            ledc_set_duty(LEDC_MODE, LEDC_L_REV_CHAN, duty);
        } else { // Parar
            ledc_set_duty(LEDC_MODE, LEDC_L_FWD_CHAN, 0);
            ledc_set_duty(LEDC_MODE, LEDC_L_REV_CHAN, 0);
        }
        // Atualiza o duty cycle
        ledc_update_duty(LEDC_MODE, LEDC_L_FWD_CHAN);
        ledc_update_duty(LEDC_MODE, LEDC_L_REV_CHAN);
    } 
    else if (motor_idx == 1) { // Motor Direito
        if (speed > 0) { // Frente
            ledc_set_duty(LEDC_MODE, LEDC_R_FWD_CHAN, duty);
            ledc_set_duty(LEDC_MODE, LEDC_R_REV_CHAN, 0);
        } else if (speed < 0) { // Ré
            ledc_set_duty(LEDC_MODE, LEDC_R_FWD_CHAN, 0);
            ledc_set_duty(LEDC_MODE, LEDC_R_REV_CHAN, duty);
        } else { // Parar
            ledc_set_duty(LEDC_MODE, LEDC_R_FWD_CHAN, 0);
            ledc_set_duty(LEDC_MODE, LEDC_R_REV_CHAN, 0);
        }
        // Atualiza o duty cycle
        ledc_update_duty(LEDC_MODE, LEDC_R_FWD_CHAN);
        ledc_update_duty(LEDC_MODE, LEDC_R_REV_CHAN);
    }
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

/**
 * @brief Task do servidor TCP para controle dos motores.
 * Recebe comandos "linear,angular" e os converte em velocidade
 * para o motor esquerdo e direito.
 */
static void motor_tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(MOTOR_TCP_PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "MotorSrv: Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "MotorSrv: Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "MotorSrv: Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "MotorSrv: Socket bound, port %d", MOTOR_TCP_PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "MotorSrv: Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (true) {
        ESP_LOGI(TAG, "MotorSrv: Socket listening... waiting for client.");

        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "MotorSrv: Unable to accept connection: errno %d", errno);
            break; 
        }

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "MotorSrv: Client connected from: %s", addr_str);

        // --- Loop de Recebimento de Comandos ---
        char rx_buffer[MOTOR_RX_BUF_SIZE];
        while (true) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            if (len < 0) {
                ESP_LOGE(TAG, "MotorSrv: recv failed: errno %d", errno);
                break; // Sai do loop de recebimento
            } else if (len == 0) {
                ESP_LOGW(TAG, "MotorSrv: Connection closed");
                break; // Sai do loop de recebimento
            } else {
                rx_buffer[len] = 0; // Adiciona terminador nulo

                // Protocolo simples: "linear,angular\n"
                // Ex: "500,-200"
                int linear_cmd = 0;
                int angular_cmd = 0;

                if (sscanf(rx_buffer, "%d,%d", &linear_cmd, &angular_cmd) == 2) {
                    // Lógica de mixagem (Differential Drive)
                    int left_speed = linear_cmd - angular_cmd;
                    int right_speed = linear_cmd + angular_cmd;

                    // "Clamp" - Limita os valores ao máximo/mínimo
                    left_speed = (left_speed > 1000) ? 1000 : (left_speed < -1000) ? -1000 : left_speed;
                    right_speed = (right_speed > 1000) ? 1000 : (right_speed < -1000) ? -1000 : right_speed;

                    ESP_LOGI(TAG, "MotorCmd: L:%d, A:%d -> Left:%d, Right:%d", linear_cmd, angular_cmd, left_speed, right_speed);

                    // Define a velocidade dos motores
                    set_motor_speed(0, left_speed); // Motor 0 = Esquerdo
                    set_motor_speed(1, right_speed); // Motor 1 = Direito

                } else {
                    ESP_LOGW(TAG, "MotorSrv: Received malformed data: %s", rx_buffer);
                }
            }
        } // Fim do loop de recebimento

        // Cliente desconectou
        close(sock);
        ESP_LOGI(TAG, "MotorSrv: Client disconnected. Stopping motors.");
        // Para os motores por segurança
        set_motor_speed(0, 0);
        set_motor_speed(1, 0);
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

    // Initialize Motor PWM Control
    motor_pwm_init();

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
    xTaskCreate(motor_tcp_server_task, "motor_server", 4096, NULL, 5, NULL);
}
