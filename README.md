# 🤖 autobot\_controller (Firmware da ESP32)

Este repositório contém o firmware da ESP32 para o projeto **Autobot**. Este código é o "cérebro" de baixo nível do robô, responsável por controlar o hardware (motores, encoders, IMU) e atuar como uma ponte de comunicação via Wi-Fi para o computador principal que executa o ROS 2.

Este projeto foi desenvolvido usando **PlatformIO** com o framework ESP-IDF.

Ele foi projetado para funcionar em conjunto com o workspace ROS 2 **[autobot\_ws](https://github.com/AutoBotsUnicamp/autobot_ws)** (substitua pelo link do seu repositório ROS). Este firmware *não* é um nó ROS; ele é o servidor de hardware com o qual os nós ROS se comunicam.

## 🏗️ Arquitetura e Funcionalidades

O firmware implementa uma arquitetura de multitarefa (usando FreeRTOS) que gerencia múltiplas conexões de rede e periféricos de hardware simultaneamente.

### 1\. Conectividade Wi-Fi

  * **Função:** Conecta a ESP32 à sua rede Wi-Fi local como um cliente (station).
  * **Implementação:** `wifi_init_sta()` e `wifi_event_handler()`.
  * **Credenciais:** As credenciais de Wi-Fi **não** estão no código. Elas são gerenciadas pelo PlatformIO através de um arquivo `secrets.ini` (veja a seção de Compilação).

### 2\. Servidor 1: Ponte do Lidar (UART -\> TCP)

  * **Porta:** `TCP:8888`
  * **Função:** Expõe os dados seriais brutos do YDLidar X3 (conectado à UART2) através de um socket TCP.
  * **Implementação:**
      * `tcp_server_task`: Aguarda uma única conexão de cliente (o nó `socat` do ROS).
          * **Ao Conectar:** Aciona o pino `LIDAR_M_CTR_PIN` (GPIO 17) para o nível ALTO, aumentando a velocidade do motor do Lidar.
          * **Ao Desconectar:** Aciona o pino para o nível BAIXO, reduzindo a velocidade do motor do Lidar.
      * `uart_to_tcp_task`: Lê todos os dados que chegam da UART do Lidar e os retransmite imediatamente para o cliente TCP conectado.
      * `tcp_to_uart_task`: Atualmente, descarta quaisquer dados recebidos *do* ROS (a comunicação do Lidar é unidirecional).
  * **Contraparte ROS:** O comando `socat` no `autobot_base.launch.py` se conecta a esta porta para criar o dispositivo serial virtual `/dev/ydlidar_wifi`.

### 3\. Servidor 2: Controle de Motores (TCP -\> PWM)

  * **Porta:** `TCP:8889`
  * **Função:** Recebe comandos de velocidade do ROS e os traduz em sinais PWM para os drivers de motor (Ponte H).
  * **Implementação:**
      * `motor_tcp_server_task`: Aguarda uma única conexão de cliente (o nó `cmd_vel_tcp_bridge_node` do ROS).
      * **Protocolo:** Espera strings formatadas como `"linear,angular\n"`, por exemplo: `"500,-200"`.
      * **Lógica:**
        1.  Analisa a string para extrair os valores `linear_cmd` e `angular_cmd`.
        2.  Realiza a **mixagem de acionamento diferencial**:
              * `left_speed = linear_cmd - angular_cmd`
              * `right_speed = linear_cmd + angular_cmd`
        3.  Limita ("clampa") os valores entre -1000 e 1000.
        4.  Chama `set_motor_speed()` para aplicar a velocidade usando `LEDC` (PWM).
        5.  **Importante:** Armazena o IP deste cliente para ser usado pelo Servidor 3 (Odometria).
  * **Contraparte ROS:** O nó `cmd_vel_tcp_bridge_node` se conecta a esta porta e envia esses comandos.

### 4\. Servidor 3: Publicador de Odometria e IMU (Hardware -\> UDP)

  * **Porta:** `UDP:8890` (enviado para o IP do cliente da porta 8889)
  * **Frequência:** 50Hz (a cada `ODOM_SEND_PERIOD_MS` = 20ms)
  * **Função:** Coleta dados dos sensores (encoders e IMU) em alta frequência e os transmite em um único pacote UDP para o nó ROS.
  * **Implementação:**
      * `odom_udp_task`: Uma task periódica que:
        1.  Só é executada se um cliente estiver conectado ao servidor de motores (`g_ros_client_connected == true`).
        2.  Lê os contadores de pulso (ticks) dos encoders esquerdo e direito usando o periférico `PCNT`.
        3.  Zera os contadores do `PCNT` para a próxima leitura.
        4.  Lê os dados brutos do acelerômetro e giroscópio do MPU-6050 via `I2C`.
        5.  Agrupa todos esses dados (`left_ticks`, `right_ticks`, `imu_raw_data_t`) na struct `robot_data_t`.
        6.  Envia a `struct` inteira como um único pacote binário UDP.
  * **Contraparte ROS:** O nó `odom_udp_receiver_node` escuta na porta UDP 8890 e decodifica exatamente esta `struct`.

### 5\. Lógica Adicional: Proteção de Motor Travado (Stuck Motor)

  * **Função:** Detecta se um motor recebeu um comando de potência, mas não está se movendo (provavelmente preso por atrito ou obstáculo), e aplica um "pulso" de potência máxima para destravá-lo.
  * **Implementação:** Dentro da `odom_udp_task`, uma máquina de estados não-bloqueante:
    1.  **Monitora:** Se `abs(g_current_speed)` (comando) for alto E `count_l/r` (ticks) for zero.
    2.  **Aguarda:** Se essa condição persistir por 100ms (`STUCK_DETECT_COUNT * 20ms`).
    3.  **Ação ("Kick"):** Aplica um pulso de potência máxima (`STUCK_KICK_PWM`) na direção comandada por 50ms (`STUCK_KICK_DURATION_US`).
    4.  **Retorno:** Retorna ao comando de velocidade original.

-----

## 🔌 Mapeamento de Pinos (Hardware)

| Função | Periférico | Pinos da ESP32 |
| :--- | :--- | :--- |
| **Lidar (YDLidar X3)** | UART | TX: `GPIO 16` <br> Motor: `GPIO 17` |
| **IMU (MPU-6050)** | I2C | SDA: `GPIO 21` <br> SCL: `GPIO 22` |
| **Encoder Esquerdo** | PCNT | A: `GPIO 27` <br> B: `GPIO 23` |
| **Encoder Direito** | PCNT | A: `GPIO 14` <br> B: `GPIO 13` |
| **Motor Esquerdo** | LEDC (PWM) | Frente: `GPIO 33` <br> Ré: `GPIO 32` |
| **Motor Direito** | LEDC (PWM) | Frente: `GPIO 26` <br> Ré: `GPIO 25` |

-----

## 🚀 Compilação e Instalação (PlatformIO)

Este projeto é configurado para o VSCode com a extensão **PlatformIO**.

### 1\. Configurar Credenciais de Wi-Fi

Este projeto usa um arquivo `secrets.ini` para gerenciar as credenciais de Wi-Fi, mantendo-as fora do controle de versão.

1.  Crie um arquivo chamado `secrets.ini` na raiz deste projeto (no mesmo nível de `platformio.ini`).

2.  Adicione o seguinte conteúdo, substituindo pelos seus dados:

    ```ini
    [secrets]
    WIFI_SSID = sua_rede_wifi_aqui
    WIFI_PASS = sua_senha_de_wifi_aqui
    ```

    (Nota: O código-fonte (`main.c`) pegará automaticamente essas definições em tempo de compilação através das macros `YOUR_WIFI_SSID` e `YOUR_WIFI_PASS`.)

### 2\. Compilar e Enviar

1.  Conecte sua placa ESP32 via USB.
2.  Abra o projeto no VSCode com PlatformIO.
3.  Use os comandos do PlatformIO:
      * **Build:** Para compilar o firmware.
      * **Upload:** Para compilar e enviar o firmware para a ESP32.
      * **Upload and Monitor:** Para enviar e abrir o monitor serial para ver os logs de depuração.

-----

## 📡 Resumo do Protocolo de Rede

| Porta | Protocolo | Direção | Propósito | Contraparte ROS |
| :--- | :--- | :--- | :--- | :--- |
| **8888** | TCP | ESP ➔ ROS | Dados brutos do Lidar (UART) | `socat` (para `ydlidar_node`) |
| **8889** | TCP | ROS ➔ ESP | Comandos de velocidade (`"linear,angular\n"`) | `cmd_vel_tcp_bridge_node` |
| **8890** | UDP | ESP ➔ ROS | Pacote de Odometria (ticks) + IMU (bruto) | `odom_udp_receiver_node` |