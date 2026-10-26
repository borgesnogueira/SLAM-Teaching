#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <string.h>
#include <esp_wifi.h>
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "led_strip.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include <ctype.h>
#include "esp_camera.h"
#include "esp_netif_sntp.h"
#include "freertos/queue.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <unistd.h>

// Correção horizontal da janela do OV2640, aplicada via set_res_raw (sem patch no managed_components).
// Recorta |px| colunas do sensor: px<0 remove à direita (janela vai p/ esquerda), px>0 remove à esquerda.
// Válido apenas para framesizes 4:3 (QVGA, VGA, SVGA, ...).
static int camera_hwindow_correction = 0;

static esp_err_t camera_apply_hwindow_correction(sensor_t *sensor, int px)
{
    framesize_t fs = sensor->status.framesize;
    if (sensor->id.PID != OV2640_PID || resolution[fs].aspect_ratio != ASPECT_RATIO_4X3) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Mesmos valores que ov2640.c:set_framesize usa para 4:3 (1600x1200 em UXGA).
    int mode, max_x, max_y;
    if (fs <= FRAMESIZE_CIF) {
        mode = 2; max_x = 400; max_y = 296;   // OV2640_MODE_CIF
    } else if (fs <= FRAMESIZE_SVGA) {
        mode = 1; max_x = 800; max_y = 600;   // OV2640_MODE_SVGA
    } else {
        mode = 0; max_x = 1600; max_y = 1200; // OV2640_MODE_UXGA
    }

    int crop = abs(px);
    if (crop >= max_x - resolution[fs].width) {
        return ESP_ERR_INVALID_ARG;
    }
    int offset_x = (px > 0) ? px : 0;
    max_x -= crop;

    if (sensor->set_res_raw(sensor, mode, 0, 0, 0, offset_x, 0, max_x, max_y,
                            resolution[fs].width, resolution[fs].height, false, false) != 0) {
        return ESP_FAIL;
    }
    camera_hwindow_correction = px;
    return ESP_OK;
}
// ---------------------------------------------------------- DEFINES
// Configuração Mecânica
#define WHEEL_RADIUS            0.0823f   // Metros (4,15cm)
// #define WHEEL_RADIUS            0.084f   // Metros (4,15cm) FAKE
#define WHEEL_BASE              0.143f   // Metros (14cm entre rodas)

#define ENC_LFT_A_PIN           1
#define ENC_LFT_B_PIN           2
#define MOT_LFT_A_PIN           39
#define MOT_LFT_B_PIN           40
#define MOT_RGH_A_PIN           42
#define MOT_RGH_B_PIN           41
#define ENC_RGH_A_PIN           47
#define ENC_RGH_B_PIN           38
#define WS2812_PIN              48  // WS2812
#define WS2812_LED_NUM          1

// ======== CONFIG CAMERA (ESP32-S3-CAM) ========
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5

#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      11

#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

// PWM config
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Resolução do duty cycle
#define LEDC_FREQUENCY          20000
#define LEDC_CHANNEL_MLFTA      LEDC_CHANNEL_0
#define LEDC_CHANNEL_MLFTB      LEDC_CHANNEL_1
#define LEDC_CHANNEL_MRGHA      LEDC_CHANNEL_2
#define LEDC_CHANNEL_MRGHB      LEDC_CHANNEL_3
#define ESP_INTR_FLAG_DEFAULT   0

// Encoder
#define PCNT_LIMIT              30000
#define ENCODER_EDGES_PER_REV   1496.0f
// Conversão de pulsos para radianos: (2*PI / pulsos_por_volta)
#define ENCODER_PULSE_TO_RAD    (2.0f * M_PI / ENCODER_EDGES_PER_REV)

// Control Unit
#define SAMPLING_PERIOD_MS          25
#define PID_MOTOR_KP                37//45
#define PID_MOTOR_KI                370//450
#define PID_MOTOR_KD                0.01
#define PID_MOTOR_TAU               0//.005
#define PID_LIM_MAX                 255
#define PID_LIM_MIN                 -255

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT     BIT1
#define MQTT_CONNECTED_BIT BIT0
#define MQTT_MSG_RX_BIT   BIT1

#define PUBLISH_PERIOD_MS       100

#define QUEUE_SIZE 2
#define CAMERA_STREAM_FPS 10
#define CAMERA_CAPTURE_PERIOD_MS (1000 / CAMERA_STREAM_FPS)
#define CAMERA_XCLK_HZ 8000000
#define CAMERA_JPEG_QUALITY 18
#define CAMERA_UDP_MAX_PAYLOAD 1200
#define CAMERA_UDP_DEFAULT_PORT 5001

// ---------------------------------------------------------------------- TYPES
typedef struct {
    // --- Hardware Raw Data ---
    int16_t enc_left_delta;
    int16_t enc_right_delta;

    // --- Velocidades Atuais (Medidas) ---
    float wl_meas;          // rad/s (roda esquerda)
    float wr_meas;          // rad/s (roda direita)

    // --- Setpoints (Desejados) ---
    float wl_ref;           // rad/s
    float wr_ref;

    // --- Odometria (Global) ---
    float x;                // m
    float y;                // m
    float theta;            // rad
    uint64_t odom_ts_us;    // timestamp da amostragem usada no cálculo

    // --- Controle ---
    float pwm_left;
    float pwm_right;
} RobotState_t;

typedef struct {
    float Kp, Ki, Kd;
    float Ts, tau;
    float lim_min, lim_max;
} PID_Config;

typedef struct {
    float integrator;
    float differentiator;
    float prev_error;
    float prev_measurement;
    float out;
} PID_State;

typedef struct {
    PID_Config cfg;
    PID_State st;
} PID_t;


// ---------------------------------------------------------- GLOBALS
static const char *TAG_STATE = "STATE MACHINE";
static const char *TAG_WIFI = "WIFI MANAGER";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_CAM = "CAMERA";

// Global State protegida por Mutex
RobotState_t robot;
SemaphoreHandle_t robot_mutex;

static led_strip_handle_t led_strip;

pcnt_unit_handle_t enc_left;
pcnt_unit_handle_t enc_right;

static EventGroupHandle_t s_wifi_event_group;

static esp_mqtt_client_handle_t mqtt_client;

static EventGroupHandle_t s_mqtt_event_group;
esp_timer_handle_t control_timer;

// PIDs globais
PID_t pid_l, pid_r;

/* Controle: só após RUNNING armar o loop (timer + encoders). */
static volatile bool s_control_active = false;

// camera
static QueueHandle_t frame_queue;

typedef struct {
    camera_fb_t *fb;
    uint64_t capture_ts_us;
} CameraFrameQueueItem_t;

typedef struct {
    bool enabled;
    struct sockaddr_in addr;
    socklen_t addr_len;
} UdpStreamDest_t;

// Ver a nova doc, sugere umas coisas bem diferentes
typedef struct __attribute__((packed)) {
    uint32_t frame_id;
    uint64_t timestamp_us;
    uint16_t total_frags;
    uint16_t frag_idx;
    uint16_t payload_len;
} CameraUdpFragHeader_t;

static UdpStreamDest_t s_udp_dest = {0};
static SemaphoreHandle_t s_udp_dest_mutex;
static uint32_t s_frame_seq = 0;
static EventGroupHandle_t s_time_sync_event_group;

#define TIME_SYNC_BIT BIT0
// ---------------------------------------------------------- PROTOTYPES
void  vTaskStateINIT(void *pvParameters);
void  vTaskStateSTANDBY(void *pvParameters);
void  vTaskStateRUNNING(void *pvParameters);
void  udp_stream_task(void *pvParameters);

static bool time_is_valid(void)
{
    time_t now = time(NULL);
    return now >= 1700000000;
}

static esp_err_t start_sntp_and_wait(void)
{
    xEventGroupClearBits(s_time_sync_event_group, TIME_SYNC_BIT);

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.server_from_dhcp = false;
    config.index_of_first_server = 0;
    config.sync_cb = NULL;
    esp_netif_sntp_init(&config);

    ESP_LOGI(TAG_CAM, "Aguardando sincronizacao NTP...");
    esp_err_t err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(15000));

    if (err != ESP_OK && !time_is_valid()) {
        ESP_LOGE(TAG_CAM, "Timeout esperando NTP");
        esp_netif_sntp_deinit();
        return ESP_FAIL;
    }

    while (!time_is_valid()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG_CAM, "Horario sincronizado: %04d-%02d-%02d %02d:%02d:%02d",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    esp_netif_sntp_deinit();
    return ESP_OK;
}

// ---------------------------------------------------------- FUNCTIONS
// -------------------------------------------------- PIN CONFIGURATION
static esp_err_t camera_init(void)
{
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer   = LEDC_TIMER_0,

        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,

        .pin_xclk = XCLK_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,

        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,

        .pin_pwdn  = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,

        // 8MHz reduz ainda mais pressão de barramento/PSRAM.
        .xclk_freq_hz = CAMERA_XCLK_HZ,
        .pixel_format = PIXFORMAT_JPEG,

        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = CAMERA_JPEG_QUALITY,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_CAM, "esp_camera_init falhou: %s", esp_err_to_name(err));
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        ESP_LOGI(TAG_CAM, "Sensor detectado: PID=0x%02x", sensor->id.PID);
        esp_err_t corr_err = camera_apply_hwindow_correction(sensor, -64);
        if (corr_err == ESP_OK) {
            ESP_LOGI(TAG_CAM, "Correção horizontal aplicada: %d px", camera_hwindow_correction);
        } else {
            ESP_LOGW(TAG_CAM, "Correção horizontal não aplicada: %s", esp_err_to_name(corr_err));
        }
    } else {
        ESP_LOGW(TAG_CAM, "esp_camera_sensor_get() retornou NULL");
    }
    ESP_LOGI(TAG_CAM, "Câmera inicializada (QVGA JPEG, xclk=%lu Hz, jpeg_q=%u, fb_count=1)",
             (unsigned long)CAMERA_XCLK_HZ, CAMERA_JPEG_QUALITY);
    return ESP_OK;
}

static void encoder_pin_init(void)
{
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL<<ENC_LFT_A_PIN)|
                        (1ULL<<ENC_LFT_B_PIN)|
                        (1ULL<<ENC_RGH_A_PIN)|
                        (1ULL<<ENC_RGH_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_config);
}

pcnt_unit_handle_t pcnt_init_encoder(gpio_num_t pinA, gpio_num_t pinB, int limit)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = limit,
        .low_limit = -limit,
        .intr_priority = 0,
    };

    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_chan_config_t chA_cfg = {
        .edge_gpio_num  = pinA,
        .level_gpio_num = pinB,
    };
    pcnt_channel_handle_t chA = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chA_cfg, &chA));

    // pcnt_chan_config_t chB_cfg = {
    //     .edge_gpio_num  = pinB,
    //     .level_gpio_num = pinA,
    // };
    // pcnt_channel_handle_t chB = NULL;
    // ESP_ERROR_CHECK(pcnt_new_channel(unit, &chB_cfg, &chB));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    //     chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    // ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    //     chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));

    return unit;
}

void encoders_enable(void)
{
    pcnt_unit_clear_count(enc_left);
    pcnt_unit_clear_count(enc_right);

    pcnt_unit_start(enc_left);
    pcnt_unit_start(enc_right);
}

void encoders_disable(void)
{
    pcnt_unit_stop(enc_left);
    pcnt_unit_stop(enc_right);
}

// Configuração dos pinos de controle dos motores
static void ledc_init(void)
{
    // Configura LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Array de configs de canal
    ledc_channel_config_t ledc_channels[] = {
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_MLFTA, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOT_LFT_A_PIN, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_MLFTB, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOT_LFT_B_PIN, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_MRGHA, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOT_RGH_A_PIN, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_MRGHB, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOT_RGH_B_PIN, .duty = 0, .hpoint = 0}
    };

    // Aplicar as configs de canal
    for (size_t i = 0; i < sizeof(ledc_channels)/sizeof(ledc_channels[0]); ++i) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channels[i]));
    }
}

void set_motor(int pwm, int channelA, int channelB)
{
    if(pwm >= 0){
        ledc_set_duty(LEDC_MODE, channelA, pwm);
        ledc_set_duty(LEDC_MODE, channelB, 0);
    } else {
        ledc_set_duty(LEDC_MODE, channelA, 0);
        ledc_set_duty(LEDC_MODE, channelB, abs(pwm));
    }
    ledc_update_duty(LEDC_MODE, channelA);
    ledc_update_duty(LEDC_MODE, channelB);
}

void drive_motors(int left_pwm, int right_pwm){
    set_motor(left_pwm, LEDC_CHANNEL_MLFTA, LEDC_CHANNEL_MLFTB);
    set_motor(right_pwm, LEDC_CHANNEL_MRGHA, LEDC_CHANNEL_MRGHB);
}

void set_motor_pwm(int pwm, int chA, int chB) {
    if (pwm >= 0) {
        ledc_set_duty(LEDC_MODE, chA, pwm);
        ledc_set_duty(LEDC_MODE, chB, 0);
    } else {
        ledc_set_duty(LEDC_MODE, chA, 0);
        ledc_set_duty(LEDC_MODE, chB, abs(pwm));
    }
    ledc_update_duty(LEDC_MODE, chA);
    ledc_update_duty(LEDC_MODE, chB);
}

void apply_motors(float left, float right) {
    set_motor_pwm((int)left, LEDC_CHANNEL_MLFTA, LEDC_CHANNEL_MLFTB);
    set_motor_pwm((int)right, LEDC_CHANNEL_MRGHA, LEDC_CHANNEL_MRGHB);
}

// ------------------------------------------------------------ CONTROL
void PID_init(PID_t *pid, PID_Config cfg) {
    pid->cfg = cfg;
    pid->st.integrator = 0.0f;
    pid->st.differentiator = 0.0f;
    pid->st.prev_error = 0.0f;
    pid->st.prev_measurement = 0.0f;
    pid->st.out = 0.0f;
}

void PID_reset(PID_t *pid) {
    pid->st.integrator = 0.0f;
    pid->st.differentiator = 0.0f;
    pid->st.prev_error = 0.0f;
    pid->st.prev_measurement = 0.0f;
    pid->st.out = 0.0f;
}

float PID_update(PID_t *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    float proportional = pid->cfg.Kp * error;

    // Tustin (Trapezoidal) Integrator
    pid->st.integrator += 0.5f * pid->cfg.Ki * pid->cfg.Ts * (error + pid->st.prev_error);

    // Anti-windu´p
    float lim_max_int = pid->cfg.lim_max > proportional ? pid->cfg.lim_max - proportional : 0.0f;
    float lim_min_int = pid->cfg.lim_min < proportional ? pid->cfg.lim_min - proportional : 0.0f;

    if (pid->st.integrator > lim_max_int) pid->st.integrator = lim_max_int;
    if (pid->st.integrator < lim_min_int) pid->st.integrator = lim_min_int;

    // Derivative (com filtro passa-baixa)
    pid->st.differentiator = -(2.0f * pid->cfg.Kd * (measurement - pid->st.prev_measurement)
                             + (2.0f * pid->cfg.tau - pid->cfg.Ts) * pid->st.differentiator)
                             / (2.0f * pid->cfg.tau + pid->cfg.Ts);

    float output = proportional + pid->st.integrator + pid->st.differentiator;

    // Saturação Saída
    if (output > pid->cfg.lim_max) output = pid->cfg.lim_max;
    if (output < pid->cfg.lim_min) output = pid->cfg.lim_min;

    pid->st.prev_error = error;
    pid->st.prev_measurement = measurement;
    pid->st.out = output;

    return output;
}

// -------------------------------------------- STATE MACHINE FUNCTIONS
void ws2812_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_PIN,
        .max_leds = WS2812_LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .mem_block_symbols = 64,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

static float cjson_item_to_float(const cJSON *item)
{
    if (!item || cJSON_IsNull(item)) {
        return 0.f;
    }
    if (cJSON_IsNumber(item)) {
        return (float)item->valuedouble;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        return (float)atof(item->valuestring);
    }
    return 0.f;
}

static bool parse_udp_destination_from_payload(const char *payload, struct sockaddr_in *out_addr)
{
    if (!payload || !out_addr) {
        return false;
    }

    char ip_str[16] = {0};
    int port = CAMERA_UDP_DEFAULT_PORT;

    cJSON *root = cJSON_Parse(payload);
    if (root) {
        cJSON *ip = cJSON_GetObjectItem(root, "ip");
        cJSON *port_item = cJSON_GetObjectItem(root, "port");
        if (cJSON_IsString(ip) && ip->valuestring) {
            strncpy(ip_str, ip->valuestring, sizeof(ip_str) - 1);
        }
        if (cJSON_IsNumber(port_item)) {
            port = (int)port_item->valuedouble;
        } else if (cJSON_IsString(port_item) && port_item->valuestring) {
            port = atoi(port_item->valuestring);
        }
        cJSON_Delete(root);
    } else {
        if (sscanf(payload, "%15[^:]:%d", ip_str, &port) < 1) {
            return false;
        }
    }

    if (port <= 0 || port > 65535) {
        ESP_LOGW(TAG_CAM, "Destino UDP invalido (port=%d)", port);
        return false;
    }

    memset(out_addr, 0, sizeof(*out_addr));
    out_addr->sin_family = AF_INET;
    out_addr->sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip_str, &out_addr->sin_addr) != 1) {
        ESP_LOGW(TAG_CAM, "Destino UDP invalido (ip='%s')", ip_str);
        return false;
    }

    return true;
}

static void set_udp_destination(const struct sockaddr_in *addr)
{
    if (xSemaphoreTake(s_udp_dest_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    if (addr) {
        s_udp_dest.enabled = true;
        s_udp_dest.addr = *addr;
        s_udp_dest.addr_len = sizeof(*addr);
    } else {
        s_udp_dest.enabled = false;
        s_udp_dest.addr_len = 0;
        memset(&s_udp_dest.addr, 0, sizeof(s_udp_dest.addr));
    }

    xSemaphoreGive(s_udp_dest_mutex);
}
// ------------------------------------------- SUPERVISOR FUNCTIONS

void control_timer_callback(void* arg) {
    if (!s_control_active) {
        return;
    }

    // Aguarda mutex antes de ler/zerar PCNT para não perder pulsos se este ciclo falhar.
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    int count_l = 0, count_r = 0;
    pcnt_unit_get_count(enc_left, &count_l);
    pcnt_unit_get_count(enc_right, &count_r);
    pcnt_unit_clear_count(enc_left);
    pcnt_unit_clear_count(enc_right);

    robot.enc_left_delta = count_l;
    robot.enc_right_delta = count_r;

    struct timeval odom_tv = {0};
    gettimeofday(&odom_tv, NULL);
    robot.odom_ts_us = (uint64_t)odom_tv.tv_sec * 1000000ULL + (uint64_t)odom_tv.tv_usec;

    /*
    * 2. Cálculo de Velocidade Angular (rad/s)
    * w = (pulsos * fator_conversao) / periodo
    */
    float dt = SAMPLING_PERIOD_MS / 1000.0f;
    robot.wl_meas = (count_l * ENCODER_PULSE_TO_RAD) / dt;
    robot.wr_meas = (count_r * ENCODER_PULSE_TO_RAD) / dt;

    // 3. PID Update (references are in rad/s)
    robot.pwm_left = PID_update(&pid_l, robot.wl_ref, robot.wl_meas);
    robot.pwm_right = PID_update(&pid_r, robot.wr_ref, robot.wr_meas);

    // 4. Aplicar Motores
    apply_motors(robot.pwm_left, robot.pwm_right);

    // 5. Odometria (Cinemática Diferencial)
    float v_lin = (WHEEL_RADIUS * 0.5f) * (robot.wr_meas + robot.wl_meas);
    float w_ang = (WHEEL_RADIUS / WHEEL_BASE) * (robot.wr_meas - robot.wl_meas);

    robot.theta += w_ang * dt;
    if(robot.theta > M_PI) robot.theta -= 2*M_PI;
    else if(robot.theta <= -M_PI) robot.theta += 2*M_PI;

    robot.x += v_lin * cosf(robot.theta) * dt;
    robot.y += v_lin * sinf(robot.theta) * dt;

    xSemaphoreGive(robot_mutex);
}

// ------------------------------------------------- WIFI FUNCTIONS

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        // ESP_LOGE("WIFI", "Falha ao conectar. Razão: %d", event_data->reason);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// NVS utils ---------------------------------------------------------------
bool nvs_read_wifi(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi", NVS_READONLY, &handle);
    if (err != ESP_OK) return false;

    err = nvs_get_str(handle, "ssid", ssid, &ssid_len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }
    err = nvs_get_str(handle, "pass", pass, &pass_len);
    nvs_close(handle);
    return err == ESP_OK;
}

void nvs_write_wifi(const char *ssid, const char *pass)
{
    nvs_handle_t handle;
    nvs_open("wifi", NVS_READWRITE, &handle);
    nvs_set_str(handle, "ssid", ssid);
    nvs_set_str(handle, "pass", pass);
    nvs_commit(handle);
    nvs_close(handle);
}

// HTTP Handlers ---------------------------------------------------------
static void url_decode_inplace(char *str)
{
    char *src = str;
    char *dst = str;

    while (*src) {
        if (*src == '%' &&
            isxdigit((unsigned char)src[1]) &&
            isxdigit((unsigned char)src[2])) {
            int hi = (src[1] <= '9') ? (src[1] - '0') : (tolower((unsigned char)src[1]) - 'a' + 10);
            int lo = (src[2] <= '9') ? (src[2] - '0') : (tolower((unsigned char)src[2]) - 'a' + 10);
            *dst++ = (char)((hi << 4) | lo);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

esp_err_t root_get_handler(httpd_req_t *req)
{
    
    const char resp[] =
        "<!DOCTYPE html><html><head>"
        "<meta charset=\"UTF-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
        "<title>Robot Wi-Fi Setup</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;background:#f4f6f8;margin:0;padding:16px;}"
        ".card{max-width:420px;margin:0 auto;background:#fff;padding:20px;border-radius:12px;"
        "box-shadow:0 2px 10px rgba(0,0,0,0.08);}"
        "h2{margin:0 0 16px 0;font-size:1.4rem;}"
        "label{display:block;margin:12px 0 6px;font-weight:600;}"
        "input{width:100%;padding:12px;font-size:1rem;border:1px solid #ccc;border-radius:8px;"
        "box-sizing:border-box;}"
        "button{width:100%;margin-top:16px;padding:12px;font-size:1rem;border:none;border-radius:8px;"
        "background:#0d6efd;color:#fff;font-weight:700;}"
        "</style>"
        "</head><body>"
        "<div class=\"card\">"
        "<h2>Config Wi-Fi</h2>"
        "<form action=\"/save\" method=\"post\">"
        "<label for=\"ssid\">SSID</label>"
        "<input id=\"ssid\" name=\"ssid\" maxlength=\"31\" autocomplete=\"off\">"
        "<label for=\"pass\">Password</label>"
        "<input id=\"pass\" name=\"pass\" type=\"password\" maxlength=\"63\" autocomplete=\"off\">"
        "<button type=\"submit\">Save</button>"
        "</form>"
        "</div>"
        "</body></html>";

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t save_post_handler(httpd_req_t *req)
{
    char buf[1024];
    int total_len = req->content_len;
    if (total_len <= 0 || total_len >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid payload size");
        return ESP_FAIL;
    }

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            return ESP_FAIL;
        }
        received += ret;
    }
    buf[received] = '\0';

    char ssid[32] = {0}, pass[64] = {0};
    if (httpd_query_key_value(buf, "ssid", ssid, sizeof(ssid)) != ESP_OK ||
        httpd_query_key_value(buf, "pass", pass, sizeof(pass)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing SSID or password");
        return ESP_FAIL;
    }

    url_decode_inplace(ssid);
    url_decode_inplace(pass);

    ESP_LOGI(TAG_WIFI, "Saving SSID: %s", ssid);
    nvs_write_wifi(ssid, pass);

    httpd_resp_sendstr(req, "<h3>Credentials saved!\nReseting and trying to connect...</h3>");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.max_uri_handlers = 16;
        config.max_open_sockets = 4;
        config.max_resp_headers = 16;
        config.lru_purge_enable = true;
        config.recv_wait_timeout = 10;
        config.send_wait_timeout = 10;

        // config.max_uri_len = 1024;
        // config.max_req_hdr_len = 4096;
        // config.max_resp_hdr_len = 1024;
        config.stack_size = 10240;      // mais pilha para a task do servidor

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_save = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler };
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_save);
    }
    return server;
}

// Wi-fi Manager ------------------------------------------------------
void start_wifi_ap(void)
{
    ESP_LOGI(TAG_WIFI, "Starting as AP mode");

    esp_netif_create_default_wifi_ap();

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "Robot_Config",
            .ssid_len = 0,
            .password = "12345678",
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();

    start_webserver();
}

void start_wifi_sta(const char *ssid, const char *pass)
{
    ESP_LOGI(TAG_WIFI, "Attempting to connect as STA: %s", ssid);
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    esp_netif_create_default_wifi_sta();
    esp_event_handler_instance_t any_id, got_ip; // Pode substituir por NULL no register

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &got_ip);

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, pass);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "Successfully connected!");
        esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
        if (ps_err == ESP_OK) {
            ESP_LOGI(TAG_WIFI, "Wi-Fi power save desativado (WIFI_PS_NONE)");
        } else {
            ESP_LOGW(TAG_WIFI, "Falha ao desativar power save: %s", esp_err_to_name(ps_err));
        }
    } else {
        ESP_LOGW(TAG_WIFI, "Failed to connect. Reverting to Ap mode...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        start_wifi_ap();
    }
}

// ------------------------------------------- MQTT FUNCTIONS
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG_MQTT, "MQTT tentando conectar...");
            break;

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT Connected");
            esp_mqtt_client_publish(mqtt_client, "swarm/robot2/telemetry/status", "Online", 0, 1, 0);
            xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
            esp_mqtt_client_subscribe(mqtt_client, "swarm/robot2/cmd/wheels", 0);
            esp_mqtt_client_subscribe(mqtt_client, "swarm/robot2/cmd/odometry", 0);
            esp_mqtt_client_subscribe(mqtt_client, "swarm/robot2/cmd/mode", 0);
            esp_mqtt_client_subscribe(mqtt_client, "swarm/robot2/cmd/camera_udp", 0);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "MQTT Disconnected");
            xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
            // Parar robô por segurança
            xSemaphoreTake(robot_mutex, portMAX_DELAY);
            robot.wl_ref = 0; robot.wr_ref = 0;
            xSemaphoreGive(robot_mutex);
            break;

        case MQTT_EVENT_ERROR:
            if (event && event->error_handle) {
                ESP_LOGE(TAG_MQTT, "MQTT erro: type=%d esp_tls=%d sock_errno=%d",
                         event->error_handle->error_type,
                         event->error_handle->esp_tls_last_esp_err,
                         event->error_handle->esp_transport_sock_errno);
            } else {
                ESP_LOGE(TAG_MQTT, "MQTT erro sem detalhes");
            }
            break;

        case MQTT_EVENT_DATA:
            if (event->data_len == 0) break;
            xEventGroupSetBits(s_mqtt_event_group, MQTT_MSG_RX_BIT);
            
            // Copia payload para buffer null-terminated
            char *payload = malloc(event->data_len + 1);
            memcpy(payload, event->data, event->data_len);
            payload[event->data_len] = 0;

            if (strncmp(event->topic, "swarm/robot2/cmd/camera_udp", event->topic_len) == 0) {
                struct sockaddr_in addr = {0};
                if (parse_udp_destination_from_payload(payload, &addr)) {
                    set_udp_destination(&addr);
                    ESP_LOGI(TAG_CAM, "Destino camera UDP configurado");
                } else {
                    set_udp_destination(NULL);
                    ESP_LOGW(TAG_CAM, "Destino camera UDP desabilitado");
                }
                free(payload);
                break;
            }

            cJSON *root = cJSON_Parse(payload);
            if (root) {
                if (strncmp(event->topic, "swarm/robot2/cmd/wheels", event->topic_len) == 0) {
                    cJSON *l = cJSON_GetObjectItem(root, "l");
                    cJSON *r = cJSON_GetObjectItem(root, "r");
                    
                    if (l && r) {
                        xSemaphoreTake(robot_mutex, portMAX_DELAY);
                        // O Python envia velocidade linear da roda (m/s). 
                        // Converter para angular (rad/s): w = v / R
                        float v_l = cjson_item_to_float(l);
                        float v_r = cjson_item_to_float(r);
                        
                        robot.wl_ref = v_l;
                        robot.wr_ref = v_r;
                        xSemaphoreGive(robot_mutex);
                        ESP_LOGI(TAG_MQTT, "Ref update: %.2f, %.2f rad/s", robot.wl_ref, robot.wr_ref);
                    }
                }
                else if (strncmp(event->topic, "swarm/robot2/cmd/odometry", event->topic_len) == 0) {
                    cJSON *x_item = cJSON_GetObjectItem(root, "x");
                    cJSON *y_item = cJSON_GetObjectItem(root, "y");
                    cJSON *theta_item = cJSON_GetObjectItem(root, "theta");

                    if (x_item && y_item && theta_item) {
                        float new_x = cjson_item_to_float(x_item);
                        float new_y = cjson_item_to_float(y_item);
                        float new_theta = cjson_item_to_float(theta_item);

                        xSemaphoreTake(robot_mutex, portMAX_DELAY);
                        robot.x = new_x;
                        robot.y = new_y;
                        robot.theta = new_theta;
                        xSemaphoreGive(robot_mutex);

                        ESP_LOGI(TAG_MQTT, "Odometry update: x=%.4f, y=%.4f, theta=%.4f",
                                 new_x, new_y, new_theta);
                    }
                }
                // Tópico Stop
                else if (strncmp(event->topic, "swarm/robot2/cmd/mode", event->topic_len) == 0) {
                    xSemaphoreTake(robot_mutex, portMAX_DELAY);
                    robot.wl_ref = 0; robot.wr_ref = 0;
                    PID_reset(&pid_l); PID_reset(&pid_r);
                    xSemaphoreGive(robot_mutex);
                    ESP_LOGW(TAG_STATE, "EMERGENCY STOP RECEIVED");
                }
                cJSON_Delete(root);
            }
            free(payload);
            break;

        default: break;
    }
}

static void mqtt_start(void){
    esp_mqtt_client_config_t mqtt_cfg = {
        .network.disable_auto_reconnect = false,
        .broker.address.uri = "mqtt://192.168.0.125", // LARS broker, pedir ao dono do lab
        // .broker.address.uri = "mqtt://mqtt.eclipseprojects.io",
        // .broker.address.uri = "mqtt://test.mosquitto.org",
        // .broker.address.uri = "mqtt://192.168.1.211", // pi zero
        .broker.address.port = 1883,
        .session.keepalive = 120,   // Padrão é 120 segundos
        .session.last_will = {
            .topic = "swarm/robot2/telemetry/status",
            .msg = "Offline",
            .msg_len = strlen("Offline"),
            .retain = 0
        },
        .task.priority = 6
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// ---------------------------------------------------------- TASKS
void capture_task(void *arg)
{
    const TickType_t delay = pdMS_TO_TICKS(CAMERA_CAPTURE_PERIOD_MS);
    uint32_t ok = 0, null_fb = 0, queue_drop = 0;
    TickType_t last_log = 0;

    while (1) {
        struct timeval tv = {0};
        gettimeofday(&tv, NULL);
        uint64_t capture_ts_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            null_fb++;
            if ((null_fb == 1u) || (null_fb % 25u == 0u)) {
                ESP_LOGW(TAG_CAM, "esp_camera_fb_get falhou (%lu vezes)", (unsigned long)null_fb);
            }
            vTaskDelay(delay);
            continue;
        }

        ok++;
        CameraFrameQueueItem_t item = {
            .fb = fb,
            .capture_ts_us = capture_ts_us,
        };
        if (xQueueSend(frame_queue, &item, 0) != pdTRUE) {
            queue_drop++;
            esp_camera_fb_return(fb);
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_log >= pdMS_TO_TICKS(5000)) {
            UBaseType_t q_now = uxQueueMessagesWaiting(frame_queue);
            ESP_LOGI(TAG_CAM, "captura: frames_ok=%lu get_fail=%lu fila_cheia=%lu fila=%lu/%u",
                     (unsigned long)ok, (unsigned long)null_fb, (unsigned long)queue_drop,
                     (unsigned long)q_now, QUEUE_SIZE);
            last_log = now;
        }

        vTaskDelay(delay);
    }
}

void udp_stream_task(void *pvParameters)
{
    (void)pvParameters;
    uint32_t sent_frames = 0;
    uint32_t dropped_frames = 0;
    uint8_t packet[sizeof(CameraUdpFragHeader_t) + CAMERA_UDP_MAX_PAYLOAD];
    const size_t max_payload = CAMERA_UDP_MAX_PAYLOAD;
    int sock = -1;
    TickType_t last_log = 0;

    ESP_LOGI(TAG_CAM, "udp_stream_task iniciada (%dfps, payload=%u)",
             CAMERA_STREAM_FPS, CAMERA_UDP_MAX_PAYLOAD);

    while (1) {
        UdpStreamDest_t dest_snapshot = {0};
        if (xSemaphoreTake(s_udp_dest_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            dest_snapshot = s_udp_dest;
            xSemaphoreGive(s_udp_dest_mutex);
        }

        CameraFrameQueueItem_t item = {0};
        if (xQueueReceive(frame_queue, &item, portMAX_DELAY) != pdTRUE || item.fb == NULL) {
            continue;
        }

        camera_fb_t *fb = item.fb;
        const uint64_t capture_ts_us = item.capture_ts_us;

        if (!dest_snapshot.enabled) {
            dropped_frames++;
            esp_camera_fb_return(fb);
            continue;
        }

        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock < 0) {
                dropped_frames++;
                ESP_LOGW(TAG_CAM, "Socket UDP indisponivel (rede ainda nao pronta?)");
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            struct timeval timeout = {
                .tv_sec = 0,
                .tv_usec = 150000
            };
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        }

        uint32_t frame_id = ++s_frame_seq;
        uint16_t total_frags = (uint16_t)((fb->len + max_payload - 1) / max_payload);
        size_t offset = 0;
        bool frame_ok = true;

        for (uint16_t frag = 0; frag < total_frags; frag++) {
            size_t remaining = fb->len - offset;
            uint16_t payload_len = (uint16_t)((remaining > max_payload) ? max_payload : remaining);
            CameraUdpFragHeader_t hdr = {
                .frame_id = frame_id,
                .timestamp_us = capture_ts_us,
                .total_frags = total_frags,
                .frag_idx = frag,
                .payload_len = payload_len
            };

            memcpy(packet, &hdr, sizeof(hdr));
            memcpy(packet + sizeof(hdr), fb->buf + offset, payload_len);

            ssize_t sent = sendto(sock, packet, sizeof(hdr) + payload_len, 0,
                                  (struct sockaddr *)&dest_snapshot.addr, dest_snapshot.addr_len);
            if (sent < 0) {
                frame_ok = false;
                close(sock);
                sock = -1;
                break;
            }
            offset += payload_len;
        }

        if (frame_ok) {
            sent_frames++;
        } else {
            dropped_frames++;
        }

        TickType_t now = xTaskGetTickCount();
        if (sent_frames == 1u || now - last_log >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG_CAM, "udp: sent=%lu drop=%lu frame=%lu bytes=%u frags=%u",
                     (unsigned long)sent_frames,
                     (unsigned long)dropped_frames,
                     (unsigned long)frame_id,
                     (unsigned int)fb->len,
                     (unsigned int)total_frags);
            last_log = now;
        }

        esp_camera_fb_return(fb);
    }
}

void vTaskPublishOdometry(void *pvParam) {
    char json_buffer[128];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PUBLISH_PERIOD_MS);
    TickType_t last_pub_fail_log = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Verifica conexão MQTT
        if (xEventGroupGetBits(s_mqtt_event_group) & MQTT_CONNECTED_BIT) {
            float x, y, theta, vl, vr;
            uint64_t odom_ts_us;
            
            // Snapshot rápido dos dados
            xSemaphoreTake(robot_mutex, portMAX_DELAY);
            x = robot.x;
            y = robot.y;
            theta = robot.theta;
            odom_ts_us = robot.odom_ts_us;
            vl = robot.wl_meas; // Envia m/s para o Python
            vr = robot.wr_meas;
            xSemaphoreGive(robot_mutex);

            // 1. Publica Odometria com timestamp absoluto da amostragem
            // Mantém o campo legado "odometria" para compatibilidade.
            snprintf(json_buffer, sizeof(json_buffer), 
                "{\"odometria\": \"%.4f, %.4f, %.4f\", \"timestamp_us\": %llu}",
                x, y, theta, (unsigned long long)odom_ts_us);
            int pub_odom = esp_mqtt_client_publish(mqtt_client, "swarm/robot2/telemetry/odometry", json_buffer, 0, 0, 0);
            if (pub_odom < 0) {
                TickType_t now = xTaskGetTickCount();
                if (now - last_pub_fail_log > pdMS_TO_TICKS(3000)) {
                    ESP_LOGW(TAG_MQTT, "Falha publish odometry (client possivelmente desconectado)");
                    last_pub_fail_log = now;
                }
                continue;
            }

            // 2. Publica Velocidades
            // JSON: {"velocidades": "vl, vr"}
            snprintf(json_buffer, sizeof(json_buffer), 
                "{\"wheels\": \"%.3f, %.3f\"}", vl, vr);
            int pub_wheels = esp_mqtt_client_publish(mqtt_client, "swarm/robot2/telemetry/wheels", json_buffer, 0, 0, 0);
            if (pub_wheels < 0) {
                TickType_t now = xTaskGetTickCount();
                if (now - last_pub_fail_log > pdMS_TO_TICKS(3000)) {
                    ESP_LOGW(TAG_MQTT, "Falha publish wheels (client possivelmente desconectado)");
                    last_pub_fail_log = now;
                }
            }
        }
    }
}

void vTaskStateINIT(void *pvParameters){
    ESP_LOGI(TAG_STATE, " --> STATE INIT <--");
    // LED azul durante inicialização
    ws2812_init();
    ws2812_set_color(0, 0, 80);

    // Hardware base
    encoder_pin_init();
    ledc_init();

    enc_left  = pcnt_init_encoder(ENC_LFT_A_PIN, ENC_LFT_B_PIN, 20000);
    enc_right = pcnt_init_encoder(ENC_RGH_B_PIN, ENC_RGH_A_PIN, 20000);
    encoders_disable();

    frame_queue = xQueueCreate(QUEUE_SIZE, sizeof(CameraFrameQueueItem_t));
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG_CAM, "Falha na câmera durante INIT");
        vTaskDelete(NULL);
        return;
    }

    xTaskCreate(udp_stream_task, "udp_stream_task", 6144, NULL, 3, NULL);
    xTaskCreate(capture_task, "capture_task", 4096, NULL, 2, NULL);

    // Stack de rede
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    char ssid[32] = {0};
    char pass[64] = {0};
    bool has_saved_credentials = nvs_read_wifi(ssid, sizeof(ssid), pass, sizeof(pass));

    if (has_saved_credentials && strlen(ssid) > 0) {
        ESP_LOGI(TAG_WIFI, "Saved credentials found. Trying STA connection...");
        start_wifi_sta(ssid, pass);
    } else {
        ESP_LOGW(TAG_WIFI, "No saved Wi-Fi credentials. Starting AP provisioning.");
        start_wifi_ap();
    }

    EventBits_t wifi_bits = xEventGroupGetBits(s_wifi_event_group);
    if ((wifi_bits & WIFI_CONNECTED_BIT) == 0) {
        ESP_LOGW(TAG_STATE, "INIT em AP/provisioning: aguardando formulário Wi-Fi (sem avançar de estado)");
        while (1) {
            ws2812_set_color(0, 0, 80);
            vTaskDelay(pdMS_TO_TICKS(300));
            ws2812_set_color(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

    while (start_sntp_and_wait() != ESP_OK) {
        ESP_LOGW(TAG_STATE, "Tempo NTP ainda indisponivel. Mantendo INIT e tentando novamente...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    mqtt_start();
    ESP_LOGI(TAG_STATE, "INIT concluída, indo para STANDBY");
    xTaskCreate(vTaskStateSTANDBY, "vTaskStateSTANDBY", 4096, NULL, 7, NULL);
    
    vTaskDelete(NULL);
}

void vTaskStateSTANDBY(void *pvParameters) {
    ESP_LOGI(TAG_STATE, " --> STATE STANDBY <--");
    ws2812_set_color(0, 80, 0);
    ESP_LOGI(TAG_STATE, "Aguardando qualquer mensagem MQTT para entrar em RUNNING...");

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            s_mqtt_event_group,
            MQTT_MSG_RX_BIT,
            pdTRUE,   // clear on exit
            pdFALSE,
            pdMS_TO_TICKS(1000)
        );

        if (bits & MQTT_MSG_RX_BIT) {
            ESP_LOGI(TAG_STATE, "Mensagem MQTT recebida. Transição: STANDBY -> RUNNING");
            xTaskCreate(vTaskStateRUNNING, "vTaskStateRUNNING", 4096, NULL, 7, NULL);
            vTaskDelete(NULL);
            return;
        }
    }
}

void vTaskStateRUNNING(void *pvParameters) {
    ws2812_set_color(80, 0, 80);

    ESP_LOGI(TAG_STATE, "STATE: RUNNING");
    
    s_control_active = false;

    // 1. Inicializa Estado
    xSemaphoreTake(robot_mutex, portMAX_DELAY);
    robot.x = 0; robot.y = 0; robot.theta = 0;
    robot.wl_ref = 0; robot.wr_ref = 0;
    xSemaphoreGive(robot_mutex);

    // 2. Configura PIDs
    PID_Config pid_cfg = {
        .Kp = PID_MOTOR_KP, .Ki = PID_MOTOR_KI, .Kd = PID_MOTOR_KD,
        .Ts = SAMPLING_PERIOD_MS / 1000.0f,
        .tau = PID_MOTOR_TAU,
        .lim_min = PID_LIM_MIN, .lim_max = PID_LIM_MAX
    };
    PID_init(&pid_l, pid_cfg);
    PID_init(&pid_r, pid_cfg);

    encoders_enable();

    // 3. Inicia Timer de Controle (25ms)
    const esp_timer_create_args_t control_timer_args = {
        .callback = &control_timer_callback,
        .name = "control_loop"
    };
    ESP_ERROR_CHECK(esp_timer_create(&control_timer_args, &control_timer));
    s_control_active = true;
    ESP_ERROR_CHECK(esp_timer_start_periodic(control_timer, SAMPLING_PERIOD_MS * 1000)); // em us

    // 4. Cria task de Publicação MQTT
    xTaskCreate(vTaskPublishOdometry, "pub_odom", 4096, NULL, 6, NULL);

    // Loop infinito apenas para manter a task viva
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // adicionar: Monitoramento de saúde do sistema e bateria
    }
}

// ---------------------------------------------------------- APP MAIN

void app_main(void) {
    // Inicialização mínima; fluxo principal roda na state INIT
    robot_mutex = xSemaphoreCreateMutex();
    s_wifi_event_group = xEventGroupCreate();
    s_mqtt_event_group = xEventGroupCreate();
    s_udp_dest_mutex = xSemaphoreCreateMutex();
    s_time_sync_event_group = xEventGroupCreate();

    xTaskCreate(vTaskStateINIT, "INIT", 8192, NULL, 8, NULL);
}