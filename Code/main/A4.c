#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include <driver/dac.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "driver/timer.h"
#include "driver/spi_master.h"

#define WIFI_SSID      "RobotHouse"
#define WIFI_PASS      "Ro8otH@u53"
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to s
#define TIMER_INTERVAL0_S    0.0002 // sample test interval for the first timer
#define TIMER_INTERVAL1_S 0.00005
#define TIMER_INTERVAL2 0.005
static EventGroupHandle_t wifiEventGroup;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int retryNum = 0;
static const char *TAG = "wifi station";   
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
static const adc_atten_t atten = ADC_ATTEN_11db;

const int pinDir1 = 1;
const int pinStep1 = 2;
const int pinLCClk = 3;
const int pinLCDat = 4;
const int pinEn3 = 8;
const int pinDir2 = 9;
const int pinStep2 = 10;
const int pinEn2 = 11;
const int pinEn1 = 12;
//const int pinCharge = 13;
const int pinDir0 = 14;
const int pinStep0 = 15;
const int pinEn0 = 16;
const int pinDAC1 = 17;
const int pinStep3 = 19;
const int pinDir3 = 20;
const int pinEn4 = 21;
const int pinStep4 = 26;
const int pinDir4 = 33;
const int pinCS4 = 34;
const int pinClk = 35;
const int pinCS3 = 36;
const int pinMosi = 37;
const int pinMot1 = 38;
const int pinMot2 = 39;
const int pinMiso = 40;
const int pinCS2 = 41;
const int pinCS1 = 42;
const int pinCS0 = 45;

const bool enable[4] = {1,1,1,1};

const int pinCharge = ADC2_CHANNEL_2;

int64_t timeNow = 0;
int64_t timeSample = 0;
int64_t timePrev = 0;
float battery = 12.0;

const float speedMin = 3; // deg/s
const float speedMax[4] = {17, 17, 17, 8}; //deg/s should be 20 intervals per step
const float accel = 360.0/27.0; // deg/s2
const float angleTolerance = 0.3;

struct motor {
    float stepsPerDeg;
    float degPerStep;
    float targetPos;
    float currentPos;
    float startPos;
    float totalDis;
    int64_t startTime;
    volatile float disToTarget;
    volatile float targetSpeed;
    volatile float stepInterval;
    int pinStep;
    int pinDir;
    int pinEn;
    int pinCS;
    volatile int intrCount;
};

volatile bool masterComplete, slaveComplete;
bool newMove;

struct motor motors[4];
int m = 0, s0 = 1, s1 = 2, s2 = 3;
bool robotEnabled = false;

float totalDis = 0;
float coastDisEnd = 0;
float accelDis = 0;
volatile float targetSpeed = speedMin;
volatile float dis = 0;

//offsets
//j0 = 242.3 facing left, set to 270
//j1 = 216.2 all the way back, set to 45
//j2 = 28.6 straight up, set to 180
//j3 = 257.7 straight out, set to 90
const float offset[4] = {270-242.3, 45-216.2, 180-28.6, 90-257.7};

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retryNum < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            retryNum++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifiEventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retryNum = 0;
        xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    }
}

void wifiInit(void) {
    wifiEventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    //wait for event connected or failed
    EventBits_t bits = xEventGroupWaitBits(wifiEventGroup,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(wifiEventGroup);
}

static esp_err_t get_handler(httpd_req_t *req) {
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[]   asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_send_chunk(req, (const char *)index_html_start, index_html_size);
    /* Send empty chunk to signal HTTP response completion */
    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

static esp_err_t get_handler_2(httpd_req_t *req) {
    if (!strcmp(req->uri,"/battery")) {
        //return the battery voltage
        int len = snprintf(NULL, 0, "%.2f", battery);
        char *result = (char *)malloc(len + 1);
        snprintf(result, len + 1, "%.2f", battery);
        httpd_resp_sendstr(req, result);
    } else if (!strcmp(req->uri,"/pos")) {
        int len = snprintf(NULL, 0, "J0: %.2f\t J1: %.2f\t J2: %.2f\t J3: %.2f", 
            motors[0].currentPos, motors[1].currentPos, motors[2].currentPos, motors[3].currentPos);
        char *result = (char *)malloc(len + 1);
        snprintf(result, len + 1, "J0: %.2f\t J1: %.2f\t J2: %.2f\t J3: %.2f", 
            motors[0].currentPos, motors[1].currentPos, motors[2].currentPos, motors[3].currentPos);
        httpd_resp_sendstr(req, result);
    } else {
        printf("URI: ");
        printf(req->uri);
        printf("\n");
    }
    return ESP_OK;
}

int maxIndex() {
    float max = 0;
    int maxIndex = 0;
    for (int i = 0; i < 4; i++) {
        if(motors[i].totalDis > max) {
            max = motors[i].totalDis;
            maxIndex = i;
        }
    }
    return maxIndex;
}

void planMove(float j0, float j1, float j2, float j3) {
    motors[0].targetPos = j0;
    motors[1].targetPos = j1;
    motors[2].targetPos = j2;
    motors[3].targetPos = j3;

    for (int i = 0; i < 4; i++) {
        motors[i].startPos = motors[i].currentPos;
        motors[i].totalDis = fabs(motors[i].targetPos - motors[i].startPos);
        motors[i].targetSpeed = speedMin;
        motors[i].stepInterval = 10000;
    }

    m = maxIndex();
    if(m == 0) {
        s0 = 1;
        s1 = 2;
        s2 = 3;
    } else if (m == 1) {
        s0 = 0;
        s1 = 2;
        s2 = 3;
    } else if (m == 2) {
        s0 = 0;
        s1 = 1;
        s2 = 3;
    } else {
        s0 = 0;
        s1 = 1;
        s2 = 2;
    }

    float accelTime = (speedMax[m]-speedMin)/accel;
    accelDis = speedMin*accelTime + 0.5*accel*accelTime*accelTime;
    totalDis = motors[m].totalDis;
    if(accelDis > totalDis/2.0) accelDis = totalDis/2.0;
    coastDisEnd = totalDis - accelDis;

    if(enable[0]) gpio_set_level(motors[0].pinEn, 0);
    if(enable[1]) gpio_set_level(motors[1].pinEn, 0);
    if(enable[2]) gpio_set_level(motors[2].pinEn, 0);
    if(enable[3]) gpio_set_level(motors[3].pinEn, 0);

    if(!robotEnabled) {
        timer_start(TIMER_GROUP_0, TIMER_0);
        robotEnabled = true;
    }
}

static esp_err_t get_handler_move(httpd_req_t *req) {
    char*  buf;
    size_t buf_len;
    float j0 = motors[0].currentPos;
    float j1 = motors[1].currentPos;
    float j2 = motors[2].currentPos;
    float j3 = motors[3].currentPos;
    /* Read URL query string length and allocate memory for length + 1,
    * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "j0", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => j0=%s", param);
                if(enable[0]) j0 = atof(param);
            }
            if (httpd_query_key_value(buf, "j1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => j1=%s", param);
                if(enable[1]) j1 = atof(param);
            }
            if (httpd_query_key_value(buf, "j2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => j2=%s", param);
                if(enable[2]) j2 = atof(param);
            }
            if (httpd_query_key_value(buf, "j3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => j3=%s", param);
                if(enable[3]) j3 = atof(param);
            }
            planMove(j0, j1, j2, j3);
        }
        free(buf);
    }
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t move_uri = {
            .uri       = "/moveRelative*",  // Match all URIs of type /path/to/file
            .method    = HTTP_GET,
            .handler   = get_handler_move,
            .user_ctx  = NULL   // Pass server data as context
        };
        httpd_register_uri_handler(server, &move_uri);
        httpd_uri_t get_uri_1 = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &get_uri_1);
        /* URI handler for getting uploaded files */
        httpd_uri_t file_download = {
            .uri       = "/*",  // Match all URIs of type /path/to/file
            .method    = HTTP_GET,
            .handler   = get_handler_2,
            .user_ctx  = NULL   // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_download);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server) {
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, 
                            int32_t event_id, void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void IRAM_ATTR setDir(int i) {
    if(motors[i].disToTarget > 0) {
        if(i == 2) {
            gpio_set_level(motors[i].pinDir, 1);
        } else {
            gpio_set_level(motors[i].pinDir, 0);
        }
    } else {
        if(i == 2) {
            gpio_set_level(motors[i].pinDir, 0);
        } else {
            gpio_set_level(motors[i].pinDir, 1);
        }
    }
}

void IRAM_ATTR timer_0_0_isr(void *para) {
    timer_spinlock_take(TIMER_GROUP_0);

    for(int i = 0; i<4; i++) {
        motors[i].intrCount++;
        if(motors[i].intrCount >= motors[i].stepInterval && fabs(motors[i].disToTarget) > angleTolerance) {
            motors[i].intrCount = 0;
            gpio_set_level(motors[i].pinStep, 1);
        }
    }

    //set timer 1 to reset step pins
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_1);
    timer_counter_value += (uint64_t) (TIMER_INTERVAL1_S * TIMER_SCALE);
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_1, timer_counter_value);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_spinlock_give(TIMER_GROUP_0);
}

void IRAM_ATTR timer_0_1_isr(void *para) {
    timer_spinlock_take(TIMER_GROUP_0);
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    gpio_set_level(motors[0].pinStep, 0);
    gpio_set_level(motors[1].pinStep, 0);
    gpio_set_level(motors[2].pinStep, 0);
    gpio_set_level(motors[3].pinStep, 0);
    timer_spinlock_give(TIMER_GROUP_0);
}

//calculate desired speed for each axis at this time
void IRAM_ATTR timer_1_0_isr(void *para) {
    timer_spinlock_take(TIMER_GROUP_1);

    //compute master axis speed based on displacement
    dis = fabs(motors[m].currentPos - motors[m].startPos);
    float a, b, c, t;
    targetSpeed = speedMin;
    if(dis < accelDis) {
        //estimate time from displacement to calculate desired velocity
        a = 0.5*accel;
        b = speedMin;
        c = -dis;
        t = (-b+sqrt(b*b - 4*a*c))/(2*a);
        targetSpeed = speedMin + accel*t;
    } else if (dis < coastDisEnd) {
        targetSpeed = speedMax[m];
    } else if (dis < totalDis) {
        a = 0.5*accel;
        b = speedMin;
        c = -(totalDis - dis);
        t = (-b+sqrt(b*b - 4*a*c))/(2*a);
        targetSpeed = speedMin + accel*t;
    }
    motors[m].targetSpeed = targetSpeed;

    if(motors[m].targetSpeed > speedMax[m]) motors[m].targetSpeed = speedMax[m];
    if(motors[m].targetSpeed < speedMin) motors[m].targetSpeed = speedMin;

    // deg/s*step/deg -> step/s step/s*s/interval -> step/interval
    motors[m].stepInterval = 1.0/(motors[m].targetSpeed*motors[m].stepsPerDeg*TIMER_INTERVAL0_S);
    motors[s0].stepInterval = motors[m].stepInterval*motors[m].totalDis/motors[s0].totalDis;
    motors[s1].stepInterval = motors[m].stepInterval*motors[m].totalDis/motors[s1].totalDis;
    if(s2 == 3) {
        motors[s2].targetSpeed = motors[m].targetSpeed*motors[s2].totalDis/motors[m].totalDis;
        motors[s2].stepInterval = 1.0/(motors[s2].targetSpeed*motors[s2].stepsPerDeg*TIMER_INTERVAL0_S);
    } else {
        motors[s2].stepInterval = motors[m].stepInterval*motors[m].totalDis/motors[s2].totalDis;
    }

    for(int i = 0; i<4; i++) {
        motors[i].disToTarget = motors[i].targetPos - motors[i].currentPos;
        setDir(i);
    }

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_0);
    timer_spinlock_give(TIMER_GROUP_1);
}


static void tg_timer_init(timer_group_t tg, int timer_idx,
                                   bool auto_reload, double timer_interval_ms) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(tg, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(tg, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(tg, timer_idx, timer_interval_ms * TIMER_SCALE);
    timer_enable_intr(tg, timer_idx);
}

void motorInit(int i, int pinEn, int pinStep, int pinDir, int pinCS, float gearRatio) {
    motors[i].pinEn = pinEn;
    motors[i].pinStep = pinStep;
    motors[i].pinDir = pinDir;
    motors[i].pinCS = pinCS;
    motors[i].stepsPerDeg = 200.0*gearRatio/360.0;
    motors[i].degPerStep = 1.0/motors[i].stepsPerDeg;
    motors[i].intrCount = 0;
    motors[i].totalDis = 0;
}

void getAngle(spi_device_handle_t spi, int i) {
    if(!enable[i]) {
        motors[i].currentPos = 0;
        motors[i].targetPos = 0;
        return;
    }

    //get encoder angle
    spi_transaction_t t;
    gpio_set_level(motors[i].pinCS, 0);
    memset(&t, 0, sizeof(t));
    t.length = 16;
    // 0x3fff = 11111111111111, set 14 to 1 for read, set parity bit to 1
    uint16_t cmd = 0x3fff|1<<14|1<<15;
    t.tx_buffer = &cmd;
    t.flags = SPI_TRANS_USE_RXDATA;
    spi_device_transmit(spi, &t);
    gpio_set_level(motors[i].pinCS, 1);
    //check parity
    bool even = true;
    for(int j=0; j<2; j++) {
        for(int i=0; i<8; i++) {
            if(t.rx_data[j] & 1<<i) even = !even;
        }
    }
    if(even) {
        uint16_t trim = 0b0011111111111111;
        uint16_t angle = ((t.rx_data[0]<<8) + t.rx_data[1]) & trim;
        float tempAngle = (float)angle/16383.0*360.0 + offset[i];
        if(tempAngle < 0) tempAngle += 360;
        if(tempAngle > 360) tempAngle -= 360;
        motors[i].currentPos = tempAngle;
    }
}

static void enc0_task(void *arg) {
    //set CS pins high to start
    gpio_set_level(pinCS0, 1);
    gpio_set_level(pinCS1, 1);
    gpio_set_level(pinCS2, 1);
    gpio_set_level(pinCS3, 1);
    //create SPI
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=pinMiso,
        .mosi_io_num=pinMosi,
        .sclk_io_num=pinClk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1000000,           //Clock out at 10 MHz
        .mode=1,                                //SPI mode 1
        .spics_io_num=-1,               //CS pins manually implemented
        .address_bits = 0,
        .command_bits = 0,
        .queue_size = 7
    };
    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    while(1) {
        for(int i=0; i<4; i++) {
            getAngle(spi, i);
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    //connect to wifi
    wifiInit();

    static httpd_handle_t server = NULL;

    //Register event handlers to stop the server when Wi-Fi is disconnected
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();

    //configure outputs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = 1ULL<<pinDir0|1ULL<<pinStep0|1ULL<<pinEn0|1ULL<<pinCS0;
    io_conf.pin_bit_mask |= 1ULL<<pinDir1|1ULL<<pinStep1|1ULL<<pinEn1|1ULL<<pinCS1;
    io_conf.pin_bit_mask |= 1ULL<<pinDir2|1ULL<<pinStep2|1ULL<<pinEn2|1ULL<<pinCS2;
    io_conf.pin_bit_mask |= 1ULL<<pinDir3|1ULL<<pinStep3|1ULL<<pinEn3|1ULL<<pinCS3;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(pinEn0, 1);
    gpio_set_level(pinEn1, 1);
    gpio_set_level(pinEn2, 1);
    gpio_set_level(pinEn3, 1);

    //configure ADC
    adc2_config_channel_atten((adc2_channel_t)pinCharge, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_2, atten, width, DEFAULT_VREF, adc_chars);

    motorInit(0, pinEn0, pinStep0, pinDir0, pinCS0, 26.0+103.0/121.0);
    motorInit(1, pinEn1, pinStep1, pinDir1, pinCS1, 26.0+103.0/121.0);
    motorInit(2, pinEn2, pinStep2, pinDir2, pinCS2, 26.0+103.0/121.0);
    motorInit(3, pinEn3, pinStep3, pinDir3, pinCS3, 26.0+103.0/121.0);

    tg_timer_init(TIMER_GROUP_0, TIMER_0, 1, TIMER_INTERVAL0_S);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_0_0_isr,
       (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    tg_timer_init(TIMER_GROUP_0, TIMER_1, 0, TIMER_INTERVAL1_S);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timer_0_1_isr,
       (void *) TIMER_1, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_1);

    tg_timer_init(TIMER_GROUP_1, TIMER_0, true, TIMER_INTERVAL2);
    timer_isr_register(TIMER_GROUP_1, TIMER_0, timer_1_0_isr,
       (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_1, TIMER_0);

    xTaskCreate(enc0_task, "enc0_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    while(1) {
        uint32_t adc_reading = 0;
        int raw;
        adc2_get_raw((adc2_channel_t)pinCharge, width, &raw);
        adc_reading = raw;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        battery = (float)voltage*57.0/10000.0;
        //if voltage is too low, deep sleep indefinitely
        if(battery < 9.6) {
            printf("Low battery: %.2f V\n", battery);
            esp_deep_sleep_start();
        }
        // printf("motor1 stepsToTarget: %d\n", motors[1].stepsToTarget);
        // printf("motor0, 1, fraction: %.2f, %.2f\n", motors[0].fractionComplete, motors[1].fractionComplete);

        printf("J0 %.1f\t J1 %.1f\t J2 %.1f\t J3 %.1f\n", motors[0].currentPos, motors[1].currentPos, 
            motors[2].currentPos, motors[3].currentPos);
        // printf("J0 %.0f\t speed %.1f\n", motors[0].stepInterval, motors[0].targetSpeed);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}