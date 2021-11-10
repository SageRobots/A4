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

const bool enableJ0 = true;
const bool enableJ1 = true;
const bool enableJ2 = false;
const bool enableJ3 = false;

const int pinCharge = ADC2_CHANNEL_2;

int64_t timeNow = 0;
int64_t timeSample = 0;
int64_t timePrev = 0;
float battery = 12.0;

const float speedMin = 12.0/27.0; // deg/s
const float accel = 360.0/27.0; // deg/s2
const int minStepInterval = 20;
const float angleTolerance = 0.3;

struct motor {
    float stepsPerDeg;
    float degPerStep;
    float targetPos;
    float currentPos;
    float startPos;
    float movePos;
    volatile int32_t currentSteps;
    volatile float accelFraction;
    int32_t targetSteps;
    volatile int32_t stepsToTarget;
    volatile float targetSpeed;
    volatile float stepInterval;
    int32_t moveSteps;
    volatile bool accelFractionSet;
    int pinStep;
    int pinDir;
    int pinEn;
    volatile float numer;
    volatile float denom;
    volatile float fractionComplete;
};

volatile int intr_count = 0;
volatile int intr_count1 = 0;
volatile int intr_count2 = 0;
volatile bool masterComplete, slaveComplete;
bool newMove;

struct motor motors[3];
int m = 0, s0 = 1, s1 = 2;
bool robotEnabled = false;

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
        int len = snprintf(NULL, 0, "J0:%.2f\t J1:%.2f\t", motors[0].currentPos, motors[1].currentPos);
        char *result = (char *)malloc(len + 1);
        snprintf(result, len + 1, "J0:%.2f\t J1:%.2f\t", motors[0].currentPos, motors[1].currentPos);
        httpd_resp_sendstr(req, result);
    } else {
        printf("URI: ");
        printf(req->uri);
        printf("\n");
    }
    return ESP_OK;
}

void planMove(float j0, float j1, float j2) {
    motors[0].targetPos = j0;
    motors[1].targetPos = j1;
    motors[2].targetPos = j2;
    for (int i = 0; i < 3; i++) {
        motors[i].accelFraction = 0.5;
        motors[i].startPos = motors[i].currentPos;
        motors[i].movePos = motors[i].targetPos - motors[i].currentPos;
        motors[i].moveSteps = abs(motors[i].movePos*motors[i].stepsPerDeg);
        motors[i].targetSpeed = speedMin;
        motors[i].stepInterval = 1.0/TIMER_INTERVAL0_S/motors[i].targetSpeed*motors[i].degPerStep; // intervals/s s/deg deg/step
        motors[i].accelFractionSet = false;
        gpio_set_level(motors[i].pinEn, 0);
    }

    if(abs(motors[0].movePos) > abs(motors[1].movePos) && abs(motors[0].movePos) > abs(motors[2].movePos)) {
        m = 0;
        s0 = 1;
        s1 = 2;
    } else if (abs(motors[1].movePos) > abs(motors[2].movePos)) {
        m = 1;
        s0 = 0;
        s1 = 2;
    } else {
        m = 2;
        s0 = 0;
        s1 = 1;
    }

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
                if(enableJ0) j0 = atof(param);
            }
            if (httpd_query_key_value(buf, "j1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => j1=%s", param);
                if(enableJ1) j1 = atof(param);
            }
            if (httpd_query_key_value(buf, "j2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => j2=%s", param);
                if(enableJ2) j2 = atof(param);
            }
            planMove(j0, j1, j2);
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

void IRAM_ATTR timer_0_isr(void *para) {
    timer_spinlock_take(TIMER_GROUP_0);
    bool stepMaster = false;
    bool stepSlave = false;
    intr_count++;
    intr_count1++;
    intr_count2++;
    motors[m].numer = abs(motors[m].currentPos-motors[m].startPos)*motors[m].stepsPerDeg+(float)intr_count/motors[m].stepInterval;
    motors[m].denom = (float)motors[m].moveSteps-angleTolerance*motors[m].stepsPerDeg;
    if(motors[m].denom == 0) {
        motors[m].fractionComplete = 1;
    } else {
        motors[m].fractionComplete = motors[m].numer/motors[m].denom;
    }
    if(intr_count >= motors[m].stepInterval) {
        intr_count = 0;
        //compute steps to target
        motors[m].stepsToTarget = round((motors[m].targetPos - motors[m].currentPos)*motors[m].stepsPerDeg);
        
        if(abs(motors[m].stepsToTarget) >= angleTolerance*motors[m].stepsPerDeg) {
            //set direction
            if(motors[m].stepsToTarget > 0) {
                gpio_set_level(motors[m].pinDir, 0);
            } else {
                gpio_set_level(motors[m].pinDir, 1);
            }
            //update speed level
            if((motors[m].fractionComplete < motors[m].accelFraction)
                 || (abs(motors[m].stepsToTarget) > 0.5*motors[m].moveSteps)) {
                // accel in deg/s2 stepInterval is n*.0002s
                motors[m].targetSpeed += accel*motors[m].stepInterval*TIMER_INTERVAL0_S;
                if(motors[m].stepInterval == minStepInterval) {
                    if(!motors[m].accelFractionSet) {
                        motors[m].accelFraction = motors[m].fractionComplete;
                        motors[m].accelFractionSet = true;
                    }
                }
            } else {
                //decelerate
                motors[m].targetSpeed -= accel*motors[m].stepInterval*TIMER_INTERVAL0_S;
                if(motors[m].targetSpeed <= speedMin) {
                    motors[m].targetSpeed = speedMin;
                }
            }
            //compute time per step for target speed
            motors[m].stepInterval = 1.0/TIMER_INTERVAL0_S/motors[m].targetSpeed*motors[m].degPerStep; // intervals/s s/deg deg/step
            if(motors[m].stepInterval < minStepInterval) motors[m].stepInterval = minStepInterval;
            stepMaster = true;           
        } else {
            masterComplete = true;
        }
    }

    motors[s0].numer = abs(motors[s0].currentPos-motors[s0].startPos);
    motors[s0].denom = abs(motors[s0].movePos)-angleTolerance;
    if(motors[s0].denom == 0) {
        motors[s0].fractionComplete = 1;
    } else {
        motors[s0].fractionComplete = motors[s0].numer/motors[s0].denom;
    }

    motors[s0].stepsToTarget = (motors[s0].targetPos - motors[s0].currentPos)*motors[s0].stepsPerDeg;
    if(motors[s0].fractionComplete <= motors[m].fractionComplete 
        && abs(motors[s0].stepsToTarget) > angleTolerance*motors[s0].stepsPerDeg) {
        if(intr_count1 >= motors[m].stepInterval*motors[m].moveSteps/motors[s0].moveSteps) {
            stepSlave = true;
            intr_count1 = 0;            
        }
        if(motors[s0].stepsToTarget > 0) {
            gpio_set_level(motors[s0].pinDir, 0);
        } else {
            gpio_set_level(motors[s0].pinDir, 1);
        }
    }

    if(motors[s0].stepsToTarget == 0) {
        slaveComplete = true;
    }

    if(stepMaster) gpio_set_level(motors[m].pinStep, 1);
    if(stepSlave) gpio_set_level(motors[s0].pinStep, 1);
    if(stepMaster | stepSlave) {
        //set timer 1 to reset step pins
        uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_1);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL1_S * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_1, timer_counter_value);
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);
    }

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_spinlock_give(TIMER_GROUP_0);
}

void IRAM_ATTR timer_1_isr(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    gpio_set_level(motors[0].pinStep, 0);
    gpio_set_level(motors[1].pinStep, 0);
    timer_spinlock_give(TIMER_GROUP_0);
}

static void tg0_timer_init(int timer_idx,
                                   bool auto_reload, double timer_interval_ms) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_ms * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
}

void motorInit(int i, int pinEn, int pinStep, int pinDir, float gearRatio) {
    motors[i].currentSteps = 0;
    motors[i].pinEn = pinEn;
    motors[i].pinStep = pinStep;
    motors[i].pinDir = pinDir;
    motors[i].fractionComplete = 0;
    motors[i].stepsPerDeg = 200.0*gearRatio/360.0;
    motors[i].degPerStep = 1.0/motors[i].stepsPerDeg;
}

void csLow(int pinCS) {
    gpio_set_level(pinCS, 0);
}

void csHigh(int pinCS) {
    gpio_set_level(pinCS, 1);
}

float getAngle(spi_device_handle_t spi, int pinCS) {
    //get encoder angle
    spi_transaction_t t;
    csLow(pinCS);
    memset(&t, 0, sizeof(t));
    t.length = 16;
    // 0x3fff = 11111111111111, set 14 to 1 for read, set parity bit to 1
    uint16_t cmd = 0x3fff|1<<14|1<<15;
    t.tx_buffer = &cmd;
    t.flags = SPI_TRANS_USE_RXDATA;
    spi_device_transmit(spi, &t);
    csHigh(pinCS);
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
        return (float)angle/16383.0*360.0;
    } else {
        return 999;
    }

}

static void enc0_task(void *arg) {
    //set CS pins high to start
    csHigh(pinCS0);
    csHigh(pinCS1);
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
        .clock_speed_hz=10000000,           //Clock out at 10 MHz
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
        float angle;
        if(enableJ0) {
            angle = getAngle(spi, pinCS0);
            if(angle != 999) {
                if(abs(motors[0].currentPos - angle) > 10) {
                    motors[0].currentPos = 0.8*motors[0].currentPos + 0.2*angle;
                } else {
                    motors[0].currentPos = angle;
                }
            }
        } else {
            motors[0].currentPos = 0;
            motors[0].targetPos = 0;
        }

        if(enableJ1) {
            angle = getAngle(spi, pinCS1);
            if(angle != 999) {
                if(abs(motors[1].currentPos - angle) > 10) {
                    motors[1].currentPos = 0.8*motors[1].currentPos + 0.2*angle;
                } else {
                    motors[1].currentPos = angle;
                }
            }
        } else {
            motors[1].currentPos = 0;
            motors[1].targetPos = 0;
        }

        if(enableJ2) {
            angle = getAngle(spi, pinCS2);
            if(angle != 999) {
                if(abs(motors[2].currentPos - angle) > 10) {
                    motors[2].currentPos = 0.8*motors[2].currentPos + 0.2*angle;
                } else {
                    motors[2].currentPos = angle;
                }
            }
        } else {
            motors[2].currentPos = 0;
            motors[2].targetPos = 0;
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
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = 1ULL<<pinDir0|1ULL<<pinStep0|1ULL<<pinEn0|1ULL<<pinCS0;
    io_conf.pin_bit_mask |= 1ULL<<pinDir1|1ULL<<pinStep1|1ULL<<pinEn1|1ULL<<pinCS1;
    io_conf.pin_bit_mask |= 1ULL<<pinDir2|1ULL<<pinStep2|1ULL<<pinEn2|1ULL<<pinCS2;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(pinEn0, 1);
    gpio_set_level(pinEn1, 1);
    gpio_set_level(pinEn2, 1);

    //configure ADC
    adc2_config_channel_atten((adc2_channel_t)pinCharge, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_2, atten, width, DEFAULT_VREF, adc_chars);

    motorInit(0, pinEn0, pinStep0, pinDir0, 26.0+103.0/121.0);
    motorInit(1, pinEn1, pinStep1, pinDir1, 26.0+103.0/121.0);
    motorInit(2, pinEn2, pinStep2, pinDir2, 26.0+103.0/121.0);

    tg0_timer_init(TIMER_0, 1, TIMER_INTERVAL0_S);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_0_isr,
       (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    tg0_timer_init(TIMER_1, 0, TIMER_INTERVAL1_S);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timer_1_isr,
       (void *) TIMER_1, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_1);

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

        printf("J0 %.1f\t J1 %.1f\t J2 %.1f\n", motors[0].currentPos, motors[1].currentPos, motors[2].currentPos);
        // printf("J0 %.2f\t J1 %.2f\t\n", motors[0].fractionComplete, motors[1].fractionComplete);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}