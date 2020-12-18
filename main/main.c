#include <stdio.h>
#include <stdlib.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "esp_wifi.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include "esp_intr_alloc.h"
//#include "servo.h"

#include <math.h>

//arms in mm
#define r1 50.5
#define r2 95

#define SSID "ESP32AP"
#define PASSWORD "password"

struct angles {
    int theta1, theta2;
};

struct polarCoords {
    int r, theta;
};

#define PWM_HZ 50

#define MOTOR_PWM_TIMER_HIGH_SPEED LEDC_TIMER_1
#define MOTOR_PWM_TIMER_LOW_SPEED LEDC_TIMER_2
#define MOTOR_PWM_BIT_NUM LEDC_TIMER_16_BIT

#define SERVO_HIGH_0_GPIO_NUM GPIO_NUM_2
#define SERVO_HIGH_1_GPIO_NUM GPIO_NUM_15
#define SERVO_HIGH_2_GPIO_NUM GPIO_NUM_18
#define SERVO_HIGH_3_GPIO_NUM GPIO_NUM_19
#define SERVO_HIGH_4_GPIO_NUM GPIO_NUM_33
#define SERVO_HIGH_5_GPIO_NUM GPIO_NUM_32
#define SERVO_HIGH_6_GPIO_NUM GPIO_NUM_27
#define SERVO_HIGH_7_GPIO_NUM GPIO_NUM_14
#define SERVO_LOW_0_GPIO_NUM GPIO_NUM_4
#define SERVO_LOW_1_GPIO_NUM GPIO_NUM_5
#define SERVO_LOW_2_GPIO_NUM GPIO_NUM_25
#define SERVO_LOW_3_GPIO_NUM GPIO_NUM_26

void motor_pwm_init_timers()
{
    ledc_timer_config_t ledc_timer_high_speed = {0};
    ledc_timer_high_speed.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer_high_speed.bit_num = MOTOR_PWM_BIT_NUM;
    ledc_timer_high_speed.timer_num = MOTOR_PWM_TIMER_HIGH_SPEED;
    ledc_timer_high_speed.freq_hz = 50;

    ledc_timer_config_t ledc_timer_low_speed = {0};
    ledc_timer_low_speed.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer_low_speed.bit_num = MOTOR_PWM_BIT_NUM;
    ledc_timer_low_speed.timer_num = MOTOR_PWM_TIMER_LOW_SPEED;
    ledc_timer_low_speed.freq_hz = 50;

    ESP_ERROR_CHECK( ledc_timer_config(&ledc_timer_high_speed) );
    ESP_ERROR_CHECK( ledc_timer_config(&ledc_timer_low_speed) );
}

void motor_pwm_init(ledc_channel_config_t ledcChannel, ledc_channel_t channelNum, gpio_num_t gpioNum, bool speedMode) //true is high for speedMode
{
    //ledc_channel_config_t ledc_channel_left = {0}, ledc_channel_right = {0};

    ledcChannel.gpio_num = gpioNum;
    ledcChannel.channel = channelNum;
    ledcChannel.intr_type = LEDC_INTR_DISABLE;
    ledcChannel.duty = 0;

    if(speedMode) {
        ledcChannel.speed_mode = LEDC_HIGH_SPEED_MODE;
        ledcChannel.timer_sel = LEDC_TIMER_1;
    }
    else {
        ledcChannel.speed_mode = LEDC_LOW_SPEED_MODE;
        ledcChannel.timer_sel = LEDC_TIMER_2;
    }

    ESP_ERROR_CHECK( ledc_channel_config(&ledcChannel));
}

struct LegPos {
    int x;
    int y;
    int theta;
};

struct LegPos legs[4] = {};

void motor_pwm_set(float duty_fraction, ledc_mode_t speedMode, ledc_channel_t channelNum, ledc_fade_mode_t fadeMode) {
    uint32_t max_duty = (1 << MOTOR_PWM_BIT_NUM) - 1;
    uint32_t duty = lroundf(duty_fraction/100 * (float)max_duty);
    
    ESP_ERROR_CHECK( ledc_set_fade_with_time(speedMode, channelNum, duty, 400) );
    ESP_ERROR_CHECK( ledc_fade_start(speedMode, channelNum, fadeMode) );
}

/*double servoGetDutyByPercentage(double percentage){
    if (percentage <= 0){
        return 0;
    }
    if (percentage > 100){
        percentage = 100;
    }
    printf("made it 3\n");
    return (percentage / 100.0) * ((2<<(LEDC_TIMER_16_BIT-1))-1);
}*/

double servoGetDutyByuS(double uS){
    return ((uS * 100.0)/(1000000/PWM_HZ));
}

void servoWrite(ledc_mode_t speedMode, ledc_channel_t channelNum, unsigned int value, ledc_fade_mode_t fadeMode) {
    // 0 = MinServoAngle ; 180 = Max ServoAngle;
    int scale = (value) * (2550 - 500) / (180 - 0) + 500;
    motor_pwm_set(servoGetDutyByuS(scale), speedMode, channelNum, fadeMode);
}

static struct angles calcAngle(int x, int y) {
    //flip x and y and subtract 90 degrees
    double thetaT = atan((double)y/x);
    double rT = sqrt(x * x + y * y);
    double theta1T = acos((r1 * r1 + rT * rT - r2 * r2)/(2.0 * r1 * rT));
    double theta2 = acos((r1 * r1 + r2 * r2 - rT * rT)/(2.0 * r1 * r2));
    double theta1 = theta1T - thetaT;
    /*printf("%d\n", x);
    printf("%d\n", y);
    printf("%d\n", (int)(theta1T * 180 / M_PI));*/
    struct angles a = {(int)(theta1 * 180 / M_PI) /*- 90*/, (int)(theta2 * 180 / M_PI)};
    return a;
}

static struct polarCoords calcStraightLine(int curR, int curTheta, int mR, int mTheta){
    double x = curR * cos(curTheta * M_PI / 180) +  mR * cos(mTheta * M_PI / 180);
    double y = curR * sin(curTheta * M_PI / 180) +  mR * sin(mTheta * M_PI / 180);
    printf("%f\n", x);
    printf("%f\n", y);
    double theta = atan2(y, x);
    double r = sqrt(x * x + y * y);
    struct polarCoords p = {(int)(r), (int)(theta * 180 / M_PI)};
    return p;
}

int modulo( int value, unsigned m) {
    int mod = value % (int)m;
    if (mod < 0) {
        mod += m;
    }
    return mod;
}

/*void moveLeg(int x, int y, int theta, int leg){
    printf("%d\n", legs[leg].x);
    printf("%d\n", legs[leg].theta + 90*leg);
    struct polarCoords newCoords = calcStraightLine(legs[leg].x, legs[leg].theta + 90*leg, x, theta);
    printf("%d\n", newCoords.r);
    printf("%d\n", newCoords.theta);
    printf("%d\n", newCoords.theta - 90*leg);
    printf("%d\n", modulo(newCoords.theta - 90*leg, 360));
    struct angles a = calcAngle(newCoords.r, y);
    ledc_channel_t tibia = LEDC_CHANNEL_0;
    ledc_channel_t femur = LEDC_CHANNEL_1;
    ledc_channel_t coxa = LEDC_CHANNEL_0;
    switch(leg){
        case 0:
            tibia = LEDC_CHANNEL_1;
            femur = LEDC_CHANNEL_0;
            coxa = LEDC_CHANNEL_0;
            break;
        case 1:
            tibia = LEDC_CHANNEL_3;
            femur = LEDC_CHANNEL_2;
            coxa = LEDC_CHANNEL_1;
            break;
        case 2:
            tibia = LEDC_CHANNEL_5;
            femur = LEDC_CHANNEL_4;
            coxa = LEDC_CHANNEL_2;
            break;
        case 3:
            tibia = LEDC_CHANNEL_7;
            femur = LEDC_CHANNEL_6;
            coxa = LEDC_CHANNEL_3;
            break;
    }
    if((a.theta1 <= 90 && a.theta1 >= 0) && (a.theta2 <= 180 && a.theta2 >= 0)){
        servoWrite(LEDC_HIGH_SPEED_MODE, femur, a.theta1 + 90, LEDC_FADE_NO_WAIT);
        servoWrite(LEDC_HIGH_SPEED_MODE, tibia, 180 - a.theta2, LEDC_FADE_NO_WAIT);
        servoWrite(LEDC_LOW_SPEED_MODE, coxa, modulo(newCoords.theta - 90*leg, 360), LEDC_FADE_WAIT_DONE);
    }
    legs[leg].x = newCoords.r;
    legs[leg].y = y;
    legs[leg].theta = newCoords.theta - 90*leg;
}*/

void moveLeg(int x, int y, int theta, int leg, bool wait){
    struct angles a = calcAngle(x, y);
    ledc_channel_t tibia = LEDC_CHANNEL_0;
    ledc_channel_t femur = LEDC_CHANNEL_1;
    ledc_channel_t coxa = LEDC_CHANNEL_0;
    switch(leg){
        case 0:
            tibia = LEDC_CHANNEL_1;
            femur = LEDC_CHANNEL_0;
            coxa = LEDC_CHANNEL_0;
            break;
        case 1:
            tibia = LEDC_CHANNEL_3;
            femur = LEDC_CHANNEL_2;
            coxa = LEDC_CHANNEL_1;
            break;
        case 2:
            tibia = LEDC_CHANNEL_5;
            femur = LEDC_CHANNEL_4;
            coxa = LEDC_CHANNEL_2;
            break;
        case 3:
            tibia = LEDC_CHANNEL_7;
            femur = LEDC_CHANNEL_6;
            coxa = LEDC_CHANNEL_3;
            break;
    }
    if((a.theta1 <= 90 && a.theta1 >= 0) && (a.theta2 <= 180 && a.theta2 >= 0)){
        servoWrite(LEDC_HIGH_SPEED_MODE, femur, a.theta1 + 90, LEDC_FADE_NO_WAIT);
        servoWrite(LEDC_HIGH_SPEED_MODE, tibia, 180 - a.theta2, LEDC_FADE_NO_WAIT);
        if(wait){
            servoWrite(LEDC_LOW_SPEED_MODE, coxa, theta, LEDC_FADE_WAIT_DONE);
        }
        else{
            servoWrite(LEDC_LOW_SPEED_MODE, coxa, theta, LEDC_FADE_NO_WAIT);
        }
    }
    legs[leg].x = x;
    legs[leg].y = y;
    legs[leg].theta = theta;
}

static esp_err_t getHandler(httpd_req_t *req)
{ 
    /* Send response with custom headers and body set as the
     * string passed in user context*/
    //const char* resp_str = (const char*) req->user_ctx;
    //httpd_resp_send(req, resp_str, strlen(resp_str));

    char*  buf;
    size_t buf_len;
    int x = 0;
    int y = 0;
    int theta = 0;
    int leg = 0;

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            printf("Found URL query => %s\n", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "x", param, sizeof(param)) == ESP_OK) {
                printf("Found URL query parameter => x=%s\n", param);
                x = atoi(param);
            }
            if (httpd_query_key_value(buf, "y", param, sizeof(param)) == ESP_OK) {
                printf("Found URL query parameter => y=%s\n", param);
                y = atoi(param);                
            }
            if (httpd_query_key_value(buf, "theta", param, sizeof(param)) == ESP_OK) {
                printf("Found URL query parameter => theta=%s\n", param);
                theta = atoi(param);                
            }
            if (httpd_query_key_value(buf, "leg", param, sizeof(param)) == ESP_OK) {
                printf("Found URL query parameter => leg=%s\n", param);
                leg = atoi(param);
            }
        }
        free(buf);
    }

    char* data1="<!DOCTYPE html><html>";
    char* data2="<head><title>ESP32 Web Server</title></head>";
    char* data3="<body><h1>ESP32 Web Server</h1><p>This is the first test for a server from ESP32.</p></body></html>";
    char* end_data = "";
 
    httpd_resp_send_chunk(req,data1,strlen(data1));
    httpd_resp_send_chunk(req,data2,strlen(data2));
    httpd_resp_send_chunk(req,data3,strlen(data3));
    httpd_resp_send_chunk(req,end_data,strlen(end_data));

    //struct angles a = calcAngle(x,y);
    //printf("Theta1: %d, Theta2: %d\n", a.theta1 + 90, a.theta2);
    /*servoWrite(servo1, a.theta1);
    servoWrite(servo2, a.theta2);*/
    //if (r1 + r2 >= sqrt(x * x + y * y) && sqrt(x * x + y * y) >= r2 - r1){
        //servoWrite(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, a.theta1 + 90);
        //servoWrite(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 180 - a.theta2);
    struct polarCoords newCoords0 = calcStraightLine(40, 120, 30, 45);
    struct polarCoords newCoords1 = calcStraightLine(40, 60, 30, -45);
    struct polarCoords newCoords2 = calcStraightLine(40, 120, 30, 225);
    struct polarCoords newCoords3 = calcStraightLine(40, 60, 30, 135);

    moveLeg(40, 40, 120, 0, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords0.r, 40, newCoords0.theta, 0, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords0.r, 70, newCoords0.theta, 0, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 60, 1, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords1.r, 40, newCoords1.theta, 1, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords1.r, 70, newCoords1.theta, 1, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 120, 2, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords2.r, 40, newCoords2.theta, 2, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords2.r, 70, newCoords2.theta, 2, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 60, 3, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords3.r, 40, newCoords3.theta, 3, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords3.r, 70, newCoords3.theta, 3, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);


    moveLeg(40, 70, 120, 0, false);
    moveLeg(40, 70, 60, 1, false);
    moveLeg(40, 70, 120, 2, false);
    moveLeg(40, 70, 60, 3, false);
        /*moveLeg(x, y, theta, 1);
        moveLeg(x, y, theta, 2);
        moveLeg(x, y, theta, 3);*/
    /*}
    else {
        printf("Failed. rT:%f\n", sqrt(x * x + y * y));
    }*/

    return ESP_OK;
}

static esp_err_t forwardHandler(httpd_req_t *req)
{ 
    char*  buf;
    size_t buf_len;
    int d = 0;

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            printf("Found URL query => %s\n", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "d", param, sizeof(param)) == ESP_OK) {
                printf("Found URL query parameter => d=%s\n", param);
                d = atoi(param);                
            }
        }
        free(buf);
    }

    char* data1="<!DOCTYPE html><html>";
    char* data2="<head><title>ESP32 Web Server</title></head>";
    char* data3="<body><h1>ESP32 Web Server</h1><p>This is the first test for a server from ESP32.</p></body></html>";
    char* end_data = "";
 
    httpd_resp_send_chunk(req,data1,strlen(data1));
    httpd_resp_send_chunk(req,data2,strlen(data2));
    httpd_resp_send_chunk(req,data3,strlen(data3));
    httpd_resp_send_chunk(req,end_data,strlen(end_data));

    struct polarCoords newCoords0 = calcStraightLine(40, 60, d, 45);
    struct polarCoords newCoords1 = calcStraightLine(40, 120, d, -45);
    struct polarCoords newCoords2 = calcStraightLine(40, 60, d, 225);
    struct polarCoords newCoords3 = calcStraightLine(40, 120, d, 135);

    moveLeg(40, 40, 60, 0, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords0.r, 40, newCoords0.theta, 0, true);
    moveLeg(newCoords0.r, 70, newCoords0.theta, 0, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 120, 1, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords1.r, 40, newCoords1.theta, 1, true);
    moveLeg(newCoords1.r, 70, newCoords1.theta, 1, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 60, 2, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords2.r, 40, newCoords2.theta, 2, true);
    moveLeg(newCoords2.r, 70, newCoords2.theta, 2, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 120, 3, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(newCoords3.r, 40, newCoords3.theta, 3, true);
    moveLeg(newCoords3.r, 70, newCoords3.theta, 3, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 70, 60, 0, false);
    moveLeg(40, 70, 120, 1, false);
    moveLeg(40, 70, 60, 2, false);
    moveLeg(40, 70, 120, 3, false);
    return ESP_OK;
}

static esp_err_t turnHandler(httpd_req_t *req)
{ 
    char*  buf;
    size_t buf_len;
    int theta = 0;

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            printf("Found URL query => %s\n", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "theta", param, sizeof(param)) == ESP_OK) {
                printf("Found URL query parameter => theta=%s\n", param);
                theta = atoi(param);                
            }
        }
        free(buf);
    }

    char* data1="<!DOCTYPE html><html>";
    char* data2="<head><title>ESP32 Web Server</title></head>";
    char* data3="<body><h1>ESP32 Web Server</h1><p>This is the first test for a server from ESP32.</p></body></html>";
    char* end_data = "";
 
    httpd_resp_send_chunk(req,data1,strlen(data1));
    httpd_resp_send_chunk(req,data2,strlen(data2));
    httpd_resp_send_chunk(req,data3,strlen(data3));
    httpd_resp_send_chunk(req,end_data,strlen(end_data));

    moveLeg(40, 40, 60, 0, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(40, 40, 60 + theta, 0, true);
    moveLeg(40, 70, 60 + theta, 0, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 120, 1, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(40, 40, 120 + theta, 1, true);
    moveLeg(40, 70, 120 + theta, 1, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 60, 2, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(40, 40, 60 + theta, 2, true);
    moveLeg(40, 70, 60 + theta, 2, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 40, 120, 3, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    moveLeg(40, 40, 120 + theta, 3, true);
    moveLeg(40, 70, 120 + theta, 3, true);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    moveLeg(40, 70, 60, 0, false);
    moveLeg(40, 70, 120, 1, false);
    moveLeg(40, 70, 60, 2, false);
    moveLeg(40, 70, 120, 3, false);
    return ESP_OK;
}

static const httpd_uri_t get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = getHandler,
    .user_ctx = NULL
};

static const httpd_uri_t forward = {
    .uri = "/forward",
    .method = HTTP_GET,
    .handler = forwardHandler,
    .user_ctx = NULL
};

static const httpd_uri_t turn = {
    .uri = "/turn",
    .method = HTTP_GET,
    .handler = turnHandler,
    .user_ctx = NULL
};

static httpd_handle_t startWebserver()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    if(httpd_start(&server, &config) == ESP_OK)
    {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &get));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &forward));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &turn));
        return server;
    }
    printf("http server error\n");
    return NULL;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        startWebserver();
    } /*else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_ERROR_CHECK(httpd_stop())
    }*/
}

static void startSoftAP()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifiInitConfig));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t apConfig = {
        .ap = {
            .ssid = SSID,
            .ssid_len = strlen(SSID),
            .channel = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .password = PASSWORD,
            .ssid_hidden = 0,
            .max_connection = 4,
            .beacon_interval = 100
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &apConfig));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("AP Configuration Successful\n");
}

void app_main()
{
    motor_pwm_init_timers();

    /*legs[0].x = 0;
    legs[0].y = 0;
    legs[0].theta = 90;

    legs[1].x = 0;
    legs[1].y = 0;
    legs[1].theta = 90;

    legs[2].x = 0;
    legs[2].y = 0;
    legs[2].theta = 90;

    legs[3].x = 0;
    legs[3].y = 0;
    legs[3].theta = 90;*/

    ESP_ERROR_CHECK( ledc_fade_func_install(0) );

    ledc_channel_config_t ledc_channel_0 = {0}, ledc_channel_1 = {0}, ledc_channel_2 = {0}, ledc_channel_3 = {0}, ledc_channel_4 = {0}, ledc_channel_5 = {0}, ledc_channel_6 = {0}, ledc_channel_7 = {0}, ledc_channel_8 = {0}, ledc_channel_9 = {0}, ledc_channel_10 = {0}, ledc_channel_11 = {0};
    motor_pwm_init(ledc_channel_0, LEDC_CHANNEL_0, SERVO_HIGH_0_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_1, LEDC_CHANNEL_1, SERVO_HIGH_1_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_2, LEDC_CHANNEL_2, SERVO_HIGH_2_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_3, LEDC_CHANNEL_3, SERVO_HIGH_3_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_4, LEDC_CHANNEL_4, SERVO_HIGH_4_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_5, LEDC_CHANNEL_5, SERVO_HIGH_5_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_6, LEDC_CHANNEL_6, SERVO_HIGH_6_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_7, LEDC_CHANNEL_7, SERVO_HIGH_7_GPIO_NUM, true);
    motor_pwm_init(ledc_channel_8, LEDC_CHANNEL_0, SERVO_LOW_0_GPIO_NUM, false);
    motor_pwm_init(ledc_channel_9, LEDC_CHANNEL_1, SERVO_LOW_1_GPIO_NUM, false);
    motor_pwm_init(ledc_channel_10, LEDC_CHANNEL_2, SERVO_LOW_2_GPIO_NUM, false);
    motor_pwm_init(ledc_channel_11, LEDC_CHANNEL_3, SERVO_LOW_3_GPIO_NUM, false);

    moveLeg(40, 70, 60, 0, false);//increase each by 90 starting at 90
    moveLeg(40, 70, 120, 1, false);
    moveLeg(40, 70, 60, 2, false);
    moveLeg(40, 70, 120, 3, false);

    /*legs[0].x = 40;
    legs[0].y = 70;
    legs[0].theta = 90;

    legs[1].x = 40;
    legs[1].y = 70;
    legs[1].theta = 90;

    legs[2].x = 40;
    legs[2].y = 70;
    legs[2].theta = 90;

    legs[3].x = 40;
    legs[3].y = 70;
    legs[3].theta = 90;*/

    ESP_ERROR_CHECK(nvs_flash_init());
    startSoftAP();
}
