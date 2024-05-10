#include <stdio.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"

#include "wifi_connecter.h"
//------    environment
#include <stdint.h>
#include <unistd.h>
#include "iot_i2c.h"
#include "iot_gpio.h"
#include "iot_pwm.h"
#include "iot_errno.h"
#include "hi_io.h"
//#include "iot_adc.h"
#include "aht20.h"
#include "oled_ssd1306.h"
#include "hi_adc.h"


#include <stdlib.h>
#include <dtls_al.h>
#include <mqtt_al.h>
#include <oc_mqtt_al.h>
#include <oc_mqtt_profile.h>

#include <cJSON.h>

#define CONFIG_APP_SERVERIP "291874cd92.st1.iotda-device.cn-north-4.myhuaweicloud.com"

#define CONFIG_APP_SERVERPORT "8883"

#define CONFIG_APP_DEVICEID "660a239b71d845632a035258_zhai_smoker" 

#define CONFIG_APP_DEVICEPWD "510da5c2f8b4838ab3a089275b1bb24fa8daf33369fd4cc8fb5b2c21116e3594" 

#define CONFIG_APP_LIFETIME 60 // seconds

#define CONFIG_QUEUE_TIMEOUT (5 * 1000)

#define MSGQUEUE_COUNT 16
#define MSGQUEUE_SIZE 10


osMessageQueueId_t mid_MsgQueue; // message queue id






   
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) sizeof(a)/sizeof(a[0])
#endif

#define MS_PER_S 1000

#define BEEP_TIMES 3
#define BEEP_DURATION 100
#define BEEP_PWM_DUTY 30000
#define BEEP_PWM_FREQ 4000
#define BEEP_PIN_NAME 9
#define BEEP_PIN_FUNCTION 5
#define WIFI_IOT_PWM_PORT_PWM0 0

#define GAS_SENSOR_CHAN_NAME 5
// #define GAS_SENSOR_PIN_NAME WIFI_IOT_IO_NAME_GPIO_11

#define AHT20_BAUDRATE 400*1000
#define AHT20_I2C_IDX 0

#define ADC_RESOLUTION 2048

unsigned int g_sensorStatus = 0;

//---------

//--------- 
typedef enum {
    en_msg_cmd = 0,
    en_msg_report,
    en_msg_conn,
    en_msg_disconn,
} en_msg_type_t;

typedef struct {
    char* request_id;
    char* payload;
} cmd_t;

typedef struct {
    float krqt_num;
    float temp_num;
    float humi_num;
} report_t;

typedef struct {
    en_msg_type_t msg_type;
    union {
        cmd_t cmd;
        report_t report;
    } msg;
} app_msg_t;

typedef struct {
    osMessageQueueId_t app_msg;
    int connected;
    int led;
    int motor;
} app_cb_t;
static app_cb_t g_app_cb;
 
report_t g_senser_num;


static void deal_report_msg(report_t* report)
{
    oc_mqtt_profile_service_t service;
    oc_mqtt_profile_kv_t krqt;
    oc_mqtt_profile_kv_t temp;
    oc_mqtt_profile_kv_t humi;

    if (g_app_cb.connected != 1) {
        return;
    }

    service.event_time = NULL;
    service.service_id = "krqt_senser";
    service.service_property = &krqt;
    service.nxt = NULL;

    krqt.key = "krqt_num";
    krqt.value = &report->krqt_num;
    krqt.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    krqt.nxt = &temp;

    temp.key = "temp_num";
    temp.value = &report->temp_num;
    temp.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    temp.nxt = &humi;

    humi.key = "humi_num";
    humi.value = &report->humi_num;
    humi.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    humi.nxt = NULL;

    oc_mqtt_profile_propertyreport(NULL, &service);
    return;
}

static int msg_rcv_callback(oc_mqtt_profile_msgrcv_t* msg)
{
    int ret = 0;
    char* buf;
    int buf_len;
    app_msg_t* app_msg;

    if ((msg == NULL) || (msg->request_id == NULL) || (msg->type != EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_COMMANDS)) {
        return ret;
    }

    buf_len = sizeof(app_msg_t) + strlen(msg->request_id) + 1 + msg->msg_len + 1;
    buf = malloc(buf_len);
    if (buf == NULL) {
        return ret;
    }
    app_msg = (app_msg_t*)buf;
    buf += sizeof(app_msg_t);

    app_msg->msg_type = en_msg_cmd;
    app_msg->msg.cmd.request_id = buf;
    buf_len = strlen(msg->request_id);
    buf += buf_len + 1;
    memcpy_s(app_msg->msg.cmd.request_id, buf_len, msg->request_id, buf_len);
    app_msg->msg.cmd.request_id[buf_len] = '\0';

    buf_len = msg->msg_len;
    app_msg->msg.cmd.payload = buf;
    memcpy_s(app_msg->msg.cmd.payload, buf_len, msg->msg, buf_len);
    app_msg->msg.cmd.payload[buf_len] = '\0';

    ret = osMessageQueuePut(g_app_cb.app_msg, &app_msg, 0U, CONFIG_QUEUE_TIMEOUT);
    if (ret != 0) {
        free(app_msg);
    }

    return ret;
}




static void oc_cmdresp(cmd_t* cmd, int cmdret)
{
    oc_mqtt_profile_cmdresp_t cmdresp;
    ///< do the response
    cmdresp.paras = NULL;
    cmdresp.request_id = cmd->request_id;
    cmdresp.ret_code = cmdret;
    cmdresp.ret_name = NULL;
    (void)oc_mqtt_profile_cmdresp(NULL, &cmdresp);
}

//static void deal_alarm_cmd(cmd_t* cmd, cJSON* obj_root)
//{
//    cJSON* obj_paras;
//    cJSON* obj_para;
//    int cmdret;
//
//    obj_paras = cJSON_GetObjectItem(obj_root, "Paras");
//    if (obj_paras == NULL) {
//        cJSON_Delete(obj_root);
//    }
//    obj_para = cJSON_GetObjectItem(obj_paras, "alarm_state");
//    if (obj_para == NULL) {
//        cJSON_Delete(obj_root);
//    }
//    ///< operate the Motor here
//    if (strcmp(cJSON_GetStringValue(obj_para), "ON") == 0) {
//        static char line[32] = { 0 };
//        uint32_t retval = 0;
//        OledInit();
//        OledFillScreen(0);
//        IoTI2cInit(AHT20_I2C_IDX, AHT20_BAUDRATE);
//        IoTGpioInit(BEEP_PIN_NAME);
//        retval = hi_io_set_func(BEEP_PIN_NAME, BEEP_PIN_FUNCTION);
//        if (retval != IOT_SUCCESS) {
//            printf("IoTGpioInit(9) failed, %0X!\n", retval);
//        }
//        IoTGpioSetDir(9, IOT_GPIO_DIR_OUT);
//        IoTPwmInit(WIFI_IOT_PWM_PORT_PWM0);
//
//        for (int i = 0; i < BEEP_TIMES; i++) {
//            snprintf(line, sizeof(line), "beep %d/%d", (i + 1), BEEP_TIMES);
//            OledShowString(0, 0, line, 1);
//
//            IoTPwmStart(WIFI_IOT_PWM_PORT_PWM0, 50, BEEP_PWM_FREQ);
//            usleep(BEEP_DURATION * 1000);
//            IoTPwmStop(WIFI_IOT_PWM_PORT_PWM0);
//            usleep((1000 - BEEP_DURATION) * 1000);
//        }
//        printf("alarm On!\r\n");
//    }
//    else {
//        printf("alarm Off!\r\n");
//    }
//    cmdret = 0;
//    oc_cmdresp(cmd, cmdret);
//
//_ERR:
//    cJSON_Delete(obj_root);
//    return;
//}

//static void deal_cmd_msg(cmd_t* cmd)
//{
//    cJSON* obj_root;
//    cJSON* obj_cmdname;
//
//    int cmdret = 1;
//    obj_root = cJSON_Parse(cmd->payload);
//    if (obj_root == NULL) {
//        oc_cmdresp(cmd, cmdret);
//    }
//    obj_cmdname = cJSON_GetObjectItem(obj_root, "command_name");
//    if (obj_cmdname == NULL) {
//        cJSON_Delete(obj_root);
//    }
//    /*if (strcmp(cJSON_GetStringValue(obj_cmdname), "Agriculture_Control_light") == 0) {
//        deal_light_cmd(cmd, obj_root);
//    } else if (strcmp(cJSON_GetStringValue(obj_cmdname), "Agriculture_Control_Motor") == 0) {
//        deal_motor_cmd(cmd, obj_root);
//    }*/
//    if (strcmp(cJSON_GetStringValue(obj_cmdname), "alarm") == 0) {
//        deal_alarm_cmd(cmd, obj_root);
//    }
//    return;
//}










//--------environment    
static float ConvertToVoltage(unsigned short data)
{
    return (float)data * 1.8 * 4 / 4096;
}

static void EnvironmentTask(void* arg)
{
    (void)arg;
    app_msg_t* app_msg;
    uint32_t retval = 0;
    float humidity = 0.0f;
    float temperature = 0.0f;
    float gasSensorResistance = 0.0f;
    static char line[32] = { 0 };

    OledInit();
    OledFillScreen(0);
    IoTI2cInit(AHT20_I2C_IDX, AHT20_BAUDRATE);

    // set BEEP pin as PWM function
    IoTGpioInit(BEEP_PIN_NAME);
    retval = hi_io_set_func(BEEP_PIN_NAME, BEEP_PIN_FUNCTION);
    if (retval != IOT_SUCCESS) {
        printf("IoTGpioInit(9) failed, %0X!\n", retval);
    }
    IoTGpioSetDir(9, IOT_GPIO_DIR_OUT);
    IoTPwmInit(WIFI_IOT_PWM_PORT_PWM0);

    for (int i = 0; i < BEEP_TIMES; i++) {
        snprintf(line, sizeof(line), "beep %d/%d", (i + 1), BEEP_TIMES);
        OledShowString(0, 0, line, 1);

        IoTPwmStart(WIFI_IOT_PWM_PORT_PWM0, 50, BEEP_PWM_FREQ);
        usleep(BEEP_DURATION * 1000);
        IoTPwmStop(WIFI_IOT_PWM_PORT_PWM0);
        usleep((1000 - BEEP_DURATION) * 1000);
    }

    while (IOT_SUCCESS != AHT20_Calibrate()) {
        printf("AHT20 sensor init failed!\r\n");
        usleep(1000);
    }

    while (1) {
        retval = AHT20_StartMeasure();
        if (retval != IOT_SUCCESS) {
            printf("trigger measure failed!\r\n");
        }

        retval = AHT20_GetMeasureResult(&temperature, &humidity);
        if (retval != IOT_SUCCESS) {
            printf("get humidity data failed!\r\n");
        }

        unsigned short data = 0;
        int ret;
        ret = hi_adc_read(GAS_SENSOR_CHAN_NAME, &data, HI_ADC_EQU_MODEL_4, HI_ADC_CUR_BAIS_DEFAULT, 0);
        if (ret == IOT_SUCCESS) {
            float Vx = ConvertToVoltage(data);

            // Vcc            ADC            GND
            //  |    ______   |     ______   |
            //  +---| MG-2 |---+---| 1kom |---+
            //       ------         ------
            //     ԭ  ͼ  ADC     λ   1K      ȼ        ֮ 䣬ȼ          һ ˽    5V   Դ      
            //       ·  ѹ    ֹ     ȣ 
            // Vx / 5 == 1kom / (1kom + Rx)
            //   => Rx + 1 == 5/Vx
            //   =>  Rx = 5/Vx - 1
            gasSensorResistance = 5 / Vx - 1;
            printf("\r\n hi_adc_read ok, data=%d, vx=%f, gasSensorResistance=%f", data, Vx, gasSensorResistance);
        }
        else {
            printf("\r\n hi_adc_read fail, ret=%d", ret);
        }
        g_senser_num.krqt_num = gasSensorResistance;
        g_senser_num.temp_num = temperature;
        g_senser_num.humi_num = humidity;
        int res[3] = { g_senser_num.krqt_num, g_senser_num.temp_num, g_senser_num.humi_num };
        app_msg = malloc(sizeof(app_msg_t));
        printf("SENSOR:krqt_senser:%f  %f  %f\r\n", res[0], res[1], res[2]);
        if (app_msg != NULL) {
            app_msg->msg_type = en_msg_report;
            app_msg->msg.report.krqt_num = res[0];
            app_msg->msg.report.temp_num = res[1];
            app_msg->msg.report.humi_num = res[2];
            if (osMessageQueuePut(g_app_cb.app_msg, &app_msg, 0U, CONFIG_QUEUE_TIMEOUT != 0)) {
                free(app_msg);
            }
        }
        OledShowString(0, 0, "Sensor values:", 1);

        snprintf(line, sizeof(line), "temp: %.2f", temperature);
        OledShowString(0, 1, line, 1);

        snprintf(line, sizeof(line), "humi: %.2f", humidity);
        OledShowString(0, 2, line, 1);

        snprintf(line, sizeof(line), "gas: %.2f kom", gasSensorResistance);
        OledShowString(0, 3, line, 1);

        if (temperature > 35.0 || temperature < 0) {
            g_sensorStatus++;
            OledShowString(0, 5, "temp abnormal!!", 1);
        }

        if (humidity < 20.0 || humidity > 50.0) {
            g_sensorStatus++;
            if (temperature > 35.0 || temperature < 0) {
                OledShowString(0, 6, "humi abnormal!!", 1);
            }
            else {
                OledShowString(0, 5, "humi abnormal!!", 1);
            }
        }

        if (g_sensorStatus > 0) {
            IoTPwmStart(WIFI_IOT_PWM_PORT_PWM0, 50, 4000);
            usleep(500000);
            IoTPwmStop(WIFI_IOT_PWM_PORT_PWM0);
            g_sensorStatus = 0;
        }

        usleep(500000);
    }
}



//static int SensorTaskEntry(void)
//{
//    app_msg_t* app_msg;
//
//    while (1) {
//        int res[3] = { g_senser_num.krqt_num, g_senser_num.temp_num, g_senser_num.humi_num };
//        app_msg = malloc(sizeof(app_msg_t));
//        printf("SENSOR:krqt_senser:%f  %f  %f\r\n", res[0], res[1], res[2]);
//        if (app_msg != NULL) {
//            app_msg->msg_type = en_msg_report;
//            app_msg->msg.report.krqt_num = res[0];
//            app_msg->msg.report.temp_num = res[1];
//            app_msg->msg.report.humi_num = res[2];
//            if (osMessageQueuePut(g_app_cb.app_msg, &app_msg, 0U, CONFIG_QUEUE_TIMEOUT != 0)) {
//                free(app_msg);
//            }
//        }
//        osal_task_sleep(2000);
//    }
//    return 0;
//}

static int CloudMainTaskEntry(void)
{
    app_msg_t* app_msg;
    uint32_t ret;


    g_app_cb.app_msg = osMessageQueueNew(MSGQUEUE_COUNT, MSGQUEUE_SIZE, NULL);
    if (g_app_cb.app_msg == NULL) {
        printf("Create receive msg queue failed");
    }
    oc_mqtt_profile_connect_t connect_para;
    (void)memset_s(&connect_para, sizeof(connect_para), 0, sizeof(connect_para));

    connect_para.boostrap = 0;
    connect_para.device_id = CONFIG_APP_DEVICEID;
    connect_para.device_passwd = CONFIG_APP_DEVICEPWD;
    connect_para.server_addr = CONFIG_APP_SERVERIP;
    connect_para.server_port = CONFIG_APP_SERVERPORT;
    connect_para.life_time = CONFIG_APP_LIFETIME;
    connect_para.rcvfunc = msg_rcv_callback;
    connect_para.security.type = EN_DTLS_AL_SECURITY_TYPE_NONE;
    ret = oc_mqtt_profile_connect(&connect_para);
    printf("ret = %u\n", ret);
    if ((ret == (int)en_oc_mqtt_err_ok)) {
        g_app_cb.connected = 1;
        printf("oc_mqtt_profile_connect succed!\r\n");
    }
    else {
        printf("oc_mqtt_profile_connect faild!\r\n");
    }

    while (1) {
        app_msg = NULL;
        (void)osMessageQueueGet(g_app_cb.app_msg, (void**)&app_msg, NULL, 0xFFFFFFFF);
        if (app_msg != NULL) {
            switch (app_msg->msg_type) {
            /*case en_msg_cmd:
                deal_cmd_msg(&app_msg->msg.cmd);
                break;*/
            case en_msg_report:
                deal_report_msg(&app_msg->msg.report);
                break;
            default:
                break;
            }
            free(app_msg);
        }
    }
    return 0;
}

static void WifiConnectTask(void *arg)
{
    (void)arg;

    osDelay(10);

    // setup your AP params
    WifiDeviceConfig apConfig = {0};
    strcpy(apConfig.ssid, "vivo S18 Pro");
    strcpy(apConfig.preSharedKey, "zyt090016");
    apConfig.securityType = WIFI_SEC_TYPE_PSK;

//    int netId = ConnectToHotspot(&apConfig);

//    int timeout = 60;
//    while (timeout--) {
//        printf("After %d seconds I will disconnect with AP!\r\n", timeout);
//        osDelay(100);
//    }

//    DisconnectWithHotspot(netId);
}

static void CloudMainTask(void* arg) {
    (void)arg;
    CloudMainTaskEntry();
}

//static void SensorTask(void* arg) {
//    (void)arg;
//    SensorTaskEntry();
//}




static void WifiConnectDemo(void)
{
    osThreadAttr_t attr;
    osThreadAttr_t attr_environment;
    osThreadAttr_t attr_cloud;
    //osThreadAttr_t attr_sensor;


    //--------
    IoTGpioInit(BEEP_PIN_NAME);
    hi_io_set_func(BEEP_PIN_NAME, BEEP_PIN_FUNCTION);
    IoTPwmInit(WIFI_IOT_PWM_PORT_PWM0);
    //-------


    attr.name = "WifiConnectTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew(WifiConnectTask, NULL, &attr) == NULL) {
        printf("[WifiConnectDemo] Falied to create WifiConnectTask!\n");
    }



    //--------environment ߳ 
    attr_environment.name = "EnvironmentTask";
    attr_environment.attr_bits = 0U;
    attr_environment.cb_mem = NULL;
    attr_environment.cb_size = 0U;
    attr_environment.stack_mem = NULL;
    attr_environment.stack_size = 4096;
    attr_environment.priority = osPriorityNormal;


    if (osThreadNew(EnvironmentTask, NULL, &attr_environment) == NULL) {
        printf("[EnvironmentDemo] Falied to create EnvironmentTask!\n");
    }


    attr_cloud.name = "CloudMainTask";
    attr_cloud.attr_bits = 0U;
    attr_cloud.cb_mem = NULL;
    attr_cloud.cb_size = 0U;
    attr_cloud.stack_mem = NULL;
    attr_cloud.stack_size = 1024 * 10; //       Ҫ    ջ  С
    attr_cloud.priority = osPriorityNormal;

    //          ߳ 
    if (osThreadNew(CloudMainTask, NULL, &attr_cloud) == NULL) {
        printf("[AppFeatureInit] Failed to create CloudMainTask!\n");
    }


    //attr_sensor.name = "SensorTask";
    //attr_sensor.attr_bits = 0U;
    //attr_sensor.cb_mem = NULL;
    //attr_sensor.cb_size = 0U;
    //attr_sensor.stack_mem = NULL;
    //attr_sensor.stack_size = 1024 * 5; 
    //attr_sensor.priority = osPriorityLow;

    //if (osThreadNew(SensorTask, NULL, &attr_sensor) == NULL) {
    //    printf("[AppFeatureInit] Failed to create SensorTask!\n");
    //}

}

SYS_RUN(WifiConnectDemo);
