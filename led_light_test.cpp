/*
 * Copyright (c) 2016-2017 zzg@idealte.com.  All Rights Reserved.
 */

// system includes
#include <cmath>
#include <cstdbool>
#include <cstring>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <string>
#include <sys/un.h>
#include <stddef.h>
#include <sys/time.h>
#include <inttypes.h>
//#include <mysql/mysql.h>


// Waypoint utilities
#include "snav_waypoint_utils.hpp"
// Snapdragon Navigator
#include "snapdragon_navigator.h"

using namespace std;


#define __DEBUG

#ifdef __DEBUG
#define DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format, ...)
#endif

/**************Static Function switcher defines******************/
#define USE_SNAV_DEV                                // use the snav or snav-dev dpkg
#define LOW_BATTERY_AUTO_LANDING                    // auto land when in low battery status
#define CIRCLE_HEIGHT_LIMIT_FLAG
#define RGB_RED_GREEN_OPPOSITE
#define DYNAMIC_GPS_MODE_HEIGHT
//#define USE_REVISE_HEIGHT
//#define LOW_SAMPLE_SIZE_SWITCH_ALT_MODE
//#define HEIGHT_LIMIT
//#define AUTO_REDUCE_HEIGHT
//#define AUTO_FACE_TAKE_OFF
//#define AUTO_ALT_MODE_SWITCH
//#define DISABLE_INFRARED_TAKEOFF

#define CSV_LOG_FOR_XIAOYI                          // for XiaoYi Development
#define ZZG_DEBUG_FLAG

//#define VEL_LIMIT_FLAG
//#define SPEED_LIMIT_FLAG
//#define LOW_HEIGHT_LIMIT_VEL


/********************Dynamic variables**************************/
/* low battery set */
const float low_battery_led_warning = 6.9f;        //6.75f
const float force_landing_battery_indoor = 6.75f;   //6.55f;
const float force_landing_battery_outdoor = 6.8f;   //6.7f;

/* udp timeout set */
struct timeval timeout_udp = {0, 20000};            //20ms
struct timeval timeout_follow = {0, 300000};        //300ms cuiyc

/* time interval set */
/*const*/ double time_interval_of_sonar_valid = 0.5;        // S: for switch to optic-flow-mode
/*const*/ double time_interval_of_gps_valid = 2;        // S: for switch to gps-mode
const double time_interval_of_imu_invalid = 0.2;    // S: for stop propers when drone is inverted
const double time_interval_of_low_spin = 0.2;       // S: for stop propers more faster when landing on the ground
const double time_for_spin_on_ground = 10;          // S: for auto stop propers when keep on the ground
const double time_interval_of_face_takeoff = 4;     // S: for face auto takeoff

/* log file limit */
const int log_file_size_total_limit = 100;  //1024;         // 1G
const int log_file_size_single_limit = 20;  //200;         // 200M

/* hover vel and brake cmd limit */
const float hover_vel_limit = 0.2;                  // over the vel limit will auto reverse the drone
const float hover_brake_limit = 0.2;                // cmd limit of the auto reverse

const float control_msg_max_diff = 0.199;   //0.099;           // s

#define PATH_FLAG                   "/"

#define SNAV_LOG_PATH               "/var/log/snav/flight"
#define FC_LOG_PATH                 "/home/linaro"
#define FC_LOG_CT_NAME              "flightctrl_proxy_count"
#define FC_LOG_CT_MAX               8000

#define SNAV_DEV_PKG_NAME           "snav-dev"
#define SNAV_PKG_NAME               "snav"
#define FC_PKG_NAME                 "flightctrl-proxy"
#define CAMERA_PKG_NAME             "mm-video"

#define WIFI_FILE_NAME              "/etc/hostapd.conf"
#define CAM_CFG_FILE_NAME           "/etc/camera.cfg"
#define SNAV_UPDATE_FILE_NAME       "/tmp/update-snav.zip"
#define LINARO_UPDATE_FILE_NAME     "/tmp/update-linaro.zip"

#define SNAV_CFG_FILE_NAME          "/usr/share/data/adsp/200qc_runtime_params.xml"



/* ***********************************************************************************/
#define SERVER_UDP_PORT                         14559
#define QCAM_DOMAIN_PORT                        16889
#define CAM_SUPER_PORT                          16885
#define OTA_UDP_PORT                            14888

#define TRACKING_SERVER_PORT                    7777
#define TRACKING_FORAPP_PORT                    17455

#define MAX_BUFF_LEN                            512
#define TMP_BUFF_LEN                            128

#define MIN_GPS_POSITION_NUM                    2
#define MAX_GPS_POSITION_NUM                    10

#define DOMAIN_BUFF_SIZE                        16

#define STR_SEPARATOR                           ","


// CMD from App
#define SNAV_CMD_CONROL                         "1000"

#define SNAV_CMD_TAKE_OFF                       "1001"
#define SNAV_CMD_LAND                           "1002"
#define SNAV_CMD_RETURN                         "1003"
#define SNAV_CMD_CIRCLE                         "1004"
#define SNAV_CMD_TRAIL_NAVIGATION               "1005"
#define SNAV_CMD_GPS_FOLLOW                     "1006"
#define SNAV_CMD_PANORAMA                       "1007"
#define SNAV_CMD_MAG_CALIBRATE                  "1008"
#define SNAV_CMD_HOR_CALIBRATE                  "1009"
#define SNAV_CMD_MODIFY_SSID_PWD                "1025"
#define SNAV_CMD_CHECK_WIFI_MODE                "1026"
#define SNAV_CMD_MODIFY_WIFI_5G                 "1027"
#define SNAV_CMD_MODIFY_WIFI_2G                 "1028"
#define SNAV_CMD_FACE_FOLLOW                    "1100"
#define SNAV_CMD_FACE_FOLLOW_MODE               "1110"
#define SNAV_CMD_BODY_FOLLOW                    "1101"
#define SNAV_CMD_CHECK_GPS_STATUS               "1102"
#define SNAV_CMD_OPEN_GPS                       "1103"
#define SNAV_CMD_CLOSE_GPS                      "1104"
#define SNAV_CMD_CHECK_CAM_FREQ                 "1105"
#define SNAV_CMD_MODIFY_CAM_FREQ                "1106"
#define SNAV_CMD_CUSTOMIZED_PLAN                "1107"
#define SNAV_CMD_FACE_TAKE_OFF_SWITCH           "1108"
#define SNAV_CMD_OPTIC_FLOW_CALIB               "1201"
#define SNAV_CMD_FLY_TEST                       "1202"
#define SNAV_CMD_ROTATION_TEST                  "1203"

#define SNAV_CMD_RETURN_CONROL                  "2000"

#define SNAV_CMD_RETURN_TAKE_OFF                "2001"
#define SNAV_CMD_RETURN_LAND                    "2002"
#define SNAV_CMD_RETURN_RETURN                  "2003"
#define SNAV_CMD_RETURN_CIRCLE                  "2004"
#define SNAV_CMD_RETURN_TRAIL_NAVIGATION        "2005"
#define SNAV_CMD_RETURN_GPS_FOLLOW              "2006"
#define SNAV_CMD_RETURN_PANORAMA                "2007"
#define SNAV_CMD_RETURN_MAG_CALIBRATE           "2008"
#define SNAV_CMD_RETURN_HOR_CALIBRATE           "2009"
#define SNAV_CMD_RETURN_MODIFY_SSID_PWD         "2025"
#define SNAV_CMD_RETURN_CHECK_WIFI_MODE         "2026"
#define SNAV_CMD_RETURN_MODIFY_WIFI_5G          "2027"
#define SNAV_CMD_RETURN_MODIFY_WIFI_2G          "2028"
#define SNAV_CMD_RETURN_FACE_FOLLOW             "2100"
#define SNAV_CMD_RETURN_FACE_FOLLOW_MODE        "2110"
#define SNAV_CMD_RETURN_BODY_FOLLOW             "2101"
#define SNAV_CMD_RETURN_CHECK_GPS_STATUS        "2102"
#define SNAV_CMD_RETURN_OPEN_GPS                "2103"
#define SNAV_CMD_RETURN_CLOSE_GPS               "2104"
#define SNAV_CMD_RETURN_CHECK_CAM_FREQ          "2105"
#define SNAV_CMD_RETURN_MODIFY_CAM_FREQ         "2106"
#define SNAV_CMD_RETURN_CUSTOMIZED_PLAN         "2107"
#define SNAV_CMD_RETURN_FACE_TAKE_OFF_SWITCH    "2108"
#define SNAV_CMD_RETURN_OPTIC_FLOW_CALIB        "2201"
#define SNAV_CMD_RETURN_FLY_TEST                "2202"
#define SNAV_CMD_RETURN_ROTATION_TEST           "2203"


#define SNAV_TASK_GET_INFO                      "8001"
#define SNAV_TASK_GET_SNAV_PROXY_VERSION        "8002"
#define SNAV_TASK_CONFIRM_LAND                  "8003"
#define SNAV_TASK_SHOW_LAND_CONFIRM             "8004"
#define SNAV_TASK_SNAV_UPDATE                   "8008"
#define SNAV_TASK_LINARO_UPDATE                 "8009"
#define SNAV_TASK_GET_LINARO_VERSION            "8010"
#define SNAV_TASK_GET_SNAV_VERSION              "8011"
#define SNAV_TASK_GET_QCAM_VERSION              "8012"
#define SNAV_TASK_GET_STORAGE                   "8013"
#define SNAV_TASK_GET_SD_STATUS                 "8014"
#define SNAV_TASK_GET_SD_STORAGE                "8015"
#define SNAV_TASK_GET_HW_VERSION                "8016"
#define SNAV_TASK_GET_SN                        "8017"



#define SNAV_TASK_GET_INFO_RETURN               "9001"
#define SNAV_TASK_GET_SNAV_PROXY_VERSION_RETURN "9002"
#define SNAV_TASK_CONFIRM_LAND_RETURN           "9003"
#define SNAV_TASK_SNAV_UPDATE_RETURN            "9008"
#define SNAV_TASK_LINARO_UPDATE_RETURN          "9009"
#define SNAV_TASK_GET_LINARO_VERSION_RETURN     "9010"
#define SNAV_TASK_GET_SNAV_VERSION_RETURN       "9011"
#define SNAV_TASK_GET_QCAM_VERSION_RETURN       "9012"
#define SNAV_TASK_GET_STORAGE_RETURN            "9013"
#define SNAV_TASK_GET_SD_STATUS_RETURN          "9014"
#define SNAV_TASK_GET_SD_STORAGE_RETURN         "9015"
#define SNAV_TASK_GET_HW_VERSION_RETURN         "9016"
#define SNAV_TASK_GET_SN_RETURN                 "9017"



#define SNAV_INFO_OVER_SAFE_HEIGHT              "9101"

#define SNAV_OPEN_GPS_RESULT                    "9103"
#define SNAV_CLOSE_GPS_RESULT                   "9104"
#define SNAV_INFO_MAG_CALIBRATE_RESULT          "9110"
#define SNAV_INFO_HOR_CALIBRATE_RESULT          "9111"
#define SNAV_INFO_OPTIC_FLOW_CALIB_RESULT       "9201"
#define SNAV_RETURN_MISSION_PAUSE               "9202"
#define SNAV_WARNING_SAMPLE_SIZE                "9203"

#define SNAV_WARNING_TAKEOFF_FORBIDDEN          "9210"


// Send to client
#define SNAV_TASK_SHOW_MOTER_ERROR              "3001"
#define SNAV_TASK_SHOW_GPS_RETURN_ERROR_ONE     "3002"
#define SNAV_TASK_SHOW_GPS_RETURN_ERROR_TWO     "3003"

#define SNAV_TASK_SHOW_PLAN_STEP_COMPLETE       "3107"
#define SNAV_TASK_RESET_FACE_TAKEOFF            "3108"

//send to tracker
#define SNAV_TASK_START_TRACKER                 6001
#define SNAV_TASK_STOP_TRACKER                  6101
#define SNAV_TASK_START_GESTURE                 6002
#define SNAV_TASK_STOP_GESTURE                  6102

// For customized plan
#define PLAN_LEFT                               "l"
#define PLAN_RIGHT                              "r"
#define PLAN_FRONT                              "f"
#define PLAN_BACK                               "b"
#define PLAN_UP                                 "u"
#define PLAN_DOWN                               "d"
#define PLAN_CLOCKWISE                          "s"
#define PLAN_ANTI_CLOCKWISE                     "t"
#define PLAN_ZOOM_IN                            "i"
#define PLAN_ZOOM_OUT                           "o"

#define OPEN_GPS                                "open_gps"
#define CLOSE_GPS                               "close_gps"

#define SDCARD_DIR                              "/media/sdcard"
#define SDCARD_MOUNT_PATH                       "/mnt/sdcard"

//cuiyc
//#define FOLLOW_BAOHONG
#define GESTURE_TAKEPHOTO  101
#define GESTURE_BACKANDUP  201
#define GESTURE_LAND       301

#ifdef FOLLOW_BAOHONG
#define FOLLOW_IMG_WIDTH                1280
#define FOLLOW_IMG_HEIGHT               720
//#define FOLLOW_RESERVE_AREA             10
#else
#define FOLLOW_IMG_WIDTH                640
#define FOLLOW_IMG_HEIGHT               360
//#define FOLLOW_RESERVE_AREA             5
#endif

#define BAROMETER_LINEAR_MACRO(x)       (0.00003*(x)*(x) - 0.0708*(x) + 0.8176)

#define VEL_LINEAR_LIMIT_GPS_MACRO(x)   (-0.003*(x)*(x) + 0.0812*(x) + 0.5013)
#define VEL_LINEAR_LIMIT_OPTIC_MACRO(x) (-0.003*(x)*(x) + 0.0812*(x) + 0.5013)

#define CMD_INPUT_LIMIT(x,y)      (x>y?y:(x<(-y)?(-y):x))

typedef unsigned char byte;

// States used to control mission
enum class MissionState
{
  UNKNOWN,
  ON_GROUND,
  STARTING_PROPS,
  TAKEOFF,
  LOITER,
  LANDING,
  IN_MOTION
};

// States of Drone
enum class DroneState
{
  NORMAL,
  MOTOR_ERROR,
  CPU_OVER_HEAT,
  IMU_ERROR,
  BARO_ERROR,
  MAG_ERROR,
  GPS_ERROR,
  SONAR_ERROR,
  OPTIC_FLOW_ERROR,
  EMERGENCY_LANDING_MODE,
  EMERGENCY_KILL_MODE,
  MODE_ERROR,
  UNKNOWN
};

// LED COLOR
enum class LedColor
{
  UNKNOWN,
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  LED_COLOR_BLUE,
  LED_COLOR_WHITE,
  LED_COLOR_YELLOW,
  LED_COLOR_PURPLE,
  LED_COLOR_BLUE_EX
};

struct Position
{
  float x;   // m
  float y;   // m
  float z;   // m
  float yaw;
};

struct PlanPosition
{
  float x;   // m
  float y;   // m
  float z;   // m
  float yaw;
  bool yaw_only;
};


struct GpsPosition
{
  int latitude;   // xxx.xxxxxx
  int longitude;  // xxx.xxxxxx
  int altitude;   // xxx.xxxxxx
  float yaw;
};

struct NavigationPosition
{
  float latitude;
  float longitude;
};

//cuiyc face detect
struct body_info
{
  bool have_face;
  bool have_body;
  int  body_flag;       //1000 upperbody 1001 fullbody
  int  handle_gesture;  //0 nothing / 101 take photo / 201 back and high / 301 landing
  bool newP;
  float distance;       // m
  float velocity;       // m/s
  float hegith_calib;   // m for height need to changed to center
  float angle;          // m
};

// Global variables
static LedColor led_color_status = LedColor::UNKNOWN;
static bool bNeedLedColorCtl = false;

struct body_info cur_body;
static bool face_follow_switch = false;
static bool body_follow_switch = false;
static bool body_follow_start = false;
static bool hand_gesture_switch = true;
static bool face_rotate_switch = false; // false: drone will parallel; true:drone will first rotate to face then close
static bool body_follow_prallel = false; //prallel fly
const float safe_distance = 1.4f;
const float min_angle_offset = 0.087f;// about 5
const float safe_distanceB = 2.5f; //body distance
static bool adjust_people_height = true;
const float face_height_limit = 2.2f;
const float face_vel_limit = 1.0f;   //m/sec
const float body_speed_limit = 2.0f; //m/s  10km/h   2.78*2.5 25km/h
static float init_width,init_height; //body init
static bool follow_reset_yaw = false;

float speed_last =0;
static bool face_detect = false;
static bool face_takeoff_flag = false;

/****************Log control************************/
int log_count = 0;
int current_log_count = 0;
char log_filename[TMP_BUFF_LEN];

int csv_log_count = 0;
char csv_log_filename[TMP_BUFF_LEN];

typedef struct
{
 char  head[4]={'T','R','C','K'};
 unsigned short x = 0;
 unsigned short y = 0;
 unsigned short width = 0;
 unsigned short height = 0;
 unsigned char trackStatus = 0;//0:stopping, 1:tracking, 2:lost
 unsigned char reserved[3];
} S_TRACK_RESULT;

S_TRACK_RESULT track_result;
static int bd_start_counter;
//cuiyc  face detect

static bool send_panorama_flag = false;
static char panorama_buff[DOMAIN_BUFF_SIZE];

static bool send_face_follow_swither_flag = false;
static char face_follow_swither_buff[DOMAIN_BUFF_SIZE];

static bool send_body_follow_swither_flag = false;
static char body_follow_swither_buff[DOMAIN_BUFF_SIZE];

static bool send_take_photo_flag = false;
static char take_photo_buff[DOMAIN_BUFF_SIZE];

static bool send_gesture_swither_flag = false;
static char gesture_swither_buff[DOMAIN_BUFF_SIZE];

static bool send_ota_linaro_flag = false;
static char ota_linaro_path_buff[DOMAIN_BUFF_SIZE];

static bool send_ota_snav_flag = false;
static char ota_snav_path_buff[DOMAIN_BUFF_SIZE];

static bool send_restart_snav = false;
static char ota_restart_snav[DOMAIN_BUFF_SIZE];

static bool send_fpv_flag = false;
static char fpv_switcher_buff[DOMAIN_BUFF_SIZE];

// *****************tool functions start******************************
vector<string> split(const string& s, const string& delim)
{
    vector<string> elems;

    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();

    if (delim_len == 0)
    {
        return elems;
    }

    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }

    return elems;
}

//angle transfrom to radian
float rad(double d)
{
    const float PI = 3.1415926;
    return d*PI/180.0;
}

float CalcDistance(float fLati1, float fLong1, float fLati2, float fLong2)
{
    const float EARTH_RADIUS = 6378137;     //m

    double radLat1 = rad(fLati1);
    double radLat2 = rad(fLati2);
    double lati_diff = radLat1 - radLat2;
    double long_diff = rad(fLong1) - rad(fLong2);
    double s = 2*asin(sqrt(pow(sin(lati_diff/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(long_diff/2),2)));
    s = s*EARTH_RADIUS;
    return s;
}

float CalcAxisDistance(float f1,  float f2)
{
    const float EARTH_RADIUS = 6378137.0;       // r =6378.137km earth radius
    const float PI = 3.1415926;

    double s =(f1-f2)*PI*EARTH_RADIUS/180.0;    // n*pi*r/180
    return s;
}

void Get_ip_address(unsigned long address,char* ip)
{
    sprintf(ip,"%d.%d.%d.%d",
                (int)(address>>24),
                (int)((address&0xFF0000)>>24),
                (int)((address&0xFF00)>>24),
                (int)(address&0xFF));
}

//high byte first
int bytesToInt(byte src[], int offset)
{
    int value;

    value = (int)(((src[offset] & 0xFF)<<24)
                    |((src[offset+1] & 0xFF)<<16)
                    |((src[offset+2] & 0xFF)<<8)
                    |(src[offset+3] & 0xFF));

    return value;
}


#define FALSE  -1
#define TRUE   0

/*******************led control************************************/
int main(int argc, char* argv[])
{
    DEBUG("ThreadLedControl start\n");

    uint8_t led_colors[3] = {0, 0, 0};  //R, G, B
    int32_t timeout = 1000000;          // 1S, timeout for flight controller to take over LED control after API commands stop

    SnavCachedData* snav_data = NULL;
    if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
    {
        DEBUG("\nFailed to get flight data pointer!\n");
        return -1;
    }

    if (argc >= 2)
    {
        while (true)
        {
            if (sn_update_data() != 0)
            {
                printf("sn_update_data failed!\n");
            }
            else
            {
                if (argc >= 4)
                {
                    led_colors[0] = atoi(argv[1]);
                    led_colors[1] = atoi(argv[2]);
                    led_colors[2] = atoi(argv[3]);

                    printf("*********%s,%s,%s*********\n", argv[1],argv[2],argv[3]);
                }
                else if (argc == 3)
                {
                    led_colors[0] = atoi(argv[1]);
                    led_colors[1] = atoi(argv[2]);

                    printf("*********%s,%s*********\n", argv[1],argv[2]);
                }
                else if (argc == 2)
                {
                    led_colors[0] = atoi(argv[1]);

                    printf("*********%s*********\n", argv[1]);
                }

                int ret = sn_set_led_colors(led_colors, sizeof(led_colors), timeout);

                if (ret != 0)
                {
                    printf("sn_set_led_colors returned %d\n",ret);
                }

                usleep(10000);  // 10ms note that commands should only be sent as often as needed (minimize message traffic)
            }
        }
    }
    /*
    else if (argc == 2)
    {
        while (true)
        {
            if (sn_update_data() != 0)
            {
                printf("sn_update_data failed!\n");
            }
            else
            {
                if ((strcmp(argv[1],"r")==0) || (strcmp(argv[1],"R")==0))
                {
                    led_colors[0] = 255;
                    led_colors[1] = 0;
                    led_colors[2] = 0;
                    printf("sn_set_led_colors red!\n");
                }
                else if ((strcmp(argv[1],"g")==0) || (strcmp(argv[1],"G")==0))
                {
                    led_colors[0] = 0;
                    led_colors[1] = 255;
                    led_colors[2] = 0;
                    printf("sn_set_led_colors green!\n");
                }
                else if ((strcmp(argv[1],"b")==0) || (strcmp(argv[1],"B")==0))
                {
                    led_colors[0] = 0;
                    led_colors[1] = 0;
                    led_colors[2] = 255;
                    printf("sn_set_led_colors blue!\n");
                }
                else
                {
                    led_colors[0] = 255;
                    led_colors[1] = 255;
                    led_colors[2] = 255;
                    printf("sn_set_led_colors white!\n");
                }

                int ret = sn_set_led_colors(led_colors, sizeof(led_colors), timeout);

                if (ret != 0)
                {
                    printf("sn_set_led_colors returned %d\n",ret);
                }

                usleep(10000);  // 10ms note that commands should only be sent as often as needed (minimize message traffic)
            }
        }
    }
    */
    else
    {
        printf("plese run as: led_light_test 255 255 0 \n");
    }

}
