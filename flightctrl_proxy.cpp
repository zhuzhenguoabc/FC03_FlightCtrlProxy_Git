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
#include <dirent.h>
#include "sql_array.h"

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

//#define NO_FLY_ZONE
//#define RGB_RED_GREEN_OPPOSITE
#define RGB_TEMP

//#define DYNAMIC_GPS_MODE_HEIGHT

//#define USE_REVISE_HEIGHT
#define HEIGHT_LIMIT
//#define AUTO_REDUCE_HEIGHT
//#define AUTO_FACE_TAKE_OFF
//#define DISABLE_INFRARED_TAKEOFF

#define ALT_CMD_LIMIT
#define OPTIC_FLOW_CMD_LIMIT
#define DISCONN_AUTO_RETURN

#define CSV_LOG_FOR_XIAOYI                          // for XiaoYi Development

#define VIO_OPTIC_FLOW_SWITCH

//#define ZZG_DEBUG_FLAG
//#define ZZG_CMD_DEBUG_FLAG
//#define ZZG_TIMESTAMP_DEBUG_FLAG
//#define ZZG_SONAR_DEBUG_FLAG
//#define ZZG_FINAL_CMD_FLAG

//#define VEL_LIMIT_FLAG
//#define SPEED_LIMIT_FLAG
//#define LOW_HEIGHT_LIMIT_VEL


/********************Dynamic variables**************************/
/* low battery set */
const float low_battery_led_warning = 7.1;  //6.9f;        //6.75f
const float force_landing_battery_outdoor = 7.0;    //6.8f;   //6.7f;
const float force_landing_battery_indoor = 6.95;    //6.75f;   //6.55f;

const float close_fan_battery = 6.9;    //6.75;

/*const*/ float distance_disconn_auto_return = 10;

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
const int log_file_size_total_limit = 500;  //1024;          // M
const int log_file_size_single_limit = 100; //200;           // M

/* hover vel and brake cmd limit */
const float hover_vel_limit = 0.2;                  // over the vel limit will auto reverse the drone
const float hover_brake_limit = 0.2;                // cmd limit of the auto reverse

const float control_msg_max_diff = 0.199;   //0.099;           // s

#define PATH_FLAG                   "/"

#define SNAV_LOG_PATH               "/var/log/snav/flight"
#define FC_LOG_PATH                 "/home/linaro"
#define FC_LOG_CT_NAME              "flightctrl_proxy_count"
#define FC_LOG_CT_MAX               8000

#define SNAV_OEM_PKG_NAME           "snav-oem"
#define SNAV_DEV_PKG_NAME           "snav-dev"
#define SNAV_PKG_NAME               "snav"
#define FC_PKG_NAME                 "flightctrl-proxy"
#define CAMERA_PKG_NAME             "mm-video"

#define WIFI_FILE_NAME              "/etc/hostapd.conf"
#define CAM_CFG_FILE_NAME           "/etc/camera.cfg"
#define SNAV_UPDATE_FILE_NAME       "/tmp/update-snav.zip"
#define LINARO_UPDATE_FILE_NAME     "/tmp/update-linaro.zip"

#define SNAV_CFG_FILE_NAME          "/usr/share/data/adsp/200qc_runtime_params.xml"
#define EAGLE_CFG_FILE_NAME         "/usr/share/data/adsp/eagle_p2.xml"

#define LOG_FOR_XIAOYI_PATH         "/home/linaro/log_yi"

/* ***********************************************************************************/
#define SERVER_UDP_PORT                         14559
#define QCAM_DOMAIN_PORT                        16889
#define CAM_SUPER_PORT                          16887
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


#define SNAV_OPEN_GPS_RESULT                    "9103"
#define SNAV_CLOSE_GPS_RESULT                   "9104"
#define SNAV_INFO_MAG_CALIBRATE_RESULT          "9110"
#define SNAV_INFO_HOR_CALIBRATE_RESULT          "9111"
#define SNAV_INFO_OPTIC_FLOW_CALIB_RESULT       "9201"


/******************hidden in doc for customer********************/
//recv GESTURE swtich from app
#define SNAV_CMD_GESTURE                        "1501"
#define SNAV_CMD_RETURN_GESTURE                 "2501"

#define SNAV_INFO_OVER_SAFE_HEIGHT              "9101"
#define SNAV_RETURN_MISSION_PAUSE               "9202"
#define SNAV_WARNING_SAMPLE_SIZE                "9203"

#define SNAV_WARNING_TAKEOFF_FORBIDDEN          "9210"
#define SNAV_WARNING_CIRCLE_OVER_HEIGHT         "9211"
#define SNAV_WARNING_NO_FLY_ZONE                "9212"

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

static bool send_csv_pic_flag = false;
static char csv_pic_buff[DOMAIN_BUFF_SIZE];


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


// For Infrared start
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#define UART_DEVICE     "/dev/ttyHSL2"

#define FALSE  -1
#define TRUE   0

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[]  = { 115200,  38400,  19200,  9600,  4800,  2400,  1200,  300};
int fd;

static unsigned char  read_buf[128];
static unsigned char  cont_buf[128];
static float    ir_distance = 0;
static float    ir_safe_distance = 1.5;

#define IR_VEL_LIMIT        0.2
#define IR_BRAKE_CMD        0.2     // 0.3

#define IR_VEL_FLAG         0.8     // m/s
#define IR_FORBID_HIGH_VEL  2;      //1.5
#define IR_FORBID_LOW_VEL   1.5;    //1.0


// For fan control
#define SYSFS_GPIO_EXPORT           "/sys/class/gpio/export"
#define SYSFS_GPIO_SFILE            "/sys/class/gpio/gpio115"
#define SYSFS_GPIO_RST_PIN_VAL      "115"
#define SYSFS_GPIO_RST_DIR          "/sys/class/gpio/gpio115/direction"
#define SYSFS_GPIO_RST_DIR_VAL      "OUT"
#define SYSFS_GPIO_RST_VAL          "/sys/class/gpio/gpio115/value"
#define SYSFS_GPIO_RST_VAL_H        "1"
#define SYSFS_GPIO_RST_VAL_L        "0"

int is_file_exist(const char *file_path)
{
    if (file_path == NULL)
    {
        return 0;
    }

    if (access(file_path, F_OK) == 0)
    {
        return 1;
    }

    return 0;
}

int switchFan(bool bFlag)
{
    /*
    int fd;

    if (!is_file_exist(SYSFS_GPIO_SFILE))
    {
        fd = open(SYSFS_GPIO_EXPORT, O_WRONLY);

        if(fd == -1)
        {
            DEBUG("ERR: Radio hard reset pin open error.\n");
            return -1;
        }

        write(fd, SYSFS_GPIO_RST_PIN_VAL ,sizeof(SYSFS_GPIO_RST_PIN_VAL));
        write(fd, SYSFS_GPIO_RST_DIR_VAL, sizeof(SYSFS_GPIO_RST_DIR_VAL));
        close(fd);
        DEBUG(" Determines that the file does not exist, creates and sets the output\n");
    }

    if (bFlag == true)
    {
        fd = open(SYSFS_GPIO_EXPORT, O_WRONLY);
        if(fd == -1)
        {
            DEBUG("ERR: Radio hard reset pin open error.\n");
            return -1;
        }
        write(fd, SYSFS_GPIO_RST_VAL_H, sizeof(SYSFS_GPIO_RST_VAL_H));
        close(fd);
        DEBUG(" pull up \n");
    }
    else
    {
        fd = open(SYSFS_GPIO_EXPORT, O_WRONLY);
        if(fd == -1)
        {
            DEBUG("ERR: Radio hard reset pin open error.\n");
            return -1;
        }
        write(fd, SYSFS_GPIO_RST_VAL_L, sizeof(SYSFS_GPIO_RST_VAL_L));
        DEBUG(" pull down  \n");
    }
    */

    if (!is_file_exist(SYSFS_GPIO_SFILE))
    {
        system("echo 115 > sys/class/gpio/export");
        system("echo out  > /sys/class/gpio/gpio115/direction");
    }
    else
    {
        if (bFlag == true)
        {
            system("echo 1 > sys/class/gpio/gpio115/value");
            system("sync");
        }
        else
        {
            system("echo 0 > sys/class/gpio/gpio115/value");
            system("sync");
        }
    }

    return 0;
}

#ifdef NO_FLY_ZONE
bool isInNoFlyZone(float latitude, float longitude)
{
    DEBUG("Input: lat, lng: [%f, %f]\n", latitude, longitude);

    float distance_to_nfz = 0;
    float diff_between_lat = 0;

    int array_ct = sizeof(NFZPos)/sizeof(string);
    int unit_ct = sizeof(NFZPos[0])/sizeof(string);
    int item_ct = array_ct/unit_ct;
    printf("array_ct=%d, uint_ct=%d, item_ct=%d\n", array_ct, unit_ct, item_ct);

    for (int i = 0; i < item_ct; i++)
    {
        float lat = stof(NFZPos[i][3], 0);
        float lng = stof(NFZPos[i][4], 0);
        float radius = stof(NFZPos[i][5], 0);
        float lat_diff = fabs(latitude - lat);
        float lng_diff = fabs(longitude - lng);

        printf("Db: lat, lng, radius: [%f, %f, %f]\n", lat, lng, radius);
        printf("lat_diff, lng_diff: [%f, %f]\n", lat_diff, lng_diff);

        if ((lat_diff > 1) || (lng_diff > 1))
        {
            printf("Too far away! Ignore!");
            continue;
        }
        else
        {
            distance_to_nfz = CalcDistance(lat, lng, lat, lng);

            printf("distance_to_nfx=[%f]\n", distance_to_nfz);

            if (distance_to_nfz <= radius + no_fly_zone_warning_distance)
            {
                return true;
            }
        }
    }

    return false;
}

#endif

/*set serial port transport speed*/
void set_speed(int fd, int speed)
{
    int   i;
    int   status;
    int   size;
    struct termios   Opt;
    tcgetattr(fd, &Opt);

    size = sizeof(speed_arr)/sizeof(int);

    for (i = 0; i < size; i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);

            if (status != 0)
            {
                DEBUG("tcsetattr fd1");
                return;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
}

/*set serial port databits(7/8), stopbits(1/2), paritybit(N/E/O/S)*/
int set_Parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr(fd,&options) != 0)
    {
        DEBUG("SetupSerial 1");
        return(FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n"); return (FALSE);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 'S':
        case 's':  /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return (FALSE);
        }

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
           break;
        default:
             fprintf(stderr,"Unsupported stop bits\n");
             return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    return (TRUE);
}

void  init_uart()
{
    DEBUG("Start  init UART ...\n");
    fd = open(UART_DEVICE, O_RDWR|O_NONBLOCK);

    if (fd < 0) {
        perror(UART_DEVICE);
        DEBUG("perror(UART_DEVICE)...\n");
    }

    DEBUG("Open...\n");
    set_speed(fd,115200);
    if (set_Parity(fd,8,1,'N') == FALSE)  {
        DEBUG("Set Parity Error\n");
    }
}

void debug_uart()
{
    DEBUG("%s ****\n",__func__);
    system("mount  -t debugfs none /sys/kernel/debug");
    //system("echo 1 >  /sys/kernel/debug/msm_serial_hs/loopback.0");
}
// For Infrared end

// *****************tool functions end******************************


//cyc people detect begin
void* ThreadGetVideoFaceFollowParam(void*)
{
    //video follow socket begin
    const char* UNIX_DOMAIN ="/tmp/vedio.domain";
    static int recv_php_buf[16];
    static int recv_php_num=0;

    const float window_degree = 101.9747f; //117  degree   101;  83 degree 72
    const float camera_rad =window_degree*M_PI/180; //camera 83.5 degree
    const float camera_vert_rad =(9.0f/16)*window_degree*M_PI/180; //camera 83.5 degree
    const float face_winth =0.150; //camera 83.5 degree
    const float angle_camera =0;//25*M_PI/180;//angle of inclination for y , 0 or 25
    int listen_fd;
    int ret=0;
    int i;

    struct sockaddr_un clt_addr;
    struct sockaddr_un srv_addr;
    socklen_t len =sizeof(clt_addr);

    DEBUG("ThreadGetVideoFaceFollowParam start\n");

    while(1)
    {
        //listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
        listen_fd=socket(AF_UNIX,SOCK_DGRAM,0);
        if(listen_fd<0)
        {
            DEBUG("cannot create listening socket");
            continue;
        }
        else
        {
            while(1)
            {
                srv_addr.sun_family=AF_UNIX;
                strncpy(srv_addr.sun_path,UNIX_DOMAIN,sizeof(srv_addr.sun_path)-1);
                unlink(UNIX_DOMAIN);
                ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));

                if(ret==-1)
                {
                    DEBUG("cannot bind server socket");
                    //close(listen_fd);
                    unlink(UNIX_DOMAIN);
                    break;
                }

                while(true)
                {
                    recv_php_num = recvfrom(listen_fd, recv_php_buf, sizeof(recv_php_buf),
                                            0, (struct sockaddr *)&clt_addr, &len);
                    DEBUG("\n=====face info=====\n");
                    //0,flag 1,center-x 2,center-y,3,face_winth,4,_face_height,5,video_winth,6,video_height
                    for(i=0;i<16;i++)
                    {
                        DEBUG("%d ",recv_php_buf[i]);
                    }
                    DEBUG("\n");

                    if(recv_php_buf[0] == 1)
                    {
                        // normal  face 18cm && camera 83.5 degree
                        float distance,angle,height_cal,vert_offset_angle;
                        cur_body.have_face=true;

                        if(recv_php_buf[6] == 720) //720p
                        {
                            distance = face_winth/tan((recv_php_buf[3]*camera_rad/1280));
                        }
                        else if(recv_php_buf[6] == 1080) //1080p
                        {
                            distance = face_winth/tan((recv_php_buf[3]*camera_rad/1920));
                        }
                        else if(recv_php_buf[5] != 0)
                        {
                            distance = face_winth/tan((recv_php_buf[3]*camera_rad/recv_php_buf[5]));
                        }
                        //face y center ,height need add to calibration to 1/3 height for image
                        vert_offset_angle = angle_camera -
                            camera_vert_rad*(recv_php_buf[6]/2 -recv_php_buf[2])/recv_php_buf[6];

                        height_cal =distance*tan(angle_camera) - distance * tan(vert_offset_angle);

                        if(!adjust_people_height)height_cal = 0;

                        //printf("tan(angle_camera):%f ,tan offset vertical:%f \n",tan(angle_camera),
                        //  tan(vert_offset_angle));

                        printf("angle_camera %f,vert_offset_angle:%f \n",angle_camera,vert_offset_angle);

                        //dgree for window 72.7768
                        angle = window_degree*((recv_php_buf[5]/2 - recv_php_buf[1])*1.0)/recv_php_buf[5];
                        DEBUG("face distance :%f angle:%f \n",distance,angle);

                        if(face_takeoff_flag && !face_detect)
                        {
                            DEBUG("face_detect :%d center:%d ,%d \n",face_detect,recv_php_buf[1],recv_php_buf[2]);
                            if(face_detect)continue;
                            if(recv_php_buf[1] > recv_php_buf[5]*0.25 &&
                               recv_php_buf[1] < recv_php_buf[5]*0.75 &&
                               recv_php_buf[2] > recv_php_buf[6]*0.25 &&
                               recv_php_buf[2] < recv_php_buf[6]*0.75 )
                               {
                                   face_detect = true;
                                   //face_takeoff_flag = false;
                               }
                        }
                        else
                        {
                            if((cur_body.angle != angle) || (cur_body.distance != distance))
                            {
                                cur_body.distance = distance;
                                cur_body.angle  = angle;
                                cur_body.newP = true;
                                cur_body.hegith_calib = height_cal;
                            }
                            else
                            {
                                cur_body.newP = false;
                            }
                        }
                    }
                    else
                    {
                        cur_body.have_face=false;
                    }
                }
            }
        }
    }
    //video follow socket end
}

void* ThreadGetVideoBodyFollowParam(void*)
{
    //video follow socket begin
    const float window_degree = 101.9747f;
    struct timeval tv;

    static int recv_php_num=0;
    static float velocity_forward =0; //ralative speed to drone

    socklen_t len;
    int init_cx,init_cy;
    long long lasttime;
    int last_width;

    struct sockaddr_in track_server ,track_client;
    char followdata[128];
    bool have_apk_client = false;
    int send_apk_result ;
    int socket_track;

    //get track info from BNtrack or kcf tracker
    bzero(&track_server, sizeof(track_server));
    track_server.sin_family = AF_INET;
    track_server.sin_addr.s_addr = inet_addr("127.0.0.1");
    track_server.sin_port = htons(TRACKING_SERVER_PORT);
    socket_track = socket(AF_INET, SOCK_DGRAM, 0);
    len=sizeof(track_server);

    if(bind(socket_track, (const sockaddr*)&track_server, sizeof(track_server)) == -1)
    {
        DEBUG("socket_track bind error");
    }

    //send follow info to apk
    struct sockaddr_in server_apk_address,bh_apk_client;
    char apk_confirm[16];
    int server_apk_sockfd;
    int server_apk_len;

    server_apk_address.sin_family=AF_INET;
    server_apk_address.sin_addr.s_addr=inet_addr("192.168.1.1");
    server_apk_address.sin_port=htons(TRACKING_FORAPP_PORT);
    server_apk_len=sizeof(server_apk_address);
    server_apk_sockfd=socket(AF_INET,SOCK_DGRAM,0);

    int bind_result = bind(server_apk_sockfd,(struct sockaddr*)&server_apk_address,server_apk_len);
    if(bind_result==-1)
    {
        DEBUG("cannot bind server_apk_sockfd socket");
    }

    //300MS avoid of udp missing data
    setsockopt(server_apk_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_follow, sizeof(struct timeval));
    setsockopt(server_apk_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_udp, sizeof(struct timeval));

    while(true)
    {
        while(!have_apk_client && !face_takeoff_flag) //take off on palm not need apk client
        {
            recv_php_num = recvfrom(server_apk_sockfd, apk_confirm, sizeof(apk_confirm),
                0, (struct sockaddr *)&bh_apk_client, (socklen_t *)&server_apk_len);

            if((strncmp(apk_confirm,"APK",3) == 0))
            {
                have_apk_client = true;
                DEBUG("\n get APK client received data from %s, %d:\n",
                    inet_ntoa(bh_apk_client.sin_addr), ntohs(bh_apk_client.sin_port));
                break;
            }
        }

        while(true)
        {
            recv_php_num = recvfrom(socket_track, (S_TRACK_RESULT *)&track_result, sizeof(track_result),
                0, (struct sockaddr *)&track_client, &len);
            DEBUG("\n=====follow info===== %d\n" ,recv_php_num);

            if((strncmp(track_result.head,"TRCK",4) == 0)
                && body_follow_switch)
            {
                gettimeofday(&tv,NULL);
                long long temp = tv.tv_sec*1e3 + tv.tv_usec*1e-3; //ms
                printf("follow cost time :%lld ms\n",temp - lasttime);
                lasttime = temp;

                if(track_result.trackStatus == 1)
                {
                    int center_x, center_y;

                    if(init_width <= 1.0f || init_height <= 1.0f)
                    {
                        if(body_follow_start)
                        {
                            init_width = track_result.width;
                            init_height = track_result.height;
                            init_cx = track_result.x + track_result.width/2;
                            init_cy = track_result.y + track_result.height/2;
                            last_width = track_result.width;

                            DEBUG("follow init_width :%f,init_height:%f init cx:%d cy:%d\n",
                                init_width,init_height,init_cx,init_cy);

                            cur_body.angle= (FOLLOW_IMG_WIDTH*0.5 -init_cx)*window_degree/FOLLOW_IMG_WIDTH;
                        }
                    }
                    else
                    {
                        cur_body.have_body=true;
                        center_x= track_result.x + track_result.width/2;
                        center_y= track_result.y + track_result.height/2;

                        DEBUG("follow track_result x:%d y:%d,width:%d,height:%d\n",
                            track_result.x,track_result.y,track_result.width,track_result.height);
                        DEBUG("follow track_result center_x:%d center_y:%d\n",center_x,center_y);

                        if(track_result.width < init_width && track_result.width != 0)
                            velocity_forward = (init_width/track_result.width -1.0)*10;
                        else
                            velocity_forward = 0;

                        if(velocity_forward > body_speed_limit) velocity_forward = body_speed_limit;
                        if(velocity_forward < -1*body_speed_limit) velocity_forward = -1*body_speed_limit;

                        if(fabs(velocity_forward) <0.05 )
                            velocity_forward = 0;

                        if(!body_follow_start || init_width < 1)
                            cur_body.velocity = 0;
                        else if(velocity_forward > 0.05)
                            cur_body.velocity = 0.8f*cur_body.velocity+0.2f*velocity_forward;
                        else
                            cur_body.velocity = 0;

                        cur_body.angle= (FOLLOW_IMG_WIDTH*0.5 -center_x)*window_degree/FOLLOW_IMG_WIDTH;
                    }
                }
                else if(!body_follow_switch)
                {
                    init_width = 0;
                    init_height = 0;
                    cur_body.angle = 0;
                    cur_body.velocity = 0;
                    cur_body.have_body=false;
                }
                else
                {
                    DEBUG("track_result missing \n");
                    cur_body.have_body=false;
                    cur_body.velocity= 0;
                    cur_body.angle = 0;
                }

                DEBUG("follow velocity:%f angle:%f \n",cur_body.velocity,cur_body.angle);
                if(have_apk_client)
                {
                    int j=0;
                    memset(followdata,0,sizeof(followdata));

                    if(track_result.trackStatus == 1)
                    {
                        j  = sprintf( followdata,  "%d,", 5102 );

#ifdef FOLLOW_BAOHONG
                        j += sprintf( followdata + j, "%d,", (int)(track_result.x*0.5f));
                        j += sprintf( followdata + j, "%d,", (int)(track_result.y*0.5f));
                        j += sprintf( followdata + j, "%d,", (int)(track_result.width*0.5f));
                        j += sprintf( followdata + j, "%d",  (int)(track_result.height*0.5f));
#else
                        j += sprintf( followdata + j, "%d,", (int)(track_result.x));
                        j += sprintf( followdata + j, "%d,", (int)(track_result.y));
                        j += sprintf( followdata + j, "%d,", (int)(track_result.width));
                        j += sprintf( followdata + j, "%d",  (int)(track_result.height));

                        if(!body_follow_start)
                        j += sprintf( followdata + j, ",%s",  "go");
#endif
                    }
                    else if(track_result.trackStatus == 2)
                    {
                        j  = sprintf( followdata,  "%d,", 5102 );

                        j += sprintf( followdata + j, "%d,", 0 );
                        j += sprintf( followdata + j, "%d,", 0 );
                        j += sprintf( followdata + j, "%d,", 0 );
                        j += sprintf( followdata + j, "%d", 0 );
                    }
                    else if(track_result.trackStatus == 0)
                    {
                        //lost too long need user reselect
                        j  = sprintf( followdata,  "%d,", 5103 );
                        j += sprintf( followdata + j, "%d,", 0 );
                        j += sprintf( followdata + j, "%d,", 0 );
                        j += sprintf( followdata + j, "%d,", 0 );
                        j += sprintf( followdata + j, "%d", 0 );
                    }

                    send_apk_result = sendto(server_apk_sockfd, followdata, strlen(followdata), 0,
                       (struct sockaddr*)&bh_apk_client, sizeof(bh_apk_client));
                    printf("follow info: %s to app result:%d ,port:%d\n", followdata,send_apk_result,ntohs(bh_apk_client.sin_port));
                }
            }
            else if(strncmp(track_result.head,"TKPH",4) == 0)
            {
                printf("handgesture taking photo\n");
                send_take_photo_flag= true;
                memset(take_photo_buff, 0, DOMAIN_BUFF_SIZE);
                strcpy(take_photo_buff, "cont1");
                cur_body.handle_gesture = GESTURE_TAKEPHOTO;
                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_PURPLE;
            }
            else if(strncmp(track_result.head,"LAND",4) == 0)
            {
                printf("handgesture landing\n");
                cur_body.handle_gesture = GESTURE_LAND;
                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_PURPLE;
            }
            else if(strncmp(track_result.head,"BACK",4) == 0)
            {
                printf("handgesture back \n");
                cur_body.handle_gesture = GESTURE_BACKANDUP;
                cur_body.have_body=true;
                cur_body.velocity= -0.95;
                cur_body.angle = 0;
                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_PURPLE;

                sleep(1); // fly 1s
                cur_body.have_body = false;
                cur_body.velocity = 0;
                cur_body.angle = 0;
                cur_body.handle_gesture = 0;
            }
            else //if we have face ,set no body
            {
                printf("something unknow \n");
                cur_body.have_body=false;
                cur_body.velocity= 0;
                cur_body.angle = 0;
            }
        }
    }
    //video follow socket end
}
//cyc people detect end


/*******************interact with qcamvid************************************/
void* ThreadInteractWithQcamvid(void*)
{
    DEBUG("ThreadInteractWithQcamvid start\n");

    int socket_cli;

    struct sockaddr_in address;
    bzero(&address, sizeof(address));
    address.sin_family      = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port        = htons(QCAM_DOMAIN_PORT);

    struct sockaddr_in address_cam_super;
    bzero(&address_cam_super, sizeof(address_cam_super));
    address_cam_super.sin_family      = AF_INET;
    address_cam_super.sin_addr.s_addr = inet_addr("127.0.0.1");
    address_cam_super.sin_port        = htons(CAM_SUPER_PORT);

    socket_cli = socket(AF_INET, SOCK_DGRAM, 0);
    int send_num = 0;

    while (true)
    {
        /*
        if (send_panorama_flag)
        {
            send_num = sendto(socket_cli, panorama_buff, strlen(panorama_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("panorama_buff=%s send_num=%d\n", panorama_buff,send_num);
            send_panorama_flag = false;
        }
        */

        if (send_fpv_flag)
        {
            if (strncmp(fpv_switcher_buff, "exit", 4) == 0)
            {
                send_num = sendto(socket_cli, fpv_switcher_buff, strlen(fpv_switcher_buff), 0, (struct sockaddr*)&address, sizeof(address));
            }
            else
            {
                send_num = sendto(socket_cli, fpv_switcher_buff, strlen(fpv_switcher_buff), 0, (struct sockaddr*)&address_cam_super, sizeof(address_cam_super));
            }
            DEBUG("fpv_switcher_buff=%s, send_num=%d\n", fpv_switcher_buff, send_num);
            send_fpv_flag = false;
        }

        if (send_csv_pic_flag)
        {
            send_num = sendto(socket_cli, csv_pic_buff, strlen(csv_pic_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("csv_pic_buff=%s, send_num=%d\n", csv_pic_buff, send_num);
            send_csv_pic_flag= false;
        }

        if (send_face_follow_swither_flag)
        {
            send_num = sendto(socket_cli, face_follow_swither_buff, strlen(face_follow_swither_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("face_follow_swither_buff=%s, send_num=%d\n", face_follow_swither_buff, send_num);
            send_face_follow_swither_flag = false;
        }

        if (send_body_follow_swither_flag)
        {
            send_num = sendto(socket_cli, body_follow_swither_buff, strlen(body_follow_swither_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("body_follow_swither_buff=%s, send_num=%d\n", body_follow_swither_buff, send_num);
            send_body_follow_swither_flag = false;
        }

        if (send_gesture_swither_flag)
        {
            send_num = sendto(socket_cli, gesture_swither_buff, strlen(gesture_swither_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("gesture_swither_buff=%s, send_num=%d\n", gesture_swither_buff, send_num);
            send_gesture_swither_flag = false;
        }

        if (send_take_photo_flag)
        {
            usleep(500000); // purple 500ms
            printf("take_photo led confirm \n");

            bNeedLedColorCtl = true;
            led_color_status = LedColor::LED_COLOR_YELLOW;
            printf("take_photo led LED_COLOR_YELLOW ");

            usleep(3000000); // yellow spark 3s

            send_num = sendto(socket_cli, take_photo_buff, strlen(take_photo_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("take_photo_buff=%s, send_num=%d\n", take_photo_buff, send_num);
            send_take_photo_flag = false;
            cur_body.handle_gesture = 0;

            usleep(500000); // wihte ,finish take photo
            led_color_status = LedColor::LED_COLOR_WHITE;
        }

        usleep(20000);      //20ms
    }
}

void* ThreadInteractWithOtaAndLimitLog(void*)
{
    DEBUG("ThreadInteractWithOtaAndLimitLog start\n");

    int socket_cli;

    struct sockaddr_in address;
    bzero(&address, sizeof(address));
    address.sin_family      = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port        = htons(OTA_UDP_PORT);

    socket_cli = socket(AF_INET, SOCK_DGRAM, 0);
    int send_num = 0;

    // For limit log
    char get_log_file_total_cmd[TMP_BUFF_LEN] = "du -m  /home/linaro/log_flightctrl* | awk '{sum+=$1}; END{print sum}'";
    char log_file_total[TMP_BUFF_LEN];
    int log_file_total_size = 0;

    char get_current_log_cmd[TMP_BUFF_LEN];
    char current_log[TMP_BUFF_LEN];
    int current_log_size = 0;

    char get_log_cmd[TMP_BUFF_LEN];
    char get_log[TMP_BUFF_LEN];
    int log_size = 0;

    while (true)
    {
#ifdef CSV_LOG_FOR_XIAOYI
        // *****************Limit csv logs size***********************
        memset(get_log_cmd, 0, sizeof(get_log_cmd));
        sprintf(get_log_cmd, "du -m /home/linaro/log_yi/csv_log* | awk '{sum+=$1}; END{print sum}'");
        FILE *fp_log = popen(get_log_cmd, "r");

        if (fp_log != NULL)
        {
            fgets(get_log, sizeof(get_log), fp_log);
        }
        pclose(fp_log);

        if ((strlen(get_log) >= 1) && (get_log[strlen(get_log)-1] == '\n'))
        {
            get_log[strlen(get_log)-1] = '\0';
        }

        log_size = atoi(get_log);

        // du -m : 1.1M will output 2
        if (log_size > log_file_size_total_limit)
        {
            // Delete the log one by one
            system("find /home/linaro/log_yi/ -type f -name 'csv_log*'|xargs -r ls -l|head -n 1|awk '{print $9}'| xargs rm -rf");
        }
#endif

        // *****************Limit total logs size***********************
        FILE *fp = popen(get_log_file_total_cmd, "r");

        if (fp != NULL)
        {
            fgets(log_file_total, sizeof(log_file_total), fp);
        }
        pclose(fp);

        if ((strlen(log_file_total) >= 1) && (log_file_total[strlen(log_file_total)-1] == '\n'))
        {
            log_file_total[strlen(log_file_total)-1] = '\0';
        }

        log_file_total_size = atoi(log_file_total);

        // du -m : 1.1M will output 2
        if (log_file_total_size > log_file_size_total_limit)
        {
            // Delete the log one by one
            system("find /home/linaro/ -type f -name 'log_flightctrl*'|xargs -r ls -l|head -n 1|awk '{print $9}'| xargs rm -rf");
        }

        // *****************Limit current log size***********************
        memset(get_current_log_cmd, 0, sizeof(get_current_log_cmd));
        sprintf(get_current_log_cmd, "du -m  %s | awk '{sum+=$1}; END{print sum}'", log_filename);

        FILE *fp_current = popen(get_current_log_cmd, "r");
        if (fp_current != NULL)
        {
            fgets(current_log, sizeof(current_log), fp_current);
        }
        pclose(fp_current);

        if ((strlen(current_log) >= 1) && (current_log[strlen(current_log)-1] == '\n'))
        {
           current_log[strlen(current_log)-1] = '\0';
        }

        current_log_size = atoi(current_log);

        if (current_log_size > log_file_size_single_limit)
        {
            current_log_count++;
            memset(log_filename, 0, TMP_BUFF_LEN);
            sprintf(log_filename, "/home/linaro/log_flightctrl_%04d_%02d", log_count, current_log_count);

            freopen(log_filename, "a", stdout);
            setbuf(stdout, NULL);       //needn't cache and fflush, output immediately
            freopen(log_filename, "a", stderr);
            setbuf(stderr, NULL);       //needn't cache and fflush, output immediately
        }


        if (send_ota_linaro_flag)
        {
            send_num = sendto(socket_cli, ota_linaro_path_buff, strlen(ota_linaro_path_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("ota_linaro_path_buff=%s send_num=%d\n", ota_linaro_path_buff, send_num);
            send_ota_linaro_flag = false;
        }

        if (send_ota_snav_flag)
        {
            send_num = sendto(socket_cli, ota_snav_path_buff, strlen(ota_snav_path_buff), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("ota_snav_path_buff=%s send_num=%d\n", ota_snav_path_buff, send_num);
            send_ota_snav_flag = false;
        }

        if (send_restart_snav)
        {
            send_num = sendto(socket_cli, ota_restart_snav, strlen(ota_restart_snav), 0, (struct sockaddr*)&address, sizeof(address));
            DEBUG("ota_restart_snav=%s send_num=%d\n", ota_restart_snav, send_num);
            send_restart_snav = false;
        }

        usleep(200000);     //200ms
    }
}




/*******************led control************************************/
void* ThreadLedControl(void*)
{
    DEBUG("ThreadLedControl start\n");

    uint8_t led_colors[3] = {0, 0, 0};  //R, G, B
    int32_t timeout = 1000000;          // 1S, timeout for flight controller to take over LED control after API commands stop

    int continue_red_count = 0;
    int continue_green_count = 0;
    int continue_blue_count = 0;
    int continue_white_count = 0;
    int continue_yellow_count = 0;
    int continue_purple_count = 0;
    int continue_blue_ex_count = 0;

    while (true)
    {
        switch (led_color_status)
        {
            case LedColor::LED_COLOR_RED:
            {
                continue_red_count++;
                continue_green_count = 0;
                continue_blue_count = 0;
                continue_white_count = 0;
                continue_yellow_count = 0;
                continue_purple_count = 0;
                continue_blue_ex_count = 0;

                led_colors[0] = 255;
                led_colors[1] = 0;
                led_colors[2] = 0;
/*
#ifdef RGB_RED_GREEN_OPPOSITE
                led_colors[0] = 0;
                led_colors[1] = 255;
#else
                led_colors[0] = 255;
                led_colors[1] = 0;
#endif
                led_colors[2] = 0;
*/

                break;
            }
            case LedColor::LED_COLOR_GREEN:
            {
                continue_red_count = 0;
                continue_green_count++;
                continue_blue_count = 0;
                continue_white_count = 0;
                continue_yellow_count = 0;
                continue_purple_count = 0;
                continue_blue_ex_count = 0;

#ifdef RGB_TEMP
                led_colors[0] = 0;
                led_colors[1] = 0;
                led_colors[2] = 255;
#else
                led_colors[0] = 0;
                led_colors[1] = 255;
                led_colors[2] = 0;
#endif

/*
#ifdef RGB_RED_GREEN_OPPOSITE
                led_colors[0] = 255;
                led_colors[1] = 0;
#else
                led_colors[0] = 0;
                led_colors[1] = 255;
#endif
                led_colors[2] = 0;
*/

                break;
            }
            case LedColor::LED_COLOR_BLUE:
            {
                continue_red_count = 0;
                continue_green_count = 0;
                continue_blue_count++;
                continue_white_count = 0;
                continue_yellow_count = 0;
                continue_purple_count = 0;
                continue_blue_ex_count = 0;

#ifdef RGB_TEMP
                led_colors[0] = 0;
                led_colors[1] = 255;
                led_colors[2] = 0;
#else
                led_colors[0] = 0;
                led_colors[1] = 0;
                led_colors[2] = 255;
#endif
                break;
            }
            case LedColor::LED_COLOR_WHITE:
            {
                continue_red_count = 0;
                continue_green_count = 0;
                continue_blue_count = 0;
                continue_white_count++;
                continue_yellow_count = 0;
                continue_purple_count = 0;
                continue_blue_ex_count = 0;

                led_colors[0] = 255;
                led_colors[1] = 255;
                led_colors[2] = 255;

                break;
            }
            case LedColor::LED_COLOR_YELLOW:
            {
                continue_red_count = 0;
                continue_green_count = 0;
                continue_blue_count = 0;
                continue_white_count = 0;
                continue_yellow_count++;
                continue_purple_count = 0;
                continue_blue_ex_count = 0;

#ifdef RGB_TEMP
                led_colors[0] = 255;
                led_colors[1] = 0;
                led_colors[2] = 255;
#else
                led_colors[0] = 255;
                led_colors[1] = 255;
                led_colors[2] = 0;
#endif
                break;
            }
            case LedColor::LED_COLOR_PURPLE:
            {
                continue_red_count = 0;
                continue_green_count = 0;
                continue_blue_count = 0;
                continue_white_count = 0;
                continue_yellow_count = 0;
                continue_purple_count++;
                continue_blue_ex_count = 0;

#ifdef RGB_TEMP
                led_colors[0] = 255;
                led_colors[1] = 255;
                led_colors[2] = 0;
#else
                led_colors[0] = 0;
                led_colors[1] = 255;
                led_colors[2] = 255;
#endif
                break;
            }
            case LedColor::LED_COLOR_BLUE_EX:       // will not twinkle different from blue twinkle
            {
                continue_red_count = 0;
                continue_green_count = 0;
                continue_blue_count = 0;
                continue_white_count = 0;
                continue_yellow_count = 0;
                continue_purple_count = 0;
                continue_blue_ex_count ++;

#ifdef RGB_TEMP
                led_colors[0] = 0;
                led_colors[1] = 255;
                led_colors[2] = 0;
#else
                led_colors[0] = 0;
                led_colors[1] = 0;
                led_colors[2] = 255;
#endif
                break;
            }
            default:
            {
                continue_red_count = 0;
                continue_green_count = 0;
                continue_blue_count = 0;
                continue_white_count = 0;
                continue_yellow_count = 0;
                continue_purple_count = 0;
                continue_blue_ex_count = 0;

                led_colors[0] = 255;
                led_colors[1] = 255;
                led_colors[2] = 255;
                break;
            }
        }

        // Red/Blue/Yellow/Purple color twinkle
        if (((continue_red_count%100 >= 50) && (continue_red_count%100 < 100))
            || ((continue_blue_count%100 >= 50) && (continue_blue_count%100 < 100))
            || ((continue_yellow_count%100 >= 50) && (continue_yellow_count%100 < 100))
            || ((continue_purple_count%100 >= 50) && (continue_purple_count%100 < 100))
            || ((continue_blue_ex_count%100 >= 50) && (continue_blue_ex_count%100 < 100))
            || ((continue_green_count%100 >= 50) && (continue_green_count%100 < 100))
            || ((continue_white_count%100 >= 50) && (continue_white_count%100 < 100)))
        {
            led_colors[0] = 0;
            led_colors[1] = 0;
            led_colors[2] = 0;
        }

        if (bNeedLedColorCtl)
        {
            int ret = sn_set_led_colors(led_colors, sizeof(led_colors), timeout);

            if (ret != 0)
            {
                DEBUG("sn_set_led_colors returned %d\n",ret);
            }
        }

        usleep(10000);  // 10ms note that commands should only be sent as often as needed (minimize message traffic)
    }
}

/*******************ThreadInfrared************************************/
void* ThreadInfrared(void*)
{
    DEBUG("ThreadInfrared start\n");

    int   infrared_size = 0;
    char  buf[256];

    int   loop_cter = 0;
    int   data_miss_count = 0;

    //debug_uart();
    init_uart();

    while (true)
    {
        memset(buf, 0, 256);
        memset(read_buf, 0, 128);
        memset(cont_buf, 0, 128);

        int count = 0;

        tcflush(fd, TCIOFLUSH);

        /* usleep for avoiding to read ir-value too often
            (value between the safe and warning will make the drone shake)*/
        if (ir_distance <= 2*ir_safe_distance)
        {
            usleep(300000); //300ms
        }
        else
        {
            usleep(100000); //100ms
        }

        while (count < 3)
        {
            infrared_size = read(fd, buf, 255);

            if (infrared_size > 1)
            {
                break;
            }
            count ++;
        }

        if (infrared_size > 1)
        {
            DEBUG("[%d] Infrared size = %d buf = %s\n", loop_cter, infrared_size, buf);

            data_miss_count = 0;

            memcpy(read_buf, buf+10, 4);
            memcpy(cont_buf, buf, 1);
            ir_distance = atoi((const char*)read_buf)*0.01;
            DEBUG("[%d] cont_buf = %s, ir_distance = %f\n", loop_cter, cont_buf,ir_distance);
        }
        else
        {
            data_miss_count++;

            if (data_miss_count > 3)
            {
                ir_distance = 0;
                DEBUG("[%d] infrared_size < 1, data ignore!\n", loop_cter);
            }
        }

        loop_cter ++;
    }

    close(fd);
}


int main(int argc, char* argv[])
{
    /********************Dynamic variables Start**************************/
    float use_infrared = 1;                 // use infrared data to obstacle avoidance

    int use_reduce_height = 0;              // need open macro AUTO_REDUCE_HEIGHT first

    const float fTakeOffHeight = 1.6;       // m: loiter height when take off
    const float fTrarilHeight = 1.8;        // m
    const float kTakeoffSpeed = 0.7;        // cmd2
    const float kLandingSpeed = -0.5;       // cmd2

    float sonar_valid_data_min = 0.33;      //0.2;
    float sonar_valid_data_max = 3.95;      //2;

    int use_revise_height = 0;              // revise height with barometer and sonar, need to open USE_REVISE_HEIGHT first

    int reverse_rule_sample_size = 0;       // Optic flow mode reverse rule: sample_size missing
    int reverse_rule_desire = 0;            // Optic flow mode reverse rule: desire diff with estimate
    int reverse_rule_linacc = 0;            // Optic flow mode reverse rule: linacc overlimit

    int reverse_full_flag = 1;  //0;        // reverse with all abnormal situation
    int reverse_ctrl_flag = 1;  //0;        // reverse with realtime control

    float gps_mode_height = 5;              // m: switch to gps mode over this height
    float circle_height_limit = 4;          // m: circle height limit, must lower than gps_mode_height

    float speed_coefficient = 1.0f;         // speed coefficient of cmd
    float yaw_coefficient = 0.3;            // cmd3 coefficient

    const float fConstHeightLimit = 30;     // gps-mode-height limit
    float height_limit = 5; //20;           // m: dynamic height limit

    const float fMaxCmdValue = 0.6;
    const float fMaxHoverBrakeCmd = 0.2;
    const float fMaxAltCmdValue = 0.1;
    const float fMaxOFCmdValue = 0.1;
    const float fMaxOFHeight = 5;

    float vel_target = 0.75;                // m/s
    /********************Dynamic variables End**************************/

    SnMode last_mode = SN_OPTIC_FLOW_POS_HOLD_MODE;

    char result_to_client[MAX_BUFF_LEN];
    char current_udp_client_addr[TMP_BUFF_LEN];
    double last_client_msg_time = 0;
    int  udpOverTimeCount = 0;
    bool bHaveUdpClient = false;
    bool bReceivedUdpMsg = false;

    bool confirm_land = false;

    // Fly test mission
    bool fly_test_mission = false;
    int  fly_test_count = 0;
    bool rotation_test_mission = false;
    int  rotation_test_count = 0;

    // 0(indoor:  low_voltage_force_land, low_speed)
    // 1(outdoor: high_voltage_force_land, high_speed)
    int outdoor_mode = 1;

    // Circle mission
    int circle_cam_point_direct = 1;        // point to inside by default 1(inside) -1(outside)
    bool circle_mission = false;
    bool calcCirclePoint = false;
    vector<Position> circle_positions;
    float radius = 2.5f;                    // m: default-circle-radius
    int point_count = 36;                   // default-circle-point-count
    float angle_per = 2*M_PI/point_count;
    int clockwise = 1;                      // anticlockwise = -1

    // Panorama mission
    bool panorama_mission = false;
    static bool calcPanoramaPoint = false;
    vector<Position> panorama_positions;

    // Trail follow with gps array
    bool trail_navigation_mission = false;

    // Customized plan
    bool customized_plan_mission = false;
    bool calcPlanPoint = false;
    vector<string> customized_plan_steps;
    vector<PlanPosition> customized_plan_positions;
    int plan_step_total = 0;
    int plan_step_count = 0;
    float plan_unit = 1;                    // m: per step length

    // Auto reduce the height when disconnect with client
    bool auto_reduce_height_mission = false;
    Position auto_reduce_height_position;

    // Return home mission
    bool return_mission = false;
    bool fly_home = false;
    float gohome_x_vel_des = 0;
    float gohome_y_vel_des = 0;
    float gohome_z_vel_des = 0;
    float gohome_yaw_vel_des = 0;
    uint8_t wp_goal_ret = 0b11111111;
    uint8_t wp_goal_mask = 0b11111111;
    float yaw_target_home, distance_home_squared;
    float yaw_gps_target_home, distance_gps_home_squared;
    float distance_home_squared_threshold = 1;
    FlatVars output_vel;
    bool take_off_with_gps_valid = false;

    static double t_face_detect_valid = 0;
    static bool t_face_detect_flag = false;

    // People detect cuiyc begin
    bool face_mission=false;
    bool body_mission=false;

    // Position at startup
    static float x_est_startup = 0;
    static float y_est_startup = 0;
    static float z_est_startup = 0;
    static float yaw_est_startup = 0;

    static float x_est_gps_startup = 0;
    static float y_est_gps_startup = 0;
    static float z_est_gps_startup = 0;
    static float yaw_est_gps_startup = 0;

    /*
    static float x_est_vio_startup = 0;
    static float y_est_vio_startup = 0;
    static float z_est_vio_startup = 0;
    static float yaw_est_vio_startup = 0;
    */

    // Mission State Machine
    static size_t current_position = 0;

    // Time to loiter
    const float kLoiterTime = 3;            // s

    // Gps params
    GpsPosition posLast;
    GpsPosition posGpsCurrent;
    GpsPosition posGpsDestination;
    float destyaw = 0;
    float speed = 0;
    float distance_to_dest = 0;

    // Gps postion fly
    vector<GpsPosition> gps_positions;
    vector<NavigationPosition> trail_navigation_positions;

    char drone_state_error[MAX_BUFF_LEN];
    char d_state_err[TMP_BUFF_LEN];

    DroneState drone_state = DroneState::NORMAL;
    MissionState state = MissionState::ON_GROUND;   // UNKNOWN;
    int loop_counter = 0;

    // optic flow sample size limit when takeoff
    int propers_start_count = 0;
    int of_ct = 15;
    int of_valid_value = 10;
    int optic_flow_sample_size_sum = 0;
    int optic_flow_sample_average = 0;

    static bool landing_near_ground = false;        // for ignore the sonar wrong data below 0.3m

    float revise_height = 0;                        // height calced with baro and sonar
    float baro_groud = 0;
    float baro_diff  = 0;

    double t_sonar_invalid = 0;

    double t_gps_invalid = 0;
    double t_gps_height_invalid = 0;

    double t_z_rotation_valid = 0;
    double t_prop_spin_loiter = 0;

    double t_normal_rpm = 0;

    double t_client_valid = 0;
    double t_have_client = 0;

    double t_ir_brake = 0;

    // Add by wlh
    float estimated_xy_sqrt = 0;
    float desired_xy_sqrt = 0;
    int baro_count = 0;
    float baro_value_sum = 0;
    int reverse_ctrl_count = 0;
    float old_cmd0 = 0;
    float old_cmd1 = 0;
    float old_cmd00 = 0;
    float old_cmd11 = 0;
    int sample_size_count = 0;
    int sample_size_missing_count = 0;
    int reverse_ctrl_step = 0;
    int linacc_sample_count = 0;
    float linacc_total_value = 0;
    float imu_lin_acc = 0;
    float old_pitch = 0;
    float old_roll = 0;
    float last_pitch = 0;
    float last_roll = 0;
    const int stop_control_num = 199;

    int low_baro_count = 0;
    float low_baro_height_sum = 0;

    const float go_pitch_roll_limit = 0.1;          //0.15;
    const float go_cmd_offset_limit = 0.1;          //0.05;         //0.02;         //5;

    const float cmd_offset_limit = 0.02;

    float last_cmd0 = 0;
    float last_cmd1 = 0;

    int last_sample_sizes_num = 60;
    int last_sample_sizes[60] = {0};
    int last_sample_sizes_flag = 0;

    int v_simple_size_overage = 0;
    const int min_sample_size = 60;
    // Add end

    /* ***********************************************************************************************************/
    /* Read the 200qc_runtime_params.xml to set the sonar-min/max value and gps-mode-height, circle-height-limit */
    /* ***********************************************************************************************************/
    char get_sonar_min_value[TMP_BUFF_LEN] = "grep 'min_sonar_range' " SNAV_CFG_FILE_NAME " | cut -d '\"' -f 4";
    char get_sonar_max_value[TMP_BUFF_LEN] = "grep 'max_sonar_range' " SNAV_CFG_FILE_NAME " | cut -d '\"' -f 4";

    char current_sonar_min_value[TMP_BUFF_LEN];
    char current_sonar_max_value[TMP_BUFF_LEN];
    memset(current_sonar_min_value, 0, TMP_BUFF_LEN);
    memset(current_sonar_max_value, 0, TMP_BUFF_LEN);

    FILE *fp_get_sonar_min_value = popen(get_sonar_min_value, "r");
    if (fp_get_sonar_min_value != NULL)
    {
        fgets(current_sonar_min_value, sizeof(current_sonar_min_value), fp_get_sonar_min_value);
    }
    pclose(fp_get_sonar_min_value);

    if ((strlen(current_sonar_min_value) >= 1) && (current_sonar_min_value[strlen(current_sonar_min_value)-1] == '\n'))
    {
        current_sonar_min_value[strlen(current_sonar_min_value)-1] = '\0';
    }

    FILE *fp_get_sonar_max_value = popen(get_sonar_max_value, "r");
    if (fp_get_sonar_max_value != NULL)
    {
        fgets(current_sonar_max_value, sizeof(current_sonar_max_value), fp_get_sonar_max_value);
    }
    pclose(fp_get_sonar_max_value);

    if ((strlen(current_sonar_max_value) >= 1) && (current_sonar_max_value[strlen(current_sonar_max_value)-1] == '\n'))
    {
        current_sonar_max_value[strlen(current_sonar_max_value)-1] = '\0';
    }

    if (strlen(current_sonar_min_value) >= 1)
    {
        float sonar_min = atof(current_sonar_min_value);

        if (sonar_min > 0)
        {
            sonar_valid_data_min = sonar_min;
        }
    }

    if (strlen(current_sonar_max_value) >= 1)
    {
        float sonar_max = atof(current_sonar_max_value);

        if (sonar_max > 0)
        {
            sonar_valid_data_max = sonar_max;
        }
    }

#ifdef DYNAMIC_GPS_MODE_HEIGHT
    gps_mode_height = sonar_valid_data_max*0.7;
    circle_height_limit = sonar_valid_data_max*0.8;
#endif

    /****************************************************************************/
    /****************************************************************************/
    /**************************Confirm logfile name start************************/
    /****************************************************************************/
    /****************************************************************************/
    FILE *fp_count_read, *fp_count_write, *fp_csv_log;

    if ((fp_count_read = fopen(FC_LOG_PATH PATH_FLAG FC_LOG_CT_NAME, "a+")) != NULL)
    {
        if (fscanf(fp_count_read, "%d", &log_count) != EOF)
        {
            DEBUG("flightctrl_proxy_count=%d\n", log_count);

            if ((log_count >= 0) && (log_count < FC_LOG_CT_MAX))
            {
                log_count++;
            }
            else
            {
                log_count = 0;
            }
        }
        else
        {
            DEBUG("create flightctrl_proxy_count = 1\n");
            log_count = 1;
        }
        fclose(fp_count_read);

        char str[16];
        if ((fp_count_write = fopen(FC_LOG_PATH PATH_FLAG FC_LOG_CT_NAME, "w+")) != NULL)
        {
            snprintf(str, sizeof(str), "%d", log_count);
            fwrite(str, strlen(str), 1, fp_count_write);
            fclose(fp_count_write);
        }
    }

    /*
    char current_snav_log_name[TMP_BUFF_LEN];
    FILE *fp_current = popen("find " SNAV_LOG_PATH " -type f -name 'snav*'|xargs -r ls -l|tail -n 1|awk '{print $9}'", "r");
    if (fp_current != NULL)
    {
        fgets(current_snav_log_name, sizeof(current_snav_log_name), fp_current);
    }
    pclose(fp_current);
    DEBUG("current_snav_log_name=%s\n", current_snav_log_name);
    */

    sleep(20);

    char snav_log_count[TMP_BUFF_LEN];
    FILE *fp_snav_log_count = popen("find " SNAV_LOG_PATH " -type f -name 'snav*'|xargs -r ls -l|tail -n 1| awk '{split($9,a,\"_\"); print a[2]}'", "r");
    if (fp_snav_log_count != NULL)
    {
        fgets(snav_log_count, sizeof(snav_log_count), fp_snav_log_count);
    }
    pclose(fp_snav_log_count);

    if (atoi(snav_log_count) > 0)
    {
        log_count = atoi(snav_log_count);
    }

    current_log_count++;
    memset(log_filename, 0, TMP_BUFF_LEN);
    sprintf(log_filename, FC_LOG_PATH PATH_FLAG "log_flightctrl_%04d_%02d", log_count, current_log_count);

    freopen(log_filename, "a", stdout);
    setbuf(stdout, NULL);       //needn't cache and fflush, output immediately
    freopen(log_filename, "a", stderr);
    setbuf(stderr, NULL);       //needn't cache and fflush, output immediately
    // Confirm logfile name end

    DEBUG("sonar_valid_data_min=%f, sonar_valid_data_max=%f, gps_mode_height=%f, circle_height_limit=%f\n",
            sonar_valid_data_min, sonar_valid_data_max, gps_mode_height, circle_height_limit);
    DEBUG("snav_log_count=%s, log_filename=%s\n", snav_log_count, log_filename);

#ifdef ZZG_DEBUG_FLAG
    FILE *fp_zzg_debug_log;
    // Only keep the last 3 log files-----------keep 2 and add a new one.
    system("find " FC_LOG_PATH  " -type f -name 'zzg_debug_log*'|xargs -r ls -l|head -n -2|awk '{print $9}'| xargs rm -rf");
    char zzg_debug_log_filename[TMP_BUFF_LEN];
    memset(zzg_debug_log_filename, 0, sizeof(zzg_debug_log_filename));
    sprintf(zzg_debug_log_filename, FC_LOG_PATH PATH_FLAG "zzg_debug_log_%04d.csv", log_count);
    DEBUG("zzg_debug_log_filename=%s\n", zzg_debug_log_filename);

    if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
    {
#ifdef ZZG_CMD_DEBUG_FLAG
        char zzg_debug_title[TMP_BUFF_LEN]="index,cmd0,cmd1,cmd2,cmd3,ir_distance,timestamp\n";
#elif defined ZZG_TIMESTAMP_DEBUG_FLAG
        char zzg_debug_title[TMP_BUFF_LEN]="index,timestamp\n";
#elif defined ZZG_SONAR_DEBUG_FLAG
        char zzg_debug_title[TMP_BUFF_LEN]="index,sonar,timestamp\n";
#elif defined ZZG_FINAL_CMD_FLAG
        char zzg_debug_title[TMP_BUFF_LEN]="index,cmd0,cmd1,cmd2,cmd3,timestamp\n";
#endif
        fwrite(zzg_debug_title, strlen(zzg_debug_title), 1, fp_zzg_debug_log);
        fclose(fp_zzg_debug_log);
    }
#endif

#ifdef CSV_LOG_FOR_XIAOYI
    if (NULL == opendir(LOG_FOR_XIAOYI_PATH))
    {
        mkdir(LOG_FOR_XIAOYI_PATH, 0777);
        chmod(LOG_FOR_XIAOYI_PATH, 0777);
    }
#endif

    switchFan(true);

    // Create the face_body_follow process of snav
    bool face_body_follow_flag = false;
    while (!face_body_follow_flag)
    {
        pthread_t face_follow_thread, body_follow_thread;
        pthread_attr_t thread_attr;

        if (pthread_attr_init(&thread_attr) != 0)
        {
            perror("face_body_follow Attribute init failed");
            continue;
        }

        if (pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED) != 0)
        {
            perror("face_body_follow Setting detached attribute failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        if (pthread_create(&face_follow_thread, &thread_attr, ThreadGetVideoFaceFollowParam, NULL) != 0)
        {
            perror("face_follow_thread create failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        if (pthread_create(&body_follow_thread, &thread_attr, ThreadGetVideoBodyFollowParam, NULL) != 0)
        {
            pthread_cancel(face_follow_thread);

            perror("body_follow_thread create failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        face_body_follow_flag = true;
        pthread_attr_destroy(&thread_attr);
    }

    // Create the interact with qcamvid process
    bool interact_with_qcamvid_flag = false;
    while (!interact_with_qcamvid_flag)
    {
        pthread_t interact_with_qcamvid_thread;
        pthread_attr_t thread_attr;

        if (pthread_attr_init(&thread_attr) != 0)
        {
            perror("interact_with_qcamvid_thread Attribute init failed");
            continue;
        }

        if (pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED) != 0)
        {
            perror("interact_with_qcamvid_thread Setting detached attribute failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        if (pthread_create(&interact_with_qcamvid_thread, &thread_attr, ThreadInteractWithQcamvid, NULL) != 0)
        {
            perror("interact_with_qcamvid_thread create failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        interact_with_qcamvid_flag = true;
        pthread_attr_destroy(&thread_attr);
    }

    // Create the interact with ota process
    bool interact_with_ota_flag = false;
    while (!interact_with_ota_flag)
    {
        pthread_t interact_with_ota_thread;
        pthread_attr_t thread_attr;

        if (pthread_attr_init(&thread_attr) != 0)
        {
            perror("interact_with_ota_thread Attribute init failed");
            continue;
        }

        if (pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED) != 0)
        {
            perror("interact_with_ota_thread Setting detached attribute failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        if (pthread_create(&interact_with_ota_thread, &thread_attr, ThreadInteractWithOtaAndLimitLog, NULL) != 0)
        {
            perror("interact_with_ota_and_limit_log_thread create failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        interact_with_ota_flag = true;
        pthread_attr_destroy(&thread_attr);
    }

    // Create the led control process
    bool led_ctl_flag = false;
    while (!led_ctl_flag)
    {
        pthread_t led_ctl_thread;
        pthread_attr_t thread_attr;

        if (pthread_attr_init(&thread_attr) != 0)
        {
            perror("led_ctl_thread Attribute init failed");
            continue;
        }

        if (pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED) != 0)
        {
            perror("led_ctl_thread Setting detached attribute failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        if (pthread_create(&led_ctl_thread, &thread_attr, ThreadLedControl, NULL) != 0)
        {
            perror("Thread led_ctl_thread create failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        led_ctl_flag = true;
        pthread_attr_destroy(&thread_attr);
    }

    // Create the led Infrared process
    if (use_infrared == 1)
    {
        bool Infrared_flag = false;
        while (!Infrared_flag)
        {
            pthread_t Infrared_thread;
            pthread_attr_t thread_attr;

            if (pthread_attr_init(&thread_attr) != 0)
            {
                perror("Infrared_thread Attribute init failed");
                continue;
            }

            if (pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED) != 0)
            {
                perror("Infrared_thread Setting detached attribute failed");
                pthread_attr_destroy(&thread_attr);
                continue;
            }

            if (pthread_create(&Infrared_thread, &thread_attr, ThreadInfrared, NULL) != 0)
            {
                perror("Thread Infrared_thread create failed");
                pthread_attr_destroy(&thread_attr);
                continue;
            }

            Infrared_flag = true;
            pthread_attr_destroy(&thread_attr);
        }
    }

    SnavCachedData* snav_data = NULL;
    if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
    {
        DEBUG("\nFailed to get flight data pointer!\n");
        return -1;
    }

    // Udp communicate with tracker --cuiyc
    int tracker_udp_sockfd;
    struct sockaddr_in address_tracker;
    bzero(&address_tracker, sizeof(address_tracker));
    address_tracker.sin_family      = AF_INET;
    address_tracker.sin_addr.s_addr = inet_addr("192.168.1.1");
    address_tracker.sin_port        = htons(17555);
    tracker_udp_sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    // Udp communicate with Android/IOS app
    int server_udp_sockfd;
    int server_udp_len;
    struct sockaddr_in server_udp_address;

    bzero(&server_udp_address, sizeof(server_udp_address));
    server_udp_address.sin_family       = AF_INET;
    server_udp_address.sin_addr.s_addr  = htonl(INADDR_ANY);
    server_udp_address.sin_port         = htons(SERVER_UDP_PORT);
    server_udp_len                      = sizeof(server_udp_address);

    server_udp_sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    const int kMaxNumAttempts = 10;
    while (bind(server_udp_sockfd, (struct sockaddr*)&server_udp_address, server_udp_len) != 0)
    {
        static int attempt_number = 0;
        DEBUG("Attempt %d to bind udp failed!\n", attempt_number);

        if (attempt_number >= kMaxNumAttempts)
        {
            DEBUG("Unable to bind after %d attempts.\n", attempt_number);
            return -1;
        }

        ++attempt_number;
        usleep(1e6);
    }


    // Set overtime avoid of block
    setsockopt(server_udp_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_udp, sizeof(struct timeval));
    setsockopt(server_udp_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_udp, sizeof(struct timeval));


#ifdef ZZG_DEBUG_FLAG
#ifdef ZZG_TIMESTAMP_DEBUG_FLAG
    double last_udp_msg_time = 0;
#endif
#endif

    while (true)
    {
        int length = 0;
        struct sockaddr_in remote_addr;
        int sin_size = sizeof(struct sockaddr_in);
        char udp_buff_data[MAX_BUFF_LEN];

        //receive the udp data
        length = recvfrom(server_udp_sockfd, udp_buff_data, MAX_BUFF_LEN-1, 0, (struct sockaddr *)&remote_addr, (socklen_t*)&sin_size);

        DEBUG("\n\n");

        // Handle the overtime and limit the client
        if (length > 0)
        {
            struct timeval time_val;
            gettimeofday(&time_val, NULL);
            double time_now = time_val.tv_sec + time_val.tv_usec*1e-6;

#ifdef ZZG_DEBUG_FLAG
#ifdef ZZG_TIMESTAMP_DEBUG_FLAG
            if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
            {
                if ((time_now-last_udp_msg_time) < 20)
                {
                    char csv_info[TMP_BUFF_LEN];

                    memset(csv_info, 0, sizeof(csv_info));
                    sprintf(csv_info, "%d,%lf\n", loop_counter,(time_now-last_udp_msg_time));
                    fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                    fclose(fp_zzg_debug_log);
                }
            }
            last_udp_msg_time = time_now;
#endif
#endif

            udp_buff_data[length] = '\0';
            DEBUG("[%d] udp receive data from %s, %d\n", loop_counter, inet_ntoa(remote_addr.sin_addr), ntohs(remote_addr.sin_port));
            DEBUG("[%d] udp recvfrom udp_buff_data=%s, time_now=%lf\n", loop_counter, udp_buff_data, time_now);

            bReceivedUdpMsg = true;

            //********************ignore the other udp connect******************************
            if (bHaveUdpClient)
            {
                DEBUG("[%d] Current client addr=%s\n", loop_counter, current_udp_client_addr);

                //ignore the other udp client when have one
                if (strncmp(current_udp_client_addr, inet_ntoa(remote_addr.sin_addr), strlen(current_udp_client_addr)) != 0)
                {
                    DEBUG("[%d] udp recvfrom ignore the other udp addr=%s\n", loop_counter, inet_ntoa(remote_addr.sin_addr));
                    continue;
                }
                else
                {
                    udpOverTimeCount = 0;       //Reset the overtime counter to 0
                }
            }
            // Lock the udp client ip when the first udp connect
            else
            {
                bHaveUdpClient = true;
                udpOverTimeCount = 0;

                memset(current_udp_client_addr, 0, TMP_BUFF_LEN);
                memcpy(current_udp_client_addr, inet_ntoa(remote_addr.sin_addr), TMP_BUFF_LEN);
                DEBUG("[%d] udp recvfrom the first client addr=%s\n", loop_counter, inet_ntoa(remote_addr.sin_addr));
            }
        }
        else    /* overtime/disconnect/other issue */
        {
#ifdef ZZG_DEBUG_FLAG
#ifdef ZZG_TIMESTAMP_DEBUG_FLAG
            if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
            {
                char csv_info[TMP_BUFF_LEN];

                memset(csv_info, 0, sizeof(csv_info));
                sprintf(csv_info, "%d,%d\n", loop_counter, 0);
                fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                fclose(fp_zzg_debug_log);
            }
#endif
#endif
            struct timeval time_val;
            gettimeofday(&time_val, NULL);
            double time_now = time_val.tv_sec + time_val.tv_usec*1e-6;

            DEBUG("[%d] udp overtime or issue: length=%d, errno=%d, time_now=%lf\n", loop_counter, length, errno, time_now);

            bReceivedUdpMsg = false;

            //************************************************************
            udpOverTimeCount++;

            DEBUG("[%d] udpOverTimeCount=%d\n", loop_counter, udpOverTimeCount);

            if (udpOverTimeCount >= 150)    //150*20ms=3s
            {
                DEBUG("[%d] current udp client overtime and discard\n", loop_counter);
                bHaveUdpClient = false;
                udpOverTimeCount = 0;
            }
        }

        // Record the time between current and last udp msg
        struct timeval tm_temp;
        gettimeofday(&tm_temp, NULL);
        double time_temp_now = tm_temp.tv_sec + tm_temp.tv_usec*1e-6;


        /* Reply to 1000 control */
        vector<string> udp_client_msg;
        udp_client_msg = split(udp_buff_data, STR_SEPARATOR);

        if ((udp_client_msg.size() >= 6) && (udp_client_msg[0].compare(SNAV_CMD_CONROL) == 0))
        {
            char time_now[TMP_BUFF_LEN];
            memset(time_now, 0, TMP_BUFF_LEN);
            sprintf(time_now, "%lf", time_temp_now);

            memset(result_to_client, 0, MAX_BUFF_LEN);
            sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CONROL);
            strcat(result_to_client, STR_SEPARATOR);
            strcat(result_to_client, time_now);

            length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
                             , 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
            DEBUG("[%d] udp sendto SNAV_CMD_RETURN_CONROL length=%d, time_now=%s\n", loop_counter, length, time_now);
        }


        /* Will make the msg between 6ms(min valid msg) and 100ms(over time) */
        // Avoid to send cmd too quick to snav, snav will ignore these cmds
        if ((time_temp_now > 0)
            && (last_client_msg_time > 0)
            && ((time_temp_now - last_client_msg_time) > 0.000001)
            && ((time_temp_now - last_client_msg_time) < 5))
        {
            DEBUG("[%d] diff_time_between_udp_msg=%lf. time_now=%lf, last_client_msg_time=%lf.\n",
                        loop_counter, (time_temp_now - last_client_msg_time), time_temp_now, last_client_msg_time);

            vector<string> udp_array;
            udp_array = split(udp_buff_data, STR_SEPARATOR);

            if (((time_temp_now - last_client_msg_time) < 0.006)    //6ms
                && (udp_array.size() >= 6)
                && (udp_array[0].compare(SNAV_CMD_CONROL) == 0))    //only ignore the too quick control cmd
            {
                DEBUG("[%d] send cmd too quick, ignore this cmd.  time_diff=%lf.\n",
                            loop_counter, (time_temp_now - last_client_msg_time));
                continue;
            }
        }

        // Avoid to send cmd too slow to snav
        if (!fly_test_mission
            && !rotation_test_mission
            && !circle_mission
            && !panorama_mission
            && !trail_navigation_mission
            && !customized_plan_mission
            && !return_mission
            && !face_mission
            && !body_mission)
        {
            if ((udpOverTimeCount > 0)
                && (time_temp_now > 0)
                && (last_client_msg_time > 0)
                && ((time_temp_now - last_client_msg_time) > 0.000001)
                && ((time_temp_now - last_client_msg_time) < control_msg_max_diff))
            {
                continue;
            }

            /*last_client_msg_time = time_temp_now;*/
        }

        last_client_msg_time = time_temp_now;


        /* Dispatch the overlate control msg */
        if ((udp_client_msg.size() >= 7) && (udp_client_msg[0].compare(SNAV_CMD_CONROL) == 0))
        {
            double time_stamp = atof(udp_client_msg[6].c_str());

            DEBUG("[%d] time_stamp=%lf\n", loop_counter, time_stamp);

            if ((time_temp_now - time_stamp) > 0.3) //300ms
            {
                // Discard this overtime valid udp msg
                bReceivedUdpMsg = false;
                DEBUG("[%d] time_temp_now - time_stamp=%lf, discard!!!\n", loop_counter, (time_temp_now - time_stamp));
            }
        }


        // Always need to call this
        if (sn_update_data() != 0)
        {
            DEBUG("[%d] sn_update_data failed!!!\n", loop_counter);
        }
        else
        {
            // Get the current mode
            SnMode mode = (SnMode)snav_data->general_status.current_mode;

            // Get the current state of the propellers
            SnPropsState props_state = (SnPropsState)snav_data->general_status.props_state;

            // Get the source of the RC input (spektrum vs API) here
            SnRcCommandSource rc_cmd_source = (SnRcCommandSource)(snav_data->rc_active.source);

            if (use_infrared == 1)
            {
                DEBUG("[%d] Infrared distance=%f \n", loop_counter, ir_distance);
            }

            // Get the gps status
            int gps_enabled = snav_data->gps_pos_vel.is_enabled;
            //sn_is_gps_enabled(&gps_enabled);

            if ((int)snav_data->gps_0_raw.fix_type == 3)
            {
                posGpsCurrent.latitude  = (int)snav_data->gps_0_raw.latitude;
                posGpsCurrent.longitude = (int)snav_data->gps_0_raw.longitude;
                posGpsCurrent.altitude  = (int)snav_data->gps_0_raw.altitude;
                posGpsCurrent.yaw       = (float)snav_data->gps_pos_vel.yaw_estimated;
            }

            // Get the current battery voltage
            float voltage = snav_data->general_status.voltage;

            // Get the current sample size
            int sample_size = snav_data->optic_flow_0_raw.sample_size;

            // Get the sensors status
            SnDataStatus imu_status         = (SnDataStatus)snav_data->data_status.imu_0_status;
            SnDataStatus baro_status        = (SnDataStatus)snav_data->data_status.baro_0_status;
            SnDataStatus mag_status         = (SnDataStatus)snav_data->data_status.mag_0_status;
            SnDataStatus gps_status         = (SnDataStatus)snav_data->data_status.gps_0_status;
            SnDataStatus sonar_status       = (SnDataStatus)snav_data->data_status.sonar_0_status;
            SnDataStatus optic_flow_status  = (SnDataStatus)snav_data->data_status.optic_flow_0_status;

            bool mag_usable = true;
            if ((mag_status != SN_DATA_VALID) && (mag_status != SN_DATA_WARNING))
            {
                mag_usable = false;
            }

            // Get the "on ground" flag
            int on_ground_flag = (int)snav_data->general_status.on_ground;

            // Record the gps_invalid_time
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            double t_now_for_gps = tv_now.tv_sec + tv_now.tv_usec*1e-6;

            if (!gps_enabled || (gps_status != SN_DATA_VALID))
            {
                t_gps_invalid = t_now_for_gps;
            }


            // Record the barometer data
            if (on_ground_flag == 1)
            {
                baro_groud = snav_data->barometer_0_raw.pressure;
            }

            if (baro_groud != 0)
            {
                baro_diff = snav_data->barometer_0_raw.pressure - baro_groud;
            }

            // Add by wlh -----Revise the z position with linear formula of barometer start
#ifdef USE_REVISE_HEIGHT
            float barometer_height = BAROMETER_LINEAR_MACRO(baro_diff);

            if (on_ground_flag == 1 || snav_data->sonar_0_raw.range  < sonar_valid_data_min)
            {
                barometer_height = 0;
            }

            if (on_ground_flag == 0 && snav_data->sonar_0_raw.range > sonar_valid_data_min && baro_groud != 0)
            {
                if ((revise_height < sonar_valid_data_max) && (snav_data->sonar_0_raw.range < sonar_valid_data_max))
                {
                    if (fabs(revise_height - snav_data->sonar_0_raw.range) < 0.5)
                    {
                        if (fabs(barometer_height - snav_data->sonar_0_raw.range) < 0.8)    //1.2
                        {
                            revise_height = snav_data->sonar_0_raw.range;
                        }
                        else
                        {
                            if (low_baro_count == 0)
                            {
                                low_baro_height_sum = 0;
                                low_baro_count++;
                            }
                            else
                            {
                                low_baro_height_sum += barometer_height;
                                low_baro_count++;
                                if(low_baro_count > 5)
                                {
                                    low_baro_count = 0;
                                    revise_height = low_baro_height_sum/5;
                                }
                            }
                        }
                    }
                    else
                    {
                        if (fabs(barometer_height - revise_height) < 0.5)
                        {
                            if (barometer_height > 0.3)
                            {
                                revise_height = barometer_height;
                            }
                        }
                        else
                        {
                            //air_heigh = snav_data->sonar_0_raw.range;
                            //printf("444air_heigh[%.2f]sonar_0_raw.range[%.2f]\n", air_heigh,snav_data->sonar_0_raw.range);
                        }
                    }
                }
                else
                {
                    if (baro_count == 0)
                    {
                        baro_value_sum = 0;
                        baro_count++;
                    }
                    else
                    {
                        baro_value_sum += 0.000007*baro_diff*baro_diff - 0.081320*baro_diff - 0.027264;
                        baro_count++;
                        if (baro_count > 10)
                        {
                            baro_count = 0;

                            if (fabs((fabs(baro_value_sum/10) - revise_height)) < 0.8)
                            {
                                revise_height = baro_value_sum/10;
                            }

                            if (revise_height < 0)
                            {
                                revise_height = barometer_height;
                            }
                        }
                    }
                }
            }
#endif
            /* Cal the height with baro_diff and sonar End */

            /* Print necessary msg */
            float vel_est_optic_0 = snav_data->optic_flow_pos_vel.velocity_estimated[0];
            float vel_est_optic_1 = snav_data->optic_flow_pos_vel.velocity_estimated[1];

            if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
            {
                DEBUG("[%d] revise_height=%f, baro_groud=%f, baro_diff=%f, snav_data->sonar_0_raw.range=%f, vel_est_optic_0=%f, vel_est_optic_1=%f, mode=SN_OPTIC_FLOW_POS_HOLD_MODE.\n",
                            loop_counter, revise_height, baro_groud, baro_diff, snav_data->sonar_0_raw.range, vel_est_optic_0, vel_est_optic_1);
            }
            else if (mode == SN_VIO_POS_HOLD_MODE)
            {
                float vel_est_vio_0 = snav_data->vio_pos_vel.velocity_estimated[0];
                float vel_est_vio_1 = snav_data->vio_pos_vel.velocity_estimated[1];

                DEBUG("[%d] revise_height=%f, baro_groud=%f, baro_diff=%f, snav_data->sonar_0_raw.range=%f, vel_est_vio_0=%f, vel_est_vio_1=%f, mode=SN_VIO_POS_HOLD_MODE.\n",
                            loop_counter, revise_height, baro_groud, baro_diff, snav_data->sonar_0_raw.range, vel_est_vio_0, vel_est_vio_1);
            }
            else if (mode == SN_GPS_POS_HOLD_MODE)
            {
                float vel_est_gsp_0 = snav_data->gps_pos_vel.velocity_estimated[0];
                float vel_est_gsp_1 = snav_data->gps_pos_vel.velocity_estimated[1];

                DEBUG("[%d] revise_height=%f, baro_groud=%f, baro_diff=%f, snav_data->sonar_0_raw.range=%f, vel_est_gsp_0=%f, vel_est_gsp_1=%f, mode=SN_GPS_POS_HOLD_MODE.\n",
                            loop_counter, revise_height, baro_groud, baro_diff, snav_data->sonar_0_raw.range, vel_est_gsp_0, vel_est_gsp_1);
            }
            else if (mode == SN_ALT_HOLD_MODE)
            {
                DEBUG("[%d] revise_height=%f, baro_groud=%f, baro_diff=%f, snav_data->sonar_0_raw.range=%f, vel_est_optic_0=%f, vel_est_optic_1=%f, mode=SN_ALT_HOLD_MODE.\n",
                            loop_counter, revise_height, baro_groud, baro_diff, snav_data->sonar_0_raw.range, vel_est_optic_0, vel_est_optic_1);
            }
            else
            {
                DEBUG("[%d] revise_height=%f, baro_groud=%f, baro_diff=%f, snav_data->sonar_0_raw.range=%f, vel_est_optic_0=%f, vel_est_optic_1=%f, mode=%d.\n",
                            loop_counter, revise_height, baro_groud, baro_diff, snav_data->sonar_0_raw.range, vel_est_optic_0, vel_est_optic_1, mode);
            }

            // Get the current gps estimated and desired position and yaw
            float x_est_gps, y_est_gps, z_est_gps, yaw_est_gps;
            float x_des_gps, y_des_gps, z_des_gps, yaw_des_gps;

            if (gps_enabled)
            {
                x_est_gps = (float)snav_data->gps_pos_vel.position_estimated[0];
                y_est_gps = (float)snav_data->gps_pos_vel.position_estimated[1];
                z_est_gps = (float)snav_data->gps_pos_vel.position_estimated[2];
                yaw_est_gps = (float)snav_data->gps_pos_vel.yaw_estimated;

                x_des_gps = (float)snav_data->gps_pos_vel.position_desired[0];
                y_des_gps = (float)snav_data->gps_pos_vel.position_desired[1];
                z_des_gps = (float)snav_data->gps_pos_vel.position_desired[2];
                yaw_des_gps = (float)snav_data->gps_pos_vel.yaw_desired;
            }
            else
            {
                x_est_gps = 0;
                y_est_gps = 0;
                z_est_gps = 0;
                yaw_est_gps = 0;

                x_des_gps = 0;
                y_des_gps = 0;
                z_des_gps = 0;
                yaw_des_gps = 0;
            }

            // Get the current vio estimated and desired position and yaw
            /*
            float x_est_vio, y_est_vio, z_est_vio, yaw_est_vio;
            float x_des_vio, y_des_vio, z_des_vio, yaw_des_vio;

            if (mode == SN_VIO_POS_HOLD_MODE)
            {
                x_est_vio = (float)snav_data->vio_pos_vel.position_estimated[0];
                y_est_vio = (float)snav_data->vio_pos_vel.position_estimated[1];
                z_est_vio = (float)snav_data->vio_pos_vel.position_estimated[2];
                yaw_est_vio = (float)snav_data->vio_pos_vel.yaw_estimated;

                x_des_vio = (float)snav_data->vio_pos_vel.position_desired[0];
                y_des_vio = (float)snav_data->vio_pos_vel.position_desired[1];
                z_des_vio = (float)snav_data->vio_pos_vel.position_desired[2];
                yaw_des_vio = (float)snav_data->vio_pos_vel.yaw_desired;
            }
            else
            {
                x_est_vio = 0;
                y_est_vio = 0;
                z_est_vio = 0;
                yaw_est_vio = 0;

                x_des_vio = 0;
                y_des_vio = 0;
                z_des_vio = 0;
                yaw_des_vio = 0;
            }
            */

            // Get the current estimated position and yaw
            float x_est, y_est, z_est, yaw_est;
            x_est = (float)snav_data->optic_flow_pos_vel.position_estimated[0];
            y_est = (float)snav_data->optic_flow_pos_vel.position_estimated[1];
            z_est = (float)snav_data->optic_flow_pos_vel.position_estimated[2];
            yaw_est = (float)snav_data->optic_flow_pos_vel.yaw_estimated;


            // Get the current desired position and yaw
            // NOTE this is the setpoint that will be controlled by sending
            // velocity commands
            float x_des, y_des, z_des, yaw_des;
            x_des = (float)snav_data->optic_flow_pos_vel.position_desired[0];
            y_des = (float)snav_data->optic_flow_pos_vel.position_desired[1];
            z_des = (float)snav_data->optic_flow_pos_vel.position_desired[2];
            yaw_des = (float)snav_data->optic_flow_pos_vel.yaw_desired;

            struct timeval tv_for_des;
            gettimeofday(&tv_for_des, NULL);
            double t_des_now = tv_for_des.tv_sec + tv_for_des.tv_usec * 1e-6;

            /* Wlh--Calc the optic-flow-sample-size value */
            last_sample_sizes[last_sample_sizes_flag] = snav_data->optic_flow_0_raw.sample_size;
            last_sample_sizes_flag++;

            if (last_sample_sizes_flag > last_sample_sizes_num - 1)
            {
                last_sample_sizes_flag = 0;
            }

            if (last_sample_sizes_flag == last_sample_sizes_num - 1)
            {
                v_simple_size_overage = 0;

                for (int i = 0; i < last_sample_sizes_num; i++)
                {
                    v_simple_size_overage += last_sample_sizes[i];
                }

                v_simple_size_overage = v_simple_size_overage/last_sample_sizes_num;
            }

            if (on_ground_flag == 1)
            {
                v_simple_size_overage = 0;
            }

            DEBUG("[%d] v_simple_size_overage=%d, t_des_now=%lf.\n", loop_counter, v_simple_size_overage, t_des_now);

            if (bHaveUdpClient)
            {
                t_have_client = t_des_now;
            }

#ifdef AUTO_REDUCE_HEIGHT
            if (use_reduce_height == 1)
            {
                DEBUG("[%d] bHaveUdpClient=%d, t_client_valid=%f\n", loop_counter, bHaveUdpClient, t_client_valid);

                // For auto_reduce_height_mission
                if (bHaveUdpClient)
                {
                    t_client_valid = t_des_now;
                    auto_reduce_height_mission = false;
                }
                else if ((state == MissionState::LOITER) || (state == MissionState::IN_MOTION))
                {
                    DEBUG("[%d] bHaveUdpClient==false, auto_reduce_height_mission=%d\n", loop_counter, auto_reduce_height_mission);

                    if (auto_reduce_height_mission)
                    {
                        state = MissionState::IN_MOTION;
                    }
                    else
                    {
                        Position pos_current;
                        if (mode == SN_GPS_POS_HOLD_MODE)
                        {
                            pos_current.x = x_est_gps-x_est_gps_startup;
                            pos_current.y = y_est_gps-y_est_gps_startup;
                            pos_current.z = z_est_gps;
                            pos_current.yaw = yaw_est_gps;
                        }
                        /*
                        else if (mode == SN_VIO_POS_HOLD_MODE)
                        {
                            pos_current.x = x_est_vio-x_est_vio_startup;
                            pos_current.y = y_est_vio-y_est_vio_startup;
                            pos_current.z = z_est_vio;
                            pos_current.yaw = yaw_est_vio;
                        }
                        */
                        else
                        {
                            pos_current.x = x_est-x_est_startup;
                            pos_current.y = y_est-y_est_startup;
                            pos_current.z = z_est;
                            pos_current.yaw = yaw_est;
                        }

                        DEBUG("[%d] bHaveUdpClient==false, t_diff:%f, revise_height:%f, pos_current:[%f,%f,%f,%f]\n",
                                loop_counter, (t_des_now - t_client_valid), revise_height,
                                pos_current.x, pos_current.y,
                                pos_current.z, pos_current.yaw);

                        if (((t_client_valid != 0)  && (t_des_now - t_client_valid) >= 60) && ((t_des_now - t_client_valid) < 120))
                        {
                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                if ((z_est_gps - z_est_gps_startup) >= 15 /*1.8*/)
                                {
                                    pos_current.z = 12 + z_est_gps_startup;  /*1.5*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                            else
                            {
                                if ((z_est - z_est_startup) >= 15 /*1.8*/)
                                {
                                    pos_current.z = 12 + z_est_startup;  /*1.5*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                        }
                        else if (((t_client_valid != 0)  && (t_des_now - t_client_valid) >= 120) && ((t_des_now - t_client_valid) < 180))
                        {
                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                if ((z_est_gps - z_est_gps_startup) >= 12 /*1.3*/)
                                {
                                    pos_current.z = 10 + z_est_gps_startup;  /*1.2*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                            else
                            {
                                if ((z_est - z_est_startup) >= 12 /*1.3*/)
                                {
                                    pos_current.z = 10 + z_est_startup;  /*1.2*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                        }
                        else if (((t_client_valid != 0)  && (t_des_now - t_client_valid) >= 180) && ((t_des_now - t_client_valid) < 240))
                        {
                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                if ((z_est_gps - z_est_gps_startup) >= 10 /*1*/)
                                {
                                    pos_current.z = 8 + z_est_gps_startup;  /*0.8*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                            else
                            {
                                if ((z_est - z_est_startup) >= 10 /*1*/)
                                {
                                    pos_current.z =  + z_est_startup;  /*0.8*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                        }
                        else if ((t_client_valid != 0)  && (t_des_now - t_client_valid) > 240)
                        {
                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                if ((z_est_gps - z_est_gps_startup) >= 5 /*0.6*/)
                                {
                                    pos_current.z = 5 + z_est_gps_startup;  /*0.4*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                            else
                            {
                                if ((z_est - z_est_startup) >= 5 /*0.6*/)
                                {
                                    pos_current.z = 5 + z_est_startup;  /*0.4*/
                                    auto_reduce_height_position = pos_current;
                                    auto_reduce_height_mission = true;
                                    state = MissionState::IN_MOTION;
                                }
                            }
                        }
                    }
                }
            }
#endif

            // Record the time for stop propers when drone was inverted
            if (snav_data->attitude_estimate.rotation_matrix[8] > -0.7)
            {
                t_z_rotation_valid = t_des_now;
            }

            // Record the time for stop propers when landing
            if ( (snav_data->esc_raw.rpm[0] >= 7500)
                || (snav_data->esc_raw.rpm[1] >= 7500)
                || (snav_data->esc_raw.rpm[2] >= 7500)
                || (snav_data->esc_raw.rpm[3] >= 7500))
            {
                t_normal_rpm = t_des_now;
            }

            DEBUG("[%d] t_des_now_diff_with_z_rotation_valid=%f\n", loop_counter, (t_des_now - t_z_rotation_valid));

            // Stop propers when spin over 10s on ground for safety
            if (props_state == SN_PROPS_STATE_SPINNING)
            {
                if (on_ground_flag != 1)
                {
                    t_prop_spin_loiter = t_des_now;
                }

                if ((on_ground_flag == 1)
                    && (t_prop_spin_loiter != 0)
                    && ((t_des_now - t_prop_spin_loiter) > time_for_spin_on_ground))
                {
                    // Reset all the missions
                    current_position =0;

                    fly_test_mission = false;
                    rotation_test_mission = false;

                    circle_mission = false;
                    calcCirclePoint = false;

                    panorama_mission = false;
                    calcPanoramaPoint = false;

                    trail_navigation_mission = false;
                    customized_plan_mission = false;
                    calcPlanPoint = false;

                    return_mission = false;

                    face_mission = false;
                    body_mission = false;

                    face_follow_switch = false;
                    body_follow_switch = false;

                    /*if (!send_fpv_flag)*/
                    {
                        send_fpv_flag = true;
                        memset(fpv_switcher_buff, 0, DOMAIN_BUFF_SIZE);
                        strncpy(fpv_switcher_buff, "exit", 4);

                        send_face_follow_swither_flag = true;
                        memset(face_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                        strncpy(face_follow_swither_buff, "fdoff", 5);

                        send_body_follow_swither_flag = true;
                        memset(body_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                        strncpy(body_follow_swither_buff, "bdoff", 5);
                    }

                    sn_stop_props();
                    state = MissionState::ON_GROUND;
                }
            }

            // Stop propers when drone is inverted
            if (((props_state == SN_PROPS_STATE_SPINNING) || (props_state == SN_PROPS_STATE_STARTING))
                && ((t_des_now - t_z_rotation_valid) > time_interval_of_imu_invalid))
            {
                // Reset all the mission
                current_position =0;

                fly_test_mission = false;
                rotation_test_mission = false;

                circle_mission = false;
                calcCirclePoint = false;

                panorama_mission = false;
                calcPanoramaPoint = false;

                trail_navigation_mission = false;
                customized_plan_mission = false;
                calcPlanPoint = false;

                return_mission = false;

                face_mission = false;
                body_mission = false;

                face_follow_switch = false;
                body_follow_switch = false;

                /*if (!send_fpv_flag)*/
                {
                    send_fpv_flag = true;
                    memset(fpv_switcher_buff, 0, DOMAIN_BUFF_SIZE);
                    strcpy(fpv_switcher_buff, "exit");

                    send_face_follow_swither_flag = true;
                    memset(face_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                    strcpy(face_follow_swither_buff, "fdoff");

                    send_body_follow_swither_flag = true;
                    memset(body_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                    strcpy(body_follow_swither_buff, "bdoff");
                }

                sn_stop_props();
                state = MissionState::ON_GROUND;
            }

#ifdef  AUTO_FACE_TAKE_OFF
            if (!bHaveUdpClient
                && (on_ground_flag == 1)
                && (props_state == SN_PROPS_STATE_NOT_SPINNING)
                && ((t_des_now - t_z_rotation_valid) > time_interval_of_face_takeoff)
                && !face_takeoff_flag)
            {
                DEBUG("[%d] face_follow_switch start!\n", loop_counter);

                send_fpv_flag = true;
                memset(fpv_switcher_buff, 0, DOMAIN_BUFF_SIZE);
                strncpy(fpv_switcher_buff, "nofpv_on", 8);

                sleep(3);

                face_follow_switch = true;
                body_follow_switch = false;

                send_face_follow_swither_flag = true;
                memset(face_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
                strncpy(face_follow_swither_buff, "fdon", 4);

                face_takeoff_flag = true;
            }
#endif

            // Revise print message and whether to use it
            DEBUG("[%d] use_revise_height=%d, revise_height=%f, z_est_startup=%f, z_est-z_est_startup=%f,z_des-z_est_startup=%f\n",
                        loop_counter, use_revise_height, revise_height,
                        z_est_startup, (z_est-z_est_startup), (z_des-z_est_startup));

            // Whether to use the revise_height or the z_est_height data
            if (use_revise_height != 1)
            {
                if (gps_enabled && (gps_status == SN_DATA_VALID) && (take_off_with_gps_valid))
                {
                    revise_height = (z_est_gps-z_est_gps_startup);
                }
                /*
                else if (mode == SN_VIO_POS_HOLD_MODE)
                {
                    revise_height = (z_est_vio -z_est_vio_startup);
                }
                */
                else
                {
                    revise_height = (z_est-z_est_startup);
                }
            }

            if (revise_height < 0)
            {
                revise_height = 0;
            }

            // Record the time for gps/optic-flow mode switch
            //if ((revise_height <= gps_mode_height) || (snav_data->sonar_0_raw.range <= gps_mode_height))
            if (revise_height <= gps_mode_height)
            {
                t_gps_height_invalid = t_des_now;
            }

            if ((sonar_status != SN_DATA_VALID) || (snav_data->sonar_0_raw.range > sonar_valid_data_max))
            {
                t_sonar_invalid = t_des_now;
            }

            // Add by wlh-----Judge whether need to reverse the drone start
            const float reverse_plan_num = 10;
            float optic_flow_vel_est = fabs(sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                 + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]));

            float gps_vel_est = fabs(sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                 + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]));

            // Sample-size error handle start
            if (reverse_rule_sample_size == 1
                && (mode == SN_OPTIC_FLOW_POS_HOLD_MODE
                    || mode == SN_ALT_HOLD_MODE))
            {
                // Avoid to reverse the drone when takeoff with sample-size bad
                if ((state == MissionState::LOITER) || (state == MissionState::IN_MOTION))
                {
                    if (sample_size >= 0)
                    {
                        if ((revise_height > sonar_valid_data_min) && (reverse_ctrl_count == 0) && (on_ground_flag == 0))
                        {
                            sample_size_count++;

                            if ((sample_size_count > 0) && (sample_size < 5))
                            {
                                sample_size_missing_count++;
                            }

                            if (sample_size_count > 20)
                            {
                                if (sample_size_missing_count > 9)
                                {
                                    if (reverse_ctrl_step == 0)
                                    {
                                        reverse_ctrl_count++;

                                        DEBUG("sample_size_missing ---first old_cmd0[%f], old_cmd1[%f], reverse_ctrl_count:%d\n",
                                                    old_cmd0, old_cmd1, reverse_ctrl_count);

                                        if (fabs(old_cmd0) < 0.3 && fabs(old_cmd1) < 0.3)
                                        {
                                            old_cmd0 = snav_data->attitude_estimate.pitch*6;
                                            old_cmd1 = -snav_data->attitude_estimate.roll*6;
                                        }

                                        DEBUG("debug_flag control session 111 reverse_ctrl_count=%d.\n", reverse_ctrl_count);

                                        DEBUG("sample_size_missing ---final old_cmd0[%f]old_cmd1[%f], reverse_ctrl_count:%d\n",
                                                    old_cmd0, old_cmd1, reverse_ctrl_count);

                                        old_pitch = snav_data->attitude_estimate.pitch;
                                        old_roll  = -snav_data->attitude_estimate.roll;
                                    }
                                }

                                sample_size_count = 0;
                                sample_size_missing_count = 0;
                            }

                            // optic-flow over vel limit
                            if ((optic_flow_vel_est > 3) && (reverse_ctrl_step == 0))
                            {
                                reverse_ctrl_count++;

                                DEBUG("debug_flag control session 222 reverse_ctrl_count=%d.\n", reverse_ctrl_count);

                                DEBUG("optic-flow over_speed > 3------optic_flow_vel_est[%f], reverse_ctrl_count:%d \n", optic_flow_vel_est, reverse_ctrl_count);
                                old_pitch = snav_data->attitude_estimate.pitch;
                                old_roll  = -snav_data->attitude_estimate.roll;
                            }

                        }
                    }

                    // Safe speed limit in the end.
                    old_cmd0 = CMD_INPUT_LIMIT(old_cmd0, fMaxCmdValue);
                    old_cmd1 = CMD_INPUT_LIMIT(old_cmd1, fMaxCmdValue);
                }
            }
            // Sample-size error handle end

            // Diff between pos_est and pos_des is abnormal handle start
            if (reverse_rule_desire == 1 && mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
            {
                estimated_xy_sqrt = sqrt(snav_data->optic_flow_pos_vel.position_estimated[0]*snav_data->optic_flow_pos_vel.position_estimated[0]
                                        + snav_data->optic_flow_pos_vel.position_estimated[1]*snav_data->optic_flow_pos_vel.position_estimated[1]);
                desired_xy_sqrt   = sqrt(snav_data->optic_flow_pos_vel.position_desired[0]* snav_data->optic_flow_pos_vel.position_desired[0]
                                        + snav_data->optic_flow_pos_vel.position_desired[1]*snav_data->optic_flow_pos_vel.position_desired[1]);

                if(fabs(estimated_xy_sqrt)> 1
                    && fabs(desired_xy_sqrt) > 1
                    &&(fabs(estimated_xy_sqrt - desired_xy_sqrt) > revise_height/2)
                    && (fabs(estimated_xy_sqrt - desired_xy_sqrt) > 0.5)
                    && (revise_height > sonar_valid_data_min)
                    && (snav_data->general_status.on_ground == 0)
                    && (reverse_ctrl_count == 0))
                {
                    if(reverse_ctrl_step == 0)
                    {
                        reverse_ctrl_count++;

                        DEBUG("debug_flag control session 333 reverse_ctrl_count=%d.\n", reverse_ctrl_count);

                        DEBUG("Diff between pos_est and pos_des is abnormal---estimated_xy_sqrt[%f] desired_xy_sqrt[%f]--sample_size[%d]-revise_height/2=[%f], reverse_ctrl_count:%d.\n",
                                estimated_xy_sqrt,desired_xy_sqrt,sample_size,revise_height/2,reverse_ctrl_count);
                        old_pitch = snav_data->attitude_estimate.pitch;
                        old_roll  = -snav_data->attitude_estimate.roll;
                    }
                }
            }
            // Diff between pos_est and pos_des is abnormal handle end

            // Imu_lin_acc overlimit handle start
            if (reverse_rule_linacc == 1 && mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
            {
                if (imu_status == SN_DATA_VALID)
                {
                    imu_lin_acc = sqrt(snav_data->imu_0_compensated.lin_acc[0]*snav_data->imu_0_compensated.lin_acc[0]
                                        + snav_data->imu_0_compensated.lin_acc[1]*snav_data->imu_0_compensated.lin_acc[1]);

                    if (linacc_sample_count < 30)
                    {
                        linacc_sample_count++;
                        linacc_total_value += fabs(imu_lin_acc);
                    }
                    else
                    {
                        if((linacc_total_value/30)/1.44 > 2
                            && (revise_height> sonar_valid_data_min)
                            && snav_data->general_status.on_ground == 0
                            && reverse_ctrl_count == 0)
                        {
                            reverse_ctrl_count++;

                            DEBUG("debug_flag control session 444 reverse_ctrl_count=%d.\n", reverse_ctrl_count);

                            DEBUG("Imu_lin_acc overlimit--linacc_total_value/30[%f], reverse_ctrl_count:%d.\n",
                                        linacc_total_value/30, reverse_ctrl_count);
                            old_pitch = snav_data->attitude_estimate.pitch;
                            old_roll  = -snav_data->attitude_estimate.roll;
                        }
                        linacc_sample_count = 0;
                        linacc_total_value = 0;
                    }
                }
                else
                {
                    imu_lin_acc = -108;
                }
            }
            // Imu_lin_acc overlimit handle end
            // Add end-----Judge whether need to reverse the drone end



            /* *******************************************************************/
            /* *******Prepare current drone status for sending to client**********/
            /* *******************************************************************/
            drone_state = DroneState::NORMAL;                           /*Reinit the drone status every cicle*/

            memset(drone_state_error, 0, MAX_BUFF_LEN);
            memset(d_state_err, 0, TMP_BUFF_LEN);

            sprintf(drone_state_error, "%s", "drone_state_error:0");    /*Reinit the drone error state*/
            sprintf(d_state_err, "%s", "0");

            if (props_state == SN_PROPS_STATE_UNKNOWN)                  /*Get current proper status*/
            {
                drone_state = DroneState::MOTOR_ERROR;
                strcat(drone_state_error, ":1");
                strcat(d_state_err, ":1");
            }

            CpuStats cpu_status = snav_data->cpu_stats;                 /*Get current cpu temp*/
            for (int j = 0; j < 10; j++)
            {
                if (cpu_status.temp[j] >= 80)
                {
                    drone_state = DroneState::CPU_OVER_HEAT;
                    strcat(drone_state_error, ":2");
                    strcat(d_state_err, ":2");
                    break;
                }
            }

            if (imu_status != SN_DATA_VALID)                            /*Get current imu status*/
            {
                drone_state = DroneState::IMU_ERROR;
                strcat(drone_state_error, ":3");
                strcat(d_state_err, ":3");
            }

            if (baro_status != SN_DATA_VALID)                           /*Get current barometer status*/
            {
                drone_state = DroneState::BARO_ERROR;
                strcat(drone_state_error, ":4");
                strcat(d_state_err, ":4");
            }

            if (gps_enabled)                                            /*Get current gps/mag status*/
            {
                if ((mag_status != SN_DATA_VALID) && (mag_status != SN_DATA_WARNING))
                {
                    drone_state = DroneState::MAG_ERROR;
                    strcat(drone_state_error, ":5");
                    strcat(d_state_err, ":5");
                }

                if (gps_status != SN_DATA_VALID)
                {
                    drone_state = DroneState::GPS_ERROR;
                    strcat(drone_state_error, ":6");
                    strcat(d_state_err, ":6");
                }
            }

            if (sonar_status != SN_DATA_VALID)                          /*Get current sonar status*/
            {
                drone_state = DroneState::SONAR_ERROR;
                strcat(drone_state_error, ":7");
                strcat(d_state_err, ":7");
            }

            if ((optic_flow_status != SN_DATA_VALID) && (optic_flow_status != SN_DATA_WARNING))             /*Get current optic-flow status*/
            {
                drone_state = DroneState::OPTIC_FLOW_ERROR;
                strcat(drone_state_error, ":8");
                strcat(d_state_err, ":8");
            }

            if ((mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
                && (mode != SN_GPS_POS_HOLD_MODE)
                && (mode != SN_THRUST_ANGLE_MODE)
                && (mode != SN_ALT_HOLD_MODE)
                && (mode != SN_VIO_POS_HOLD_MODE))                    /*Get current mode*/
            {
                if (mode == SN_EMERGENCY_LANDING_MODE)
                {
                    drone_state = DroneState::EMERGENCY_LANDING_MODE;
                    strcat(drone_state_error, ":9");
                    strcat(d_state_err, ":9");
                }
                else if (mode == SN_EMERGENCY_KILL_MODE)
                {
                    drone_state = DroneState::EMERGENCY_KILL_MODE;
                    strcat(drone_state_error, ":10");
                    strcat(d_state_err, ":10");
                }
                else if (mode != SN_GPS_POS_HOLD_MODE)
                {
                    drone_state = DroneState::MODE_ERROR;
                    strcat(drone_state_error, ":11");
                    strcat(d_state_err, ":11");
                }
            }

            DEBUG("[%d] drone_state_error=%s\n", loop_counter, drone_state_error);

            /*
            if (state == MissionState::STARTING_PROPS)
            {
                switchFan(false);
            }
            else
            {
                switchFan(true);
            }
            */

            /* **********************************/
            /* *******Led light control**********/
            /* **********************************/
#ifdef  AUTO_FACE_TAKE_OFF
            if (face_takeoff_flag)
            {
                bNeedLedColorCtl = true;
                if (face_detect)
                {
                    led_color_status = LedColor::LED_COLOR_YELLOW;
                }
                else
                {
                    led_color_status = LedColor::LED_COLOR_BLUE_EX;
                }
            }
#else
            if (face_takeoff_flag && face_detect)
            {
                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_YELLOW;
            }
#endif
            else if(cur_body.handle_gesture == GESTURE_TAKEPHOTO)
            {
                bNeedLedColorCtl = true;
            }
            else if(cur_body.handle_gesture == GESTURE_BACKANDUP)
            {
                cur_body.velocity = cur_body.velocity*0.97f;
            }
            else if(cur_body.handle_gesture == GESTURE_LAND)
            {
                bNeedLedColorCtl = true;

                confirm_land = true;
                state = MissionState::LANDING;

                cur_body.handle_gesture = 0;

                // Send to camera_super stop
                send_gesture_swither_flag = true;
                memset(gesture_swither_buff, 0, DOMAIN_BUFF_SIZE);
                strncpy(gesture_swither_buff, "gsoff", 5);

                // Send to gesture stop
                unsigned short func[32];
                func[0] = SNAV_TASK_STOP_GESTURE;
                length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                             (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                DEBUG("[%d] SNAV_TASK_STOP_GESTURE func=%d, length=%d\n", loop_counter, func[0], length);
            }
#if 0
            else if (((on_ground_flag == 1) && (voltage < 7.40f))
                    || (((/*z_est - z_est_startup*/revise_height) <= 5.0f) && (voltage < 6.90f))
                    || (((/*z_est - z_est_startup*/revise_height) > 5.0f && (/*z_est - z_est_startup*/revise_height) <= 10.0f) && (voltage < 7.0f))
                    || (((/*z_est - z_est_startup*/revise_height) > 10.0f && (/*z_est - z_est_startup*/revise_height) <= 15.0f) && (voltage < 7.1f))
                    || (((/*z_est - z_est_startup*/revise_height) > 15.0f && (/*z_est - z_est_startup*/revise_height) <= 20.0f) && (voltage < 7.2f))
                    || (((/*z_est - z_est_startup*/revise_height) > 20.0f) && (voltage < 7.3f)))
#else
            else if (voltage < low_battery_led_warning)
#endif
            {
                if (voltage < close_fan_battery)
                {
                    switchFan(false);
                }

                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_BLUE;
            }
            /*
            else if (((mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
                    && (mode != SN_GPS_POS_HOLD_MODE)
                    && (mode != SN_THRUST_ANGLE_MODE)
                    && (mode != SN_ALT_HOLD_MODE)
                    && (mode != SN_VIO_POS_HOLD_MODE))
                || (drone_state == DroneState::IMU_ERROR)
                || (drone_state == DroneState::OPTIC_FLOW_ERROR)
                || (drone_state == DroneState::SONAR_ERROR)
                || (drone_state == DroneState::BARO_ERROR)
                || (drone_state == DroneState::GPS_ERROR)
                || (drone_state == DroneState::MAG_ERROR)
                || (drone_state == DroneState::MOTOR_ERROR)
                || (drone_state == DroneState::CPU_OVER_HEAT))
            {
                bNeedLedColorCtl = true;

                if ((mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
                    && (revise_height < fMaxOFHeight))
                {
                    led_color_status = LedColor::LED_COLOR_GREEN;
                }
                else
                {
                    led_color_status = LedColor::LED_COLOR_RED;
                }
            }
            */
            else if ((mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
                    && (mode != SN_GPS_POS_HOLD_MODE)
                    && (mode != SN_THRUST_ANGLE_MODE)
                    && (mode != SN_ALT_HOLD_MODE)
                    && (mode != SN_VIO_POS_HOLD_MODE))
            {
                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_RED;
            }
            else if ((drone_state == DroneState::IMU_ERROR)
                    || (drone_state == DroneState::OPTIC_FLOW_ERROR)
                    || (drone_state == DroneState::SONAR_ERROR)
                    || (drone_state == DroneState::BARO_ERROR)
                    || (drone_state == DroneState::GPS_ERROR)
                    || (drone_state == DroneState::MAG_ERROR)
                    || (drone_state == DroneState::MOTOR_ERROR)
                    || (drone_state == DroneState::CPU_OVER_HEAT))
            {
                bNeedLedColorCtl = true;

                if ((drone_state == DroneState::GPS_ERROR)
                    && ((mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
                        || (mode == SN_VIO_POS_HOLD_MODE))
                    && (revise_height < fMaxOFHeight))
                {
                    if (bHaveUdpClient)
                    {
                        led_color_status = LedColor::LED_COLOR_GREEN;
                    }
                    else
                    {
                        led_color_status = LedColor::LED_COLOR_YELLOW;
                    }
                }
                else
                {
                    led_color_status = LedColor::LED_COLOR_RED;
                }
            }
            else if ((cur_body.have_face && face_follow_switch)
                    || (cur_body.have_body && body_follow_switch))
            {
                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_GREEN;
            }
            else if (bHaveUdpClient)
            {
                bNeedLedColorCtl = true;

                /*
                if (on_ground_flag == 1)
                {
                    led_color_status = LedColor::LED_COLOR_GREEN;
                }
                else
                {
                    led_color_status = LedColor::LED_COLOR_WHITE;
                }
                */

                led_color_status = LedColor::LED_COLOR_GREEN;
            }
            else
            {
                //bNeedLedColorCtl = false;
                bNeedLedColorCtl = true;
                led_color_status = LedColor::LED_COLOR_YELLOW;
            }
            DEBUG("[%d] bNeedLedColorCtl:%d, led_color_status:%d\n", loop_counter, bNeedLedColorCtl, led_color_status);

#ifdef  LOW_BATTERY_AUTO_LANDING
            // Low battery force landing
            if ((outdoor_mode == 0)
                && (props_state == SN_PROPS_STATE_SPINNING)
                && (voltage < force_landing_battery_indoor)
                && (state == MissionState::LOITER
                    || state == MissionState::IN_MOTION))
            {
                confirm_land = true;
                state = MissionState::LANDING;
            }
            else if ((outdoor_mode == 1)
                && (props_state == SN_PROPS_STATE_SPINNING)
                && (voltage < force_landing_battery_outdoor)
                && (state == MissionState::LOITER
                    || state == MissionState::IN_MOTION))
            {
                confirm_land = true;
                state = MissionState::LANDING;
            }
#endif

            /*
            if (((state == MissionState::STARTING_PROPS) || (state == MissionState::TAKEOFF))
                 && (mode != SN_GPS_POS_HOLD_MODE)
                        && (mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
                        && (mode != SN_VIO_POS_HOLD_MODE))
            {
                confirm_land = true;
                state = MissionState::LANDING;
            }
            */

            if (state != MissionState::LANDING)
            {
                landing_near_ground = false;
            }

            // Handle the udp msg received from the client
            string recv_udp_cmd;
            vector<string> udp_msg_array;

            // Handle the udp msg start
            if (bReceivedUdpMsg)
            {
                recv_udp_cmd = udp_buff_data;
                udp_msg_array = split(recv_udp_cmd, STR_SEPARATOR);

                if ((state == MissionState::LANDING) && (on_ground_flag == 1))
                {
                    // Ignore the control msg when langding and the drone is on gounrd.
                }
                else if ((udp_msg_array.size() >= 6) && (udp_msg_array[0].compare(SNAV_CMD_CONROL) == 0))
                {
                    struct timeval tv_tmp;
                    gettimeofday(&tv_tmp, NULL);
                    double t_current = tv_tmp.tv_sec + tv_tmp.tv_usec*1e-6;


                    DEBUG("[%d] ir_brake control [distance  t_diff][%f, %lf]-------------[t_current, t_ir_brake = [%lf, %lf]\n",
                                        loop_counter, ir_distance, (t_current - t_ir_brake), t_current, t_ir_brake);

                    // If any valid control msg have been received from the client
                    // Not the loiter msg or other task msg
                    if (!((udp_msg_array[1].compare("0")==0)
                        && (udp_msg_array[2].compare("0")==0)
                        && (udp_msg_array[3].compare("0")==0)
                        && (udp_msg_array[4].compare("500")==0)
                        && (udp_msg_array[5].compare("0")==0)))
                    {
                        // Reset the state when get udp control msg
                        current_position =0;

                        fly_test_mission = false;
                        rotation_test_mission = false;

                        circle_mission = false;
                        calcCirclePoint = false;

                        panorama_mission = false;
                        calcPanoramaPoint = false;

                        trail_navigation_mission = false;
                        customized_plan_mission = false;
                        calcPlanPoint = false;

                        return_mission = false;

                        face_mission = false;
                        body_mission = false;

                        face_follow_switch = false;
                        //body_follow_switch = false;

                        // For snav control
                        int rolli = -1;
                        int pitchi = -1;
                        int yawi = -1;
                        int thrusti = -1;
                        int buttons = -1;

                        bool land_cmd_flag = false;

                        const float kMin = -1;
                        const float kMax = 1;

                        float cmd0 = 0;
                        float cmd1 = 0;
                        float cmd2 = 0;
                        float cmd3 = 0;

                        rolli   = atoi(udp_msg_array[1].c_str());
                        pitchi  = atoi(udp_msg_array[2].c_str());
                        yawi    = atoi(udp_msg_array[3].c_str());
                        thrusti = atoi(udp_msg_array[4].c_str());
                        buttons = atoi(udp_msg_array[5].c_str());

                        DEBUG("[%d] SNAV_CMD_CONROL rolli, pitchi, yawi, thrusti, buttons: %d, %d, %d, %d, %d\n",
                                      loop_counter, rolli, pitchi, yawi, thrusti, buttons);

                        if (props_state == SN_PROPS_STATE_SPINNING)
                        {
                            if (buttons == 1)   //landing
                            {
                                land_cmd_flag = true;
                            }
                            else
                            {
                                land_cmd_flag = false;
                            }

                            cmd0 = -((float)(pitchi+441)*(kMax-kMin)/882.+ kMin);
                            cmd1 = -((float)(rolli+441)*(kMax-kMin)/882.+ kMin);
                            cmd2 = (float)(thrusti)*(kMax-kMin)/1000.+ kMin;
                            cmd3 = -((float)(yawi+250)*(kMax-kMin)/500.+ kMin);

                            if ((use_infrared == 1) && ((t_current - t_ir_brake) < 2) && cmd0 > 0)
                            {
                                cmd0 = 0;
                                cmd1 = 0;
                                cmd2 = 0;
                                cmd3 = 0;
                            }

                            if (land_cmd_flag)
                            {
                                // If user touches roll/pitch stick, stop landing
                                if (fabs(cmd0) > 1e-4 || fabs(cmd1) > 1e-4)
                                {
                                    land_cmd_flag = false;
                                }
                                else
                                {
                                    cmd0 = 0;
                                    cmd1 = 0;
                                    cmd2 = -0.5;
                                    cmd3 = 0;
                                }
                            }
#ifdef ZZG_DEBUG_FLAG
#ifdef ZZG_CMD_DEBUG_FLAG
                            if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
                            {
                                char csv_info[TMP_BUFF_LEN];
                                memset(csv_info, 0, sizeof(csv_info));
                                sprintf(csv_info, "%d,%f,%f,%f,%f,%f,%lf\n", loop_counter,cmd0,cmd1,cmd2,cmd3,ir_distance,t_now_for_gps);
                                fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                                fclose(fp_zzg_debug_log);

                            }
#elif defined ZZG_SONAR_DEBUG_FLAG
                            if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
                            {
                                char csv_info[TMP_BUFF_LEN];
                                memset(csv_info, 0, sizeof(csv_info));
                                sprintf(csv_info, "%d,%f,%lf\n", loop_counter, snav_data->sonar_0_raw.range, t_now_for_gps);
                                fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                                fclose(fp_zzg_debug_log);

                            }
#endif
#endif

                            DEBUG("[%d] debug_flag control origin [cmd0,cmd1,cmd2,cmd3]: [%f,%f,%f,%f]\n"
                                        , loop_counter, cmd0, cmd1, cmd2, cmd3);
#ifdef SPEED_LIMIT_FLAG
                            if (face_mission == true || body_mission == true)
                            {
                                cmd0 = cmd0*speed_coefficient;
                                cmd1 = cmd1*speed_coefficient;
                            }
                            else
                            {
                                if (revise_height <= 5.0f)
                                {
                                    //avoid the optic flow error
                                    if ((revise_height <= 1.0f) || snav_data->sonar_0_raw.range <= 1.0f)
                                    {
                                        cmd0 = cmd0*0.5f*speed_coefficient;
                                        cmd1 = cmd1*0.5f*speed_coefficient;
                                    }
                                    else
                                    {
                                        if (circle_mission || return_mission || fly_test_mission || rotation_test_mission)
                                        {
                                            cmd0 = cmd0*speed_coefficient;
                                            cmd1 = cmd1*speed_coefficient;
                                        }
                                        else
                                        {
                                            cmd0 = cmd0*speed_coefficient;
                                            cmd1 = cmd1*speed_coefficient;
                                        }
                                    }

                                    if (mode != SN_GPS_POS_HOLD_MODE)
                                    {
                                        cmd0 = cmd0*0.6;
                                        cmd1 = cmd1*0.6;
                                    }
                                }
                                else
                                {
                                    // Slow down when height grow
                                    cmd0 = cmd0*speed_coefficient*0.25f*(20.0/revise_height);
                                    cmd1 = cmd1*speed_coefficient*0.25f*(20.0/revise_height);
                                }

                            }
#endif
                            DEBUG("[%d] debug_flag control sn_send_rc_command before [cmd0,cmd1,cmd2,cmd3]: [%f,%f,%f,%f], reverse_ctrl_count=%d\n",
                                                loop_counter, cmd0, cmd1, cmd2, cmd3, reverse_ctrl_count);

#ifdef HEIGHT_LIMIT
                            // Limit the height
                            if (gps_enabled
                                && (gps_status == SN_DATA_VALID)
                                && ((t_now_for_gps - t_gps_invalid) > time_interval_of_gps_valid)
                                /*&& ((t_des_now - t_gps_height_invalid) > time_interval_of_gps_valid)*/)
                            {
                                height_limit = fConstHeightLimit;
                            }
                            else if (mode == SN_VIO_POS_HOLD_MODE)
                            {
                                height_limit = 10;
                            }
                            else
                            {
                                height_limit = 5;
                            }


                            if ((revise_height >= height_limit) && (cmd2 > 0))
                            {
                                DEBUG("[%d] The drone have reached the limit height.\n", loop_counter);

                                cmd2 = 0;

                                memset(result_to_client,0,MAX_BUFF_LEN);
                                memcpy(result_to_client, SNAV_INFO_OVER_SAFE_HEIGHT, MAX_BUFF_LEN);
                                length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                            }
#endif

#ifdef LOW_HEIGHT_LIMIT_VEL
                            // Limit the height
                            if ((revise_height <= 1) && snav_data->sonar_0_raw.range <= 1.0f)
                            {
                                DEBUG("[%d] The drone have reached the limit height.\n", loop_counter);

                                cmd0 = 0;
                                cmd1 = 0;

                                // Reverse the drone when speed is over the low-height-limit
                                if (mode == SN_GPS_POS_HOLD_MODE)
                                {
                                    // Use gps data
                                    float current_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                            + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);

                                    float current_vel_x_est = snav_data->gps_pos_vel.velocity_estimated[0];
                                    float current_vel_y_est = snav_data->gps_pos_vel.velocity_estimated[1];

                                    if (fabs(current_vel_est) > hover_vel_limit)
                                    {
                                        float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est_gps) - current_vel_y_est*sin(-yaw_est_gps);
                                        float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est_gps) + current_vel_y_est*cos(-yaw_est_gps);

                                        cmd0 = -hover_brake_limit*current_vel_x_yawed;
                                        cmd1 = -hover_brake_limit*current_vel_y_yawed;

                                        cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                                        cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                                        DEBUG("[%d] OPTIC_FLOW FORMAL_OVER_SPEED[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                                  loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                                  current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                                    }
                                }
                                else
                                {
                                    // Use optic_flow data
                                    float current_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                                + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                                    float current_vel_x_est = snav_data->optic_flow_pos_vel.velocity_estimated[0];
                                    float current_vel_y_est = snav_data->optic_flow_pos_vel.velocity_estimated[1];

                                    if (fabs(current_vel_est) > hover_vel_limit)
                                    {
                                        float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est) - current_vel_y_est*sin(-yaw_est);
                                        float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est) + current_vel_y_est*cos(-yaw_est);

                                        cmd0 = -hover_brake_limit*current_vel_x_yawed;
                                        cmd1 = -hover_brake_limit*current_vel_y_yawed;

                                        cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                                        cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                                        DEBUG("[%d] OPTIC_FLOW FORMAL_OVER_SPEED[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                                  loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                                  current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                                    }
                                }
                            }
#endif

                            // Add by wlh-----------limit the speed to make the drone fly smoothly start
                            /*
                            if (snav_data->general_status.on_ground == 0
                                && snav_data->sonar_0_raw.range > 0.30
                                && (cmd0 !=0 || cmd1 !=0)
                                && reverse_ctrl_count == 0)
                            */
#ifdef VEL_LIMIT_FLAG
                            {
                                DEBUG("[%d] Enter control recal cmd.\n", loop_counter);

                                float now_vel_est = 0;

                                if (mode == SN_GPS_POS_HOLD_MODE)
                                {
                                    now_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                        + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);
                                    if (now_vel_est > VEL_LINEAR_LIMIT_GPS_MACRO(revise_height))
                                    {
                                        DEBUG("[%d] control recal before session 1 now_vel_est, vel_gps_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_GPS_MACRO(revise_height), cmd0, cmd1);

                                        cmd0 = cmd0*fabs(VEL_LINEAR_LIMIT_GPS_MACRO(revise_height)/now_vel_est);
                                        cmd1 = cmd1*fabs(VEL_LINEAR_LIMIT_GPS_MACRO(revise_height)/now_vel_est);

                                        DEBUG("[%d] control recal after session 1 now_vel_est, vel_gps_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_GPS_MACRO(revise_height), cmd0, cmd1);
                                    }
                                }

                                if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
                                {
                                    now_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                        + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                                    if (now_vel_est > VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height))
                                    {
                                        DEBUG("[%d] control recal before session 1 now_vel_est, vel_optic_flow_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height), cmd0, cmd1);

                                        cmd0 = cmd0*fabs(VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height)/now_vel_est);
                                        cmd1 = cmd1*fabs(VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height)/now_vel_est);

                                        DEBUG("[%d] control recal after session 1 now_vel_est, vel_optic_flow_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height), cmd0, cmd1);
                                    }

                                    float sqrt_pitch_roll = sqrt(snav_data->attitude_estimate.pitch*snav_data->attitude_estimate.pitch
                                                                 + snav_data->attitude_estimate.roll*snav_data->attitude_estimate.roll);
                                    if (sqrt_pitch_roll > go_pitch_roll_limit)
                                    {
                                        DEBUG("[%d] control recal before session 1 sqrt_pitch_roll, go_pitch_roll_limit: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                                    loop_counter, sqrt_pitch_roll, go_pitch_roll_limit, cmd0, cmd1);

                                        cmd0 = cmd0*fabs(go_pitch_roll_limit/sqrt_pitch_roll);
                                        cmd1 = cmd1*fabs(go_pitch_roll_limit/sqrt_pitch_roll);

                                        DEBUG("[%d] control recal after session 1 sqrt_pitch_roll, go_pitch_roll_limit: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                                    loop_counter, sqrt_pitch_roll, go_pitch_roll_limit, cmd0, cmd1);
                                    }
                                }

                                // cmd limit
                                float speed_level = 0.6;
                                float cmd_mag = sqrt(cmd0*cmd0 + cmd1*cmd1);

                                cmd_mag = sqrt(cmd0*cmd0 + cmd1*cmd1);
                                if (cmd_mag > 1.414)
                                {
                                    DEBUG("[%d] control recal before session 2 cmd0,cmd1:[%f, %f], cmd_mag:[%f].\n",
                                                loop_counter, cmd0, cmd1, cmd_mag);

                                    cmd0 = cmd0*(speed_level/cmd_mag);
                                    cmd1 = cmd1*(speed_level/cmd_mag);

                                    DEBUG("[%d] control recal after session 2 cmd0,cmd1:[%f, %f].\n",
                                                loop_counter, cmd0, cmd1);
                                }

                                // last cmd limit
                                float last_cmd_mag = sqrt(last_cmd0*last_cmd0 + last_cmd1*last_cmd1);

                                if (fabs(cmd_mag - last_cmd_mag) > go_cmd_offset_limit
                                    || fabs(cmd0 - last_cmd0) > go_cmd_offset_limit
                                    || fabs(cmd1 - last_cmd1) > go_cmd_offset_limit)
                                {
                                    DEBUG("[%d] control recal before session 3 cmd_mag, last_cmd_mag:[%f, %f], cmd0, lastcmd0:[%f, %f], cmd1, last_cmd1:[%f, %f].\n",
                                                loop_counter, cmd_mag, last_cmd_mag, cmd0, last_cmd0, cmd1, last_cmd1);

                                    DEBUG("[%d] control recal before session 3 cmd0,cmd1:[%f, %f].\n", loop_counter, cmd0, cmd1);

                                    float p_bate_offset_cmd0 = fabs(cmd0 - last_cmd0);
                                    float p_bate_offset_cmd1 = fabs(cmd1 - last_cmd1);

                                    if (p_bate_offset_cmd0 > 0)
                                    {
                                        cmd0 = last_cmd0+(cmd0 - last_cmd0)*go_cmd_offset_limit/p_bate_offset_cmd0;
                                    }

                                    if (p_bate_offset_cmd1 > 0)
                                    {
                                        cmd1 = last_cmd1+(cmd1 - last_cmd1)*go_cmd_offset_limit/p_bate_offset_cmd1;
                                    }

                                    DEBUG("[%d] control recal after session 3 cmd0,cmd1:[%f, %f].\n", loop_counter, cmd0, cmd1);
                                }

                                // cmd check again
                                cmd_mag = sqrt(cmd0*cmd0 + cmd1*cmd1);
                                if(cmd_mag > 1.414)
                                {
                                    DEBUG("[%d] control recal before session 4 cmd0,cmd1:[%f, %f], cmd_mag:[%f].\n",
                                                loop_counter, cmd0, cmd1, cmd_mag);

                                    cmd0 = cmd0*(speed_level/cmd_mag);
                                    cmd1 = cmd1*(speed_level/cmd_mag);

                                    DEBUG("[%d] control recal before session 4 cmd0,cmd1:[%f, %f].\n", loop_counter, cmd0, cmd1);
                                }
                            }
                            // Add end-----------limit the speed to make the drone fly smoothly end
#endif


                            cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxCmdValue);
                            cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxCmdValue);

                            // Limit the minimal z_vel to -0.6
                            if (cmd2 < -fMaxCmdValue)
                            {
                                cmd2 = -fMaxCmdValue;
                            }

                            cmd3 = cmd3*yaw_coefficient;

#ifdef NO_FLY_ZONE
                            if (gps_enabled && (gps_status == SN_DATA_VALID))
                            {
                                float lat = snav_data->gps_0_raw.latitude;
                                float lng = snav_data->gps_0_raw.longitude;

                                if (isInNoFlyZone(lat, lng))
                                {
                                    DEBUG("You are near the No-Fly-Zone!!!");

                                    memset(result_to_client, 0, MAX_BUFF_LEN);
                                    sprintf(result_to_client, "%s", SNAV_WARNING_NO_FLY_ZONE);

                                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                                    DEBUG("[%d] SNAV_WARNING_NO_FLY_ZONE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                                }
                            }
#endif
                            if (use_infrared == 1)
                            {
                                float current_vel_est = 0;

                                if (mode == SN_GPS_POS_HOLD_MODE)
                                {
                                    // Use gps data
                                    current_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                       + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);
                                }

                                else
                                {
                                    // Use optic_flow data
                                    current_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                       + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                                }

                                if (fabs(current_vel_est) > IR_VEL_FLAG)
                                {
                                    ir_safe_distance = IR_FORBID_HIGH_VEL;
                                }
                                else
                                {
                                    ir_safe_distance = IR_FORBID_LOW_VEL;
                                }
                            }


                            if ((use_infrared == 1)
                                && (ir_distance > 0)
                                && (ir_distance <= 2*ir_safe_distance))
                            {
                                struct timeval tv_tmp;
                                gettimeofday(&tv_tmp, NULL);
                                double t_current = tv_tmp.tv_sec + tv_tmp.tv_usec*1e-6;

                                if (cmd0 > 0)
                                {
                                    if (mode == SN_GPS_POS_HOLD_MODE)
                                    {
                                        // Use gps data
                                        float current_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                                + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);

                                        float current_vel_x_est = snav_data->gps_pos_vel.velocity_estimated[0];
                                        float current_vel_y_est = snav_data->gps_pos_vel.velocity_estimated[1];

                                        if (fabs(current_vel_est) > IR_VEL_LIMIT)
                                        {
                                            float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est_gps) - current_vel_y_est*sin(-yaw_est_gps);
                                            float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est_gps) + current_vel_y_est*cos(-yaw_est_gps);

                                            cmd0 = -IR_BRAKE_CMD*current_vel_x_yawed;
                                            cmd1 = -IR_BRAKE_CMD*current_vel_y_yawed;

                                            cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                                            cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                                            DEBUG("[%d] GPS_HOLD FORMAL_OVER_SPEED[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                                      loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                                      current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);

                                            DEBUG("[%d] GPS_HOLD INFRAED_DEBUG Control Brake[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                                        loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                                        current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                                        }
                                        else
                                        {
                                            if (ir_distance <= ir_safe_distance)
                                            {
                                                DEBUG("[%d] GPS_HOLD INFRAED_DEBUG Control Forbidden cmd0 and cmd1.\n", loop_counter);
                                                cmd0 = 0;
                                                cmd1 = 0;
                                            }
                                            else
                                            {
                                                DEBUG("[%d] GPS_HOLD INFRAED_DEBUG Control Forbidden cmd0 only.\n", loop_counter);
                                                cmd0 = 0;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        // Use optic_flow data
                                        float current_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                                    + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                                        float current_vel_x_est = snav_data->optic_flow_pos_vel.velocity_estimated[0];
                                        float current_vel_y_est = snav_data->optic_flow_pos_vel.velocity_estimated[1];

                                        if (fabs(current_vel_est) > IR_VEL_LIMIT)
                                        {
                                            float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est) - current_vel_y_est*sin(-yaw_est);
                                            float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est) + current_vel_y_est*cos(-yaw_est);

                                            cmd0 = -IR_BRAKE_CMD*current_vel_x_yawed;
                                            cmd1 = -IR_BRAKE_CMD*current_vel_y_yawed;

                                            cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                                            cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                                            DEBUG("[%d] OPTIC_FLOW INFRAED_DEBUG Control Brake[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                                        loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                                        current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                                        }
                                        else
                                        {
                                            if (ir_distance <= ir_safe_distance)
                                            {
                                                DEBUG("[%d] OPTIC_FLOW INFRAED_DEBUG Control Forbidden cmd0 and cmd1.\n", loop_counter);
                                                cmd0 = 0;
                                                cmd1 = 0;
                                            }
                                            else
                                            {
                                                DEBUG("[%d] OPTIC_FLOW INFRAED_DEBUG Control Forbidden cmd0 only.\n", loop_counter);
                                                cmd0 = 0;
                                            }
                                        }
                                    }

                                    t_ir_brake = t_current;
                                }
                                else if (ir_distance <= ir_safe_distance)
                                {
                                    cmd1 = 0;
                                }

#ifdef OPTIC_FLOW_CMD_LIMIT
                                if ((mode == SN_OPTIC_FLOW_POS_HOLD_MODE) && (revise_height >= fMaxOFHeight))
                                {
                                    cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxOFCmdValue);
                                    cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxOFCmdValue);
                                }
#endif

#ifdef ALT_CMD_LIMIT
                                if ((mode != SN_GPS_POS_HOLD_MODE)
                                    && (mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
                                    && (mode != SN_VIO_POS_HOLD_MODE))
                                {
                                    cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxAltCmdValue);
                                    cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxAltCmdValue);
                                }
#endif

                                if (gps_enabled && mag_usable
                                    && (gps_status == SN_DATA_VALID)
                                    && ((t_now_for_gps - t_gps_invalid) > time_interval_of_gps_valid)
                                    && ((t_des_now - t_gps_height_invalid) > time_interval_of_gps_valid))
                                {
                                    if (revise_height >= gps_mode_height)
                                    {
                                        sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                    }
                                    else
                                    {
                                        if ((sonar_status == SN_DATA_VALID)
                                             && (snav_data->sonar_0_raw.range < sonar_valid_data_max)
                                             && ((t_des_now - t_sonar_invalid) > time_interval_of_sonar_valid))
                                        {
#ifdef VIO_OPTIC_FLOW_SWITCH
                                            if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                                            {
                                                sn_send_rc_command(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                            }
                                            else
#endif
                                            {
                                                sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                            }
                                        }
                                        else
                                        {
                                            sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                        }
                                    }

                                }
                                else
                                {
#ifdef VIO_OPTIC_FLOW_SWITCH
                                    if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                                    {
                                        sn_send_rc_command(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                    }
                                    else
#endif
                                    {
                                        sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                    }
                                }
                            }
                            else
                            {
                                if (gps_enabled && mag_usable
                                    && (gps_status == SN_DATA_VALID)
                                    && ((t_now_for_gps - t_gps_invalid) > time_interval_of_gps_valid)
                                    && ((t_des_now - t_gps_height_invalid) > time_interval_of_gps_valid))
                                {
                                    if (revise_height >= gps_mode_height)
                                    {
                                        sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                    }
                                    else
                                    {
                                        if ((sonar_status == SN_DATA_VALID)
                                             && (snav_data->sonar_0_raw.range < sonar_valid_data_max)
                                             && ((t_des_now - t_sonar_invalid) > time_interval_of_sonar_valid))
                                        {
#ifdef VIO_OPTIC_FLOW_SWITCH
                                            if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                                            {
                                                sn_send_rc_command(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                            }
                                            else
#endif
                                            {
                                                sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                            }
                                        }
                                        else
                                        {
                                            sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                        }
                                    }

                                }
                                else
                                {
#ifdef VIO_OPTIC_FLOW_SWITCH
                                    if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                                    {
                                        sn_send_rc_command(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                    }
                                    else
#endif
                                    {
                                        sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                                    }
                                }

                            }

#ifdef ZZG_DEBUG_FLAG
#ifdef ZZG_FINAL_CMD_FLAG
                            if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
                            {
                                char csv_info[TMP_BUFF_LEN];
                                memset(csv_info, 0, sizeof(csv_info));
                                sprintf(csv_info, "%d,%f,%f,%f,%f,%lf\n", loop_counter,cmd0,cmd1,cmd2,cmd3,t_now_for_gps);
                                fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                                fclose(fp_zzg_debug_log);

                            }
#endif
#endif

                            // Add by wlh
                            last_cmd0 = cmd0;
                            last_cmd1 = cmd1;
                            // Add end

                            last_mode = (SnMode)snav_data->general_status.current_mode;

                            state = MissionState::LOITER;


                            /* ***************************************/
                            /* *************Debug Message*************/
                            /* ***************************************/
                            struct timeval tv_now_tmp;
                            gettimeofday(&tv_now_tmp, NULL);
                            double time_now_tmp = tv_now_tmp.tv_sec + tv_now_tmp.tv_usec * 1e-6;

                            DEBUG("[%d] debug_flag control time=%lf [x_est-start,y_est-start,z_est-start,y_est-start]: [%f,%f,%f,%f]\n",
                                        loop_counter,
                                        time_now_tmp,
                                        x_est-x_est_startup,
                                        y_est-y_est_startup,
                                        z_est-z_est_startup,
                                        yaw_est-yaw_est_startup);


                            DEBUG("[%d] debug_flag control sn_send_rc_command final [cmd0,cmd1,cmd2,cmd3]: [%f,%f,%f,%f]\n",
                                        loop_counter, cmd0, cmd1, cmd2, cmd3);

                            DEBUG("[%d] debug_flag control sn_send_rc_command timestamp: %" PRId64 "\n",
                                        loop_counter, snav_data->esc_raw.time);
                        }

#ifdef CSV_LOG_FOR_XIAOYI
                        if (((state == MissionState::LOITER) || (state == MissionState::IN_MOTION))
                             && ((fp_csv_log = fopen(csv_log_filename, "a+")) != NULL))
                        {
                            int index = 0;
                            int32_t latitude = 0;
                            int32_t longitude = 0;
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            float roll = 0;
                            float pitch = 0;
                            float yaw = 0;
                            int f_state = 0;
                            float velocity;
                            float battery;
                            double time;

                            index = loop_counter;
                            latitude = snav_data->gps_0_raw.latitude;
                            longitude = snav_data->gps_0_raw.longitude;

                            if (state == MissionState::ON_GROUND)
                            {
                                x = 0;
                                y = 0;
                                z = 0;
                            }
                            else
                            {
                                if (gps_enabled && (gps_status == SN_DATA_VALID) && (take_off_with_gps_valid))
                                {
                                    x = x_est_gps - x_est_gps_startup;
                                    y = y_est_gps - y_est_gps_startup;
                                    z = revise_height;
                                }
                                /*
                                else if (mode == SN_VIO_POS_HOLD_MODE)
                                {
                                    x = x_est_vio - x_est_vio_startup;
                                    y = y_est_vio - y_est_vio_startup;
                                    z = revise_height;
                                }*/
                                else
                                {
                                    x = x_est - x_est_startup;
                                    y = y_est - y_est_startup;
                                    z = revise_height;
                                }
                            }

                            roll = snav_data->attitude_estimate.roll;
                            pitch = snav_data->attitude_estimate.pitch;
                            yaw = snav_data->attitude_estimate.yaw;

                            f_state = (int)state;

                            if (gps_enabled && (gps_status == SN_DATA_VALID))
                            {
                                float vel_realtime = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                         + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);
                                velocity = vel_realtime;
                            }
                            else
                            {
                                float vel_realtime = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                         + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                                velocity = vel_realtime;
                            }
                            battery = voltage;
                            time = t_des_now;


                            char csv_info[MAX_BUFF_LEN];
                            memset(csv_info, 0, TMP_BUFF_LEN);
                            if (gps_enabled != 1)
                            {
                                sprintf(csv_info, "%d,null,null,%f,%f,%f,%f,%f,%f,%d,%s,%f,%f,%lf\n",
                                                                index,
                                                                x,
                                                                y,
                                                                z,
                                                                roll,
                                                                pitch,
                                                                yaw,
                                                                f_state,
                                                                d_state_err,
                                                                velocity,
                                                                battery,
                                                                time);
                            }
                            else
                            {
                                sprintf(csv_info, "%d,%d,%d,%f,%f,%f,%f,%f,%f,%d,%s,%f,%f,%lf\n",
                                                                index,
                                                                latitude,
                                                                longitude,
                                                                x,
                                                                y,
                                                                z,
                                                                roll,
                                                                pitch,
                                                                yaw,
                                                                f_state,
                                                                d_state_err,
                                                                velocity,
                                                                battery,
                                                                time);
                            }

                            fwrite(csv_info, strlen(csv_info), 1, fp_csv_log);
                            fclose(fp_csv_log);
                        }
#endif

                        loop_counter++;
                        continue;
                    }
                }
                // Other udp msg
                else if (udp_msg_array.size() >= 2 && (udp_msg_array[0].compare(SNAV_TASK_GET_INFO) == 0))
                {
                    // Update current cam_point_direct from client normal report
                    circle_cam_point_direct = atoi(udp_msg_array[1].c_str());
                    if ((circle_cam_point_direct != 1) && (circle_cam_point_direct != -1))
                    {
                        circle_cam_point_direct = 1;
                    }

                    // Update outdoor/indoor mode from client normal report
                    if (udp_msg_array.size() >= 3)
                    {
                        outdoor_mode = atoi(udp_msg_array[2].c_str());
                        if ((outdoor_mode != 0) && (outdoor_mode != 1))
                        {
                            outdoor_mode = 1;
                        }
                    }

                    char battery_info[TMP_BUFF_LEN];
                    char rpm_info[TMP_BUFF_LEN];
                    char sonar_info[TMP_BUFF_LEN];
                    char gps_info[TMP_BUFF_LEN];
                    char satellites_info[TMP_BUFF_LEN];
                    char xyz_info[TMP_BUFF_LEN];
                    char rpy_info[TMP_BUFF_LEN];
                    char flight_state_info[TMP_BUFF_LEN];
                    char drone_state_info[TMP_BUFF_LEN];
                    char mode_info[TMP_BUFF_LEN];
                    char hor_acc_info[TMP_BUFF_LEN];
                    char sample_size_info[TMP_BUFF_LEN];
                    char ir_distance_info[TMP_BUFF_LEN];
                    char velocity_info[TMP_BUFF_LEN];
                    char yaw_info[TMP_BUFF_LEN];
                    char mission_info[TMP_BUFF_LEN];
                    char face_detect_info[TMP_BUFF_LEN];
                    char body_detect_info[TMP_BUFF_LEN];
                    char cpu_temp[TMP_BUFF_LEN];
                    char baro_temp[TMP_BUFF_LEN];
                    char imu_temp[TMP_BUFF_LEN];

                    memset(battery_info, 0, TMP_BUFF_LEN);
                    memset(rpm_info, 0, TMP_BUFF_LEN);
                    memset(sonar_info, 0, TMP_BUFF_LEN);
                    memset(gps_info, 0, TMP_BUFF_LEN);
                    memset(satellites_info, 0, TMP_BUFF_LEN);
                    memset(xyz_info, 0, TMP_BUFF_LEN);
                    memset(rpy_info, 0, TMP_BUFF_LEN);
                    memset(flight_state_info, 0, TMP_BUFF_LEN);
                    memset(drone_state_info, 0, TMP_BUFF_LEN);
                    memset(mode_info, 0, TMP_BUFF_LEN);
                    memset(hor_acc_info, 0, TMP_BUFF_LEN);
                    memset(sample_size_info, 0, TMP_BUFF_LEN);
                    memset(ir_distance_info, 0, TMP_BUFF_LEN);
                    memset(velocity_info, 0, TMP_BUFF_LEN);
                    memset(yaw_info, 0, TMP_BUFF_LEN);
                    memset(mission_info, 0, TMP_BUFF_LEN);
                    memset(face_detect_info, 0, TMP_BUFF_LEN);
                    memset(body_detect_info, 0, TMP_BUFF_LEN);
                    memset(cpu_temp, 0, TMP_BUFF_LEN);
                    memset(baro_temp, 0, TMP_BUFF_LEN);
                    memset(imu_temp, 0, TMP_BUFF_LEN);

                    sprintf(battery_info, "battery_info:%f", voltage);
                    sprintf(rpm_info, "rpm_info:%d:%d:%d:%d", snav_data->esc_raw.rpm[0],
                                                              snav_data->esc_raw.rpm[1],
                                                              snav_data->esc_raw.rpm[2],
                                                              snav_data->esc_raw.rpm[3]);
                    sprintf(sonar_info,"sonar_info:%f",snav_data->sonar_0_raw.range);
                    sprintf(rpy_info, "rpy_info:%f:%f:%f", snav_data->attitude_estimate.roll,
                                                           snav_data->attitude_estimate.pitch,
                                                           snav_data->attitude_estimate.yaw);
                    sprintf(flight_state_info, "flight_state_info:%d", state);
                    sprintf(drone_state_info, "drone_state_info:%d", drone_state);

                    if (gps_enabled != 1)
                    {
                        sprintf(gps_info, "gps_info:disable");
                        sprintf(satellites_info, "satellites:0");
                        sprintf(hor_acc_info, "hor_acc:-1");
                    }
                    else
                    {
                        if (gps_status != SN_DATA_VALID)
                        {
                            sprintf(gps_info, "gps_info:invalid");
                        }
                        else
                        {
                            sprintf(gps_info, "gps_info:%d:%d", snav_data->gps_0_raw.longitude,
                                                                snav_data->gps_0_raw.latitude);
                        }

                        sprintf(satellites_info, "satellites:%d", snav_data->gps_0_raw.num_satellites);

                        sprintf(hor_acc_info, "hor_acc:%.0f", snav_data->gps_0_raw.horizontal_acc);

                        if (NULL != strchr(hor_acc_info, '.'))
                        {
                            sprintf(hor_acc_info, "hor_acc:%.0f", snav_data->gps_0_raw.horizontal_acc*1000);
                        }
                    }

                    if (state == MissionState::ON_GROUND)
                    {
                        sprintf(xyz_info, "xyz_info:0:0:0");
                    }
                    else
                    {
                        if (gps_enabled && (gps_status == SN_DATA_VALID) && (take_off_with_gps_valid))
                        {
                            sprintf(xyz_info, "xyz_info:%f:%f:%f", (x_est_gps - x_est_gps_startup),
                                                                   (y_est_gps - y_est_gps_startup),
                                                                   (/*z_est_gps - z_est_gps_startup*/revise_height));
                        }
                        /*
                        else if (mode == SN_VIO_POS_HOLD_MODE)
                        {
                            sprintf(xyz_info, "xyz_info:%f:%f:%f", (x_est_vio - x_est_vio_startup),
                                                                   (y_est_vio - y_est_vio_startup),
                                                                   (revise_height));
                        }
                        */
                        else
                        {
                            sprintf(xyz_info, "xyz_info:%f:%f:%f", (x_est-x_est_startup),
                                                                   (y_est-y_est_startup),
                                                                   (/*z_est - z_est_startup*/revise_height));
                        }
                    }

                    if (gps_enabled && (gps_status == SN_DATA_VALID))
                    {
                        float vel_realtime = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                 + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);
                        sprintf(velocity_info, "velocity:%f", vel_realtime);

                        float yaw = snav_data->gps_pos_vel.yaw_estimated;

                        if (yaw > M_PI)
                        {
                            yaw = yaw - 2*M_PI;
                        }
                        else if (yaw < -M_PI)
                        {
                            yaw = yaw + 2*M_PI;
                        }

                        sprintf(yaw_info, "yaw:%f", yaw);
                    }
                    else
                    {
                        float vel_realtime = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                 + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                        sprintf(velocity_info, "velocity:%f", vel_realtime);


                        float yaw = snav_data->optic_flow_pos_vel.yaw_estimated;

                        if (yaw > M_PI)
                        {
                            yaw = yaw - 2*M_PI;
                        }
                        else if (yaw < -M_PI)
                        {
                            yaw = yaw + 2*M_PI;
                        }

                        sprintf(yaw_info, "yaw:%f", yaw);
                    }

                    sprintf(mode_info, "mode:%d", mode);
                    sprintf(sample_size_info, "sample_size:%d", sample_size);
                    sprintf(ir_distance_info, "ir_distance:%f", ir_distance);

                    if (circle_mission)
                    {
                        sprintf(mission_info, "mission:1");
                    }
                    else if (face_mission)
                    {
                        sprintf(mission_info, "mission:2");
                    }
                    else if (body_mission)
                    {
                        sprintf(mission_info, "mission:3");
                    }
                    else if (return_mission)
                    {
                        sprintf(mission_info, "mission:4");
                    }
                    else
                    {
                        sprintf(mission_info, "mission:0");
                    }

                    sprintf(face_detect_info, "face_detect:%d:%d:%d", face_follow_switch, face_rotate_switch, cur_body.have_face);
                    sprintf(body_detect_info, "body_detect:%d:%d", body_follow_switch, cur_body.have_body);

                    float cpu_max_temp = 0;
                    cpu_max_temp = cpu_status.temp[0];
                    for (int i = 0; i < 10; i++)
                    {
                        if (cpu_status.temp[i] >= cpu_max_temp)
                        {
                            cpu_max_temp = cpu_status.temp[i];
                        }
                    }
                    sprintf(cpu_temp, "cpu_temp:%.0f", cpu_max_temp);
                    sprintf(baro_temp, "baro_temp:%.0f", snav_data->barometer_0_raw.temp);
                    sprintf(imu_temp, "imu_temp:%.0f", snav_data->imu_0_raw.temp);


                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_INFO_RETURN);

                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, battery_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, rpm_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, sonar_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, gps_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, satellites_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, xyz_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, rpy_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, flight_state_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, drone_state_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, drone_state_error);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, mode_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, hor_acc_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, sample_size_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, ir_distance_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, velocity_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, yaw_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, mission_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, face_detect_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, body_detect_info);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, cpu_temp);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, baro_temp);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, imu_temp);

                    // Sendback to udp client
                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));

                    DEBUG("[%d] SNAV_TASK_GET_INFO_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                    continue;
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SNAV_PROXY_VERSION) == 0))
                {
                    // Check current version print $2,$3
                    char get_version_cmd[TMP_BUFF_LEN] = "dpkg -l " FC_PKG_NAME " | tail -n 1 | awk '{print $3}'";
                    char current_version[TMP_BUFF_LEN];

                    FILE *fp = popen(get_version_cmd, "r");
                    if (fp != NULL)
                    {
                        fgets(current_version, sizeof(current_version), fp);
                    }
                    pclose(fp);

                    if ((strlen(current_version) >= 1) && (current_version[strlen(current_version)-1] == '\n'))
                    {
                        current_version[strlen(current_version)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_SNAV_PROXY_VERSION_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, current_version);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_SNAV_PROXY_VERSION_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_LINARO_VERSION) == 0))
                {
                    /*
                    FILE *version_fp;
                    char expected_version[TMP_BUFF_LEN];

                    if ((version_fp = fopen("/etc/systeminfo.cfg", "r")) != NULL)
                    {
                        int i=0;

                        while (fgets(expected_version, sizeof(expected_version), version_fp) != NULL)
                        {
                            DEBUG("[%d] linaro version:%s\n", loop_counter, expected_version);
                        }

                        fclose(version_fp);
                    }

                    if ((strlen(expected_version) >= 1) && (expected_version[strlen(expected_version)-1] == '\n'))
                    {
                        expected_version[strlen(expected_version)-1] = '\0';
                    }
                    */

                    char expected_version[TMP_BUFF_LEN] = "1.2.0";

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_LINARO_VERSION_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, expected_version);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_LINARO_VERSION_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SNAV_VERSION) == 0))
                {
#ifdef USE_SNAV_DEV
                    //check current version print $2,$3
                    //char get_version_cmd[TMP_BUFF_LEN] = "dpkg -l " SNAV_DEV_PKG_NAME " | tail -n 1 | awk '{print $3}'";
                    char get_version_cmd[TMP_BUFF_LEN] = "dpkg -l " SNAV_OEM_PKG_NAME " | tail -n 1 | awk '{print $3}'";
#else
                    char get_version_cmd[TMP_BUFF_LEN] = "dpkg -l " SNAV_PKG_NAME " | tail -n 1 | awk '{print $3}'";
#endif

                    char current_version[TMP_BUFF_LEN];

                    FILE *fp = popen(get_version_cmd, "r");
                    if (fp != NULL)
                    {
                        fgets(current_version, sizeof(current_version), fp);
                    }
                    pclose(fp);

                    if ((strlen(current_version) >= 1) && (current_version[strlen(current_version)-1] == '\n'))
                    {
                        current_version[strlen(current_version)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_SNAV_VERSION_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, current_version);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_SNAV_VERSION_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_QCAM_VERSION) == 0))
                {
                    //check current version print $2,$3
                    char get_version_cmd[TMP_BUFF_LEN] = "dpkg -l " CAMERA_PKG_NAME " | tail -n 1 | awk '{print $3}'";
                    char current_version[TMP_BUFF_LEN];

                    FILE *fp = popen(get_version_cmd, "r");
                    if (fp != NULL)
                    {
                        fgets(current_version, sizeof(current_version), fp);
                    }
                    pclose(fp);

                    if ((strlen(current_version) >= 1) && (current_version[strlen(current_version)-1] == '\n'))
                    {
                        current_version[strlen(current_version)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_QCAM_VERSION_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, current_version);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_QCAM_VERSION_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_HW_VERSION) == 0))
                {
                    char current_version[TMP_BUFF_LEN] = "FC02-HW-1.1.0";

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_HW_VERSION_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, current_version);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_HW_VERSION_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SN) == 0))
                {
                    char sn_number[TMP_BUFF_LEN] = "43242343251";

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_SN_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, sn_number);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_SN_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_STORAGE) == 0))
                {
                    char get_storage_total[TMP_BUFF_LEN] = "df -h | grep -w '/' | head -n 1 | awk '{print $2}'";
                    char get_storage_free[TMP_BUFF_LEN] = "df -h | grep -w '/' | head -n 1 | awk '{print $4}'";
                    char current_storage_total[TMP_BUFF_LEN];
                    char current_storage_free[TMP_BUFF_LEN];

                    memset(current_storage_total, 0, TMP_BUFF_LEN);
                    memset(current_storage_free, 0, TMP_BUFF_LEN);

                    FILE *fp_total = popen(get_storage_total, "r");
                    if (fp_total != NULL)
                    {
                        fgets(current_storage_total, sizeof(current_storage_total), fp_total);
                    }
                    pclose(fp_total);

                    FILE *fp_free = popen(get_storage_free, "r");
                    if (fp_free != NULL)
                    {
                        fgets(current_storage_free, sizeof(current_storage_free), fp_free);
                    }
                    pclose(fp_free);

                    if ((strlen(current_storage_total) >= 1) && (current_storage_total[strlen(current_storage_total)-1] == '\n'))
                    {
                        current_storage_total[strlen(current_storage_total)-1] = '\0';
                    }

                    if ((strlen(current_storage_free) >= 1) && (current_storage_free[strlen(current_storage_free)-1] == '\n'))
                    {
                        current_storage_free[strlen(current_storage_free)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_STORAGE_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, current_storage_total);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, current_storage_free);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_STORAGE_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SD_STORAGE) == 0))
                {
                    char get_sd_storage_total[TMP_BUFF_LEN] = "df -h | grep -w '/mnt/sdcard' | head -n 1 | awk '{print $2}'";
                    char get_sd_storage_free[TMP_BUFF_LEN] = "df -h | grep -w '/mnt/sdcard' | head -n 1 | awk '{print $4}'";
                    char sd_storage_total[TMP_BUFF_LEN];
                    char sd_storage_free[TMP_BUFF_LEN];


                    memset(sd_storage_total, 0, TMP_BUFF_LEN);
                    memset(sd_storage_free, 0, TMP_BUFF_LEN);

                    FILE *fp_total = popen(get_sd_storage_total, "r");
                    if (fp_total != NULL)
                    {
                        fgets(sd_storage_total, sizeof(sd_storage_total), fp_total);
                    }
                    pclose(fp_total);

                    FILE *fp_free = popen(get_sd_storage_free, "r");
                    if (fp_free != NULL)
                    {
                        fgets(sd_storage_free, sizeof(sd_storage_free), fp_free);
                    }
                    pclose(fp_free);

                    if ((strlen(sd_storage_total) >= 1) && (sd_storage_total[strlen(sd_storage_total)-1] == '\n'))
                    {
                        sd_storage_total[strlen(sd_storage_total)-1] = '\0';
                    }

                    if ((strlen(sd_storage_free) >= 1) && (sd_storage_free[strlen(sd_storage_free)-1] == '\n'))
                    {
                        sd_storage_free[strlen(sd_storage_free)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_SD_STORAGE_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, sd_storage_total);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, sd_storage_free);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_SD_STORAGE_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_GET_SD_STATUS) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_GET_SD_STATUS_RETURN);
                    if (access(/*SDCARD_DIR*/SDCARD_MOUNT_PATH, F_OK) == 0)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "1");
                    }
                    else
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "-1");
                    }

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_GET_SD_STATUS_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_SSID_PWD) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MODIFY_SSID_PWD);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_MODIFY_SSID_PWD result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_CHECK_WIFI_MODE) == 0))
                {
                    char get_current_wifi_mode[TMP_BUFF_LEN] = "cat " WIFI_FILE_NAME " | grep 'hw_mode=g'";

                    char current_wifi_mode[TMP_BUFF_LEN];
                    memset(current_wifi_mode, 0, TMP_BUFF_LEN);

                    FILE *fp_get_mode = popen(get_current_wifi_mode, "r");
                    if (fp_get_mode != NULL)
                    {
                        fgets(current_wifi_mode, sizeof(current_wifi_mode), fp_get_mode);
                    }
                    pclose(fp_get_mode);

                    if ((strlen(current_wifi_mode) >= 1) && (current_wifi_mode[strlen(current_wifi_mode)-1] == '\n'))
                    {
                        current_wifi_mode[strlen(current_wifi_mode)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CHECK_WIFI_MODE);
                    if (strncmp(current_wifi_mode, "#hw_mode=g", 10) == 0)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "5");
                    }
                    else if (strncmp(current_wifi_mode, "hw_mode=g", 9) == 0)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "2");
                    }
                    else    /* by default */
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "2");
                    }

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_CHECK_WIFI_MODE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_5G) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MODIFY_WIFI_5G);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_MODIFY_WIFI_5G result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_2G) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MODIFY_WIFI_2G);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_MODIFY_WIFI_2G result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_CHECK_GPS_STATUS) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CHECK_GPS_STATUS);
                    if (gps_enabled != 1)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "0");
                    }
                    else
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "1");
                    }

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_CHECK_GPS_STATUS result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_OPEN_GPS) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_OPEN_GPS);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_OPEN_GPS result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_CLOSE_GPS) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CLOSE_GPS);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_CLOSE_GPS result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_CHECK_CAM_FREQ) == 0))
                {
                    char get_current_cam_freq[TMP_BUFF_LEN] = "cat " CAM_CFG_FILE_NAME " | grep 'Frequency'";

                    char current_cam_freq[TMP_BUFF_LEN];
                    memset(current_cam_freq, 0, TMP_BUFF_LEN);

                    FILE *fp_get_freq = popen(get_current_cam_freq, "r");
                    if (fp_get_freq != NULL)
                    {
                        fgets(current_cam_freq, sizeof(current_cam_freq), fp_get_freq);
                    }
                    pclose(fp_get_freq);

                    if ((strlen(current_cam_freq) >= 1) && (current_cam_freq[strlen(current_cam_freq)-1] == '\n'))
                    {
                        current_cam_freq[strlen(current_cam_freq)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CHECK_CAM_FREQ);
                    if (strncmp(current_cam_freq, "Frequency=50Hz", 14) == 0)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "0");
                    }
                    else if (strncmp(current_cam_freq, "Frequency=60Hz", 14) == 0)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "1");
                    }
                    else    /* by default */
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "0");
                    }

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_CHECK_CAM_FREQ result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_TAKE_OFF) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);

#ifdef  DISABLE_INFRARED_TAKEOFF
                    if ((use_infrared == 1)
                        && (ir_distance > 0)
                        && (ir_distance <= ir_safe_distance))
                    {
                        DEBUG("[%d] INFRAED_DEBUG Forbidden takeoff.\n", loop_counter);

                        sprintf(result_to_client, "%s", SNAV_WARNING_TAKEOFF_FORBIDDEN);
                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_CMD_RETURN_TAKE_OFF result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);

                        continue;
                    }
                    else
#endif
                    {
                        sprintf(result_to_client, "%s", SNAV_CMD_RETURN_TAKE_OFF);
                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_CMD_RETURN_TAKE_OFF result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                    }
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_FACE_TAKE_OFF_SWITCH) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_FACE_TAKE_OFF_SWITCH);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, udp_msg_array[1].c_str());

                    if (udp_msg_array[1].compare("1") == 0)
                    {
                        face_takeoff_flag = true;
                    }
                    else
                    {
                        face_takeoff_flag = false;
                    }

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_FACE_TAKE_OFF_SWITCH result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_LAND) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_LAND);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_LAND result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 3) && (udp_msg_array[0].compare(SNAV_CMD_CIRCLE) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CIRCLE);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_CIRCLE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_PANORAMA) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_PANORAMA);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, udp_msg_array[1].c_str());

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_PANORAMA result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_MAG_CALIBRATE) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MAG_CALIBRATE);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_MAG_CALIBRATE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_HOR_CALIBRATE) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_HOR_CALIBRATE);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_HOR_CALIBRATE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_OPTIC_FLOW_CALIB) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_OPTIC_FLOW_CALIB);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_OPTIC_FLOW_CALIB result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_FLY_TEST) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_FLY_TEST);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_FLY_TEST result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_ROTATION_TEST) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_ROTATION_TEST);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_ROTATION_TEST result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_RETURN) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_RETURN);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_TRAIL_NAVIGATION);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_TRAIL_NAVIGATION result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                 else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_CMD_CUSTOMIZED_PLAN) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_CUSTOMIZED_PLAN);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_CUSTOMIZED_PLAN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_FACE_FOLLOW);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, udp_msg_array[1].c_str());

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_FACE_FOLLOW result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW_MODE) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_FACE_FOLLOW_MODE);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, udp_msg_array[1].c_str());

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_FACE_FOLLOW_MODE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_BODY_FOLLOW) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_BODY_FOLLOW);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, udp_msg_array[1].c_str());

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_BODY_FOLLOW result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_CMD_GESTURE) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_GESTURE);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, udp_msg_array[1].c_str());

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_GESTURE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare(SNAV_TASK_CONFIRM_LAND) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_CONFIRM_LAND_RETURN);
                    strcat(result_to_client, STR_SEPARATOR);
                    strcat(result_to_client, udp_msg_array[1].c_str());

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_CONFIRM_LAND_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_SNAV_UPDATE) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_SNAV_UPDATE_RETURN);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_SNAV_UPDATE_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                else if ((udp_msg_array.size() >= 1) && (udp_msg_array[0].compare(SNAV_TASK_LINARO_UPDATE) == 0))
                {
                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_TASK_LINARO_UPDATE_RETURN);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_LINARO_UPDATE_RETURN result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
                /* For debugging */
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("speed_limit") == 0))
                {
                    speed_coefficient = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  speed_coefficient=%f\n",speed_coefficient);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("revise_height") == 0))
                {
                    use_revise_height = atoi(udp_msg_array[1].c_str());
                    DEBUG("udp receive  use_revise_height=%d\n",use_revise_height);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("reduce_height") == 0))
                {
                    use_reduce_height = atoi(udp_msg_array[1].c_str());
                    DEBUG("udp receive  use_reduce_height=%d\n",use_reduce_height);
                }
#ifdef HEIGHT_LIMIT
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("height_limit") == 0))
                {
                    height_limit = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  height_limit=%f\n",height_limit);
                }
#endif
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("gps_mode_height") == 0))
                {
                    gps_mode_height = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  gps_mode_height=%f\n",gps_mode_height);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("gps_valid_time") == 0))
                {
                    time_interval_of_gps_valid = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  time_interval_of_gps_valid=%f\n",time_interval_of_gps_valid);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("sonar_valid_time") == 0))
                {
                    time_interval_of_sonar_valid = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  time_interval_of_sonar_valid=%f\n",time_interval_of_sonar_valid);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("plan_unit") == 0))
                {
                    plan_unit = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  plan_unit=%f\n",plan_unit);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("use_infrared") == 0))
                {
                    use_infrared = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  use_infrared=%f\n",use_infrared);
                }
                else if ((udp_msg_array.size() >= 3) && (udp_msg_array[0].compare("200qc") == 0))
                {
                    DEBUG("udp receive  200qc params modify\n");
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("return_dis") == 0))
                {
                    distance_disconn_auto_return = atof(udp_msg_array[1].c_str());
                    DEBUG("udp receive  return_distance=%f\n", distance_disconn_auto_return);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("fan") == 0))
                {
                    int bflag = atoi(udp_msg_array[1].c_str());
                    DEBUG("udp receive  fan=%d\n", bflag);

                    if (bflag == 1)
                    {
                        switchFan(true);
                    }
                    else
                    {
                        switchFan(false);
                    }
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("of_ct") == 0))
                {
                    of_ct = atoi(udp_msg_array[1].c_str());
                    DEBUG("udp receive of_ct=%d\n", of_ct);
                }
                else if ((udp_msg_array.size() >= 2) && (udp_msg_array[0].compare("of_value") == 0))
                {
                    of_valid_value = atoi(udp_msg_array[1].c_str());
                    DEBUG("udp receive of_valid_value=%d\n", of_valid_value);
                }
            }
            // Handle the udp msg end

            struct timeval tv;
            gettimeofday(&tv, NULL);
            double t_now = tv.tv_sec + tv.tv_usec*1e-6;

            // Desired velocity in vehicle world frame
            float x_vel_des = 0;
            float y_vel_des = 0;
            float z_vel_des = 0;
            float yaw_vel_des = 0;

            /**********************************************************************************/
            /**********************************************************************************/
            /**********************Handle the task cmd from udp client*************************/
            /**********************************************************************************/
            /**********************************************************************************/
            // Modify ssid and pwd
            if ((udp_msg_array.size() >= 2)
                && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_SSID_PWD) == 0)
                && (props_state == SN_PROPS_STATE_NOT_SPINNING))
            {
                char ssid[TMP_BUFF_LEN];
                char pwd[TMP_BUFF_LEN];
                char sed_str[TMP_BUFF_LEN];

                memset(ssid, 0, TMP_BUFF_LEN);
                memset(pwd, 0, TMP_BUFF_LEN);
                memset(sed_str, 0, TMP_BUFF_LEN);

                memcpy(ssid, udp_msg_array[1].c_str(), TMP_BUFF_LEN);

                if (udp_msg_array.size() >= 3)
                {
                    memcpy(pwd, udp_msg_array[2].c_str(), TMP_BUFF_LEN);
                    sprintf(sed_str,
                            "sed -i 's/^ssid=.*/ssid=%s/; s/^wpa_passphrase=.*/wpa_passphrase=%s/' " WIFI_FILE_NAME,
                            ssid, pwd);
                }
                else    /* Only modify the ssid */
                {
                    sprintf(sed_str,
                            "sed -i 's/^ssid=.*/ssid=%s/' " WIFI_FILE_NAME,
                            ssid);
                }

                system(sed_str);
                system("chmod 755 " WIFI_FILE_NAME);
                //system("ps -e |grep hostapd |awk '{print $1}'| xargs kill -9");   /* can not make sense */
                system("pkill hostapd");
                sleep(10);  //s
                system("hostapd -B " WIFI_FILE_NAME);
            }

            // Switch wifi mode 5G/2.4G
            if ((udp_msg_array.size() >= 1)
                && ((udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_5G) == 0)
                    || (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_2G) == 0)))
            {
                if (props_state == SN_PROPS_STATE_NOT_SPINNING)
                {
                    if (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_5G) == 0)
                    {
                        system("sed -i 's/^hw_mode=g.*/#hw_mode=g/' " WIFI_FILE_NAME);
                        system("sed -i 's/^channel=0.*/#channel=0 # use ch0 to enable ACS/' " WIFI_FILE_NAME);
                        system("sed -i 's/^#hw_mode=a.*/hw_mode=a/' " WIFI_FILE_NAME);
                        system("sed -i 's/^#channel=165.*/channel=165 # some channel in 5Ghz band/' " WIFI_FILE_NAME);

                        system("chmod 755 " WIFI_FILE_NAME);
                        system("pkill hostapd");
                        sleep(10);
                        system("hostapd -B " WIFI_FILE_NAME);
                    }
                    else if (udp_msg_array[0].compare(SNAV_CMD_MODIFY_WIFI_2G) == 0)
                    {
                        system("sed -i 's/^#hw_mode=g.*/hw_mode=g/' " WIFI_FILE_NAME);
                        system("sed -i 's/^#channel=0.*/channel=0 # use ch0 to enable ACS/' " WIFI_FILE_NAME);
                        system("sed -i 's/^hw_mode=a.*/#hw_mode=a/' " WIFI_FILE_NAME);
                        system("sed -i 's/^channel=165.*/#channel=165 # some channel in 5Ghz band/' " WIFI_FILE_NAME);

                        system("chmod 755 " WIFI_FILE_NAME);
                        system("pkill hostapd");
                        sleep(10);
                        system("hostapd -B " WIFI_FILE_NAME);
                    }
                }
            }

            // params modify
            if ((udp_msg_array.size() >= 3)
                && (udp_msg_array[0].compare("200qc") == 0))
            {
                char item[TMP_BUFF_LEN];
                char value[TMP_BUFF_LEN];
                char sed_str[TMP_BUFF_LEN];

                memset(item, 0, TMP_BUFF_LEN);
                memset(value, 0, TMP_BUFF_LEN);
                memset(sed_str, 0, TMP_BUFF_LEN);

                memcpy(item, udp_msg_array[1].c_str(), TMP_BUFF_LEN);
                memcpy(value, udp_msg_array[2].c_str(), TMP_BUFF_LEN);

                sprintf(sed_str, "sed -i 's@\"%s\".*@\"%s\" value=\"%s\"\\/\\>@g' " SNAV_CFG_FILE_NAME, item, item, value);

                system(sed_str);

                send_restart_snav = true;
                strcpy(ota_restart_snav, "restart_snav");
            }

            // Switch GPS on/off
            if ((udp_msg_array.size() >= 1)
                && ((udp_msg_array[0].compare(SNAV_CMD_OPEN_GPS) == 0)
                    || (udp_msg_array[0].compare(SNAV_CMD_CLOSE_GPS) == 0)))
            {
                if (props_state == SN_PROPS_STATE_NOT_SPINNING)
                {
                    if (udp_msg_array[0].compare(SNAV_CMD_OPEN_GPS) == 0)
                    {
                        //system("sed -i 's/\"gnss0_device_path\".*/\"gnss0_device_path\" value=\"\/dev\/tty-4\"\\/\\>/g' " EAGLE_CFG_FILE_NAME);
                        //system("sed -i 's/\"mag0_device_path\".*/\"mag0_device_path\" value=\"\/dev\/iic-2\"\\/\\>/g' " EAGLE_CFG_FILE_NAME);

                        system("sed -i 's/\"gnss0_device_path\".*/\"gnss0_device_path\" value=\"\\/dev\\/tty-4\"\\/\\>/g' /usr/share/data/adsp/eagle_p2.xml");
                        system("sed -i 's/\"mag0_device_path\".*/\"mag0_device_path\" value=\"\\/dev\\/iic-2\"\\/\\>/g' /usr/share/data/adsp/eagle_p2.xml");

                        system("sed -i 's/\"mag0_driver_file_name\".*/\"mag0_driver_file_name\" value=\"libmag0_hmc5883l_driver.so\"\\/\\>/g' /usr/share/data/adsp/eagle_p2.xml");

                        memset(result_to_client, 0, MAX_BUFF_LEN);
                        sprintf(result_to_client, "%s", SNAV_OPEN_GPS_RESULT);
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "1");

                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
                                            , 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_OPEN_GPS_RESULT result_to_client=%s,length=%d\n", loop_counter, result_to_client,length);

                        send_restart_snav = true;
                        strcpy(ota_restart_snav, "restart_snav");
                        //strcpy(ota_restart_snav, OPEN_GPS);
                    }
                    else if (udp_msg_array[0].compare(SNAV_CMD_CLOSE_GPS) == 0)
                    {
                        //system("sed -i 's/\"gnss0_device_path\".*/\"gnss0_device_path\" value=\"null\"\\/\\>/g' " EAGLE_CFG_FILE_NAME);
                        //system("sed -i 's/\"mag0_device_path\".*/\"mag0_device_path\" value=\"null\"\\/\\>/g' " EAGLE_CFG_FILE_NAME);

                        system("sed -i 's/\"mag0_driver_file_name\".*/\"mag0_driver_file_name\" value=\"null\"\\/\\>/g' /usr/share/data/adsp/eagle_p2.xml");

                        system("sed -i 's/\"gnss0_device_path\".*/\"gnss0_device_path\" value=\"null\"\\/\\>/g' /usr/share/data/adsp/eagle_p2.xml");
                        system("sed -i 's/\"mag0_device_path\".*/\"mag0_device_path\" value=\"null\"\\/\\>/g' /usr/share/data/adsp/eagle_p2.xml");

                        memset(result_to_client, 0, MAX_BUFF_LEN);
                        sprintf(result_to_client, "%s", SNAV_CLOSE_GPS_RESULT);
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "1");

                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
                                            , 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_CLOSE_GPS_RESULT result_to_client=%s,length=%d\n", loop_counter, result_to_client,length);

                        send_restart_snav = true;
                        strcpy(ota_restart_snav, "restart_snav");
                        //strcpy(ota_restart_snav, CLOSE_GPS);
                    }
                }
            }

            // Switch cam freq
            if ((udp_msg_array.size() >= 2)
                && (udp_msg_array[0].compare(SNAV_CMD_MODIFY_CAM_FREQ) == 0))
            {
                if (props_state == SN_PROPS_STATE_NOT_SPINNING)
                {
                    if (udp_msg_array[1].compare("0") == 0)
                    {
                        system("sed -i 's/Frequency=.*/Frequency=50Hz/g' " CAM_CFG_FILE_NAME);
                    }
                    else if (udp_msg_array[1].compare("1") == 0)
                    {
                        system("sed -i 's/Frequency=.*/Frequency=60Hz/g' " CAM_CFG_FILE_NAME);
                    }

                    char get_current_cam_freq[TMP_BUFF_LEN] = "cat " CAM_CFG_FILE_NAME " | grep 'Frequency'";

                    char current_cam_freq[TMP_BUFF_LEN];
                    memset(current_cam_freq, 0, TMP_BUFF_LEN);

                    FILE *fp_get_freq = popen(get_current_cam_freq, "r");
                    if (fp_get_freq != NULL)
                    {
                        fgets(current_cam_freq, sizeof(current_cam_freq), fp_get_freq);
                    }
                    pclose(fp_get_freq);

                    if ((strlen(current_cam_freq) >= 1) && (current_cam_freq[strlen(current_cam_freq)-1] == '\n'))
                    {
                        current_cam_freq[strlen(current_cam_freq)-1] = '\0';
                    }

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_CMD_RETURN_MODIFY_CAM_FREQ);
                    if (strncmp(current_cam_freq, "Frequency=50Hz", 14) == 0)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "0");
                    }
                    else if (strncmp(current_cam_freq, "Frequency=60Hz", 14) == 0)
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "1");
                    }
                    else    /* by default */
                    {
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, "0");
                    }

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
                                        , 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_CMD_RETURN_MODIFY_CAM_FREQ result_to_client=%s,length=%d\n", loop_counter, result_to_client,length);
                }
            }

            // Notice the ThreadInteractWithOtaAndLimitLog process to send msg to OTA app
            if ((udp_msg_array.size() >= 1)
                && (udp_msg_array[0].compare(SNAV_TASK_SNAV_UPDATE) == 0)
                && (props_state == SN_PROPS_STATE_NOT_SPINNING))
            {
                system("chmod 777 " SNAV_UPDATE_FILE_NAME);
                send_ota_snav_flag = true;
                strcpy(ota_snav_path_buff, SNAV_UPDATE_FILE_NAME);
            }

            if ((udp_msg_array.size() >= 1)
                && (udp_msg_array[0].compare(SNAV_TASK_LINARO_UPDATE) == 0)
                && (props_state == SN_PROPS_STATE_NOT_SPINNING))
            {
                system("chmod 777 " LINARO_UPDATE_FILE_NAME);
                send_ota_linaro_flag = true;
                strcpy(ota_linaro_path_buff, LINARO_UPDATE_FILE_NAME);
            }

            // MAG calibrate
            if ((udp_msg_array.size() >= 1)
                && (udp_msg_array[0].compare(SNAV_CMD_MAG_CALIBRATE) == 0)
                && (props_state == SN_PROPS_STATE_NOT_SPINNING))
            {
                bool keep_going = true;
                bool calib_started = false;
                bool calib_result = false;
                unsigned int attempt_number = 0;
                const unsigned int kMaxAttempts = 10;

                while (keep_going)
                {
                    static unsigned int circle_counter = 0;

                    if (sn_update_data() != 0)
                    {
                        DEBUG("[%u] sn_update_data failed in mag_calibrate\n", circle_counter);
                        keep_going = false;
                    }
                    else
                    {
                        if (snav_data->general_status.current_mode == SN_MAGNETOMETER_CALIBRATION_MODE)
                        {
                            calib_started = true;
                            SnCalibStatus status;
                            sn_get_magnetometer_calibration_status(&status);
                            if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
                            {
                                DEBUG("[%u] Magnetometer calibration is in progress\n",circle_counter);
                            }
                        }
                        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
                        {
                            DEBUG("[%u] Magnetometer calibration was completed successfully\n",circle_counter);
                            keep_going = false;
                            calib_result = true;
                        }
                        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
                        {
                            DEBUG("[%u] Magnetometer calibration failed\n", circle_counter);
                            keep_going = false;
                            calib_result = false;
                        }
                        else
                        {
                            if (attempt_number < kMaxAttempts)
                            {
                                DEBUG("[%u] Sending command (attempt %u) to start magnetometer calibration\n",
                                                circle_counter, attempt_number);
                                sn_start_magnetometer_calibration();
                                attempt_number++;
                            }
                            else
                            {
                                DEBUG("[%u] Unable to start calibration\n", circle_counter);
                                keep_going = false;
                            }
                        }

                        circle_counter++;
                        usleep(100000); //100ms

                        if (circle_counter >= 600)
                        {
                            DEBUG("[%u] MAG Calibrate TimeOut calib_result=%d\n", circle_counter, calib_result);
                            keep_going = false;
                        }
                    }
                }

                memset(result_to_client, 0, MAX_BUFF_LEN);
                sprintf(result_to_client, "%s", SNAV_INFO_MAG_CALIBRATE_RESULT);
                strcat(result_to_client, STR_SEPARATOR);
                strcat(result_to_client, calib_result == true? "1":"0");

                length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                DEBUG("[%d] udp sendto SNAV_INFO_MAG_CALIBRATE_RESULT length=%d\n", loop_counter, length);
            }

            // OPTIC_FLOW calibrate
            if ((udp_msg_array.size() >= 1)
                && (udp_msg_array[0].compare(SNAV_CMD_OPTIC_FLOW_CALIB) == 0)
                && (props_state == SN_PROPS_STATE_NOT_SPINNING))
            {
                bool keep_going = true;
                bool calib_started = false;
                bool calib_result = false;
                unsigned int attempt_number = 0;
                const unsigned int kMaxAttempts = 10;

                while (keep_going)
                {
                    static unsigned int circle_counter = 0;

                    if (sn_update_data() != 0)
                    {
                        DEBUG("[%u] sn_update_data failed in accel_calibrate\n", circle_counter);
                        keep_going = false;
                    }
                    else
                    {
                        if (snav_data->general_status.current_mode == SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE)
                        {
                            calib_started = true;
                            SnCalibStatus status;
                            sn_get_optic_flow_camera_yaw_calibration_status(&status);
                            if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
                            {
                                DEBUG("[%u] Optic flow camera yaw calibration is in progress\n", circle_counter);
                            }
                        }
                        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
                        {
                            DEBUG("[%u] Optic flow camera yaw calibration was completed successfully\n", circle_counter);
                            keep_going = false;
                            calib_result = true;
                        }
                        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
                        {
                            DEBUG("[%u] Optic flow camera yaw calibration failed\n", circle_counter);
                            keep_going = false;
                            calib_result = false;
                        }
                        else
                        {
                            if (attempt_number < kMaxAttempts)
                            {
                                DEBUG("[%u] Sending command (attempt %u) to start optic flow camera yaw calibration\n",
                                                circle_counter, attempt_number);
                                sn_start_optic_flow_camera_yaw_calibration();
                                attempt_number++;
                            }
                            else
                            {
                                DEBUG("[%u] Unable to start calibration\n", circle_counter);
                                keep_going = false;
                            }
                        }

                        circle_counter++;
                        usleep(100000); //100ms

                        if (circle_counter >= 600)
                        {
                            DEBUG("[%u] Optic flow camera yaw Calibrate TimeOut calib_result=%d\n", circle_counter, calib_result);
                            keep_going = false;
                        }
                    }
                }

                memset(result_to_client, 0, MAX_BUFF_LEN);
                sprintf(result_to_client, "%s", SNAV_INFO_OPTIC_FLOW_CALIB_RESULT);
                strcat(result_to_client, STR_SEPARATOR);
                strcat(result_to_client, calib_result == true? "1":"0");

                length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                DEBUG("[%d] udp sendto SNAV_INFO_OPTIC_FLOW_CALIB_RESULT length=%d\n", loop_counter, length);
            }

            // HOR calibrate
            if ((udp_msg_array.size() >= 1)
                && (udp_msg_array[0].compare(SNAV_CMD_HOR_CALIBRATE) == 0)
                && (props_state == SN_PROPS_STATE_NOT_SPINNING))
            {
                bool keep_going = true;
                bool calib_started = false;
                bool calib_result = false;
                unsigned int attempt_number = 0;
                const unsigned int kMaxAttempts = 10;

                while (keep_going)
                {
                    static unsigned int circle_counter = 0;

                    if (sn_update_data() != 0)
                    {
                        DEBUG("[%u] sn_update_data failed in accel_calibrate\n", circle_counter);
                        keep_going = false;
                    }
                    else
                    {
                        if (snav_data->general_status.current_mode == SN_STATIC_ACCEL_CALIBRATION_MODE)
                        {
                            calib_started = true;
                            SnCalibStatus status;
                            sn_get_static_accel_calibration_status(&status);

                            if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
                            {
                                DEBUG("[%u] Static accel calibration is in progress\n", circle_counter);
                            }
                        }
                        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
                        {
                            DEBUG("[%u] Static accel calibration was completed successfully\n", circle_counter);
                            keep_going = false;
                            calib_result = true;
                        }
                        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
                        {
                            DEBUG("[%u] Static accel calibration failed\n", circle_counter);
                            keep_going = false;
                            calib_result = false;
                        }
                        else
                        {
                            if (attempt_number < kMaxAttempts)
                            {
                                DEBUG("[%u] Sending command (attempt %u) to start static accel calibration\n",
                                                        circle_counter, attempt_number);
                                sn_start_static_accel_calibration();
                                attempt_number++;
                            }
                            else
                            {
                                DEBUG("[%u] Unable to start calibration\n", circle_counter);
                                keep_going = false;
                            }
                        }

                        circle_counter++;
                        usleep(100000); //100ms

                        if (circle_counter >= 600)
                        {
                            DEBUG("[%u] HOR Calibrate TimeOut calib_result=%d\n", circle_counter, calib_result);
                            keep_going = false;
                        }
                    }
                }

                memset(result_to_client, 0, MAX_BUFF_LEN);
                sprintf(result_to_client, "%s", SNAV_INFO_HOR_CALIBRATE_RESULT);
                strcat(result_to_client, STR_SEPARATOR);
                strcat(result_to_client, calib_result == true? "1":"0");

                length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
                                 , 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                DEBUG("[%d] SNAV_INFO_HOR_CALIBRATE_RESULT result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
            }

            // FaceFollow switch
            if ((udp_msg_array.size() >= 2)
                && (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW) == 0))
            {
                if (strncmp(udp_msg_array[1].c_str(), "on", 2) == 0)
                {
                    face_follow_switch = true;
                    body_follow_switch = false;

                    send_face_follow_swither_flag = true;
                    memset(face_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
                    strcpy(face_follow_swither_buff, "fdon");
                }
                else
                {
                    face_follow_switch = false;

                    send_face_follow_swither_flag = true;
                    memset(face_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
                    strcpy(face_follow_swither_buff, "fdoff");
                }

                face_mission = false;
                body_mission = false;
            }

            // gesture switch
            if ((udp_msg_array.size() >= 2)
                && (udp_msg_array[0].compare(SNAV_CMD_GESTURE) == 0))
            {
                if (strcmp(udp_msg_array[1].c_str(), "on") == 0)
                {
                    send_gesture_swither_flag = true;
                    memset(gesture_swither_buff, 0, DOMAIN_BUFF_SIZE);
                    strcpy(gesture_swither_buff, "gson");

                    //send to gesture start
                    unsigned short func[32];
                    func[0] = SNAV_TASK_START_GESTURE;
                    length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                                 (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_START_GESTURE func=%d, length=%d\n", loop_counter, func[0], length);
                }
                else
                {
                    cur_body.handle_gesture = 0;

                    //send to camera_super stop
                    send_gesture_swither_flag = true;
                    memset(gesture_swither_buff, 0, DOMAIN_BUFF_SIZE);
                    strcpy(gesture_swither_buff, "gsoff");

                    //send to gesture stop
                    unsigned short func[32];
                    func[0] = SNAV_TASK_STOP_GESTURE;
                    length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                                 (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_STOP_GESTURE func=%d, length=%d\n", loop_counter, func[0], length);
                }

                face_mission = false;
                body_mission = false;
            }

            // BodyFollow switch
            if ((udp_msg_array.size() >= 2)
                && (udp_msg_array[0].compare(SNAV_CMD_BODY_FOLLOW) == 0))
            {
                if (strncmp(udp_msg_array[1].c_str(), "on", 2) == 0)
                {
                    body_follow_switch = true;
                    face_follow_switch = false;

                    send_body_follow_swither_flag = true;
                    memset(body_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
                    strcpy(body_follow_swither_buff, "bdon");

                    //send command to start tracker
                    unsigned short func[32];
                    func[0] = SNAV_TASK_START_TRACKER;
                    length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                                 (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_START_TRACKER func=%d, length=%d\n", loop_counter, func[0], length);
                }
                else if (strncmp(udp_msg_array[1].c_str(), "start", 5) == 0 /*||
                    strncmp(udp_msg_array[1].c_str(), "up", 2) == 0*/)
                {
                    body_follow_start = true;
                    init_width = 0;
                    init_height = 0;

                    DEBUG("[%d] SNAV_TASK_START_TRACKER func=%s, length=%d\n", loop_counter, udp_msg_array[1].c_str(), length);
                }
                else if (strncmp(udp_msg_array[1].c_str(), "down", 4) == 0)
                {
                    body_follow_start = false;
                    follow_reset_yaw = true;
                    body_mission = false;
                    DEBUG("[%d] SNAV_TASK_START_TRACKER func=%s, length=%d\n", loop_counter, udp_msg_array[1].c_str(), length);
                }
                else if (strncmp(udp_msg_array[1].c_str(), "off", 3) == 0)
                {
                    body_follow_switch = false;

                    send_body_follow_swither_flag = true;
                    memset(body_follow_swither_buff, 0, DOMAIN_BUFF_SIZE);
                    strcpy(body_follow_swither_buff, "bdoff");

                    init_width = 0;
                    init_height = 0;
                    body_follow_start = false;

                    cur_body.velocity = 0;
                    cur_body.angle = 0;
                    follow_reset_yaw = true;
                    body_mission = false;

                    //send command to stop tracker
                    unsigned short func[32];
                    func[0] = SNAV_TASK_STOP_TRACKER;
                    length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                                 (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_STOP_TRACKER func=%d, length=%d\n", loop_counter, func[0], length);
                }
                else if (strcmp(udp_msg_array[1].c_str(), "uroi") == 0
                   &&(udp_msg_array.size() >= 9))//user choose ROI
                {

                    //send command to stop tracker
                    unsigned short func[32];
                    func[0] = 5002;
                    func[1] = atoi(udp_msg_array[2].c_str()); // lcd Width
                    func[2] = atoi(udp_msg_array[3].c_str()); // lcd height
                    func[3] = atoi(udp_msg_array[4].c_str()); //start x
                    func[4] = atoi(udp_msg_array[5].c_str()); //start y
                    func[5] = atoi(udp_msg_array[6].c_str()); // select endx
                    func[6] = atoi(udp_msg_array[7].c_str()); // select endy
                    func[7] = atoi(udp_msg_array[8].c_str()); // index  like 0,1,2(send 3 times)

                    length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                                 (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_STOP_TRACKER func=%d, length=%d\n", loop_counter, func[0], length);
                }
                else if (strcmp(udp_msg_array[1].c_str(), "ucmd") == 0
                   &&(udp_msg_array.size() >= 9))//user command
                {

                    //send command to stop tracker
                    unsigned short func[32];
                    func[0] = 7001;
                    func[1] = atoi(udp_msg_array[2].c_str());
                    func[2] = atoi(udp_msg_array[3].c_str());
                    func[3] = atoi(udp_msg_array[4].c_str());
                    func[4] = atoi(udp_msg_array[5].c_str());
                    func[5] = atoi(udp_msg_array[6].c_str());
                    func[6] = atoi(udp_msg_array[7].c_str());
                    func[7] = atoi(udp_msg_array[8].c_str());

                    length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                                 (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_TASK_STOP_TRACKER func=%d, length=%d\n", loop_counter, func[0], length);
                }
            }

            // FaceFollow mode
            if ((udp_msg_array.size() >= 2)
                && (udp_msg_array[0].compare(SNAV_CMD_FACE_FOLLOW_MODE) == 0))
            {
                int flag = atoi(udp_msg_array[1].c_str());

                if (flag == 1)
                {
                    face_rotate_switch = false;
                }
                else
                {
                    face_rotate_switch = true;
                }
            }

            /*************************************************************************
            **********************Hover or Other Mission handle***********************
            **************************************************************************/
            static bool mission_has_begun = false;

            if ((props_state == SN_PROPS_STATE_NOT_SPINNING)
                && (on_ground_flag == 1)
                && (!mission_has_begun))
            {
                state = MissionState::ON_GROUND;
            }

            // Reset the take_off_with_gps_valid flag
            if (state == MissionState::ON_GROUND)
            {
                take_off_with_gps_valid = false;
            }

            // Reset face_takeoff and face_detect when have take off
            if ((state == MissionState::LOITER) || (state == MissionState::IN_MOTION))
            {
                face_takeoff_flag = false;
                face_detect = false;
            }

            if (state == MissionState::ON_GROUND)
            {
                // Send zero velocity while vehicle sits on ground
                x_vel_des = 0;
                y_vel_des = 0;
                z_vel_des = 0;
                yaw_vel_des = 0;

                if (face_takeoff_flag && face_detect && !t_face_detect_flag)
                {
                    t_face_detect_valid = t_des_now;
                    t_face_detect_flag = true;
                }

                if (t_face_detect_valid > 0)
                {
                    DEBUG("[%d] ON_GROUND t_face_detect_valid=%f, dirr=%f\n", loop_counter,
                                t_face_detect_valid, (t_des_now-t_face_detect_valid));
                }

                if (face_takeoff_flag && face_detect && (t_face_detect_valid > 0) &&((t_des_now-t_face_detect_valid) > 3))
                {
                    t_face_detect_flag = false;

                    mission_has_begun = true;
                    state = MissionState::STARTING_PROPS;
                }
                else if (udp_msg_array.size() >= 1)
                {
                    if (udp_msg_array[0].compare(SNAV_CMD_TAKE_OFF) == 0)
                    {
                        revise_height = 0.15;

                        mission_has_begun = true;
                        state = MissionState::STARTING_PROPS;
                    }
                    else if (udp_msg_array[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0)
                    {
                        int position_num = 0;
                        int i = 0;
                        int lati, longi;

                        position_num = atoi(udp_msg_array[1].c_str());

                        DEBUG("Trail Navigation position_num:%d\n", position_num);

                        if ((position_num >= MIN_GPS_POSITION_NUM) && (position_num >= MAX_GPS_POSITION_NUM))
                        {
                            continue;
                        }

                        for (i = 0; i < 2*position_num; i += 2)
                        {
                            lati = atoi(udp_msg_array[2+i].c_str());
                            longi = atoi(udp_msg_array[2+i+1].c_str());

                            DEBUG("Trail Navigation [%d]-lati,logi:%d,%d\n", i/2, lati, longi);

                            NavigationPosition pos;
                            pos.latitude = lati;
                            pos.longitude = longi;

                            if (pos.latitude !=0 && pos.longitude !=0)
                            {
                                trail_navigation_positions.push_back(pos);
                            }
                        }

                        mission_has_begun = true;
                        state = MissionState::STARTING_PROPS;

                        trail_navigation_mission = true;
                    }
                    else
                    {
                        state = MissionState::ON_GROUND;
                    }
                }
            }
            else if (state == MissionState::STARTING_PROPS)
            {
                x_vel_des = 0;
                y_vel_des = 0;
                z_vel_des = 0;
                yaw_vel_des = 0;

                if (udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
                {
                    state = MissionState::LANDING;
                    loop_counter++;
                    continue;
                }

                if (face_takeoff_flag && face_detect)
                {
                    if (props_state == SN_PROPS_STATE_NOT_SPINNING)
                    {
                        sn_spin_props();
                    }
                    else if (props_state == SN_PROPS_STATE_STARTING)
                    {
                        //store current position to the boot position
                        x_est_startup = x_est;
                        y_est_startup = y_est;
                        z_est_startup = z_est;
                        yaw_est_startup = yaw_est;

                        if (gps_enabled && (gps_status == SN_DATA_VALID))
                        {
                            x_est_gps_startup = x_est_gps;
                            y_est_gps_startup = y_est_gps;
                            z_est_gps_startup = z_est_gps;
                            yaw_est_gps_startup = yaw_est_gps;

                            take_off_with_gps_valid = true;
                        }
                        else
                        {
                            take_off_with_gps_valid = false;
                        }
                    }
                    else if (props_state == SN_PROPS_STATE_SPINNING)
                    {
                        face_takeoff_flag = false;
                        face_detect = false;

                        state = MissionState::TAKEOFF;

                        memset(result_to_client, 0, MAX_BUFF_LEN);
                        sprintf(result_to_client, "%s", SNAV_TASK_RESET_FACE_TAKEOFF);

                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                            (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_TASK_RESET_FACE_TAKEOFF result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);

                        //==== cuiyc test for hand gesture
                        body_follow_switch = true;
                        //face_follow_switch = false;

                        send_gesture_swither_flag = true;
                        memset(gesture_swither_buff, 0, DOMAIN_BUFF_SIZE);
                        strcpy(gesture_swither_buff, "gson");

                        //send to gesture start
                        unsigned short func[32];
                        func[0] = SNAV_TASK_START_GESTURE;
                        length = sendto(tracker_udp_sockfd, func, sizeof(func), 0,
                                     (struct sockaddr*)&address_tracker, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_TASK_START_GESTURE func=%d, length=%d\n", loop_counter, func[0], length);
                        //==== cuiyc
                    }
                }
                else
                {
                    if (props_state == SN_PROPS_STATE_NOT_SPINNING)
                    {
                        sn_spin_props();
                    }
                    else if (props_state == SN_PROPS_STATE_STARTING)
                    {
                        //store current position to the boot position
                        x_est_startup = x_est;
                        y_est_startup = y_est;
                        z_est_startup = z_est;
                        yaw_est_startup = yaw_est;

                        if (gps_enabled && (gps_status == SN_DATA_VALID))
                        {
                            x_est_gps_startup = x_est_gps;
                            y_est_gps_startup = y_est_gps;
                            z_est_gps_startup = z_est_gps;
                            yaw_est_gps_startup = yaw_est_gps;

                            take_off_with_gps_valid = true;
                        }
                        /*
                        else if (mode == SN_VIO_POS_HOLD_MODE)
                        {
                            x_est_vio_startup = x_est_vio;
                            y_est_vio_startup = y_est_vio;
                            z_est_vio_startup = z_est_vio;
                            yaw_est_vio_startup = yaw_est_vio;
                        }
                        */
                        else
                        {
                            take_off_with_gps_valid = false;
                        }
                    }
                    else if (props_state == SN_PROPS_STATE_SPINNING)
                    {
                        //state = MissionState::TAKEOFF;  //MissionState::LOITER;

                        propers_start_count++;
                        DEBUG("[%d] propers_start_count=%d\n", loop_counter, propers_start_count);

                        optic_flow_sample_size_sum += sample_size;

                        if (propers_start_count >= of_ct)     // 15*20=300 ms
                        {
                            optic_flow_sample_average = optic_flow_sample_size_sum/of_ct;

                            DEBUG("[%d] optic_flow_sample_size_sum, average: [%d, %d]\n", loop_counter, optic_flow_sample_size_sum, optic_flow_sample_average);

                            if (optic_flow_sample_average < of_valid_value)
                            {
                                optic_flow_sample_size_sum = 0;
                                optic_flow_sample_average = 0;
                                propers_start_count = 0;

                                state = MissionState::LANDING;
                                confirm_land = true;
                                loop_counter++;
                                continue;
                            }
                            else
                            {
#ifdef CSV_LOG_FOR_XIAOYI
                                csv_log_count ++;
                                memset(csv_log_filename, 0, TMP_BUFF_LEN);
                                sprintf(csv_log_filename, LOG_FOR_XIAOYI_PATH PATH_FLAG "csv_log_%04d_%04d.csv", log_count, csv_log_count);

                                if ((fp_csv_log = fopen(csv_log_filename, "a+")) != NULL)
                                {
                                    chmod(csv_log_filename, 0777);
                                    //char csv_title[TMP_BUFF_LEN]="index,latitude,longtitude,height,battery,timestamp\n";
                                    char csv_title[TMP_BUFF_LEN]="index,latitude,longtitude,x,y,z,roll,pitch,yaw,f-state,d-state-err,velocity,battery,time\n";
                                    fwrite(csv_title, strlen(csv_title), 1, fp_csv_log);
                                    fclose(fp_csv_log);
                                }
#endif
                                state = MissionState::TAKEOFF;  //MissionState::LOITER;
                            }

                            propers_start_count = 0;
                            optic_flow_sample_size_sum = 0;
                            optic_flow_sample_average = 0;
                        }
                    }
                }
            }
            else if (state == MissionState::TAKEOFF)
            {
                if (udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
                {
                    state = MissionState::LANDING;
                    loop_counter++;
                    continue;
                }

                if (props_state == SN_PROPS_STATE_SPINNING)
                {
                    // Command constant positive z velocity during takeoff
                    x_vel_des = 0;
                    y_vel_des = 0;
                    yaw_vel_des = 0;

                    if (trail_navigation_mission)
                    {
                        z_vel_des = kTakeoffSpeed;

                        if (z_est - z_est_startup >= 0.6*fTrarilHeight)
                        {
                            z_vel_des = 0;
                            state = MissionState::LOITER;
                        }
                        else if(z_est - z_est_startup > 0.3f*fTrarilHeight)
                        {
                            z_vel_des = kTakeoffSpeed*0.15f;    //0.3f
                        }
                    }
                    else
                    {
                        // Baro linear data have more than 1m error, so use z data instead.
                        z_vel_des = kTakeoffSpeed;

                        if (snav_data->sonar_0_raw.range > sonar_valid_data_max)
                        {
                            if (z_est - z_est_startup >= 0.75*fTakeOffHeight)
                            {
                                z_vel_des = 0;
                                state = MissionState::LOITER;
                            }
                            else if (z_est - z_est_startup > 0.3f*fTakeOffHeight)
                            {
                                //z_vel_des = kTakeoffSpeed*0.15f;
                                z_vel_des = kTakeoffSpeed*0.5;
                            }
                        }
                        else
                        {
                            if ((z_est - z_est_startup >= 0.75*fTakeOffHeight)
                                && (snav_data->sonar_0_raw.range >= 0.75*fTakeOffHeight)
                                && (snav_data->sonar_0_raw.range < sonar_valid_data_max))
                            {
                                z_vel_des = 0;
                                state = MissionState::LOITER;
                            }
                            else if ((z_est - z_est_startup > 0.3f*fTakeOffHeight)
                                     && (snav_data->sonar_0_raw.range > 0.3f*fTakeOffHeight)
                                     && (snav_data->sonar_0_raw.range < sonar_valid_data_max))
                            {
                                //z_vel_des = kTakeoffSpeed*0.15f;
                                z_vel_des = kTakeoffSpeed*0.5;
                            }
                        }
                    }
                }
                else if (props_state == SN_PROPS_STATE_NOT_SPINNING)
                {
                    state = MissionState::STARTING_PROPS;
                }
            }
            else if (state == MissionState::LANDING)
            {
                if (props_state == SN_PROPS_STATE_SPINNING)
                {
                    // Reset all the mission
                    current_position =0;

                    fly_test_mission = false;
                    rotation_test_mission = false;

                    circle_mission = false;
                    calcCirclePoint = false;

                    panorama_mission = false;
                    calcPanoramaPoint = false;

                    trail_navigation_mission = false;
                    customized_plan_mission = false;
                    calcPlanPoint = false;

                    return_mission = false;

                    face_mission = false;
                    body_mission = false;

                    face_follow_switch = false;
                    body_follow_switch = false;

                    // Command constant negative z velocity during landing
                    x_vel_des = 0;
                    y_vel_des = 0;
                    z_vel_des = kLandingSpeed;
                    yaw_vel_des = 0;

                    /*
                    // Confirm whether to land
                    if ((snav_data->sonar_0_raw.range <= 1.5)
                        && (snav_data->sonar_0_raw.range >= sonar_valid_data_min)
                        && !confirm_land
                        && (mode != SN_EMERGENCY_LANDING_MODE))
                    {
                        state = MissionState::LOITER;

                        memset(result_to_client, 0, MAX_BUFF_LEN);
                        sprintf(result_to_client, "%s", SNAV_TASK_SHOW_LAND_CONFIRM);

                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0
                                         ,(struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] udp sendto SNAV_TASK_SHOW_LAND_CONFIRM length=%d\n", loop_counter, length);

                        continue;
                    }
                    */


                    /*
                    // Smoothly landing avoid to shake when touch the ground
                    if((snav_data->sonar_0_raw.range < (sonar_valid_data_min+0.1)) && !landing_near_ground)
                    {
                        landing_near_ground = true;
                    }

                    if (landing_near_ground)
                    {
                        z_vel_des = -1.5;       // 2/3 = 1 cmd3 z_vel_des_max=1.2, cmd3 max=0.8
                    }
                    else
                    */
                    {
                        // Slow down near the ground
                        /*
                        if(snav_data->sonar_0_raw.range < 2.5)
                        {
                            if (confirm_land)
                            {
                                z_vel_des = -0.45;
                            }
                            else
                            {
                                z_vel_des = -0.225;     // 2/3 = 0.15 cmd3
                            }
                        }
                        */

                        if(snav_data->sonar_0_raw.range < sonar_valid_data_max)
                        {
                            z_vel_des = -0.45;
                        }


                    }

                    /* More faster to stop props*/
                    if ((props_state == SN_PROPS_STATE_SPINNING)
                        && ((t_des_now - t_normal_rpm) > time_interval_of_low_spin)
                        && ((z_est - z_est_startup) < 0.5)
                        && (snav_data->rc_active.cmd[0] == 0)
                        && (snav_data->rc_active.cmd[1] == 0)
                        && (snav_data->rc_active.cmd[2] <= -0.8)    /* Landing... */
                        && (snav_data->rc_active.cmd[3] == 0))
                    {
                        // Snapdragon Navigator has determined that vehicle is on ground,
                        // so it is safe to kill the propellers

                        if (!send_fpv_flag)
                        {
                            send_fpv_flag = true;
                            memset(fpv_switcher_buff, 0, DOMAIN_BUFF_SIZE);
                            strcpy(fpv_switcher_buff, "exit");

                            face_follow_switch = false;
                            body_follow_switch = false;

                            send_face_follow_swither_flag = true;
                            memset(face_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                            strcpy(face_follow_swither_buff, "fdoff");

                            send_body_follow_swither_flag = true;
                            memset(body_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                            strcpy(body_follow_swither_buff, "bdoff");
                        }
                        sn_stop_props();
                    }
                    else if (props_state == SN_PROPS_STATE_SPINNING && on_ground_flag == 1)
                    {
                        // Snapdragon Navigator has determined that vehicle is on ground,
                        // so it is safe to kill the propellers
                        sn_stop_props();
                    }
                }
                else
                {
                    state = MissionState::ON_GROUND;

                    // Reset all the mission
                    current_position = 0;

                    fly_test_mission = false;
                    rotation_test_mission = false;

                    circle_mission = false;
                    calcCirclePoint = false;

                    panorama_mission = false;
                    calcPanoramaPoint = false;

                    trail_navigation_mission = false;
                    customized_plan_mission = false;
                    calcPlanPoint = false;

                    return_mission = false;

                    face_mission = false;
                    body_mission = false;

                    face_follow_switch = false;
                    body_follow_switch = false;

                    if (!send_fpv_flag)
                    {
                        send_fpv_flag = true;
                        memset(fpv_switcher_buff, 0, DOMAIN_BUFF_SIZE);
                        strcpy(fpv_switcher_buff, "exit");
                    }
                }
            }
            // All the mission start from the LOITER, include the points calc
            else if (state == MissionState::LOITER)
            {
                if (props_state == SN_PROPS_STATE_SPINNING)
                {
                    // Reset the confirm_land flag every time in LOITER
                    confirm_land = false;

                    // Maintain current position
                    x_vel_des = 0;
                    y_vel_des = 0;
                    z_vel_des = 0;
                    yaw_vel_des = 0;

                    static bool entering_loiter = true;
                    static double t_loiter_start = 0;

                    // Set the flag false
                    if (entering_loiter)
                    {
                        t_loiter_start = t_now;
                        entering_loiter = false;
                    }

                    static double t_csv_pic_start = 0;

#ifdef CSV_LOG_FOR_XIAOYI
                    if ((access(csv_log_filename, F_OK) == 0) && (t_now - t_loiter_start > 1))
                    {
                        memset(csv_pic_buff, 0, DOMAIN_BUFF_SIZE);
                        sprintf(csv_pic_buff, LOG_FOR_XIAOYI_PATH PATH_FLAG "csv_log_%04d_%04d.jpg", log_count, csv_log_count);

                        if ((t_now - t_csv_pic_start > 2) && (access(csv_pic_buff, F_OK) != 0))
                        {
                            t_csv_pic_start = t_now;

                            memset(csv_pic_buff, 0, DOMAIN_BUFF_SIZE);
                            sprintf(csv_pic_buff, "csv_log_%04d_%04d", log_count, csv_log_count);
                            send_csv_pic_flag = true;
                        }
                        else
                        {
                            memset(csv_pic_buff, 0, DOMAIN_BUFF_SIZE);
                            send_csv_pic_flag = false;
                        }
                    }
#endif


#ifdef DISCONN_AUTO_RETURN
                    if (!bHaveUdpClient && (t_have_client != 0) && (t_now - t_have_client) >= 10)
                    {
                        DEBUG("[%d]:DISCONN_AUTO_RETURN t_have_client/t_now/diff:[%lf, %lf, %lf]\n",
                                            loop_counter, t_have_client, t_now, (t_now - t_have_client));
                        if (gps_enabled)
                        {
                            float dis_gps_home = sqrt((x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
                                                          + (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps));

                            if ((gps_status == SN_DATA_VALID)
                                    && take_off_with_gps_valid
                                    && (dis_gps_home >= distance_disconn_auto_return)) /* || mag_status != SN_DATA_VALID) */
                            {
                                return_mission = true;
                                entering_loiter = true;
                                state = MissionState::IN_MOTION;
                            }

                            DEBUG("[%d]:gps_status[%d], take_off_with_gps_valid[%d], dis_gps_home[%f], return_mission[%d].\n",
                                            loop_counter, gps_status, take_off_with_gps_valid, dis_gps_home, return_mission);
                        }
                        /*
                        else if (mode == SN_VIO_POS_HOLD_MODE)
                        {
                            float dis_vio_home = sqrt((x_est_vio_startup-x_est_vio)*(x_est_vio_startup-x_est_vio)
                                                                + (y_est_vio_startup-y_est_vio)*(y_est_vio_startup-y_est_vio));

                            if (dis_vio_home >= distance_disconn_auto_return)
                            {
                                return_mission = true;
                                entering_loiter = true;
                                state = MissionState::IN_MOTION;
                            }

                            DEBUG("[%d]:dis_vio_home[%f], return_mission[%d].\n", loop_counter, dis_vio_home, return_mission);
                        }
                        */
                        else
                        {
                            float dis_optic_flow_home = sqrt((x_est_startup-x_est)*(x_est_startup-x_est)
                                                                + (y_est_startup-y_est)*(y_est_startup-y_est));

                            if (dis_optic_flow_home >= distance_disconn_auto_return)
                            {
                                return_mission = true;
                                entering_loiter = true;
                                state = MissionState::IN_MOTION;
                            }

                            DEBUG("[%d]:dis_optic_flow_home[%f], return_mission[%d].\n", loop_counter, dis_optic_flow_home, return_mission);
                        }

                        // Every circle recalc the distance and yaw_diff from home for Return mission
                        if (return_mission)
                        {
                            if (gps_enabled)
                            {
                                distance_gps_home_squared = (x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
                                                             + (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps);
                                yaw_gps_target_home = atan2(y_est_gps_startup-y_est_gps, x_est_gps_startup-x_est_gps);

                                DEBUG("[%d]:return_mission distance_gps_home_squared, yaw_gps_target_home:[%f, %f]\n",
                                            loop_counter, distance_gps_home_squared, yaw_gps_target_home);

                                if (distance_gps_home_squared > distance_home_squared_threshold)
                                {
                                    fly_home = false;           /*need turn the yaw to home*/
                                }
                                else
                                {
                                    fly_home = true;            /*too close, fly to home directly*/
                                }
                            }
                            /*
                            else if (mode == SN_VIO_POS_HOLD_MODE)
                            {
                                distance_home_squared = (x_est_vio_startup-x_est_vio)*(x_est_vio_startup-x_est_vio) + (y_est_vio_startup-y_est_vio)*(y_est_vio_startup-y_est_vio);
                                yaw_target_home = atan2(y_est_vio_startup-y_est_vio, x_est_vio_startup-x_est_vio);

                                DEBUG("[%d]:return_mission distance_home_squared, yaw_target_home:[%f,%f]\n",
                                            loop_counter,distance_home_squared,yaw_target_home);

                                if (distance_home_squared > distance_home_squared_threshold)
                                {
                                    fly_home = false;
                                }
                                else
                                {
                                    fly_home = true;
                                }
                            }
                            */
                            else
                            {
                                distance_home_squared = (x_est_startup-x_est)*(x_est_startup-x_est) + (y_est_startup-y_est)*(y_est_startup-y_est);
                                yaw_target_home = atan2(y_est_startup-y_est, x_est_startup-x_est);

                                DEBUG("[%d]:return_mission distance_home_squared, yaw_target_home:[%f,%f]\n",
                                            loop_counter,distance_home_squared,yaw_target_home);

                                if (distance_home_squared > distance_home_squared_threshold)
                                {
                                    fly_home = false;           /*need turn the yaw to home*/
                                }
                                else
                                {
                                    fly_home = true;            /*too close, fly to home directly*/
                                }
                            }

                            gohome_x_vel_des = 0;
                            gohome_y_vel_des = 0;
                            gohome_z_vel_des = 0;
                            gohome_yaw_vel_des = 0;
                        }
                    }
#endif

                    if (udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
                    {
                        state = MissionState::LANDING;
                        loop_counter++;
                        continue;
                    }
                    else if ((udp_msg_array.size() >= 2)
                            && (udp_msg_array[0].compare(SNAV_TASK_CONFIRM_LAND) ==0)
                            && (udp_msg_array[1].compare("yes") ==0))
                    {
                        state = MissionState::LANDING;
                        confirm_land = true;
                        loop_counter++;
                        continue;
                    }
                    // Panorama task
                    else if(udp_msg_array.size() >= 2
                            && udp_msg_array[0].compare(SNAV_CMD_PANORAMA) == 0
                            && !fly_test_mission
                            && !rotation_test_mission
                            && !circle_mission
                            && !return_mission
                            && !trail_navigation_mission
                            && !customized_plan_mission
                            && !face_mission
                            && !body_mission)
                    {
                        panorama_mission = true;
                        calcPanoramaPoint = true;

                        clockwise = atoi(udp_msg_array[1].c_str());
                        point_count = 12;
                        vel_target = 0.5;    //m/sec
                        angle_per = (2*M_PI)/(3*point_count);   //(120degree/point_count)

                        DEBUG("[%d] Panorama_mission: clockwise, point_count, angle_per:%d,%d,%f\n",
                                    loop_counter, clockwise, point_count, angle_per);
                    }
                    // Fly test task
                    else if(udp_msg_array.size() >= 1
                            && udp_msg_array[0].compare(SNAV_CMD_FLY_TEST) == 0
                            && !rotation_test_mission
                            && !circle_mission
                            && !panorama_mission
                            && !return_mission
                            && !trail_navigation_mission
                            && !customized_plan_mission
                            && !face_mission
                            && !body_mission)
                    {
                        fly_test_mission = true;
                        fly_test_count = 0;
                        DEBUG("[%d] SNAV_CMD_FLY_TEST\n", loop_counter);
                    }
                    // Rotation test task
                    else if(udp_msg_array.size() >= 1
                            && udp_msg_array[0].compare(SNAV_CMD_ROTATION_TEST) == 0
                            && !fly_test_mission
                            && !circle_mission
                            && !panorama_mission
                            && !return_mission
                            && !trail_navigation_mission
                            && !customized_plan_mission
                            && !face_mission
                            && !body_mission)
                    {
                        rotation_test_mission = true;
                        rotation_test_count = 0;
                        DEBUG("[%d] SNAV_CMD_ROTATION_TEST\n", loop_counter);
                    }
                    // Circel task
                    else if(udp_msg_array.size() >= 3
                            && udp_msg_array[0].compare(SNAV_CMD_CIRCLE) == 0
                            && !fly_test_mission
                            && !rotation_test_mission
                            && !panorama_mission
                            && !return_mission
                            && !trail_navigation_mission
                            && !customized_plan_mission
                            && !face_mission
                            && !body_mission)
                    {
#ifdef CIRCLE_HEIGHT_LIMIT_FLAG
                        if ((/*z_est - z_est_startup*/revise_height) > circle_height_limit)
                        {
                            circle_mission = false;
                            calcCirclePoint = false;

                            sprintf(result_to_client, "%s", SNAV_WARNING_CIRCLE_OVER_HEIGHT);
                            length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                            (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                            DEBUG("[%d] SNAV_WARNING_CIRCLE_OVER_HEIGHT result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);

                        }
                        else
                        {
#endif
                            circle_mission = true;
                            calcCirclePoint = true;

                            radius = atof(udp_msg_array[1].c_str());
                            clockwise = atoi(udp_msg_array[2].c_str());

                            if (radius <= 5.0)
                            {
                                point_count = 36;
                            }
                            else
                            {
                                point_count = 72;
                            }
                            vel_target = 0.75;   //m/sec
                            angle_per = 2*M_PI/point_count;

                            DEBUG("[%d] LOITER circle: radius, clockwise, point_count, vel_target,angle_per:%f,%d,%d,%f,%f\n",
                                        loop_counter, radius, clockwise, point_count, vel_target, angle_per);
#ifdef CIRCLE_HEIGHT_LIMIT_FLAG
                        }
#endif
                    }
                    // Return task
                    else if(udp_msg_array.size() >= 1
                            && udp_msg_array[0].compare(SNAV_CMD_RETURN) == 0
                            && !panorama_mission
                            && !fly_test_mission
                            && !rotation_test_mission
                            && !circle_mission
                            && !trail_navigation_mission
                            && !customized_plan_mission
                            && !face_mission
                            && !body_mission)
                    {
                        if (gps_enabled)
                        {
                            if ((gps_status != SN_DATA_VALID) || !take_off_with_gps_valid ) /* || mag_status != SN_DATA_VALID) */
                            {
                                memset(result_to_client, 0, MAX_BUFF_LEN);

                                if (gps_status != SN_DATA_VALID)
                                {
                                    sprintf(result_to_client, "%s", SNAV_TASK_SHOW_GPS_RETURN_ERROR_ONE);
                                }
                                else
                                {
                                    sprintf(result_to_client, "%s", SNAV_TASK_SHOW_GPS_RETURN_ERROR_TWO);
                                }

                                length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0
                                                    , (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                                DEBUG("[%d] udp sendto SNAV_TASK_SHOW_GPS_ERROR result_to_client=%s, length=%d\n",
                                            loop_counter, result_to_client, length);

                                //loop_counter++;
                                //continue;
                            }
                            else
                            {
                                return_mission = true;
                                entering_loiter = true;
                                state = MissionState::IN_MOTION;
                            }
                        }
                        // Use optic-flow return instead
                        else
                        {
                            return_mission = true;
                            entering_loiter = true;
                            state = MissionState::IN_MOTION;
                        }

                        // Every circle recalc the distance and yaw_diff from home for Return mission
                        if (return_mission)
                        {
                            if (gps_enabled)
                            {
                                distance_gps_home_squared = (x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
                                                             + (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps);
                                yaw_gps_target_home = atan2(y_est_gps_startup-y_est_gps, x_est_gps_startup-x_est_gps);

                                DEBUG("[%d]:return_mission distance_gps_home_squared, yaw_gps_target_home:[%f, %f]\n",
                                            loop_counter, distance_gps_home_squared, yaw_gps_target_home);

                                if (distance_gps_home_squared > distance_home_squared_threshold)
                                {
                                    fly_home = false;           /*need turn the yaw to home*/
                                }
                                else
                                {
                                    fly_home = true;            /*too close, fly to home directly*/
                                }
                            }
                            /*
                            else if (mode == SN_VIO_POS_HOLD_MODE)
                            {
                                distance_home_squared = (x_est_vio_startup-x_est_vio)*(x_est_vio_startup-x_est_vio) + (y_est_vio_startup-y_est_vio)*(y_est_vio_startup-y_est_vio);
                                yaw_target_home = atan2(y_est_vio_startup-y_est_vio, x_est_vio_startup-x_est_vio);

                                DEBUG("[%d]:return_mission distance_home_squared, yaw_target_home:[%f,%f]\n",
                                            loop_counter,distance_home_squared,yaw_target_home);

                                if (distance_home_squared > distance_home_squared_threshold)
                                {
                                    fly_home = false;
                                }
                                else
                                {
                                    fly_home = true;
                                }
                            }
                            */
                            else
                            {
                                distance_home_squared = (x_est_startup-x_est)*(x_est_startup-x_est) + (y_est_startup-y_est)*(y_est_startup-y_est);
                                yaw_target_home = atan2(y_est_startup-y_est, x_est_startup-x_est);

                                DEBUG("[%d]:return_mission distance_home_squared, yaw_target_home:[%f,%f]\n",
                                            loop_counter,distance_home_squared,yaw_target_home);

                                if (distance_home_squared > distance_home_squared_threshold)
                                {
                                    fly_home = false;           /*need turn the yaw to home*/
                                }
                                else
                                {
                                    fly_home = true;            /*too close, fly to home directly*/
                                }
                            }

                            gohome_x_vel_des = 0;
                            gohome_y_vel_des = 0;
                            gohome_z_vel_des = 0;
                            gohome_yaw_vel_des = 0;

                            DEBUG("[%d] LOITER return enter IN_MOTION\n", loop_counter);
                        }
                    }
                    // Trail_navigation task
                    else if(udp_msg_array.size() >= 1
                            && udp_msg_array[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0
                            && !panorama_mission
                            && !fly_test_mission
                            && !rotation_test_mission
                            && !customized_plan_mission
                            && !circle_mission
                            && !return_mission
                            && !face_mission
                            && !body_mission)
                    {
                        int position_num = 0;
                        int i = 0;
                        int lati, longi;

                        position_num = atoi(udp_msg_array[1].c_str());

                        DEBUG("Trail Navigation position_num:%d\n", position_num);

                        if ((position_num >= MIN_GPS_POSITION_NUM) && (position_num >= MAX_GPS_POSITION_NUM))
                        {
                            continue;
                        }

                        for (i = 0; i < 2*position_num; i += 2)
                        {
                            lati = atoi(udp_msg_array[2+i].c_str());
                            longi = atoi(udp_msg_array[2+i+1].c_str());

                            DEBUG("Trail Navigation [%d]-lati,logi:%d,%d\n", i/2, lati, longi);

                            NavigationPosition pos;
                            pos.latitude = posGpsCurrent.latitude;
                            pos.longitude = posGpsCurrent.longitude;

                            if (pos.latitude !=0 && pos.longitude !=0)
                            {
                                trail_navigation_positions.push_back(pos);
                            }
                        }

                        trail_navigation_mission = true;

                        state = MissionState::IN_MOTION;
                        entering_loiter = true;

                        DEBUG("[%d] LOITER return enter IN_MOTION\n", loop_counter);
                    }
                    // Customized plan task
                    else if(udp_msg_array.size() >= 1
                            && udp_msg_array[0].compare(SNAV_CMD_CUSTOMIZED_PLAN) == 0
                            && !panorama_mission
                            && !fly_test_mission
                            && !rotation_test_mission
                            && !trail_navigation_mission
                            && !circle_mission
                            && !return_mission
                            && !face_mission
                            && !body_mission)
                    {
                        int step_num = 0;
                        int step = 0;
                        int i = 0;

                        step_num = atoi(udp_msg_array[1].c_str());

                        DEBUG("LOITER Customized plan step_num:%d\n", step_num);

                        customized_plan_steps.clear();

                        // step_num just count the plan steps couples, have not include the step count.
                        // example: 5,f,2,b,3,u,1,d,2,s,1
                        for (i = 0; i < step_num*2; i ++)
                        {
                            if (udp_msg_array.size() >= (size_t)(2+i))
                            {
                                customized_plan_steps.push_back(udp_msg_array[2+i]);
                            }
                        }

                        customized_plan_mission = true;
                        calcPlanPoint = true;
                    }
                    // Cuiyc add face detect begin
                    else if(cur_body.have_face && face_follow_switch) // Cuiyc add face detect begin
                    {
                        float face_offset ;
                        face_offset = M_PI*cur_body.angle/180;
                        DEBUG("followme have_face\n" );

                        if((fabs(face_offset) >min_angle_offset
                                || fabs(cur_body.distance -safe_distance) >0.05
                                || fabs(cur_body.hegith_calib)>0.1)
                            && !panorama_mission
                            && !fly_test_mission
                            && !rotation_test_mission
                            && !circle_mission
                            && !return_mission
                            && !trail_navigation_mission
                            && !customized_plan_mission)
                        {
                            face_mission = true;
                            state = MissionState::IN_MOTION;
                            entering_loiter = true;
                            DEBUG("face_offset:%f   distance:%f hegith_calib:%f\n",face_offset,
                                cur_body.distance,cur_body.hegith_calib);
                            DEBUG("followme have_face LOITER -> FACE_FOLLOW\n" );
                        }
                    }
                    else if(cur_body.have_body && body_follow_switch)
                    {
                        float body_offset ;
                        body_offset = M_PI*cur_body.angle/180;
                        DEBUG("followme have_body\n" );

                        if ((fabs(body_offset)>min_angle_offset
                                || fabs(cur_body.velocity)>0.05f)
                            && !panorama_mission
                            && !fly_test_mission
                            && !rotation_test_mission
                            && !circle_mission
                            && !return_mission
                            && !trail_navigation_mission
                            && !customized_plan_mission)
                        {
                            body_mission= true;
                            state = MissionState::IN_MOTION;
                            entering_loiter = true;
                            DEBUG("body angle:%f    velocity:%f\n",cur_body.angle,cur_body.velocity);
                            DEBUG("followme have_body LOITER -> BODY_FOLLOW\n" );
                        }
                    }
                    // Cuiyc add face detect end

                    // Circle mission calc points
                    if (circle_mission && (t_now - t_loiter_start > kLoiterTime))
                    {
                        if (calcCirclePoint)
                        {
                            // Circle center point
                            float circle_center_x;
                            float circle_center_y;
                            float yaw_t =0;

                            if (circle_cam_point_direct == 1)           // Camera point inside
                            {
                                circle_center_x = x_est-x_est_startup + radius*cos(yaw_est);
                                circle_center_y = y_est-y_est_startup + radius*sin(yaw_est);

                                if ((clockwise != 1) && (clockwise != -1))
                                {
                                    clockwise = 1;
                                }

                                circle_positions.clear();               // Clear and recaculate.
                                for (int k = 0; k <= point_count; k++)  // The last position must be the same as the first position
                                {
                                    Position pos;
                                    yaw_t = yaw_est-angle_per*k*clockwise;

                                    if (yaw_t > M_PI)
                                    {
                                        yaw_t = yaw_t - 2*M_PI;
                                    }
                                    else if (yaw_t < -M_PI)
                                    {
                                        yaw_t = yaw_t + 2*M_PI;
                                    }

                                    if (yaw_t>0)
                                    {
                                        pos.x = cos(yaw_t-M_PI)*radius + circle_center_x;
                                        pos.y = sin(yaw_t-M_PI)*radius + circle_center_y;
                                    }
                                    else
                                    {
                                        pos.x = cos(yaw_t+M_PI)*radius + circle_center_x;
                                        pos.y = sin(yaw_t+M_PI)*radius + circle_center_y;
                                    }
                                    pos.z = z_des - z_est_startup;
                                    pos.yaw = yaw_t;

                                    circle_positions.push_back(pos);
                                }
                            }
                            else    // Camera point outside
                            {
                                circle_center_x = x_est-x_est_startup - radius*cos(yaw_est);
                                circle_center_y = y_est-y_est_startup - radius*sin(yaw_est);

                                if ((clockwise != 1) && (clockwise != -1))
                                {
                                    clockwise = 1;
                                }

                                circle_positions.clear();               // Clear and recaculate.
                                for (int k = 0; k <= point_count; k++)  // The last position must be the same as the first position
                                {
                                    Position pos;
                                    yaw_t = yaw_est-angle_per*k*clockwise;

                                    if (yaw_t > M_PI)
                                    {
                                        yaw_t = yaw_t -2*M_PI;
                                    }
                                    else if (yaw_t < -M_PI)
                                    {
                                        yaw_t = yaw_t + 2*M_PI;
                                    }

                                    if (yaw_t>0)
                                    {
                                        pos.x = circle_center_x - cos(yaw_t-M_PI)*radius;
                                        pos.y = circle_center_y - sin(yaw_t-M_PI)*radius;
                                    }
                                    else
                                    {
                                        pos.x = circle_center_x - cos(yaw_t+M_PI)*radius;
                                        pos.y = circle_center_y - sin(yaw_t+M_PI)*radius;
                                    }
                                    pos.z = z_des - z_est_startup;
                                    pos.yaw = yaw_t;

                                    circle_positions.push_back(pos);
                                }
                            }

                            calcCirclePoint = false;

                            DEBUG("[%d] circle_center_point [%f,%f]\n", loop_counter, circle_center_x, circle_center_y);

                            for (int k = 0; k < (int)circle_positions.size(); k++)
                            {
                                DEBUG("[%d] position #%d: [%f,%f,%f,%f]\n", loop_counter, k,
                                            circle_positions[k].x, circle_positions[k].y,
                                            circle_positions[k].z, circle_positions[k].yaw);
                            }
                        }

                        state = MissionState::IN_MOTION;
                        entering_loiter = true;
                    }
                    else if (panorama_mission && (t_now - t_loiter_start > kLoiterTime))
                    {
                        if (calcPanoramaPoint)
                        {
                            float yaw_t = 0;

                            if ((clockwise != 1) && (clockwise != -1))
                            {
                                clockwise = 1;
                            }

                            panorama_positions.clear();             //clear and recaculate.
                            for (int k = 0; k <= point_count; k++)
                            {
                                Position pos;
                                pos.x = x_est-x_est_startup;
                                pos.y = y_est-y_est_startup;

                                yaw_t = yaw_est-angle_per*k*clockwise;

                                if (yaw_t > M_PI)
                                {
                                    yaw_t = yaw_t -2*M_PI;
                                }
                                else if (yaw_t < -M_PI)
                                {
                                    yaw_t = yaw_t + 2*M_PI;
                                }
                                pos.z = z_des - z_est_startup;
                                pos.yaw = yaw_t;

                                panorama_positions.push_back(pos);
                            }

                            calcPanoramaPoint = false;

                            for(int k = 0; k < (int)panorama_positions.size(); k++)
                            {
                                DEBUG("[%d] Panorama_positions #%u: [%f,%f,%f,%f]\n",loop_counter, k
                                        , panorama_positions[k].x,panorama_positions[k].y,
                                          panorama_positions[k].z,panorama_positions[k].yaw);
                            }
                        }

                        state = MissionState::IN_MOTION;
                        entering_loiter = true;
                    }
                    else if (customized_plan_mission)
                    {
#if 0
                        if (calcPlanPoint)
                        {
                            customized_plan_positions.clear();

                            plan_step_total = (int)customized_plan_steps.size();

                            PlanPosition pos_start;

                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                pos_start.x = x_est_gps-x_est_gps_startup;
                                pos_start.y = y_est_gps-y_est_gps_startup;
                                pos_start.z = z_est_gps;    //-z_est_gps_startup;
                                pos_start.yaw = yaw_est_gps;
                            }
                            else
                            {
                                pos_start.x = x_est-x_est_startup;
                                pos_start.y = y_est-y_est_startup;
                                pos_start.z = z_est;    //-z_est_startup;
                                pos_start.yaw = yaw_est;    // - yaw_est_startup;
                            }
                            pos_start.yaw_only = false;

                            DEBUG("[%d] customized_plan_mission plan_step_total:%d, pos_start: [%f,%f,%f,%f]\n",
                                    loop_counter, plan_step_total, pos_start.x, pos_start.y, pos_start.z, pos_start.yaw);

                            if (plan_step_total > 0)
                            {
                                // Only push the first point to the array
                                if (customized_plan_steps[0].compare(PLAN_LEFT) == 0)
                                {
                                    pos_start.x   = pos_start.x - plan_unit*sin(pos_start.yaw);
                                    pos_start.y   = pos_start.y + plan_unit*cos(pos_start.yaw);
                                }
                                else if (customized_plan_steps[0].compare(PLAN_RIGHT) == 0)
                                {
                                    pos_start.x   = pos_start.x + plan_unit*sin(pos_start.yaw);
                                    pos_start.y   = pos_start.y - plan_unit*cos(pos_start.yaw);
                                }
                                else if (customized_plan_steps[0].compare(PLAN_FRONT) == 0)
                                {
                                    pos_start.x   = pos_start.x + plan_unit*cos(pos_start.yaw);
                                    pos_start.y   = pos_start.y + plan_unit*sin(pos_start.yaw);
                                }
                                else if (customized_plan_steps[0].compare(PLAN_BACK) == 0)
                                {
                                    pos_start.x   = pos_start.x - plan_unit*cos(pos_start.yaw);
                                    pos_start.y   = pos_start.y - plan_unit*sin(pos_start.yaw);
                                }
                                else if (customized_plan_steps[0].compare(PLAN_UP) == 0)
                                {
                                    pos_start.z   = pos_start.z + plan_unit;
                                }
                                else if (customized_plan_steps[0].compare(PLAN_DOWN) == 0)
                                {
                                    pos_start.z   = pos_start.z - plan_unit;
                                }
                                else if (customized_plan_steps[0].compare(PLAN_CLOCKWISE) == 0)
                                {
                                    pos_start.yaw   = pos_start.yaw - M_PI/2;

                                    if (pos_start.yaw > M_PI)
                                    {
                                        pos_start.yaw = pos_start.yaw - 2*M_PI;
                                    }
                                    else if (pos_start.yaw < -M_PI)
                                    {
                                        pos_start.yaw = pos_start.yaw + 2*M_PI;
                                    }
                                    pos_start.yaw_only = true;
                                }
                                else if (customized_plan_steps[0].compare(PLAN_ANTI_CLOCKWISE) == 0)
                                {
                                    pos_start.yaw   = pos_start.yaw + M_PI/2;

                                    if (pos_start.yaw > M_PI)
                                    {
                                        pos_start.yaw = pos_start.yaw - 2*M_PI;
                                    }
                                    else if (pos_start.yaw < -M_PI)
                                    {
                                        pos_start.yaw = pos_start.yaw + 2*M_PI;
                                    }
                                    pos_start.yaw_only = true;
                                }
                                else
                                {
                                    DEBUG("Unknown plan=%s\n", customized_plan_steps[0].c_str());
                                }

                                DEBUG("[%d] customized_plan_positions push first_pos:[%f,%f,%f,%f,%d]\n",
                                            loop_counter, pos_start.x, pos_start.y,
                                            pos_start.z, pos_start.yaw, pos_start.yaw_only);
                                customized_plan_positions.push_back(pos_start);
                            }
                        }
#else

                        if (calcPlanPoint)
                        {
                            customized_plan_positions.clear();

                            int step_count = 0;
                            plan_step_total = (int)(customized_plan_steps.size()/2);

                            PlanPosition pos_start;

                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                pos_start.x = x_est_gps-x_est_gps_startup;
                                pos_start.y = y_est_gps-y_est_gps_startup;
                                pos_start.z = z_est_gps;
                                pos_start.yaw = yaw_est_gps;
                            }
                            /*
                            else if (mode == SN_VIO_POS_HOLD_MODE)
                            {
                                pos_start.x = x_est_vio-x_est_vio_startup;
                                pos_start.y = y_est_vio-y_est_vio_startup;
                                pos_start.z = z_est_vio;
                                pos_start.yaw = yaw_est_vio;
                            }
                            */
                            else
                            {
                                pos_start.x = x_est-x_est_startup;
                                pos_start.y = y_est-y_est_startup;
                                pos_start.z = z_est;
                                pos_start.yaw = yaw_est;
                            }

                            DEBUG("[%d] customized_plan_mission plan_step_total:%d, pos_start: [%f,%f,%f,%f]\n",
                                    loop_counter, plan_step_total, pos_start.x, pos_start.y, pos_start.z, pos_start.yaw);

                            for (int k = 0; k < plan_step_total; k++)
                            {
                                pos_start.yaw_only = false;
                                step_count = atoi(customized_plan_steps[k*2+1].c_str());

                                DEBUG("[%d] plan_step, plan_count: [%s, %d]\n",
                                    loop_counter, customized_plan_steps[k*2].c_str(), step_count);

                                if (customized_plan_steps[k*2].compare(PLAN_LEFT) == 0)
                                {
                                    pos_start.x   = pos_start.x - plan_unit*step_count*sin(pos_start.yaw);
                                    pos_start.y   = pos_start.y + plan_unit*step_count*cos(pos_start.yaw);
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_RIGHT) == 0)
                                {
                                    pos_start.x   = pos_start.x + plan_unit*step_count*sin(pos_start.yaw);
                                    pos_start.y   = pos_start.y - plan_unit*step_count*cos(pos_start.yaw);
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_FRONT) == 0)
                                {
                                    pos_start.x   = pos_start.x + plan_unit*step_count*cos(pos_start.yaw);
                                    pos_start.y   = pos_start.y + plan_unit*step_count*sin(pos_start.yaw);
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_BACK) == 0)
                                {
                                    pos_start.x   = pos_start.x - plan_unit*step_count*cos(pos_start.yaw);
                                    pos_start.y   = pos_start.y - plan_unit*step_count*sin(pos_start.yaw);
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_UP) == 0)
                                {
                                    pos_start.z   = pos_start.z + plan_unit*step_count;
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_DOWN) == 0)
                                {
                                    pos_start.z   = pos_start.z - plan_unit*step_count;
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_ZOOM_IN) == 0)
                                {
                                    pos_start.x   = pos_start.x + plan_unit*step_count*0.707*cos(pos_start.yaw);
                                    pos_start.y   = pos_start.y + plan_unit*step_count*0.707*sin(pos_start.yaw);
                                    pos_start.z   = pos_start.z - plan_unit*step_count*0.707;
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_ZOOM_OUT) == 0)
                                {
                                    pos_start.x   = pos_start.x - plan_unit*step_count*0.707*cos(pos_start.yaw);
                                    pos_start.y   = pos_start.y - plan_unit*step_count*0.707*sin(pos_start.yaw);
                                    pos_start.z   = pos_start.z + plan_unit*step_count*0.707;
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_CLOCKWISE) == 0)
                                {
                                    for (int ct = 0; ct < step_count; ct++)
                                    {
                                        pos_start.yaw   = pos_start.yaw - M_PI/2;

                                        if (pos_start.yaw > M_PI)
                                        {
                                            pos_start.yaw = pos_start.yaw - 2*M_PI;
                                        }
                                        else if (pos_start.yaw < -M_PI)
                                        {
                                            pos_start.yaw = pos_start.yaw + 2*M_PI;
                                        }
                                    }
                                    pos_start.yaw_only = true;
                                }
                                else if (customized_plan_steps[k*2].compare(PLAN_ANTI_CLOCKWISE) == 0)
                                {
                                    for (int ct = 0; ct < step_count; ct++)
                                    {
                                        pos_start.yaw   = pos_start.yaw + M_PI/2;

                                        if (pos_start.yaw > M_PI)
                                        {
                                            pos_start.yaw = pos_start.yaw - 2*M_PI;
                                        }
                                        else if (pos_start.yaw < -M_PI)
                                        {
                                            pos_start.yaw = pos_start.yaw + 2*M_PI;
                                        }
                                    }
                                    pos_start.yaw_only = true;
                                }
                                else
                                {
                                    DEBUG("Unknown plan=%s\n", customized_plan_steps[0].c_str());
                                }
                                customized_plan_positions.push_back(pos_start);
                            }

                            calcPlanPoint = false;

                            for (int k = 0; k < (int)customized_plan_positions.size(); k++)
                            {
                                DEBUG("[%d] customized_plan_positions #%d: [%f,%f,%f,%f], yaw_only:%d\n", loop_counter, k,
                                            customized_plan_positions[k].x, customized_plan_positions[k].y,
                                            customized_plan_positions[k].z, customized_plan_positions[k].yaw,
                                            customized_plan_positions[k].yaw_only);
                            }
                        }
#endif
                        state = MissionState::IN_MOTION;
                        entering_loiter = true;
                    }
                    else if (trail_navigation_mission)
                    {
                        if (t_now - t_loiter_start > kLoiterTime)
                        {
                            state = MissionState::IN_MOTION;
                            entering_loiter = true;
                        }
                    }
                }
            }
            // All the missions execute in the IN_MOTION
            else if (state == MissionState::IN_MOTION)
            {
                float accel_max  = 1.5;         // m/s
                float stopping_accel = 1.0;     // m/s

                static double t_last = 0;

                static float vel_x_des_sent = 0;
                static float vel_y_des_sent = 0;
                static float vel_z_des_sent = 0;
                static float vel_yaw_des_sent = 0;

                float command_diff_x;
                float command_diff_y;
                float command_diff_z;
                float command_diff_yaw;

                static float vel_x_target = 0;
                static float vel_y_target = 0;
                static float vel_z_target = 0;
                static float vel_yaw_target = 0;

                yaw_vel_des = 0;

                // Land have the highest priority
                if (udp_msg_array.size() >= 1 && (udp_msg_array[0].compare(SNAV_CMD_LAND) ==0))
                {
                    state = MissionState::LANDING;
                    loop_counter++;
                    continue;
                }

                if (panorama_mission)
                {
                    command_diff_yaw = panorama_positions[current_position].yaw - yaw_des;

                    DEBUG("[%d] [panorama_mission yaw_des command_diff_yaw]: [%f,%f]\n",
                                loop_counter, yaw_des, command_diff_yaw);

                    if (command_diff_yaw > M_PI)
                    {
                        command_diff_yaw = command_diff_yaw - 2*M_PI;
                    }
                    else if (command_diff_yaw < -M_PI)
                    {
                        command_diff_yaw = command_diff_yaw + 2*M_PI;
                    }

                    if (fabs(command_diff_yaw) > M_PI*0.25f)            //0.25*PI
                    {
                        state = MissionState::LOITER;
                        current_position =0;
                        panorama_mission=false;
                        calcPanoramaPoint = false;
                        continue;
                    }

                    if (fabs(command_diff_yaw) < 0.03)
                    {
                        // Close enough, move on
                        current_position++;

                        if ((current_position == 3) || (current_position == 6) || (current_position == 9))
                        {
                            send_panorama_flag = true;
                            if (current_position == 3)
                            {
                                strcpy(panorama_buff, "snap,1");
                            }
                            else if (current_position == 6)
                            {
                                strcpy(panorama_buff, "snap,2");
                            }
                            else if (current_position == 9)
                            {
                                strcpy(panorama_buff, "snap,3");
                            }
                        }

                        if (current_position >= panorama_positions.size())
                        {
                            // No more panorama_positions,
                            state = MissionState::LOITER;
                            current_position = 0;
                            panorama_mission=false;
                            calcPanoramaPoint = false;
                        }
                    }

                    vel_x_target = 0;
                    vel_y_target = 0;
                    vel_z_target = 0;

                    vel_yaw_target = command_diff_yaw/(angle_per*vel_target);

                    if (vel_yaw_target < 0)
                    {
                        vel_yaw_target = -0.5;
                    }
                    else
                    {
                        vel_yaw_target = 0.5;
                    }

                    DEBUG("[%d] [panorama_mission current_position vel_yaw_target]: [%d %f]\n",
                                        loop_counter,current_position,vel_yaw_target);
                }
                else if(circle_mission)
                {
                    bool stop_flag = false;

                    FlatVars current_state = {x_des-x_est_startup, y_des-y_est_startup, z_des-z_est_startup, yaw_des};
                    FlatVars last_vel = {vel_x_target, vel_y_target, vel_z_target, vel_yaw_target};
                    FlatVars des_pos = {circle_positions[current_position].x, circle_positions[current_position].y,
                                        circle_positions[current_position].z, circle_positions[current_position].yaw};

                    // Stop only at last waypoint. If stopping at all waypoints is desired, make always true
                    if (current_position == circle_positions.size())
                    {
                        stop_flag = true;
                    }
                    else
                    {
                        stop_flag = false;
                    }

                    // Return -1 means the first point, need to set the start vel to zero.
                    if (goto_waypoint_for_circle(current_state, des_pos, last_vel, stop_flag, &output_vel, &wp_goal_ret) == -1)
                    {
                        if (current_position == 0)
                        {
                            output_vel.x = 0;
                            output_vel.y = 0;
                            output_vel.z = 0;
                            output_vel.yaw = 0;
                        }
                    }

                    vel_x_target = output_vel.x;
                    vel_y_target = output_vel.y;
                    vel_z_target = output_vel.z;
                    vel_yaw_target = output_vel.yaw;

                    /*
                    if (gps_enabled && mag_status != SN_DATA_VALID)
                    {
                        // Not allowed to circel when use gps_mode and mag is invalid
                        state = MissionState::LOITER;
                        current_position =0;
                        circle_mission=false;
                        calcCirclePoint = false;

                        loop_counter++;
                        continue;
                    }
                    */

                    // If reached waypoint, goto next
                    if ((wp_goal_ret&wp_goal_mask) == 0)
                    {
                        DEBUG("[%d][circle_mission reach point[%d]\n", loop_counter,current_position);

                        current_position++;
                        wp_goal_ret = 0b11111111;

                        if (current_position >= circle_positions.size())
                        {
                            state = MissionState::LOITER;
                            current_position    = 0;
                            circle_mission      = false;
                            calcCirclePoint     = false;
                        }
                    }

                    DEBUG("[%d][%d] [circle_mission current_position vel_x_target vel_y_target vel_z_target vel_yaw_target]: [%f %f %f %f]\n",
                                    loop_counter, current_position, vel_x_target, vel_y_target, vel_z_target, vel_yaw_target);
                }
                else if (trail_navigation_mission)
                {
                    command_diff_x = CalcAxisDistance(trail_navigation_positions[current_position].longitude/1e7,
                                                      (float)posGpsCurrent.longitude/1e7);

                    command_diff_y = CalcAxisDistance(trail_navigation_positions[current_position].latitude/1e7,
                                                      (float)posGpsCurrent.latitude/1e7);

                    command_diff_z = 0;

                    distance_to_dest = CalcDistance(trail_navigation_positions[current_position].latitude/1e7,
                                                    trail_navigation_positions[current_position].longitude/1e7,
                                                    (float)posGpsCurrent.latitude/1e7,
                                                    (float)posGpsCurrent.longitude/1e7);


                    if (distance_to_dest<0.01)
                    {
                        distance_to_dest = 0.01;    // To prevent dividing by zero
                    }

                    vel_x_target = command_diff_x/distance_to_dest*vel_target;
                    vel_y_target = command_diff_y/distance_to_dest*vel_target;
                    vel_z_target = command_diff_z/distance_to_dest*vel_target;

                    if (distance_to_dest < 2.5 )    //About 150*150   x,y 1.5m
                    {
                        // If it is on the last waypoint then slow down before stopping
                        float stopping_vel = sqrt(2*stopping_accel*distance_to_dest);
                        if (stopping_vel < vel_target)
                        {
                            vel_target = stopping_vel;
                        }
                    }

                    if (distance_to_dest < 1) // 1m
                    {
                        // Close enough, move on
                        current_position++;
                        if (current_position >= trail_navigation_positions.size())
                        {
                            state = MissionState::LOITER;
                            trail_navigation_mission = false;
                        }
                    }
                }
                else if (customized_plan_mission)
                {
#if 0
                    bool stop_flag = false;
                    bool position_yaw_only = false;

                    FlatVars current_state = {x_des-x_est_startup, y_des-y_est_startup, z_est, yaw_des};
                    FlatVars last_vel = {vel_x_target, vel_y_target, vel_z_target, vel_yaw_target};
                    FlatVars des_pos = current_state;

                    if (current_position == (size_t)(plan_step_total-1))
                    {
                        stop_flag = true;
                    }

                    if (current_position < customized_plan_positions.size())
                    {
                        des_pos.x = customized_plan_positions[current_position].x;
                        des_pos.y = customized_plan_positions[current_position].y;
                        des_pos.z = customized_plan_positions[current_position].z;
                        des_pos.yaw = customized_plan_positions[current_position].yaw;

                        position_yaw_only = customized_plan_positions[current_position].yaw_only;
                    }

                    DEBUG("[%d]  customized_plan_mission current_state: [%f,%f,%f,%f]\n",
                                loop_counter, current_state.x, current_state.y, current_state.z, current_state.yaw);
                    DEBUG("[%d]  customized_plan_mission des_pos: [%f,%f,%f,%f]\n",
                                loop_counter, des_pos.x, des_pos.y, des_pos.z, des_pos.yaw);

                    // Return -1 means the first point, need to set the start vel to zero.
                    if (goto_waypoint_for_customized_plan(current_state, des_pos, last_vel, stop_flag, &output_vel, &wp_goal_ret, position_yaw_only) == -1)
                    {
                        if (current_position == 0)
                        {
                            output_vel.x = 0;
                            output_vel.y = 0;
                            output_vel.z = 0;
                            output_vel.yaw = 0;
                        }
                    }

                    vel_x_target = output_vel.x;
                    vel_y_target = output_vel.y;
                    vel_z_target = output_vel.z;
                    vel_yaw_target = output_vel.yaw;


                    // If reached waypoint is met, increment waypoint
                    if ((wp_goal_ret&wp_goal_mask) == 0)
                    {
                        DEBUG("[%d][customized_plan_mission reach point[%d]\n", loop_counter,current_position);

                        char current_plan_step[TMP_BUFF_LEN];
                        memset(current_plan_step, 0, TMP_BUFF_LEN);
                        sprintf(current_plan_step, "%d", current_position);

                        memset(result_to_client, 0, MAX_BUFF_LEN);
                        sprintf(result_to_client, "%s", SNAV_TASK_SHOW_PLAN_STEP_COMPLETE);
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, current_plan_step);

                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                            (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_TASK_SHOW_PLAN_STEP_COMPLETE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);

                        current_position++;
                        wp_goal_ret = 0b11111111;

                        if (current_position >= (size_t)plan_step_total)
                        {
                            state = MissionState::LOITER;
                            current_position        = 0;
                            customized_plan_mission = false;
                            calcPlanPoint           = false;
                        }
                        else
                        {
                            PlanPosition pos_current;

                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                pos_current.x = x_est_gps-x_est_gps_startup;
                                pos_current.y = y_est_gps-y_est_gps_startup;
                                pos_current.z = z_est_gps;  //-z_est_gps_startup;
                                pos_current.yaw = yaw_est_gps;
                            }
                            else
                            {
                                pos_current.x = x_est-x_est_startup;
                                pos_current.y = y_est-y_est_startup;
                                pos_current.z = z_est;  //-z_est_startup;
                                pos_current.yaw = yaw_est;    // - yaw_est_startup;
                            }
                            pos_current.yaw_only = false;

                            DEBUG("[%d] customized_plan_mission current_position:%d, pos_current: [%f,%f,%f,%f]\n",
                                    loop_counter, current_position, pos_current.x, pos_current.y, pos_current.z, pos_current.yaw);

                            if (customized_plan_steps[current_position].compare(PLAN_LEFT) == 0)
                            {
                                pos_current.x   = pos_current.x - plan_unit*sin(pos_current.yaw);
                                pos_current.y   = pos_current.y + plan_unit*cos(pos_current.yaw);
                            }
                            else if (customized_plan_steps[current_position].compare(PLAN_RIGHT) == 0)
                            {
                                pos_current.x   = pos_current.x + plan_unit*sin(pos_current.yaw);
                                pos_current.y   = pos_current.y - plan_unit*cos(pos_current.yaw);
                            }
                            else if (customized_plan_steps[current_position].compare(PLAN_FRONT) == 0)
                            {
                                pos_current.x   = pos_current.x + plan_unit*cos(pos_current.yaw);
                                pos_current.y   = pos_current.y + plan_unit*sin(pos_current.yaw);
                            }
                            else if (customized_plan_steps[current_position].compare(PLAN_BACK) == 0)
                            {
                                pos_current.x   = pos_current.x - plan_unit*cos(pos_current.yaw);
                                pos_current.y   = pos_current.y - plan_unit*sin(pos_current.yaw);
                            }
                            else if (customized_plan_steps[current_position].compare(PLAN_UP) == 0)
                            {
                                pos_current.z   = pos_current.z + plan_unit;
                            }
                            else if (customized_plan_steps[current_position].compare(PLAN_DOWN) == 0)
                            {
                                pos_current.z   = pos_current.z - plan_unit;
                            }
                            else if (customized_plan_steps[current_position].compare(PLAN_CLOCKWISE) == 0)
                            {
                                pos_current.yaw   = pos_current.yaw - M_PI/2;

                                if (pos_current.yaw > M_PI)
                                {
                                    pos_current.yaw = pos_current.yaw - 2*M_PI;
                                }
                                else if (pos_current.yaw < -M_PI)
                                {
                                    pos_current.yaw = pos_current.yaw + 2*M_PI;
                                }
                                pos_current.yaw_only = true;
                            }
                            else if (customized_plan_steps[current_position].compare(PLAN_ANTI_CLOCKWISE) == 0)
                            {
                                pos_current.yaw   = pos_current.yaw + M_PI/2;

                                if (pos_current.yaw > M_PI)
                                {
                                    pos_current.yaw = pos_current.yaw - 2*M_PI;
                                }
                                else if (pos_current.yaw < -M_PI)
                                {
                                    pos_current.yaw = pos_current.yaw + 2*M_PI;
                                }
                                pos_current.yaw_only = true;
                            }
                            else
                            {
                                DEBUG("Unknown plan=%s\n", customized_plan_steps[current_position].c_str());
                            }

                            DEBUG("[%d] customized_plan_positions push pos[%d]:[%f,%f,%f,%f,%d]\n",
                                            loop_counter, current_position, pos_current.x, pos_current.y,
                                            pos_current.z, pos_current.yaw, pos_current.yaw_only);
                            customized_plan_positions.push_back(pos_current);
                        }
                    }
#else
                    bool stop_flag = false;
                    bool position_yaw_only = false;

                    FlatVars current_state = {x_des-x_est_startup, y_des-y_est_startup, z_est, yaw_des};
                    FlatVars last_vel = {vel_x_target, vel_y_target, vel_z_target, vel_yaw_target};
                    FlatVars des_pos = {customized_plan_positions[current_position].x,
                                        customized_plan_positions[current_position].y,
                                        customized_plan_positions[current_position].z,
                                        customized_plan_positions[current_position].yaw};

                    position_yaw_only = customized_plan_positions[current_position].yaw_only;

                    // Stop only at last waypoint. If stopping at all waypoints is desired, make always true
                    if ((position_yaw_only == true) || (current_position == customized_plan_positions.size()))
                    {
                        stop_flag = true;
                    }
                    else
                    {
                        stop_flag = false;
                    }

                    DEBUG("[%d]  customized_plan_mission current_state: [%f,%f,%f,%f]\n",
                                loop_counter, current_state.x, current_state.y, current_state.z, current_state.yaw);
                    DEBUG("[%d]  customized_plan_mission des_pos: [%f,%f,%f,%f]\n",
                                loop_counter, des_pos.x, des_pos.y, des_pos.z, des_pos.yaw);

                    // Return -1 means the first point, need to set the start vel to zero.
                    if (goto_waypoint_for_customized_plan(current_state, des_pos, last_vel, stop_flag, &output_vel, &wp_goal_ret, position_yaw_only) == -1)
                    {
                        if (current_position == 0)
                        {
                            output_vel.x = 0;
                            output_vel.y = 0;
                            output_vel.z = 0;
                            output_vel.yaw = 0;
                        }
                    }

                    DEBUG("[%d]  customized_plan_mission output_vel: [%f,%f,%f,%f]\n",
                                loop_counter, output_vel.x, output_vel.y, output_vel.z, output_vel.yaw);

                    vel_x_target = output_vel.x;
                    vel_y_target = output_vel.y;
                    vel_z_target = output_vel.z;
                    vel_yaw_target = output_vel.yaw;

                    // If reached waypoint, goto next
                    if ((wp_goal_ret&wp_goal_mask) == 0)
                    {
                        DEBUG("[%d][customized_plan_mission reach point[%d]\n", loop_counter,current_position);

                        char current_plan_step[TMP_BUFF_LEN];
                        memset(current_plan_step, 0, TMP_BUFF_LEN);
                        sprintf(current_plan_step, "%d", current_position);

                        memset(result_to_client, 0, MAX_BUFF_LEN);
                        sprintf(result_to_client, "%s", SNAV_TASK_SHOW_PLAN_STEP_COMPLETE);
                        strcat(result_to_client, STR_SEPARATOR);
                        strcat(result_to_client, current_plan_step);

                        length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                            (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                        DEBUG("[%d] SNAV_TASK_SHOW_PLAN_STEP_COMPLETE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);

                        current_position++;
                        wp_goal_ret = 0b11111111;

                        if (current_position >= customized_plan_positions.size())
                        {
                            state = MissionState::LOITER;
                            current_position        = 0;
                            customized_plan_mission = false;
                            calcPlanPoint           = false;
                        }
                    }
#endif
                }
#ifdef AUTO_REDUCE_HEIGHT
                else if ((use_reduce_height == 1) && auto_reduce_height_mission)
                {
                    DEBUG("auto_reduce_height_mission\n");

                    bool stop_flag = false;

                    FlatVars current_state = {x_des-x_est_startup, y_des-y_est_startup, z_des, yaw_des};
                    FlatVars last_vel = {vel_x_target, vel_y_target, vel_z_target, vel_yaw_target};
                    FlatVars des_pos = {auto_reduce_height_position.x, auto_reduce_height_position.y,
                                        auto_reduce_height_position.z, auto_reduce_height_position.yaw};

                    DEBUG("[%d]  auto_reduce_height_mission current_state: [%f,%f,%f,%f]\n",
                                loop_counter, current_state.x, current_state.y, current_state.z, current_state.yaw);
                    DEBUG("[%d]  auto_reduce_height_mission des_pos: [%f,%f,%f,%f]\n",
                                loop_counter, des_pos.x, des_pos.y, des_pos.z, des_pos.yaw);

                    // Return -1 means the first point, need to set the start vel to zero.
                    if (goto_waypoint_for_reduce_height(current_state, des_pos, last_vel, stop_flag, &output_vel, &wp_goal_ret) == -1)
                    {
                        DEBUG("[%d]  goto_waypoint_for_reduce_height return -1\n", loop_counter);

                        output_vel.x = 0;
                        output_vel.y = 0;
                        output_vel.z = 0;
                        output_vel.yaw = 0;
                    }

                    vel_x_target = output_vel.x;
                    vel_y_target = output_vel.y;
                    vel_z_target = output_vel.z;
                    vel_yaw_target = output_vel.yaw;

                    // If reached waypoint is met, increment waypoint
                    if ((wp_goal_ret&wp_goal_mask) == 0)
                    {
                        DEBUG("[%d][auto_reduce_height_mission reach point\n", loop_counter);

                        wp_goal_ret = 0b11111111;

                        state = MissionState::LOITER;
                        auto_reduce_height_mission = false;
                    }
                }
#endif
                else if(face_mission) // Cuiyc add face detect begin
                {
                    //static float f_dest_yaw,f_dest_x,f_dest_y;
                    static float distance_remain_x,distance_remain_y,distance_remain_z;
                    static float forword_dis , parallel_dis,angle_face_offset;

                    if(cur_body.newP )
                    {
                        angle_face_offset = cur_body.angle*M_PI/180;

                        if(fabs(angle_face_offset) > min_angle_offset && face_rotate_switch)
                        {
                            if(cur_body.distance >(safe_distance -0.2f))
                            {
                                distance_remain_x = 0;
                                distance_remain_y = 0;
                                distance_remain_z = 0;
                                vel_yaw_target = angle_face_offset*1.5;
                            }
                            else  // too close back first
                            {
                                forword_dis = cur_body.distance-safe_distance;
                                parallel_dis = 0;
                                distance_to_dest = sqrt(forword_dis*forword_dis + parallel_dis*parallel_dis);

                                DEBUG("[%d] face_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
                                            loop_counter,forword_dis,parallel_dis,distance_to_dest);

                                distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
                                distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;
                                distance_remain_z = 0;
                                vel_yaw_target = 0;
                            }
                        }
                        else
                        {
                            forword_dis = cur_body.distance-safe_distance;
                            parallel_dis = tan(angle_face_offset)*cur_body.distance;

                            distance_to_dest = sqrt(forword_dis*forword_dis + parallel_dis*parallel_dis);

                            DEBUG("[%d] face_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
                                    loop_counter,forword_dis,parallel_dis,distance_to_dest);

                            distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
                            distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;
                            distance_remain_z = cur_body.hegith_calib;
                            vel_yaw_target = 0;
                        }
                        cur_body.newP = false;
                    }

                    if(((fabs(angle_face_offset))< min_angle_offset && fabs(distance_to_dest) <0.05f)
                        || !cur_body.have_face)
                    {
                        if(fabs(distance_remain_z) > 0.1f)
                        {
                            vel_z_target = distance_remain_z*vel_target;
                            if(vel_z_target >vel_target) vel_z_target =vel_target;
                        }
                        else
                        {
                            state = MissionState::LOITER;
                            vel_z_target = 0;
                            DEBUG(" face_mission follow face-> LOITER\n" );
                            face_mission = false;
                            continue;
                        }
                    }

                    if(cur_body.handle_gesture == GESTURE_TAKEPHOTO) // taking photo keep LOITER
                    {
                        state = MissionState::LOITER;
                        DEBUG(" hand gesture working follow face-> LOITER\n" );
                        face_mission = false;
                        continue;
                    }

                    //for test rotate
                    distance_remain_x = distance_remain_x - vel_x_des_sent*0.02f; //20ms a tick
                    distance_remain_y = distance_remain_y - vel_y_des_sent*0.02f;
                    distance_remain_z = distance_remain_z - vel_z_des_sent*0.02f;

                    distance_to_dest = sqrt(distance_remain_x*distance_remain_x +
                                             distance_remain_y*distance_remain_y);

                    DEBUG("[%d] face_mission [distance_remain_x distance_remain_y,distance_remain_z]: [%f %f %f]\n",
                            loop_counter,distance_remain_x,distance_remain_y,distance_remain_z);

                    if(fabs(distance_remain_x) <0.05f)
                        distance_remain_x =0;
                    if(fabs(distance_remain_y) <0.05f)
                        distance_remain_y =0;
                    if(fabs(distance_remain_z) < 0.1f)
                        distance_remain_z =0;

                    if(fabs(distance_to_dest) >0.05f)
                    {
                        if(forword_dis <0)
                        {
                            vel_x_target = distance_remain_x*face_vel_limit;
                            vel_y_target = distance_remain_y*face_vel_limit;
                        }
                        else{
                            vel_x_target = distance_remain_x*face_vel_limit;
                            vel_y_target = distance_remain_y*face_vel_limit;
                        }

                        if(vel_x_target >face_vel_limit) vel_x_target =face_vel_limit;
                        if(vel_y_target >face_vel_limit) vel_y_target =face_vel_limit;
                        if(vel_x_target < -1*face_vel_limit) vel_x_target =-1*face_vel_limit;
                        if(vel_y_target < -1*face_vel_limit) vel_y_target =-1*face_vel_limit;
                    }
                    else
                    {
                        vel_x_target = 0;
                        vel_y_target = 0;
                    }

                    if(fabs(distance_remain_z) > 0.1f)
                    {
                        vel_z_target = distance_remain_z*vel_target;
                        if(vel_z_target >vel_target) vel_z_target = vel_target;
                    }
                    else
                        vel_z_target = 0;

                    if(vel_yaw_target > M_PI*0.5f)
                        vel_yaw_target = M_PI*0.5f;

                    DEBUG("[%d] face_mission [vel_x vel_y vel_z distance vel_yaw]: [%f %f %f %f %f]\n",
                        loop_counter,vel_x_target,vel_y_target,vel_z_target,distance_to_dest,vel_yaw_target);
                }
                else if(body_mission)
                {
                    //static float f_dest_yaw,f_dest_x,f_dest_y;
                    static float speed ,angle_body_offset;
                    static float backup_speed ,backup_angle;
                    angle_body_offset = cur_body.angle*M_PI/180;

                    if(body_follow_prallel)
                    {
                        if((fabs(angle_body_offset)< min_angle_offset)
                            || !cur_body.have_body)
                        {
                            state = MissionState::LOITER;
                            DEBUG("follow body_mission prallel follow body-> LOITER\n");
                            body_mission = false;
                            continue;
                        }
                        else
                        {
                            float yaw_body;

                            if (gps_enabled /*&& (gps_status == SN_DATA_VALID)*/)
                                yaw_body = (float)snav_data->gps_pos_vel.yaw_estimated;
                            else
                                yaw_body = (float)snav_data->optic_flow_pos_vel.yaw_estimated;

                            vel_x_target =  sin(yaw_body)*cur_body.angle/51.0f*body_speed_limit;
                            vel_y_target =  cos(yaw_body)*cur_body.angle/51.0f*body_speed_limit;
                            vel_z_target =0;

                            if(vel_x_target >body_speed_limit)
                                vel_x_target = body_speed_limit;

                            if(vel_y_target >body_speed_limit)
                                vel_y_target = body_speed_limit;

                            DEBUG(" [%d] follow body_mission prallel [vel_x_target vel_y_target yaw_body]: [%f %f %f]\n",
                            loop_counter,vel_x_target,vel_y_target,yaw_body);

                        }
                    }
                    else
                    {
                    speed = sqrt(vel_x_des_sent*vel_x_des_sent +vel_y_des_sent*vel_y_des_sent);

                    if((fabs(angle_body_offset)< min_angle_offset && fabs(cur_body.velocity) <0.05f)
                        || !cur_body.have_body)
                    {
                        state = MissionState::LOITER;
                        DEBUG("body_mission follow body-> LOITER\n");
                        body_mission = false;
                        continue;
                    }

                    if(fabs(angle_body_offset) > min_angle_offset)
                    {
                        vel_yaw_target = angle_body_offset*vel_target*1.5;

                        float curheight = z_est - z_est_startup;
                        float sonarheight = snav_data->sonar_0_raw.range;
                        if(sonarheight > 1.0f && sonarheight < 2.5f)
                            curheight = sonarheight;

                        DEBUG("[%d] body_mission angle_body_offset: [%f] curheight:%f,sonarheight:%f\n",
                                loop_counter,angle_body_offset,curheight,sonarheight);

                        if(true/*curheight>1.9f && curheight <2.1f*/)
                        {
                            vel_x_target = cos(yaw_est)*cur_body.velocity;
                            vel_y_target = sin(yaw_est)*cur_body.velocity;
                            vel_z_target =0;

                            if(vel_x_target >body_speed_limit)
                                vel_x_target = body_speed_limit;

                            if(vel_y_target >body_speed_limit)
                                vel_y_target = body_speed_limit;

                            DEBUG(" [%d] follow body_mission [vel_x_target vel_y_target vel_yaw_target]: [%f %f %f]\n",
                            loop_counter,vel_x_target,vel_y_target,vel_yaw_target);
                        }
                        else
                        {
                            vel_x_target = 0;
                            vel_y_target = 0;
                            vel_z_target = (2.0f - curheight)*vel_target*1.5;

                            if(vel_z_target >0.75f)vel_z_target = 0.75;
                            DEBUG(" [%d]  body_mission correct height [vel_z_target curheight vel_yaw_target]: [%f %f %f]\n",
                            loop_counter,vel_z_target,curheight,vel_yaw_target);
                        }
                    }
                    }
                }

                if(cur_body.have_body && body_follow_switch)
                {
                    float body_offset ;
                    body_offset = M_PI*cur_body.angle/180;
                    DEBUG("followme have_body\n" );

                    if ((fabs(body_offset)>min_angle_offset
                            || fabs(cur_body.velocity)>0.05f)
                        && !panorama_mission
                        && !fly_test_mission
                        && !rotation_test_mission
                        && !circle_mission
                        && !return_mission
                        && !trail_navigation_mission
                        && !customized_plan_mission)
                    {
                        body_mission= true;
                        DEBUG("body angle:%f    velocity:%f\n",cur_body.angle,cur_body.velocity);
                        DEBUG("followme  BODY_FOLLOW\n" );
                    }
                }

                if(follow_reset_yaw)
                {
                   vel_yaw_target = 0;
                   follow_reset_yaw = false;
                }// cuiyc add face detect end

                // Return mission
                if (return_mission)
                {
                    /*
                    if (gps_enabled && mag_status != SN_DATA_VALID)
                    {
                        // Not allowed to return home when use gps_mode and mag is invalid
                        state = MissionState::LOITER;
                        return_mission = false;

                        loop_counter++;
                        continue;
                    }
                    */

                    // Two step: turn yaw to home first, then fly directly to home.
                    if (fly_home)
                    {
                        if (gps_enabled)
                        {
                            float distance_gps_home = sqrt((x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
                                                            + (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps));
                            yaw_gps_target_home = atan2(y_est_gps_startup-y_est_gps, x_est_gps_startup-x_est_gps);

                            DEBUG("[%d] return_mission [distance_gps_home, yaw_gps_target_home]: [%f, %f]\n"
                                        , loop_counter,distance_gps_home, yaw_gps_target_home);

                            // Go to home waypoint
                            if (distance_gps_home > 2)
                            {
                                goto_waypoint_with_gps({x_est_gps, y_est_gps, z_est_gps, yaw_est_gps}
                                                        , {x_est_gps_startup, y_est_gps_startup, z_est_gps, yaw_gps_target_home}
                                                        , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                                        , true, &output_vel, &wp_goal_ret);
                            }
                            else
                            {
                                goto_waypoint_with_gps({x_est_gps, y_est_gps, z_est_gps, yaw_est_gps}
                                                        , {x_est_gps_startup, y_est_gps_startup, z_est_gps, yaw_est_gps}
                                                        , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                                        , true, &output_vel, &wp_goal_ret);
                            }
                        }
                        /*
                        else if (mode == SN_VIO_POS_HOLD_MODE)
                        {
                            float distance_home = sqrt((x_est_vio_startup-x_est_vio)*(x_est_vio_startup-x_est_vio)
                                                         + (y_est_vio_startup-y_est_vio)*(y_est_vio_startup-y_est_vio));

                            yaw_target_home = atan2(y_est_vio_startup-y_est_vio, x_est_vio_startup-x_est_vio);

                            DEBUG("[%d] return_mission [distance_home, yaw_target_home]: [%f, %f]\n"
                                        , loop_counter,distance_home, yaw_target_home);

                            // Go to home waypoint
                            if (distance_home > 0.3)
                            {
                                goto_waypoint({x_des_vio, y_des_vio, z_des_vio, yaw_des_vio}
                                                , {x_est_vio_startup, y_est_vio_startup, z_des_vio, yaw_target_home}
                                                , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                                , true, &output_vel, &wp_goal_ret);
                            }
                            else
                            {
                                goto_waypoint({x_des_vio, y_des_vio, z_des_vio, yaw_des_vio}
                                                , {x_est_vio_startup, y_est_vio_startup, z_des_vio, yaw_des_vio}
                                                , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                                , true, &output_vel, &wp_goal_ret);
                            }
                        }
                        */
                        else
                        {
                            float distance_home = sqrt((x_est_startup-x_est)*(x_est_startup-x_est)
                                                         + (y_est_startup-y_est)*(y_est_startup-y_est));

                            yaw_target_home = atan2(y_est_startup-y_est, x_est_startup-x_est);

                            DEBUG("[%d] return_mission [distance_home, yaw_target_home]: [%f, %f]\n"
                                        , loop_counter,distance_home, yaw_target_home);

                            // Go to home waypoint
                            if (distance_home > 0.3)
                            {
                                goto_waypoint({x_des, y_des, z_des, yaw_des}
                                                , {x_est_startup, y_est_startup, z_des, yaw_target_home}
                                                , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                                , true, &output_vel, &wp_goal_ret);
                            }
                            else
                            {
                                goto_waypoint({x_des, y_des, z_des, yaw_des}
                                                , {x_est_startup, y_est_startup, z_des, yaw_des}
                                                , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                                , true, &output_vel, &wp_goal_ret);
                            }

                        }

                        gohome_x_vel_des = output_vel.x;
                        gohome_y_vel_des = output_vel.y;
                        gohome_z_vel_des = output_vel.z;
                        gohome_yaw_vel_des = output_vel.yaw;

                        DEBUG("[%d]:return_mission direct fly_home wp_goal_ret,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des:[%f,%f,%f,%f]\n",
                                    loop_counter,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des);


                        float distance_from_home = sqrt((x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
                                                         + (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps));

                        if (gps_enabled)
                        {
                            if (((wp_goal_ret&wp_goal_mask) == 0) ||  (distance_from_home < 2))
                            {
                                // If home, enter landing
                                wp_goal_ret = 0b11111111;
                                state = MissionState::LANDING;
                                return_mission = false;

                                gohome_x_vel_des = 0;
                                gohome_y_vel_des = 0;
                                gohome_z_vel_des = 0;
                                gohome_yaw_vel_des = 0;

                                fly_home = false;
                            }
                        }
                        else
                        {
                            if ((wp_goal_ret&wp_goal_mask) == 0)
                            {
                                // If home, enter landing
                                wp_goal_ret = 0b11111111;
                                state = MissionState::LANDING;
                                return_mission = false;

                                gohome_x_vel_des = 0;
                                gohome_y_vel_des = 0;
                                gohome_z_vel_des = 0;
                                gohome_yaw_vel_des = 0;

                                fly_home = false;
                            }
                        }
                    }
                    else
                    {
                        if (gps_enabled)
                        {
                            goto_waypoint_with_gps({x_des_gps, y_des_gps, z_des_gps, yaw_des_gps}
                                                    , {x_des_gps, y_des_gps, z_des_gps, yaw_gps_target_home}
                                                    , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                                    , true, &output_vel, &wp_goal_ret);
                        }
                        else
                        {
                            goto_waypoint({x_des, y_des, z_des, yaw_des}
                                            , {x_des, y_des, z_des, yaw_target_home}
                                            , {gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}
                                            , true, &output_vel, &wp_goal_ret);
                        }

                        gohome_x_vel_des = output_vel.x;
                        gohome_y_vel_des = output_vel.y;
                        gohome_z_vel_des = output_vel.z;
                        gohome_yaw_vel_des = output_vel.yaw;

                        DEBUG("[%d]:return_mission fly_raw wp_goal_ret,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des:[%f,%f,%f,%f]\n",
                                    loop_counter,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des);

                        if ((wp_goal_ret&wp_goal_mask) == 0)
                        {
                            // If waypoint is reached (facing home), enter Fly_home
                            wp_goal_ret = 0b11111111;
                            fly_home = true;
                        }
                    }
                }
                else        /* Handle normal task in MissionState::IN_MOTION */
                {
                    float delT;

                    if (t_last != 0)
                    {
                        delT = (t_now - t_last);
                    }
                    else
                    {
                        delT = 0.02;
                    }

                    t_last = t_now;

                    // Now converge the velocity to desired velocity
                    float v_del_max = accel_max*delT;

                    float vel_x_diff = (vel_x_target - vel_x_des_sent);
                    float vel_y_diff = (vel_y_target - vel_y_des_sent);
                    float vel_z_diff = (vel_z_target - vel_z_des_sent);
                    float vel_yaw_diff = (vel_yaw_target - vel_yaw_des_sent);

                    DEBUG("[%d] [vel_x_diff,vel_y_diff,vel_z_diff,vel_yaw_diff]: [%f,%f,%f,%f] \n",
                                        loop_counter,vel_x_diff,vel_y_diff,vel_z_diff,vel_yaw_diff);

                    float vel_diff_mag = sqrt(vel_x_diff*vel_x_diff +
                                              vel_y_diff*vel_y_diff +
                                              vel_z_diff*vel_z_diff);

                    if (vel_diff_mag < 0.01)
                    {
                        vel_diff_mag = 0.01;
                    }

                    if (vel_diff_mag < v_del_max)
                    {
                        // Send through the target velocity
                        vel_x_des_sent = vel_x_target;
                        vel_y_des_sent = vel_y_target;
                        vel_z_des_sent = vel_z_target;
                    }
                    else
                    {
                        // Converge to the target velocity at the max acceleration rate
                        vel_x_des_sent += vel_x_diff/vel_diff_mag*v_del_max;
                        vel_y_des_sent += vel_y_diff/vel_diff_mag*v_del_max;
                        vel_z_des_sent += vel_z_diff/vel_diff_mag*v_del_max;
                    }

                    // Smooth accel
                    if (vel_yaw_diff < v_del_max)
                    {
                        vel_yaw_des_sent = vel_yaw_target;
                    }
                    else
                    {
                        vel_yaw_des_sent += v_del_max;
                    }

                    yaw_vel_des = vel_yaw_des_sent;

                    x_vel_des = vel_x_des_sent;
                    y_vel_des = vel_y_des_sent;
                    z_vel_des = vel_z_des_sent;

                    DEBUG("[%d] [x_vel_des,y_vel_des,z_vel_des,yaw_vel_des]: [%f,%f,%f,%f] \n",
                                        loop_counter,x_vel_des,y_vel_des,z_vel_des,yaw_vel_des);
                }
            }
            else    /* Other MissionState */
            {
                // Unknown state has been encountered
                x_vel_des = 0;
                y_vel_des = 0;
                z_vel_des = 0;
                yaw_vel_des = 0;

                if (props_state == SN_PROPS_STATE_SPINNING && on_ground_flag == 1)
                {
                    sn_stop_props();
                }
            }


            // Status Check
            if ((props_state == SN_PROPS_STATE_NOT_SPINNING)
                && (on_ground_flag == 1)
                && ((state == MissionState::IN_MOTION)
                    || (state == MissionState::LOITER)))
            {
                state = MissionState::ON_GROUND;

                // Reset all the mission
                current_position = 0;

                fly_test_mission = false;
                rotation_test_mission = false;

                circle_mission = false;
                calcCirclePoint = false;

                panorama_mission = false;
                calcPanoramaPoint = false;

                trail_navigation_mission = false;
                customized_plan_mission = false;
                calcPlanPoint = false;

                return_mission = false;

                face_mission = false;
                body_mission = false;

                face_follow_switch = false;
                body_follow_switch = false;
            }


            // Reset all the mission when disconnect with the phone.
            if (!bHaveUdpClient
                && (props_state == SN_PROPS_STATE_SPINNING)
                && (state == MissionState::IN_MOTION
                    || state == MissionState::LOITER))
            {
                current_position = 0;

                fly_test_mission = false;
                rotation_test_mission = false;

                circle_mission = false;
                calcCirclePoint = false;

                panorama_mission = false;
                calcPanoramaPoint = false;

                trail_navigation_mission = false;
                customized_plan_mission = false;
                calcPlanPoint = false;

                face_mission = false;
                body_mission = false;

#ifdef DISCONN_AUTO_RETURN
                if (!return_mission)
                {
                    state = MissionState::LOITER;
                }
#else
                return_mission = false;
                state = MissionState::LOITER;
#endif


                face_follow_switch = false;
                body_follow_switch = false;

#ifndef  AUTO_FACE_TAKE_OFF
                send_face_follow_swither_flag = true;
                memset(face_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                strcpy(face_follow_swither_buff, "fdoff");

                send_body_follow_swither_flag = true;
                memset(body_follow_swither_buff,0,DOMAIN_BUFF_SIZE);
                strcpy(body_follow_swither_buff, "bdoff");
#endif
            }

            // Rotate velocity by estimated yaw angle before sending
            // This puts velocity in body-relative Z-up frame
            float x_vel_des_yawed = x_vel_des*cos(-yaw_est) - y_vel_des*sin(-yaw_est);
            float y_vel_des_yawed = x_vel_des*sin(-yaw_est) + y_vel_des*cos(-yaw_est);

            float gohome_x_vel_des_yawed = gohome_x_vel_des*cos(-yaw_est) - gohome_y_vel_des*sin(-yaw_est);
            float gohome_y_vel_des_yawed = gohome_x_vel_des*sin(-yaw_est) + gohome_y_vel_des*cos(-yaw_est);

            float cmd0 = 0;
            float cmd1 = 0;
            float cmd2 = 0;
            float cmd3 = 0;

            // Go from the commands in real units computed above to the
            // dimensionless commands that the interface is expecting using a
            // linear mapping
            if (return_mission)
            {
                DEBUG("[sn_apply_cmd_mapping gohome_x_vel_des_yawed, gohome_y_vel_des_yawed, gohome_z_vel_des, gohome_yaw_vel_des]: [%f,%f,%f,%f]\n",
                                             gohome_x_vel_des_yawed, gohome_y_vel_des_yawed, gohome_z_vel_des, gohome_yaw_vel_des);

                if (gps_enabled && mag_usable
                    && (gps_status == SN_DATA_VALID)
                    && ((t_now_for_gps - t_gps_invalid) > time_interval_of_gps_valid)
                    && ((t_des_now - t_gps_height_invalid) > time_interval_of_gps_valid))
                {
                    if (revise_height >= gps_mode_height)
                    {
                        sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                             gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                             gohome_z_vel_des, gohome_yaw_vel_des,
                                             &cmd0, &cmd1, &cmd2, &cmd3);
                    }
                    else
                    {
                        if ((sonar_status == SN_DATA_VALID)
                             && (snav_data->sonar_0_raw.range < sonar_valid_data_max)
                             && ((t_des_now - t_sonar_invalid) > time_interval_of_sonar_valid))
                        {
                            if (fly_test_mission
                                || rotation_test_mission
                                || circle_mission
                                || panorama_mission
                                || trail_navigation_mission
                                || customized_plan_mission
                                || return_mission
                                || face_mission
                                || body_mission)
                            {
                                sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                         gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                                         gohome_z_vel_des, gohome_yaw_vel_des,
                                                         &cmd0, &cmd1, &cmd2, &cmd3);
                            }
                            else
                            {
#ifdef VIO_OPTIC_FLOW_SWITCH
                                if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                                {
                                    sn_apply_cmd_mapping(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                         gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                                         gohome_z_vel_des, gohome_yaw_vel_des,
                                                         &cmd0, &cmd1, &cmd2, &cmd3);
                                }
                                else
#endif
                                {
                                    sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                         gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                                         gohome_z_vel_des, gohome_yaw_vel_des,
                                                         &cmd0, &cmd1, &cmd2, &cmd3);
                                }
                            }
                        }
                        else
                        {
                            sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                 gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                                 gohome_z_vel_des, gohome_yaw_vel_des,
                                                 &cmd0, &cmd1, &cmd2, &cmd3);
                        }
                    }
                }
                else
                {
                    if (fly_test_mission
                        || rotation_test_mission
                        || circle_mission
                        || panorama_mission
                        || trail_navigation_mission
                        || customized_plan_mission
                        || return_mission
                        || face_mission
                        || body_mission)
                    {
                        sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                 gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                                 gohome_z_vel_des, gohome_yaw_vel_des,
                                                 &cmd0, &cmd1, &cmd2, &cmd3);
                    }
                    else
                    {
 #ifdef VIO_OPTIC_FLOW_SWITCH
                        if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                        {
                            sn_apply_cmd_mapping(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                 gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                                 gohome_z_vel_des, gohome_yaw_vel_des,
                                                 &cmd0, &cmd1, &cmd2, &cmd3);
                        }
                        else
#endif
                        {
                            sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                 gohome_x_vel_des_yawed, gohome_y_vel_des_yawed,
                                                 gohome_z_vel_des, gohome_yaw_vel_des,
                                                 &cmd0, &cmd1, &cmd2, &cmd3);
                        }
                    }
                }
            }
            else
            {
                DEBUG("[%d] [sn_apply_cmd_mapping x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des]: [%f,%f,%f,%f]\n",
                                             loop_counter, x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des);

                if (gps_enabled && mag_usable
                    && (gps_status == SN_DATA_VALID)
                    && ((t_now_for_gps - t_gps_invalid) > time_interval_of_gps_valid)
                    && ((t_des_now - t_gps_height_invalid) > time_interval_of_gps_valid))
                {
                    if (revise_height >= gps_mode_height)
                    {
                        sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                             x_vel_des_yawed, y_vel_des_yawed,
                                             z_vel_des, yaw_vel_des,
                                             &cmd0, &cmd1, &cmd2, &cmd3);
                    }
                    else
                    {
                        if ((sonar_status == SN_DATA_VALID)
                             && (snav_data->sonar_0_raw.range < sonar_valid_data_max)
                             && ((t_des_now - t_sonar_invalid) > time_interval_of_sonar_valid))
                        {
                            if (fly_test_mission
                                || rotation_test_mission
                                || circle_mission
                                || panorama_mission
                                || trail_navigation_mission
                                || customized_plan_mission
                                || return_mission
                                || face_mission
                                || body_mission)
                            {
                                sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                         x_vel_des_yawed, y_vel_des_yawed,
                                                         z_vel_des, yaw_vel_des,
                                                         &cmd0, &cmd1, &cmd2, &cmd3);
                            }
                            else
                            {
#ifdef VIO_OPTIC_FLOW_SWITCH
                                if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                                {
                                    sn_apply_cmd_mapping(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                         x_vel_des_yawed, y_vel_des_yawed,
                                                         z_vel_des, yaw_vel_des,
                                                         &cmd0, &cmd1, &cmd2, &cmd3);
                                }
                                else
#endif
                                {
                                    sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                         x_vel_des_yawed, y_vel_des_yawed,
                                                         z_vel_des, yaw_vel_des,
                                                         &cmd0, &cmd1, &cmd2, &cmd3);
                                }
                            }
                        }
                        else
                        {
                            sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                 x_vel_des_yawed, y_vel_des_yawed,
                                                 z_vel_des, yaw_vel_des,
                                                 &cmd0, &cmd1, &cmd2, &cmd3);
                        }
                    }
                }
                else
                {
                    if (fly_test_mission
                        || rotation_test_mission
                        || circle_mission
                        || panorama_mission
                        || trail_navigation_mission
                        || customized_plan_mission
                        || return_mission
                        || face_mission
                        || body_mission)
                    {
                        sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                             x_vel_des_yawed, y_vel_des_yawed,
                                             z_vel_des, yaw_vel_des,
                                             &cmd0, &cmd1, &cmd2, &cmd3);
                    }
                    else
                    {
#ifdef VIO_OPTIC_FLOW_SWITCH
                        if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                        {
                            sn_apply_cmd_mapping(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                 x_vel_des_yawed, y_vel_des_yawed,
                                                 z_vel_des, yaw_vel_des,
                                                 &cmd0, &cmd1, &cmd2, &cmd3);
                        }
                        else
#endif
                        {
                            sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                                                 x_vel_des_yawed, y_vel_des_yawed,
                                                 z_vel_des, yaw_vel_des,
                                                 &cmd0, &cmd1, &cmd2, &cmd3);
                        }
                    }
                }
            }

            if(body_follow_switch && body_follow_prallel && cmd1 != 0)cmd0 =0; //cuiyc add for people follow
            if(face_follow_switch && (cmd0 != 0 || cmd1 != 0))cmd2 = 0;

            if(body_follow_switch && body_follow_prallel
                && fabs(cur_body.angle) >5 )
                cmd1 = 0.5f*cur_body.angle/51.0f;

            DEBUG("[%d] [sn_send_rc_command cmd0 cmd1 cmd2 cmd3]: [%f,%f,%f,%f] cur_body.angle[%f]\n",
                                       loop_counter, cmd0, cmd1, cmd2, cmd3, cur_body.angle);

#ifdef ZZG_DEBUG_FLAG
#ifdef ZZG_CMD_DEBUG_FLAG
            if ((state == MissionState::LOITER)
                 || (state == MissionState::IN_MOTION))
            {
                if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
                {
                    char csv_info[TMP_BUFF_LEN];
                    memset(csv_info, 0, sizeof(csv_info));

                    if (udpOverTimeCount > 0)
                    {
                        sprintf(csv_info, "%d,-2,-2,-2,-2,%f,%lf\n", loop_counter, ir_distance, t_now);
                    }
                    else
                    {
                        sprintf(csv_info, "%d,-3,-3,-3,-3,%f,%lf\n", loop_counter, ir_distance, t_now);
                    }
                    fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                    fclose(fp_zzg_debug_log);
                }
            }
#elif defined ZZG_SONAR_DEBUG_FLAG
            if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
            {
                char csv_info[TMP_BUFF_LEN];
                memset(csv_info, 0, sizeof(csv_info));

                sprintf(csv_info, "%d,%f,%lf\n", loop_counter, snav_data->sonar_0_raw.range, t_now);
                fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                fclose(fp_zzg_debug_log);
            }
#endif
#endif


#ifdef SPEED_LIMIT_FLAG
            if (face_mission == true || body_mission == true)
            {
                cmd0 = cmd0*speed_coefficient;
                cmd1 = cmd1*speed_coefficient;
            }
            else
            {
                if ((revise_height)  <= 5.0f)
                {
                    //avoid the optic flow error
                    if (((revise_height) <= 1.0f)
                        || snav_data->sonar_0_raw.range <= 1.0f)
                    {
                        cmd0 = cmd0*0.5f*speed_coefficient;
                        cmd1 = cmd1*0.5f*speed_coefficient;
                    }
                    else
                    {
                        if (circle_mission || return_mission || fly_test_mission || rotation_test_mission)
                        {
                            cmd0 = cmd0*speed_coefficient;
                            cmd1 = cmd1*speed_coefficient;
                        }
                        else
                        {
                            cmd0 = cmd0*speed_coefficient;
                            cmd1 = cmd1*speed_coefficient;
                        }
                    }

                    if (mode != SN_GPS_POS_HOLD_MODE)
                    {
                        cmd0 = cmd0*0.6;
                        cmd1 = cmd1*0.6;

                        /*
                        if (customized_plan_mission)
                        {
                            cmd2 = cmd2*0.6;
                        }
                        */
                    }
                }
                else
                {
                    // Slow down when height grow
                    cmd0 = cmd0*speed_coefficient*0.25f*(20.0/revise_height);
                    cmd1 = cmd1*speed_coefficient*0.25f*(20.0/revise_height);

                    /*
                    if (customized_plan_mission)
                    {
                        cmd2 = cmd2*speed_coefficient*0.25f*(20.0/revise_height);
                    }
                    */
                }

                /*
                if (mode == SN_GPS_POS_HOLD_MODE)
                {
                    float vel_gps_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                             + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);

                    if (vel_gps_est > VEL_LINEAR_LIMIT_GPS_MACRO(revise_height))
                    {
                        cmd0 = cmd0*0.2f;
                        cmd1 = cmd1*0.2f;
                    }
                }
                */
            }
#endif

#ifdef HEIGHT_LIMIT
            // Limit the height
            if (gps_enabled
                && (gps_status == SN_DATA_VALID)
                && ((t_now_for_gps - t_gps_invalid) > time_interval_of_gps_valid)
                /*&& ((t_des_now - t_gps_height_invalid) > time_interval_of_gps_valid)*/)
            {
                height_limit = fConstHeightLimit;
            }
            else if (mode == SN_VIO_POS_HOLD_MODE)
            {
                height_limit = 10;
            }
            else
            {
                height_limit = 5;
            }

            if (((z_est - z_est_startup)>= height_limit) && (cmd2 > 0))
            {
                cmd2 = 0;

                DEBUG("[%d] The drone have reached the limit height.\n", loop_counter);

                memset(result_to_client,0,MAX_BUFF_LEN);
                memcpy(result_to_client, SNAV_INFO_OVER_SAFE_HEIGHT, MAX_BUFF_LEN);
                length=sendto(server_udp_sockfd,result_to_client,strlen(result_to_client),0,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr));
            }
#endif

#ifdef LOW_HEIGHT_LIMIT_VEL
            // Limit the height
            if (((revise_height) <= 1) && snav_data->sonar_0_raw.range <= 1.0f)
            {
                DEBUG("[%d] The drone have reached the limit height.\n", loop_counter);

                cmd0 = 0;
                cmd1 = 0;

                if (mode == SN_GPS_POS_HOLD_MODE)
                {
                    // Use gps data
                    float current_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                            + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);

                    float current_vel_x_est = snav_data->gps_pos_vel.velocity_estimated[0];
                    float current_vel_y_est = snav_data->gps_pos_vel.velocity_estimated[1];

                    if (fabs(current_vel_est) > hover_vel_limit)
                    {
                        float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est_gps) - current_vel_y_est*sin(-yaw_est_gps);
                        float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est_gps) + current_vel_y_est*cos(-yaw_est_gps);

                        cmd0 = -hover_brake_limit*current_vel_x_yawed;
                        cmd1 = -hover_brake_limit*current_vel_y_yawed;

                        cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                        cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                        DEBUG("[%d] OPTIC_FLOW FORMAL_OVER_SPEED[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                  loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                  current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                    }
                }
                else
                {
                    // Use optic_flow data
                    float current_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                    float current_vel_x_est = snav_data->optic_flow_pos_vel.velocity_estimated[0];
                    float current_vel_y_est = snav_data->optic_flow_pos_vel.velocity_estimated[1];

                    if (fabs(current_vel_est) > hover_vel_limit)
                    {
                        float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est) - current_vel_y_est*sin(-yaw_est);
                        float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est) + current_vel_y_est*cos(-yaw_est);

                        cmd0 = -hover_brake_limit*current_vel_x_yawed;
                        cmd1 = -hover_brake_limit*current_vel_y_yawed;

                        cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                        cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                        DEBUG("[%d] OPTIC_FLOW FORMAL_OVER_SPEED[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                  loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                  current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                    }
                }
            }
#endif
            // Stop the return mission when switch between optic-flow-mode and gps-pos-mode
            if (((last_mode == SN_OPTIC_FLOW_POS_HOLD_MODE || last_mode == SN_VIO_POS_HOLD_MODE) && (mode == SN_GPS_POS_HOLD_MODE))
                || ((last_mode == SN_GPS_POS_HOLD_MODE) && (mode == SN_OPTIC_FLOW_POS_HOLD_MODE || mode == SN_VIO_POS_HOLD_MODE)))
            {
                DEBUG("[%d] set reverse_ctrl_count to 0!, last_mode=%d, mode=%d\n", loop_counter, last_mode, mode);
                reverse_ctrl_count = 0;

                if (return_mission && gps_enabled)
                {
                    return_mission = false;

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_RETURN_MISSION_PAUSE);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client)
                                     , 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] udp sendto SNAV_RETURN_MISSION_PAUSE length=%d\n", loop_counter, length);
                }
            }

            // Add by wlh
            if (reverse_full_flag == 1 && mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
            {
                if ((fabs(cmd0) < 1e-4 && fabs(cmd1) < 1e-4 ) || reverse_ctrl_count != 0)
                {
                    if ((old_cmd0 != 0 || old_cmd1 != 0)  && reverse_ctrl_count < stop_control_num)
                    {
                        if (reverse_ctrl_step == 0)
                        {
                            reverse_ctrl_step = 1;
                        }

                        if (reverse_ctrl_step == 1
                            && (fabs(old_cmd00+old_cmd0) > 0.05
                                || fabs(old_cmd11+old_cmd1) > 0.05)
                            && (fabs(old_cmd0)+0.05 > fabs(old_cmd00))
                            && (fabs(old_cmd1)+0.05 > fabs(old_cmd11)))
                        {
                            int current_cmd_offset = reverse_plan_num;

                            if (reverse_ctrl_flag == 1)
                            {
                                if(reverse_ctrl_count == 0 && (revise_height> 0.3) && snav_data->general_status.on_ground == 0 )
                                {
                                    reverse_ctrl_count++;

                                    DEBUG("debug_flag formal session aaa reverse_ctrl_count=%d.\n", reverse_ctrl_count);
                                    old_pitch = snav_data->attitude_estimate.pitch;
                                    old_roll  = -snav_data->attitude_estimate.roll;
                                    DEBUG("*******************************[control break]------old_pitch[%f]old_roll[%f]optic_flow_vel_est[%f]\n",old_pitch,old_roll,optic_flow_vel_est);
                                }
                            }

                            if (((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.2)
                                && ((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) < 0.5))
                            {
                                current_cmd_offset = reverse_plan_num;
                            }
                            else if((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.5)
                            {
                                current_cmd_offset = reverse_plan_num/2;
                            }
                            else
                            {
                                current_cmd_offset = reverse_plan_num*2;
                            }

                            cmd0 = old_cmd00 - old_cmd0/current_cmd_offset;
                            old_cmd00 = old_cmd00 - old_cmd0/current_cmd_offset;

                            cmd1 = old_cmd11 - old_cmd1/current_cmd_offset;
                            old_cmd11 = old_cmd11 - old_cmd1/current_cmd_offset;

                            DEBUG("gozero------optic_flow_vel_est[%f]cmd0[%f]cmd1[%f]old_cmd0[%f]old_cmd1[%f]pitch[%f]roll[%f]\n",optic_flow_vel_est,cmd0,cmd1,old_cmd0,old_cmd1,snav_data->attitude_estimate.pitch,-snav_data->attitude_estimate.roll);

                            reverse_ctrl_step = 1;

                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                if ((gps_vel_est < 0.1) && (gps_vel_est > 0.01))
                                {
                                    reverse_ctrl_count = 0;
                                    old_cmd0 = 0;
                                    old_cmd1 = 0;
                                    old_cmd00 = 0;
                                    old_cmd11 = 0;
                                    DEBUG("-------------gozero_end-1--- last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]gps_vel_est[%f]\n",last_pitch,old_pitch,last_roll,old_roll,gps_vel_est);
                                }
                            }
                            else
                            {
                                if ((optic_flow_vel_est < 0.1) && (optic_flow_vel_est > 0.01))
                                {
                                    reverse_ctrl_count = 0;
                                    old_cmd0 = 0;
                                    old_cmd1 = 0;
                                    old_cmd00 = 0;
                                    old_cmd11 = 0;
                                    DEBUG("-------------gozero_end-1--- last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]optic_flow_vel_est[%f]\n",last_pitch,old_pitch,last_roll,old_roll,optic_flow_vel_est);
                                }
                            }
                        }
                        else if(fabs(old_cmd00+old_cmd0) <= 0.05 || fabs(old_cmd11+old_cmd1) <= 0.05)
                          reverse_ctrl_step = 2;

                        if(reverse_ctrl_step == 2 && (fabs(old_cmd00) > 0.05 || fabs(old_cmd11) > 0.05) && fabs(old_cmd00)+fabs(old_cmd11) > 0.05 )
                        {
                            int keep_cmd = 0;
                            int current_cmd_offset = reverse_plan_num;

                            if (((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.2)
                                && ((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) < 0.5))
                            {
                                current_cmd_offset = reverse_plan_num*2;
                            }
                            else if((fabs(sqrt(old_pitch*old_pitch + old_roll*old_roll))/1.44) >= 0.5)
                            {
                                current_cmd_offset = reverse_plan_num*3;
                            }
                            else
                            {
                                current_cmd_offset = reverse_plan_num;
                            }


                            if((fabs(last_pitch + old_pitch) > fabs(old_pitch) && fabs(old_pitch) > 0.1)  || (fabs(last_roll + old_roll) > fabs(old_roll) && fabs(old_roll) > 0.1))//还未拉反方向
                            {
                                //current_cmd_offset = current_cmd_offset*6;
                                keep_cmd = 2;
                            }
                            else
                            {
                                //printf("last_pitch[%f]pitch[%f]old_pitch[%f]\n",fabs(last_pitch),fabs(snav_data->attitude_estimate.pitch),fabs(old_pitch));
                                //printf("last_roll [%f]roll [%f]old_roll [%f]\n",fabs(last_roll),fabs(snav_data->attitude_estimate.roll),fabs(old_roll));
                                if((fabs(last_pitch) > fabs(snav_data->attitude_estimate.pitch) && fabs(old_pitch) > 0.1  && fabs(last_pitch) + 0.1 < fabs(old_pitch)) || (fabs(last_roll) > fabs(snav_data->attitude_estimate.roll) && fabs(old_roll) > 0.1 && fabs(last_roll) + 0.1 < fabs(old_roll)))//已经拉反，但是倾角还未到位
                                {
                                    keep_cmd = 1;
                                }
                                else
                                {
                                    keep_cmd = 0;
                                }
                            }


                            if(sample_size < 5)
                            {
                                keep_cmd = 3;
                                reverse_ctrl_count = 1;
                                DEBUG("*****************************************sample_size < 5***************************************");
                            }

                            if(keep_cmd > 0)
                            {
                                //current_cmd_offset = reverse_plan_num*6;
                                cmd0 = old_cmd00;
                                cmd1 = old_cmd11;
                            }
                            else
                            {
                                cmd0 = old_cmd00 + old_cmd0/current_cmd_offset;
                                old_cmd00 = old_cmd00 + old_cmd0/current_cmd_offset;

                                cmd1 = old_cmd11 + old_cmd1/current_cmd_offset;
                                old_cmd11 = old_cmd11 + old_cmd1/current_cmd_offset;
                            }


                            DEBUG("gozero------optic_flow_vel_est[%f]cmd0[%f]cmd1[%f]cmd2[%f]cmd3[%f]---reverse_ctrl_count[%d]---old_cmd0-1[%f][%f]---estimated-desired[%f]---imu_lin_acc[%f]---sample_size_missing_count[%d]---pitch[%f]roll[%f]---current_cmd_offset[%d]---keep_cmd[%d]\n",optic_flow_vel_est,cmd0,cmd1,cmd2,cmd3,reverse_ctrl_count,old_cmd0,old_cmd1,fabs(estimated_xy_sqrt - desired_xy_sqrt),fabs(imu_lin_acc),sample_size_missing_count,snav_data->attitude_estimate.pitch,-snav_data->attitude_estimate.roll,current_cmd_offset,keep_cmd);


                            if (mode == SN_GPS_POS_HOLD_MODE)
                            {
                                if ((gps_vel_est < 0.1) && (gps_vel_est > 0.05))
                                {
                                    reverse_ctrl_count = 0;
                                    old_cmd0 = 0;
                                    old_cmd1 = 0;
                                    old_cmd00 = 0;
                                    old_cmd11 = 0;
                                    DEBUG("-------------gozero-2_end---last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]gps_vel_est[%f]\n",last_pitch,old_pitch,last_roll,old_roll,gps_vel_est);
                                }
                            }
                            else
                            {
                                //if((fabs(last_pitch + old_pitch) < 0.05 && (fabs(last_roll + old_roll) < 0.05)
                                if ((optic_flow_vel_est < 0.1) && (optic_flow_vel_est > 0.05))
                                {
                                    reverse_ctrl_count = 0;//Reverse end
                                    old_cmd0 = 0;
                                    old_cmd1 = 0;
                                    old_cmd00 = 0;
                                    old_cmd11 = 0;
                                    DEBUG("-------------gozero-2_end---last_pitch[%f]old_pitch[%f]last_roll[%f]old_roll[%f]optic_flow_vel_est[%f]\n",last_pitch,old_pitch,last_roll,old_roll,optic_flow_vel_est);
                                }
                            }

                            if(fabs(old_cmd00) <= fabs(old_cmd0/current_cmd_offset) &&  fabs(old_cmd11) <= fabs(old_cmd1/current_cmd_offset))
                                DEBUG("gozero-------------------------------------------------------------end\n");

                            last_pitch = snav_data->attitude_estimate.pitch;
                            last_roll = -snav_data->attitude_estimate.roll;

                            reverse_ctrl_step = 2;
                        }
                        else if(reverse_ctrl_step > 1 && reverse_ctrl_count > 0)
                        {
                            cmd0 = old_cmd00;
                            cmd1 = old_cmd11;
                            reverse_ctrl_count = 0;
                        }

                    }
                }
                else
                {
                    if (reverse_ctrl_count == 0)
                    {
                        old_cmd0 = cmd0;
                        old_cmd1 = cmd1;

                        old_cmd00 = old_cmd0;
                        old_cmd11 = old_cmd1;
                        reverse_ctrl_step = 0;
                    }
                }

                if (reverse_ctrl_count > 0)
                {
                    reverse_ctrl_count++;
                    DEBUG("[%d] debug_flag formal session bbb [reverse_ctrl_count]: [%d].\n" ,loop_counter, reverse_ctrl_count);
                }

                if (reverse_ctrl_count > stop_control_num)
                {
                    reverse_ctrl_count = 0;
                }
            }
            //Add end

            if (return_mission)
            {
                if (gps_enabled)
                {
                    if (fly_home)
                    {
                        if ((revise_height) > 5)
                        {
                            // Slow down when height grow
                            cmd0 = 0.3*speed_coefficient*0.3f*(fConstHeightLimit/revise_height);

                            if (fabs((float)snav_data->gps_pos_vel.velocity_estimated[0]) > 2)
                            {
                                cmd0 = cmd0*0.5;
                            }
                        }
                        else
                        {
                            cmd0 = 0.3;

                            if (fabs((float)snav_data->gps_pos_vel.velocity_estimated[0]) > 2)
                            {
                                cmd0 = cmd0*0.5;
                            }
                        }

                        float distance_gps_home = sqrt((x_est_gps_startup-x_est_gps)*(x_est_gps_startup-x_est_gps)
                                                        + (y_est_gps_startup-y_est_gps)*(y_est_gps_startup-y_est_gps));

                        if (distance_gps_home <= 5)
                        {
                            cmd0 = cmd0*0.2;
                        }

                        cmd1 = 0;
                        cmd2 = 0;
                    }
                    else
                    {
                        cmd3 = cmd3*yaw_coefficient;
                    }

                    DEBUG("[%d] gps_return [cmd0:cmd1:cmd2:cmd3]: [%f, %f, %f, %f].\n", loop_counter, cmd0, cmd1, cmd2, cmd3);
                }
            }
            else if (rotation_test_mission)
            {
                rotation_test_count ++;
                DEBUG("[%d] [rotation_test_mission rotation_test_count[%d]\n", loop_counter, rotation_test_count);

                if (rotation_test_count < 600)
                {
                    cmd0 = 0;
                    cmd1 = 0;
                    cmd2 = 0;
                    cmd3 = 0.2;
                }
                else if ((rotation_test_count >= 650) && (rotation_test_count < 1250))
                {
                    cmd0 = 0;
                    cmd1 = 0;
                    cmd2 = 0;
                    cmd3 = -0.2;
                }
                else if (rotation_test_count >= 1250)
                {
                    rotation_test_count = 0;
                    rotation_test_mission = false;
                }
            }
            else if (fly_test_mission)
            {
                fly_test_count ++;
                DEBUG("[%d] [fly_test_mission fly_test_count[%d]\n", loop_counter,fly_test_count);

                if (fly_test_count < 200)
                {
                    cmd0 = 0.12;
                }
                else if ((fly_test_count >= 250) && (fly_test_count < 450))
                {
                    cmd1 = 0.12;
                }
                else if ((fly_test_count >= 500) && (fly_test_count < 700))
                {
                    cmd0 = -0.12;
                }
                else if ((fly_test_count >= 750) && (fly_test_count < 950))
                {
                    cmd1 = -0.12;
                }
                else if (fly_test_count >= 950)
                {
                    fly_test_count = 0;
                    fly_test_mission = false;
                }
            }
            else if (customized_plan_mission)
            {
                if (cmd2 < -0.1)
                {
                    cmd2 = cmd2*0.4;
                }
                cmd3 = cmd3*yaw_coefficient;
            }
#ifdef AUTO_REDUCE_HEIGHT
            else if ((use_reduce_height == 1) && auto_reduce_height_mission)
            {
                cmd2 = cmd2*0.3;
            }
#endif
            // Add by wlh

            DEBUG("[%d] debug_flag formal sn_send_rc_command first: [cmd0,cmd1,cmd2,cmd3]: [%f,%f,%f,%f].\n"
                    , loop_counter, cmd0, cmd1, cmd2, cmd3);

            /*
            if(snav_data->general_status.on_ground == 0
                && snav_data->sonar_0_raw.range > 0.30
                && reverse_ctrl_count == 0)
            */
#ifdef VEL_LIMIT_FLAG
            {
                DEBUG("[%d] Enter formal recal cmd reverse_ctrl_count=[%d].\n", loop_counter, reverse_ctrl_count);

                float now_vel_est = 0;

                if (mode == SN_GPS_POS_HOLD_MODE)
                {
                    now_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                        + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);
                    if (now_vel_est > VEL_LINEAR_LIMIT_GPS_MACRO(revise_height))
                    {
                        DEBUG("[%d] formal recal before session 1 now_vel_est, vel_gps_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_GPS_MACRO(revise_height), cmd0, cmd1);

                        cmd0 = cmd0*fabs(VEL_LINEAR_LIMIT_GPS_MACRO(revise_height)/now_vel_est);
                        cmd1 = cmd1*fabs(VEL_LINEAR_LIMIT_GPS_MACRO(revise_height)/now_vel_est);

                        DEBUG("[%d] formal recal after session 1 now_vel_est, vel_gps_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_GPS_MACRO(revise_height), cmd0, cmd1);
                    }
                }

                if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
                {
                    now_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                        + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                    if (now_vel_est > VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height))
                    {
                        DEBUG("[%d] formal recal before session 1 now_vel_est, vel_optic_flow_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height), cmd0, cmd1);

                        cmd0 = cmd0*fabs(VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height)/now_vel_est);
                        cmd1 = cmd1*fabs(VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height)/now_vel_est);

                        DEBUG("[%d] formal recal after session 1 now_vel_est, vel_optic_flow_limt: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                    loop_counter, now_vel_est, VEL_LINEAR_LIMIT_OPTIC_MACRO(revise_height), cmd0, cmd1);
                    }

                    float sqrt_pitch_roll = sqrt(snav_data->attitude_estimate.pitch*snav_data->attitude_estimate.pitch
                                                 + snav_data->attitude_estimate.roll*snav_data->attitude_estimate.roll);
                    if (sqrt_pitch_roll > go_pitch_roll_limit)
                    {
                        DEBUG("[%d] formal recal before session 1 sqrt_pitch_roll, go_pitch_roll_limit: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                    loop_counter, sqrt_pitch_roll, go_pitch_roll_limit, cmd0, cmd1);

                        cmd0 = cmd0*fabs(go_pitch_roll_limit/sqrt_pitch_roll);
                        cmd1 = cmd1*fabs(go_pitch_roll_limit/sqrt_pitch_roll);

                        DEBUG("[%d] formal recal after session 1 sqrt_pitch_roll, go_pitch_roll_limit: [%f, %f], cmd0,cmd1: [%f, %f].\n",
                                    loop_counter, sqrt_pitch_roll, go_pitch_roll_limit, cmd0, cmd1);
                    }
                }

                // cmd limit
                float speed_level = 0.6;
                float cmd_mag = sqrt(cmd0*cmd0 + cmd1*cmd1);

                cmd_mag = sqrt(cmd0*cmd0 + cmd1*cmd1);
                if (cmd_mag > 1.414)
                {
                    DEBUG("[%d] formal recal before session 2 cmd0,cmd1:[%f, %f], cmd_mag:[%f].\n", loop_counter, cmd0, cmd1, cmd_mag);

                    cmd0 = cmd0*(speed_level/cmd_mag);
                    cmd1 = cmd1*(speed_level/cmd_mag);

                    DEBUG("[%d] formal recal after session 2 cmd0,cmd1:[%f, %f].\n", loop_counter, cmd0, cmd1);
                }

                // last cmd limit
                float last_cmd_mag = sqrt(last_cmd0*last_cmd0 + last_cmd1*last_cmd1);

                if (fabs(cmd_mag - last_cmd_mag) > go_cmd_offset_limit
                    || fabs(cmd0 - last_cmd0) > go_cmd_offset_limit
                    || fabs(cmd1 - last_cmd1) > go_cmd_offset_limit)
                {
                    DEBUG("[%d] formal recal before session 3 cmd_mag, last_cmd_mag:[%f, %f], cmd0, lastcmd0:[%f, %f], cmd1, last_cmd1:[%f, %f].\n",
                                loop_counter, cmd_mag, last_cmd_mag, cmd0, last_cmd0, cmd1, last_cmd1);

                    DEBUG("[%d] formal recal before session 3 cmd0,cmd1:[%f, %f].\n", loop_counter, cmd0, cmd1);

                    float p_bate_offset_cmd0 = fabs(cmd0 - last_cmd0);
                    float p_bate_offset_cmd1 = fabs(cmd1 - last_cmd1);


                    if (p_bate_offset_cmd0 > 0)
                    {
                        cmd0 = last_cmd0+(cmd0 - last_cmd0)*go_cmd_offset_limit/p_bate_offset_cmd0;
                    }

                    if (p_bate_offset_cmd1 > 0)
                    {
                        cmd1 = last_cmd1+(cmd1 - last_cmd1)*go_cmd_offset_limit/p_bate_offset_cmd1;
                    }

                    DEBUG("[%d] formal recal after session 3 cmd0,cmd1:[%f, %f].\n", loop_counter, cmd0, cmd1);
                }

                // cmd check again
                cmd_mag = sqrt(cmd0*cmd0 + cmd1*cmd1);
                if(cmd_mag > 1.414)
                {
                    DEBUG("[%d] formal recal before session 4 cmd0,cmd1:[%f, %f], cmd_mag:[%f].\n", loop_counter, cmd0, cmd1, cmd_mag);

                    cmd0 = cmd0*(speed_level/cmd_mag);
                    cmd1 = cmd1*(speed_level/cmd_mag);

                    DEBUG("[%d] formal recal before session 4 cmd0,cmd1:[%f, %f].\n", loop_counter, cmd0, cmd1);
                }
            }
            // Add End
#endif

            cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxCmdValue);
            cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxCmdValue);

            // Limit the minimal z_vel to -0.6 except the Landing(use -1 near the ground to stop propers)
            if ((state != MissionState::LANDING) && (cmd2 < -fMaxCmdValue))
            {
                cmd2 = -fMaxCmdValue;
            }

#ifdef NO_FLY_ZONE
            if (gps_enabled && (gps_status == SN_DATA_VALID))
            {
                float lat = snav_data->gps_0_raw.latitude;
                float lng = snav_data->gps_0_raw.longitude;

                if (isInNoFlyZone(lat, lng))
                {
                    DEBUG("You are near the No-Fly-Zone!!!");

                    memset(result_to_client, 0, MAX_BUFF_LEN);
                    sprintf(result_to_client, "%s", SNAV_WARNING_NO_FLY_ZONE);

                    length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                    DEBUG("[%d] SNAV_WARNING_NO_FLY_ZONE result_to_client=%s, length=%d\n", loop_counter, result_to_client, length);
                }
            }
#endif
            if (use_infrared == 1)
            {
                float current_vel_est = 0;

                if (mode == SN_GPS_POS_HOLD_MODE)
                {
                    // Use gps data
                    current_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                       + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);
                }

                else
                {
                    // Use optic_flow data
                    current_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                       + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                }

                if (fabs(current_vel_est) > IR_VEL_FLAG)
                {
                    ir_safe_distance = IR_FORBID_HIGH_VEL;
                }
                else
                {
                    ir_safe_distance = IR_FORBID_LOW_VEL;
                }
            }

            if ((use_infrared == 1)
                && (ir_distance > 0)
                && (ir_distance <= 2*ir_safe_distance))
            {
                struct timeval tv_tmp;
                gettimeofday(&tv_tmp, NULL);
                double t_current = tv_tmp.tv_sec + tv_tmp.tv_usec*1e-6;

                if (cmd0 > 0)
                {
                    if (mode == SN_GPS_POS_HOLD_MODE)
                    {
                        // Use gps data
                        float current_vel_est = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                                + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);

                        float current_vel_x_est = snav_data->gps_pos_vel.velocity_estimated[0];
                        float current_vel_y_est = snav_data->gps_pos_vel.velocity_estimated[1];

                        if (fabs(current_vel_est) > IR_VEL_LIMIT)
                        {
                            float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est_gps) - current_vel_y_est*sin(-yaw_est_gps);
                            float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est_gps) + current_vel_y_est*cos(-yaw_est_gps);

                            cmd0 = -IR_BRAKE_CMD*current_vel_x_yawed;
                            cmd1 = -IR_BRAKE_CMD*current_vel_y_yawed;

                            cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                            cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                            DEBUG("[%d] GPS_HOLD FORMAL_OVER_SPEED[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                      loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                      current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);

                            DEBUG("[%d] GPS_HOLD INFRAED_DEBUG Formal Brake[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                        loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                        current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                        }
                        else
                        {
                            if (ir_distance <= ir_safe_distance)
                            {
                                DEBUG("[%d] GPS_HOLD INFRAED_DEBUG Formal Forbidden cmd0 and cmd1.\n", loop_counter);

                                cmd0 = 0;
                                cmd1 = 0;
                            }
                            else
                            {
                                DEBUG("[%d] GPS_HOLD INFRAED_DEBUG Formal Forbidden cmd0 and cmd1.\n", loop_counter);

                                cmd0 = 0;
                            }
                        }

                    }
                    else
                    {
                        // Use optic_flow data
                        float current_vel_est = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                                    + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                        float current_vel_x_est = snav_data->optic_flow_pos_vel.velocity_estimated[0];
                        float current_vel_y_est = snav_data->optic_flow_pos_vel.velocity_estimated[1];

                        if (fabs(current_vel_est) > IR_VEL_LIMIT)
                        {
                            float current_vel_x_yawed = current_vel_x_est*cos(-yaw_est) - current_vel_y_est*sin(-yaw_est);
                            float current_vel_y_yawed = current_vel_x_est*sin(-yaw_est) + current_vel_y_est*cos(-yaw_est);

                            cmd0 = -IR_BRAKE_CMD*current_vel_x_yawed;
                            cmd1 = -IR_BRAKE_CMD*current_vel_y_yawed;

                            cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxHoverBrakeCmd);
                            cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxHoverBrakeCmd);

                            DEBUG("[%d] OPTIC_FLOW FORMAL_OVER_SPEED[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                      loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                      current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);

                            DEBUG("[%d] OPTIC_FLOW INFRAED_DEBUG Formal Brake[current_vel_est, vel_x_est, vel_y_est]=[%f,%f,%f], [vel_x_yawed, vel_y_yawed]=[%f,%f], [cmd0, cmd1]=[%f,%f]\n",
                                            loop_counter, current_vel_est, current_vel_x_est, current_vel_y_est,
                                            current_vel_x_yawed, current_vel_y_yawed, cmd0, cmd1);
                        }
                        else
                        {
                            if (ir_distance <= ir_safe_distance)
                            {
                                DEBUG("[%d] OPTIC_FLOW INFRAED_DEBUG Formal Forbidden cmd0 and cmd1.\n", loop_counter);

                                cmd0 = 0;
                                cmd1 = 0;
                            }
                            else
                            {
                                DEBUG("[%d] OPTIC_FLOW INFRAED_DEBUG Formal Forbidden cmd0 only.\n", loop_counter);

                                cmd0 = 0;
                            }
                        }
                    }

                    t_ir_brake = t_current;
                }
                else if (ir_distance <= ir_safe_distance)
                {
                    cmd1 = 0;
                }
            }

#ifdef OPTIC_FLOW_CMD_LIMIT
            if ((mode == SN_OPTIC_FLOW_POS_HOLD_MODE) && (revise_height >= fMaxOFHeight))
            {
                cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxOFCmdValue);
                cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxOFCmdValue);
            }
#endif

#ifdef ALT_CMD_LIMIT
            if ((mode != SN_GPS_POS_HOLD_MODE)
                && (mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
                && (mode != SN_VIO_POS_HOLD_MODE))
            {
                cmd0 = CMD_INPUT_LIMIT(cmd0, fMaxAltCmdValue);
                cmd1 = CMD_INPUT_LIMIT(cmd1, fMaxAltCmdValue);
            }
#endif

            if (gps_enabled && mag_usable
                && (gps_status == SN_DATA_VALID)
                && ((t_now_for_gps - t_gps_invalid) > time_interval_of_gps_valid)
                && ((t_des_now - t_gps_height_invalid) > time_interval_of_gps_valid))
            {
                if (revise_height >= gps_mode_height)
                {
                    sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                }
                else
                {
                    if ((sonar_status == SN_DATA_VALID)
                         && (snav_data->sonar_0_raw.range < sonar_valid_data_max)
                         && ((t_des_now - t_sonar_invalid) > time_interval_of_sonar_valid))
                    {
                        if (fly_test_mission
                            || rotation_test_mission
                            || circle_mission
                            || panorama_mission
                            || trail_navigation_mission
                            || customized_plan_mission
                            || return_mission
                            || face_mission
                            || body_mission)
                        {
                            sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                        }
                        else
                        {
#ifdef VIO_OPTIC_FLOW_SWITCH
                            if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                            {
                                sn_send_rc_command(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                            }
                            else
#endif
                            {
                                sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                            }
                        }
                    }
                    else
                    {
                        sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                    }
                }

            }
            else
            {
                if (fly_test_mission
                    || rotation_test_mission
                    || circle_mission
                    || panorama_mission
                    || trail_navigation_mission
                    || customized_plan_mission
                    || return_mission
                    || face_mission
                    || body_mission)
                {
                    sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                }
                else
                {
#ifdef VIO_OPTIC_FLOW_SWITCH
                    if (snav_data->data_status.vio_0_status == SN_DATA_VALID)
                    {
                        sn_send_rc_command(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                    }
                    else
#endif
                    {
                        sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
                    }
                }
            }

#ifdef ZZG_DEBUG_FLAG
#ifdef ZZG_FINAL_CMD_FLAG
            if ((fp_zzg_debug_log = fopen(zzg_debug_log_filename, "a+")) != NULL)
            {
                char csv_info[TMP_BUFF_LEN];
                memset(csv_info, 0, sizeof(csv_info));
                sprintf(csv_info, "%d,%f,%f,%f,%f,%lf\n", loop_counter,cmd0,cmd1,cmd2,cmd3,t_now_for_gps);
                fwrite(csv_info, strlen(csv_info), 1, fp_zzg_debug_log);
                fclose(fp_zzg_debug_log);
            }
#endif
#endif


            DEBUG("[%d] debug_flag formal sn_send_rc_command final [cmd0,cmd1,cmd2,cmd3]: [%f,%f,%f,%f]. [z_vel_est, z_vel_des]:[%f, %f]\n"
                        , loop_counter, cmd0, cmd1, cmd2, cmd3, snav_data->optic_flow_pos_vel.velocity_estimated[2], snav_data->optic_flow_pos_vel.velocity_desired[2]);
            DEBUG("[%d] debug_flag formal sn_send_rc_command timestamp: %" PRId64 "\n",
                                        loop_counter, snav_data->esc_raw.time);

            last_mode = (SnMode)snav_data->general_status.current_mode;

#ifdef CSV_LOG_FOR_XIAOYI
            if (((state == MissionState::LOITER) || (state == MissionState::IN_MOTION))
                  && ((fp_csv_log = fopen(csv_log_filename, "a+")) != NULL))
            {
                int index = 0;
                int32_t latitude = 0;
                int32_t longitude = 0;
                float x = 0;
                float y = 0;
                float z = 0;
                float roll = 0;
                float pitch = 0;
                float yaw = 0;
                int f_state = 0;
                float velocity;
                float battery;
                double time;

                index = loop_counter;
                latitude = snav_data->gps_0_raw.latitude;
                longitude = snav_data->gps_0_raw.longitude;

                if (state == MissionState::ON_GROUND)
                {
                    x = 0;
                    y = 0;
                    z = 0;
                }
                else
                {
                    if (gps_enabled && (gps_status == SN_DATA_VALID) && (take_off_with_gps_valid))
                    {
                        x = x_est_gps - x_est_gps_startup;
                        y = y_est_gps - y_est_gps_startup;
                        z = revise_height;
                    }
                    /*
                    else if (mode == SN_VIO_POS_HOLD_MODE)
                    {
                        x = x_est_vio - x_est_vio_startup;
                        y = y_est_vio - y_est_vio_startup;
                        z = revise_height;
                    }
                    */
                    else
                    {
                        x = x_est - x_est_startup;
                        y = y_est - y_est_startup;
                        z = revise_height;
                    }
                }

                roll = snav_data->attitude_estimate.roll;
                pitch = snav_data->attitude_estimate.pitch;
                yaw = snav_data->attitude_estimate.yaw;

                f_state = (int)state;

                if (gps_enabled && (gps_status == SN_DATA_VALID))
                {
                    float vel_realtime = sqrt(snav_data->gps_pos_vel.velocity_estimated[0]*snav_data->gps_pos_vel.velocity_estimated[0]
                                             + snav_data->gps_pos_vel.velocity_estimated[1]*snav_data->gps_pos_vel.velocity_estimated[1]);
                    velocity = vel_realtime;
                }
                else
                {
                    float vel_realtime = sqrt(snav_data->optic_flow_pos_vel.velocity_estimated[0]*snav_data->optic_flow_pos_vel.velocity_estimated[0]
                                             + snav_data->optic_flow_pos_vel.velocity_estimated[1]*snav_data->optic_flow_pos_vel.velocity_estimated[1]);
                    velocity = vel_realtime;
                }

                battery = voltage;
                time = t_des_now;


                char csv_info[MAX_BUFF_LEN];
                memset(csv_info, 0, TMP_BUFF_LEN);
                if (gps_enabled != 1)
                {
                    sprintf(csv_info, "%d,null,null,%f,%f,%f,%f,%f,%f,%d,%s,%f,%f,%lf\n",
                                                    index,
                                                    x,
                                                    y,
                                                    z,
                                                    roll,
                                                    pitch,
                                                    yaw,
                                                    f_state,
                                                    d_state_err,
                                                    velocity,
                                                    battery,
                                                    time);
                }
                else
                {
                    sprintf(csv_info, "%d,%d,%d,%f,%f,%f,%f,%f,%f,%d,%s,%f,%f,%lf\n",
                                                    index,
                                                    latitude,
                                                    longitude,
                                                    x,
                                                    y,
                                                    z,
                                                    roll,
                                                    pitch,
                                                    yaw,
                                                    f_state,
                                                    d_state_err,
                                                    velocity,
                                                    battery,
                                                    time);
                }

                fwrite(csv_info, strlen(csv_info), 1, fp_csv_log);
                fclose(fp_csv_log);
            }
#endif

            // Add by wlh
            last_cmd0 = cmd0;
            last_cmd1 = cmd1;
            // Add end

            // Print some information
            if (mode == SN_GPS_POS_HOLD_MODE)
            {
                DEBUG("\n[%d] SN_GPS_POS_HOLD_MODE. \n", loop_counter);
            }
            else if (mode == SN_SENSOR_ERROR_MODE)
            {
                DEBUG("\n[%d] SENSOR ERROR MODE.\n", loop_counter);
            }
            else if (mode == SN_ALT_HOLD_MODE)
            {
                DEBUG("\n[%d] SN_ALT_HOLD_MODE.\n", loop_counter);
            }
            else if (mode == SN_ALT_HOLD_LOW_ANGLE_MODE)
            {
                DEBUG("\n[%d] SN_ALT_HOLD_LOW_ANGLE_MODE.\n", loop_counter);
            }
            else if (mode == SN_VIO_POS_HOLD_MODE)
            {
                DEBUG("\n[%d] SN_VIO_POS_HOLD_MODE. \n", loop_counter);
            }
            else if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
            {
                DEBUG("\n[%d] SN_OPTIC_FLOW_POS_HOLD_MODE. \n", loop_counter);
            }
            else
            {
                DEBUG("\n[%d] UNDEFINED MODE :%d \n", loop_counter,mode);
            }


            if (props_state == SN_PROPS_STATE_NOT_SPINNING)
            {
                DEBUG("Propellers NOT spinning\n");
            }
            else if (props_state == SN_PROPS_STATE_STARTING)
            {
                DEBUG("Propellers attempting to spin\n");
            }
            else if (props_state == SN_PROPS_STATE_SPINNING)
            {
                DEBUG("Propellers spinning\n");
            }
            else
            {
                DEBUG("Unknown propeller state\n");
            }

            DEBUG("[%d] Current sample_size: [%d]\n", loop_counter, sample_size);
            DEBUG("[%d] battery_voltage: %f\n", loop_counter, voltage);
            DEBUG("[%d] Current Sonar range: [%f]\n", loop_counter, snav_data->sonar_0_raw.range);
            DEBUG("[%d] commanded rates: [%f, %f, %f, %f]\n",
                        loop_counter, x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des);

            DEBUG("[%d] [Origin Estimated x_est, y_est, z_est, yaw_est]: [%f, %f, %f, %f]\n",
                        loop_counter, x_est, y_est, z_est, yaw_est);
            DEBUG("[%d] [Origin Desired x_des, y_des, z_des, yaw_des]: [%f, %f, %f, %f]\n",
                        loop_counter, x_des, y_des, z_des, yaw_des);
            DEBUG("[%d] [x_est_startup, y_est_startup, z_est_startup, yaw_est_startup]: [%f, %f, %f, %f]\n",
                        loop_counter, x_est_startup, y_est_startup, z_est_startup, yaw_est_startup);

            /*
            if (mode == SN_VIO_POS_HOLD_MODE)
            {
                DEBUG("[%d] [Origin VIO Estimated x_est_vio, y_est_vio, z_est_vio, yaw_est_vio]: [%f, %f, %f, %f]\n",
                            loop_counter, x_est_vio, y_est_vio, z_est_vio, yaw_est_vio);
                DEBUG("[%d] [Origin VIO Desired x_des_vio, y_des_vio, z_des_vio, yaw_des_vio]: [%f, %f, %f, %f]\n",
                            loop_counter, x_des_vio, y_des_vio, z_des_vio, yaw_des_vio);
                DEBUG("[%d] [VIO x_est_vio_startup, y_est_vio_startup, z_est_vio_startup, yaw_vio_est_startup]: [%f, %f, %f, %f]\n",
                            loop_counter, x_est_vio_startup, y_est_vio_startup, z_est_vio_startup, yaw_est_vio_startup);
            }
            */
            DEBUG("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
            DEBUG("[%d] [Reduced x_est, y_est, z_est, yaw_est]: [%f, %f, %f, %f]\n",
                        loop_counter, x_est-x_est_startup, y_est-y_est_startup, z_est-z_est_startup, yaw_est);
            DEBUG("[%d] [Reduced x_des, y_des, z_des, yaw_des]: [%f, %f, %f, %f]\n",
                        loop_counter, x_des-x_est_startup, y_des-y_est_startup, z_des-z_est_startup, yaw_des);

            /*
            if (mode == SN_VIO_POS_HOLD_MODE)
            {
                DEBUG("[%d] [Reduced VIO x_est_vio, y_est_vio, z_est_vio, yaw_est_vio]: [%f, %f, %f, %f]\n",
                            loop_counter, x_est_vio-x_est_vio_startup, y_est_vio-y_est_vio_startup, z_est_vio-z_est_vio_startup, yaw_est_vio);
                DEBUG("[%d] [Reduced VIO x_des_vio, y_des_vio, z_des_vio, yaw_des_vio]: [%f, %f, %f, %f]\n",
                            loop_counter, x_des_vio-x_est_vio_startup, y_des_vio-y_est_vio_startup, z_des_vio-z_est_vio_startup, yaw_des_vio);
            }
            */

            if ((SnDataStatus)snav_data->data_status.api_rc_status == SN_DATA_NOT_INITIALIZED)
            {
                DEBUG("[%d] [DataStatus api_rc_status==SN_DATA_NOT_INITIALIZED\n",loop_counter);
            }
            else
            {
                DEBUG("[%d] [DataStatus api_rc_status:[%d]\n",
                                    loop_counter,
                                    (SnDataStatus)snav_data->data_status.api_rc_status);
            }

            if ((SnDataStatus)snav_data->data_status.rc_active_status == SN_DATA_NOT_INITIALIZED)
            {
                DEBUG("[%d] [DataStatus rc_active_status==SN_DATA_NOT_INITIALIZED\n",loop_counter);
            }
            else
            {
                DEBUG("[%d] [DataStatus rc_active_status:[%d]\n",
                                    loop_counter,
                                    (SnDataStatus)snav_data->data_status.rc_active_status);
            }


            if (state == MissionState::ON_GROUND)
            {
                DEBUG("[%d] ON_GROUND\n", loop_counter);
            }
            else if (state == MissionState::STARTING_PROPS)
            {
                DEBUG("[%d] STARTING_PROPS\n", loop_counter);
            }
            else if (state == MissionState::TAKEOFF)
            {
                DEBUG("[%d] TAKEOFF\n", loop_counter);
                DEBUG("[%d] position_est_startup: [%f,%f,%f]\n",loop_counter,x_est_startup,y_est_startup,z_est_startup);
            }
            else if (state == MissionState::IN_MOTION)
            {
                DEBUG("[%d] IN_MOTION\n", loop_counter);

                if (circle_mission)
                {
                    DEBUG("[%d] circle_mission position #%u: [%f,%f,%f,%f]\n",
                                loop_counter,
                                current_position,
                                circle_positions[current_position].x,
                                circle_positions[current_position].y,
                                circle_positions[current_position].z,
                                circle_positions[current_position].yaw);
                }
            }
            else if (state == MissionState::LOITER)
            {
                DEBUG("[%d] LOITER\n", loop_counter);
            }
            else if (state == MissionState::LANDING)
            {
                DEBUG("[%d] LANDING\n", loop_counter);
            }
            else
            {
                DEBUG("[%d] STATE UNKNOWN\n", loop_counter);
            }
        }
        loop_counter++;
    }

    fclose(stdout);
    fclose(stderr);

    return 0;
}
