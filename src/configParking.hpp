#define pragma once
#include <Particle.h>

/***************************************************
  ELECTRON DEFINITIONS
****************************************************/
//VERSION
// PRODUCT_ID(9104);
// PRODUCT_VERSION(39);

//Set Electron mode to manual
SYSTEM_MODE(MANUAL);
// Reset reason
//STARTUP(System.enableFeature(FEATURE_RESET_INFO));
// Persistent RAM
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

/***************************************************
  PINS SETUP
****************************************************
+--------------+----------+
| Description  | Electron |
+--------------+----------+
| PIN_PWR0     | B5       |
| PIN_PWR1     | B3       |
| PIN_PWM_S0   | B4       |
| PIN_PWM_S1   | B2       |
|              |          |
| PIN_LED_RED  | B0       |
| PIN_LED_BLUE | B1       |
|              |          |
+--------------+----------+
*/
#define PIN_PWR0 B5
#define PIN_PWR1 B3
#define PIN_PWM_S0 B4
#define PIN_PWM_S1 B2

#define PIN_LED_R B0
#define PIN_LED_B B1

/***************************************************
  DEFINITIONS
****************************************************/

#define TIMEOUT_LED 10000          // Time LEDs power on
#define TIMEOUT_CELL 20000         // Timeout Cell conection
#define TIMEOUT_PARTICLE 15000     // Timeout Particle conection
#define TIMEOUT_WD (1 * 60 * 1000) // Watchdog timeout
#define TIMEOUT_OTA 15000          // Timeout OTA update
#define TIMEOUT_SENSOR 5000        // Timeout reading sensor
#define TIMESLEEP 60               // Sleep time
#define TIMESLEEP_READING 5        // Sleep time between reading
#define TIME_LIGHTS_ENABLE_START 9 // Ligths will be enable at X hour of day (localtime)
#define TIME_LIGHTS_ENABLE_END 21  // Ligths will be enable until X hours of day (localtime)
#define LENGHT_JSON_DATA 500       // Lenght of data
#define LENGHT_JSON_SYS 500        // Lenght of dataSys
#define DELAY_BOOT_SENSOR 300      // time needed for sensor boot
#define TIME_BETWEEN_SAMPLES 5     // We take 3 samples per read, the time between samples is X ms

/***************************************************
  CONSTANTS
****************************************************/
const int COUNTER_HEARTBEAT = 60;      // 60 times (60 minutes aprox)
const int COUNTER_SYSHEARTBEAT = 120;  // 1440times (1440 minutes = 24 Hours aprox)
const int POSSIBLE_DISTANCE = -1;      // Code possible car
const int SENSOR_READING_ERROR = -100; // Code error
const int NOWDAYS = 1545545700;        // Near timestamp to day
const int MAX_INVALIDS_READS = 20;     //
const int MAX_READS = 3;               // Num of reads
const int SAMPLES_PER_READING = 3;     //Num of times (samples) on each reading
const int MIN_VALID_CHANGES = 3;       //Changes of present state min to publish a change
/***************************************************
  VARIABLES
****************************************************/
enum Presence // States of presences
{
  NOT_PRESENT,        //0
  PRESENT,            //1
  PROHIBITED,         //2
  INFRACTION,         //3
  POSSIBLE_P,         //4
  POSSIBLE_I,         //5
  SENSOR_NOT_WORKING, //6
  NULLSTATE           //7
};

enum Task //State of task
{
  INIT,                   //0
  CHECK_RESTRICTION_TIME, //1
  READ_DISTANCE,          //2
  CONNECT_CELL,           //3
  RESET_CELL,             //4
  CONNECT_PARTICLE,       //5
  TX_DATA,                //6
  CHECK_UPDATES,          //7
  DISCONNECT_CELL,        //8
  CLEAR_FLAGS,            //9
  PWROFF                  //10
};

struct Parking
{
  float distance;
  float lastDistance;
  float rangeMin;
  float rangeMax;
  Presence presence;
  Presence lastPresence;
  int cChange;
  int cReadsInvalids;
};

// Info update changes
char *reason[] = {
    (char *)"leaving",             //0
    (char *)"arriving",            //1
    (char *)"leaving infraction",  //2
    (char *)"infraction",          //3
    (char *)"possible car",        //4
    (char *)"possible infraction", //5
    (char *)"error sensor",        //6
    (char *)"heartbeat",           //7
    (char *)"sys heartbeat",       //8
    (char *)"invalid measure",     //9
    (char *)"boot"};               //10
