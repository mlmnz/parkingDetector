#include "configParking.hpp"
#include <ArduinoJson.h>

/***************************************************
  VARIABLES
****************************************************/

//Loop Variable
Task task;
long beginDateState, endDateState;    // Timestamp for Prohibited State
uint8_t beginTimeState, endTimeState; // Time for Prohibited State in hours

// 'Flags" variables and persistent
retained bool firstPowerOn = true; // validate if the code was executed in the first flash
retained uint8_t cReading;
retained uint8_t cHeartbeat;     // Counter if a change has not been sent for COUNTER_HEARTBEAT mins
retained uint16_t cSysheartbeat; // Counter SYS Hearbeat

// Flags
bool timeOutFlag;     // Is a timer on?
bool lightsEnable;    // Turn on lights?
bool publishChanges;  // publish?
bool restrictedTime;  // Restriction time enable?
bool updateTimestamp; // Need sync the timestamp?
bool debugOn;

// System Variables
String deviceId;                                  // Unique ID
ApplicationWatchdog wd(TIMEOUT_WD, System.reset); // Set Wathcdog

// Variables to publish
retained Parking p0, p1;         // Init ParkingSensor 0,1
char *updateReason;              // What was the reason to publish
char jsonSys[LENGHT_JSON_SYS];   // JSON Format variable
char jsonData[LENGHT_JSON_DATA]; // JSON Format variable

/***************************************************
  CALLBACKS
****************************************************/
/* Callback for LEDs*/
//Blink RED Les
void cb_blinkLedR()
{
  static bool onOffR = false;
  digitalWrite(PIN_LED_R, onOffR);
  onOffR = !onOffR;
}

//Blink Blue RED
void cb_blinkLedB()
{
  static bool onOffB = false;
  digitalWrite(PIN_LED_B, onOffB);
  onOffB = !onOffB;
}

// Timer general timeout
void cb_timeOut()
{
  timeOutFlag = false;
}

/***************************************************
  TIMERS / COUNTERS
****************************************************/
uint8_t contRst;                         // Counter of cellular power on fail
unsigned long tOutLR, tOutLB;            // Counters for Timeout led RED and BLUE
Timer timerLR(300, cb_blinkLedR);        // SoftTimer RTOS for blinking led RED
Timer timerLB(500, cb_blinkLedB);        // SoftTimer RTOS for blinking led BLUE
Timer timerOut(30000, cb_timeOut, true); // SoftTimer RTOS for general timeouts withc run once

/***************************************************
  SETUP
****************************************************/

void setup()
{
  //Debug
  debugOn = true;
  if (debugOn)
  {
    Serial.begin(115200); //230400
    delay(3500);
  }

  //Turn off RGB Led
  // RGB.control(true);
  // RGB.color(0, 0, 0);

  //Inputs
  pinMode(PIN_PWM_S0, INPUT);
  pinMode(PIN_PWM_S1, INPUT);
  // Outputs
  pinMode(PIN_PWR0, OUTPUT);  // Ouput power sensor 0
  pinMode(PIN_PWR1, OUTPUT);  // Ouput power sensor 1
  pinMode(PIN_LED_B, OUTPUT); // Ouput power sensor 0
  pinMode(PIN_LED_R, OUTPUT); // Ouput power sensor 1

  //Sensor Setup

  //Device ID
  deviceId = System.deviceID();

  //Some variable init
  updateTimestamp = false;
  restrictedTime = false;
  lightsEnable = false;
  timeOutFlag = false;

  //Set time zone for local time
  Time.zone(-4);

  // Force publish first power on, update initial values
  if (firstPowerOn)
  {
    if (debugOn)
      Serial.println("Firts power on");

    //Set default variables
    p0.presence = NULLSTATE;
    p0.rangeMax = 6.0;
    p0.rangeMin = 0.5;
    p0.lastDistance = 0;
    p0.lastPresence = NULLSTATE;

    p1.presence = NULLSTATE;
    p1.rangeMax = 6.0;
    p1.rangeMin = 0.5;
    p1.lastDistance = 0;
    p1.lastPresence = NULLSTATE;

    contRst = 0;
    cReading = 0;
    cSysheartbeat = 60; //force 1st sys update
    publishChanges = true;
    updateReason = reason[10];
    firstPowerOn = false;
  }

  //For spaces L992/L993 and L994/L995 uncomment follow lines (UTC timestamp)
  // beginDateState = 1554091200; // 1st april/2019
  // endDateState = 1575172800;   // 1st december/2019
  // beginTimeState = 8;        // 8h00 localtime (12 UTC)
  // endTimeState = 9;          // 9h00 localtime (13h UTC)

  //Just for stay sure
  task = INIT;
}

/***************************************************
  LOOP
****************************************************/
void loop()
{

  switch (task)
  {

  case INIT:
    //Check if times is valid
    if (Time.now() < NOWDAYS)
    {
      //Update the time before process
      updateTimestamp = true;
      task = CONNECT_CELL;
    }
    else
      task = CHECK_RESTRICTION_TIME;

    break; //INIT

  case CHECK_RESTRICTION_TIME:
    // Check if lights will be available in this windows time
    if (TIME_LIGHTS_ENABLE_START <= Time.hour() && Time.hour() <= TIME_LIGHTS_ENABLE_END)
      lightsEnable = true;

    //The timestamp is between the dates of restrictions?
    if (beginDateState <= Time.local() && Time.local() <= endDateState)
    {
      //The timestamp is between the time of restrictions?
      if (beginTimeState <= Time.hour() && Time.hour() <= endTimeState)
        restrictedTime = true;
    }
    task = READ_DISTANCE;
    break; //CHECK_RESTRICTION_TIME

  case READ_DISTANCE:

    // Reading counter
    cReading++;

    // Check if car is present or not.
    checkChanges();

    // If the presence condition change, publish changes xD
    if (publishChanges)
      task = CONNECT_CELL;
    else if (cReading < MAX_READS)
      System.sleep(SLEEP_MODE_DEEP, TIMESLEEP_READING);
    else
      task = CLEAR_FLAGS;

    if (debugOn)
      Serial.printlnf("\nLecturas : %d      Publish: %d\n ", cReading, publishChanges);

    break; //READ_DISTANCE

  case CONNECT_CELL:
    // Turn on cellular and connect to data network
    Cellular.on();
    Cellular.connect();

    //Wait for cellular ready, if not ready X seconds, restart it
    if (waitFor(Cellular.ready, TIMEOUT_CELL))
      task = CONNECT_PARTICLE;
    else
      task = RESET_CELL;
    break; //CONNECT_CELL

  case RESET_CELL: //Mini "WD Cellular"
    //Power off modem
    Cellular.off();

    // If cellular fail twices, deep sleep for 5 seconds and restart
    if (contRst > 1)
      System.sleep(SLEEP_MODE_DEEP, 10);
    else
      contRst++;

    //Sleep for 2 seconds and continue
    System.sleep(WKP, RISING, 2);

    task = CONNECT_CELL;
    break; //RESET_CELL

  case CONNECT_PARTICLE:
    //Cycle Particle Cloud
    Particle.connect();
    Particle.process();

    // Wait until the Cloud is connected to publish, if not, reset cell
    if (waitFor(Particle.connected, TIMEOUT_PARTICLE))
    {
      //Sync time
      if (updateTimestamp)
      {
        Particle.syncTime();
        // Wait until Electron receives time from Particle Device Cloud (or connection to Particle Device Cloud is lost)
        waitUntil(Particle.syncTimeDone);
        updateTimestamp = false;
        task = INIT;
      }
      else
        task = TX_DATA;
    } //endif waitfor
    else
      task = RESET_CELL;

    break; //CONNECT_PARTICLE

  case TX_DATA:

    //Publish dataSys after
    if (cSysheartbeat == 0)
    {
      jsonGeneratorDataSys();
      Particle.publish("DataSys", jsonSys, PRIVATE);
      task = CHECK_UPDATES;
    }
    else
    {
      jsonGeneratorData();
      Particle.publish("DataSensor", jsonData, PRIVATE);
      task = DISCONNECT_CELL;
    }
    break;

  case CHECK_UPDATES:
    // Force reconnection cloud
    Particle.publish("spark/device/session/end", "", PRIVATE);
    Particle.disconnect();
    Particle.connect();
    Particle.process();

    // Wait until the Cloud is connected
    if (waitFor(Particle.connected, TIMEOUT_PARTICLE))
    {
      setTimeOut(TIMEOUT_OTA);
      while (timeOutFlag)
      {
        Particle.process();
      }
    }

    task = DISCONNECT_CELL;
    break; //CHECK_UPDATES

  case DISCONNECT_CELL:

    // Turn off Cellular and wait until Cellular is ready the next cycle of changes
    Particle.disconnect();
    Cellular.off();

    // Goto save variable
    task = CLEAR_FLAGS;

    break; //DISCONNECT_CELL

  case CLEAR_FLAGS:
    publishChanges = false;
    cReading = 0;
    task = PWROFF;
    break; //CLEAR_FLAGS

  case PWROFF:
    //Check if Timers are actives before go to sleep
    if (!publishChanges && !timerLB.isActive() && !timerLR.isActive())
    {
      unsigned int timeSleep = millis() / 1000;
      timeSleep = (TIMESLEEP > timeSleep ? (TIMESLEEP - timeSleep) : TIMESLEEP);
      System.sleep(SLEEP_MODE_DEEP, timeSleep);
    }
    break; //PWROFF

  } //end switch

  // Wait until LEDs shutdwn
  if (timerLB.isActive())
    if (millis() - tOutLB > TIMEOUT_LED)
    {
      timerLB.stop();
      digitalWrite(PIN_LED_B, LOW);
    }
  if (timerLR.isActive())
    if (millis() - tOutLR > TIMEOUT_LED)
    {
      timerLR.stop();
      digitalWrite(PIN_LED_R, LOW);
    }

  wd.checkin(); // resets the AWDT count
} //loop

/***************************************************
FUNCIONS
****************************************************/

//* Check Update Function *//
void checkChanges()
{

  // Get the distances from sensors
  getDistance(PIN_PWR0, PIN_PWM_S0, p0);
  getDistance(PIN_PWR1, PIN_PWM_S1, p1);

  //check if state condition has changed
  checkStates(p0);
  checkStates(p1);

  // Lights, publish only when MAX_READS was complete
  if (cReading >= MAX_READS)
  {
    //Only if we had 1 change, publish it. If not, publish the last distance and state only if the other parking publish
    if (p0.cChange >= MIN_VALID_CHANGES)
    {
      publishChanges = true;
      updateReason = reason[p0.presence];
      p0.lastDistance = p0.distance;
      p0.lastPresence = p0.presence;
    }
    else
    {
      p0.distance = p0.lastDistance;
      p0.presence = p0.lastPresence;
    }

    if (p1.cChange >= MIN_VALID_CHANGES)
    {
      publishChanges = true;
      updateReason = reason[p1.presence];
      p1.lastDistance = p1.distance;
      p1.lastPresence = p1.presence;
    }
    else
    {
      p1.distance = p1.lastDistance;
      p1.presence = p1.lastPresence;
    }
    // Update reason

    // Reset possible changes countrs
    p0.cChange = 0;
    p1.cChange = 0;

    // if is restrictedTime aand publish
    if (restrictedTime && publishChanges)
    {
      tOutLR = millis();
      timerLR.start();
    }
    else if (lightsEnable && publishChanges) //if not restricted time and light time is enable and publish, turn on leds
    {
      tOutLB = millis();
      timerLB.start();
    }
  }

  //If it is not update for 60 minutes, force to update.
  if (!publishChanges)
  {
    cHeartbeat++;
    if (cHeartbeat >= COUNTER_HEARTBEAT)
    {
      updateReason = reason[7]; //cHeartbeat
      publishChanges = true;
      // Reset counter for next cHeartbeat
      cHeartbeat = 0;
    }
  }

  // Counter for send hearbeat with sysdata
  if (cSysheartbeat >= COUNTER_SYSHEARTBEAT)
  {
    updateReason = reason[8]; //sysheartbeat
    publishChanges = true;
    cSysheartbeat = 0; //
  }
  //Only count minutes of sysheartbeat on the 1st read of cycle
  if (cReading == 0)
    cSysheartbeat++;

  if (debugOn)
    Serial.printlnf("Sensor 0: %f      %d\nSensor 1: %f      %d", p0.distance, p0.presence, p1.distance, p1.presence);

} //endCheckchanges

//* Get Distance from Sensor Function *//
void getDistance(int sensorPWR, int sensorPWM, Parking &pX)
{
  unsigned int rawPulses = 0;
  //Power on sensor and wait 300ms
  digitalWrite(sensorPWR, HIGH);
  delay(DELAY_BOOT_SENSOR - TIME_BETWEEN_SAMPLES);

  for (int i = 0; i < SAMPLES_PER_READING; i++)
  {
    delay(TIME_BETWEEN_SAMPLES);
    rawPulses += pulseIn(sensorPWM, HIGH);
  }

  digitalWrite(sensorPWR, LOW);
  pX.distance = rawPulses / (SAMPLES_PER_READING * 5900.0);
}

void checkStates(Parking &pX)
{
  //Save the distances values in case possible car and retrieve the orignal value
  float tDistance;
  if (-20 < pX.distance && pX.distance < 0) //-20 and 0 are limits of real distance, never we have a distance > 20m as valid
  {
    tDistance = POSSIBLE_DISTANCE;
    pX.distance = pX.distance * POSSIBLE_DISTANCE;
  }
  else
    tDistance = pX.distance;

  if (tDistance != SENSOR_READING_ERROR)
  {
    // CAR
    if ((pX.rangeMin < tDistance && tDistance < pX.rangeMax) || (tDistance == POSSIBLE_DISTANCE))
    {

      if (restrictedTime)
      {

        if (tDistance == POSSIBLE_DISTANCE && pX.presence != POSSIBLE_I)
        {
          pX.presence = POSSIBLE_I;
        }
        else if (pX.presence != INFRACTION)
        {
          pX.presence = INFRACTION;
        }
      }
      else if (tDistance == POSSIBLE_DISTANCE && pX.presence != POSSIBLE_P)
      {
        pX.presence = POSSIBLE_P;
      }
      else if (pX.presence != PRESENT)
      {
        pX.presence = PRESENT;
      }
    }
    // NO CAR
    else
    {
      if (restrictedTime)
      {
        if (pX.presence != PROHIBITED)
        {
          pX.presence = PROHIBITED;
        }
      }
      else if (pX.presence != NOT_PRESENT)
      {
        pX.presence = NOT_PRESENT;
      }
    }
  }
  else if (pX.presence != SENSOR_NOT_WORKING)
  {
    pX.presence = SENSOR_NOT_WORKING;
  }

  if (pX.presence != pX.lastPresence)
    pX.cChange++;
}
//* JSON Generator for Data Function *//
void jsonGeneratorData()
{
  StaticJsonDocument<LENGHT_JSON_DATA> dataObj;

  JsonObject parking = dataObj.createNestedObject("parking");

  JsonArray parking_distance = parking.createNestedArray("distance");
  parking_distance.add(p0.distance);
  parking_distance.add(p1.distance);

  JsonArray parking_presence = parking.createNestedArray("presence");
  parking_presence.add((int)p0.presence);
  parking_presence.add((int)p1.presence);

  char charUpdate[sizeof(updateReason)];
  strcpy(charUpdate, updateReason);
  dataObj["reason"] = charUpdate;

  serializeJson(dataObj, jsonData);
}

//* JSON Generator for DataSys Function *//
void jsonGeneratorDataSys()
{
  StaticJsonDocument<LENGHT_JSON_SYS> sysObj;

  JsonObject parking = sysObj.createNestedObject("parking");

  JsonObject parking_range = parking.createNestedObject("configured range");

  JsonArray parking_range_min = parking_range.createNestedArray("min");
  parking_range_min.add(p0.rangeMin);
  parking_range_min.add(p1.rangeMin);

  JsonArray parking_range_max = parking_range.createNestedArray("max");
  parking_range_max.add(p0.rangeMax);
  parking_range_max.add(p1.rangeMax);

  serializeJson(sysObj, jsonSys);
}

//* General Timeou Function *//
void setTimeOut(int time)
{
  timeOutFlag = true;
  timerOut.changePeriod(time);
}