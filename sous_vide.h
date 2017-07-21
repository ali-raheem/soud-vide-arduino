#ifndef SOUS_VIDE_H
#define SOUS_VIDE_H
#define SERIAL_DEBUG
#include <Arduino.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define STANDBY 0
#define COOK    1
#define CONFIG  2
#define ERR     3
#define SAVE    4
#define AUTO    5

#define BUTTON_LEFT   1<<0
#define BUTTON_RIGHT  1<<1
#define BUTTON_UP     1<<2
#define BUTTON_DOWN   1<<3

#ifdef SERIAL_DEBUG
#define BUTTON_HELP   1<<4
bool printed = false;
#endif

#define BUTTON_LEFT_PIN   28
#define BUTTON_RIGHT_PIN  29
#define BUTTON_UP_PIN     30
#define BUTTON_DOWN_PIN   31

#define ONE_WIRE_BUS 32
#define PID_PIN   33

#define TPC_WINDOW 5000

struct settings_t {
  double Kp, Ki, Kd;
  int running;
} settings;

unsigned int mode = STANDBY;
unsigned int oldMode = mode;
double temp = 0;
double target = 60;
double output = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

PID myPID(&temp, &output, &target, 0, 0, 0, P_ON_M, DIRECT);

uint32_t doOutput (uint32_t currentTime);
void eeprom_write_block(const void *data, const unsigned int addr, size_t len);
void eeprom_read_block(const void *data, const unsigned int addr, size_t len);

void printStatusSerial(void);
void disablePID(void);
void enablePID(void);
void getTemp(void);
#endif
