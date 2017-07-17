#include <EEPROM.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Debug info over Serial
//#define SERIAL_DEBUG

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

#define BUTTON_LEFT_PIN   4
#define BUTTON_RIGHT_PIN  5
#define BUTTON_UP_PIN     6
#define BUTTON_DOWN_PIN   7 

// A0 is 14.
#define TEMP_PIN  A0
#define PID_PIN   12

#define TPC_WINDOW 5000

struct settings_t {
  double Kp, Ki, Kd;
  int running;
} settings;

unsigned int mode = STANDBY;
double temp = 0;
double target = 60;
double output = 0;
unsigned int buttons = 0;

PID myPID(&temp, &output, &target, 0, 0, 0, P_ON_M, DIRECT);

uint32_t doOutput (uint32_t currentTime) {
  static unsigned long startTime;
  unsigned long now = millis();
  if (TPC_WINDOW < now - startTime) {
    startTime = now;
  }
  digitalWrite(PID_PIN, (now - startTime < output));
  return (currentTime + 50 * CORE_TICK_RATE); //50ms
}

void eeprom_write_block(const void *data, const unsigned int addr, size_t len) {
  unsigned int i;
  char *cdata = (char *) data;
  for (i = 0; len > i; ++i) {
    EEPROM.write(addr+i, cdata[i]);
  }
}

void eeprom_read_block(const void *data, const unsigned int addr, size_t len) {
  unsigned int i;
  char *cdata = (char *) data;
  for (i = 0; len > i; ++i) {
    cdata[i] = EEPROM.read(addr+i);
  }
}

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  Serial.println("Sous Vide Cooker");
  //printStatusSerial();
#endif
  eeprom_read_block((void *) &settings, 0, sizeof(settings));

  attachCoreTimerService(doOutput);
  
  setupPins();
    
  myPID.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  myPID.SetOutputLimits(0, TPC_WINDOW);
  
  disablePID();
  mode = STANDBY;
}


inline void setupPins() {
  pinMode(BUTTON_UP_PIN, INPUT);
  digitalWrite(BUTTON_DOWN_PIN, HIGH);
  pinMode(BUTTON_DOWN_PIN, INPUT);
  digitalWrite(BUTTON_RIGHT_PIN, HIGH);
  pinMode(BUTTON_RIGHT_PIN, INPUT);
  digitalWrite(BUTTON_RIGHT_PIN, HIGH);
  pinMode(BUTTON_LEFT_PIN, INPUT);
  digitalWrite(BUTTON_LEFT_PIN, HIGH);
  pinMode(TEMP_PIN, INPUT);
  pinMode(PID_PIN, OUTPUT);
}

void disablePID() {
  myPID.SetMode(MANUAL);
  settings.running = false;
  output = 0;
  detachCoreTimerService(doOutput);

  digitalWrite(PID_PIN, LOW);

}

void enablePID() {
  myPID.SetMode(AUTOMATIC);
  attachCoreTimerService(doOutput);
  settings.running = true;

//  digitalWrite(PID_PIN, LOW);

}

void getTemp() {
  // Prolly will end up with a onewire temp probe, will be asynchronous readings.
  // if temp ready read and start a new conversion,
  // otherwise return.
  int i = analogRead(TEMP_PIN);
  temp = 60; //Actually do this.
}


void getButtons() {
   buttons = 0;
  if(digitalRead(BUTTON_LEFT_PIN))
    buttons |= BUTTON_LEFT;
  if(digitalRead(BUTTON_RIGHT_PIN))
    buttons |= BUTTON_RIGHT;
  if(digitalRead(BUTTON_UP_PIN))
    buttons |= BUTTON_UP;
  if(digitalRead(BUTTON_DOWN_PIN))
    buttons |= BUTTON_DOWN;
    
#ifdef SERIAL_DEBUG
  char c = 0;
  if(Serial.available())
    c = Serial.read();
    switch (c) {
      case 'w':
        buttons |= BUTTON_UP;
        break;
      case 'a':
        buttons |= BUTTON_LEFT;
        break;
      case 's':
        buttons |= BUTTON_DOWN;
        break;
      case 'd':
        buttons |= BUTTON_RIGHT;
        break;

    }
#endif
}

void err() {
  disablePID();
}

void standby() {
#ifdef SERIAL_DEBUG
  Serial.println("Standby");
  Serial.println("d) Configuration");
  Serial.println("w) AutoTune");
  Serial.println("s) Save Config");
#endif
  disablePID();
  if(buttons & BUTTON_RIGHT)
    mode = CONFIG;
  if(buttons & BUTTON_DOWN)
    mode = SAVE;
  if(buttons & BUTTON_UP)
    mode = AUTO;
}

#ifdef SERIAL_DEBUG
inline void printStatusSerial() {
  Serial.println("------------");
  Serial.println("Temp\tXX");
  Serial.println("Target\tXX");
  if(settings.running)
    Serial.println("PID\tON");
  else
    Serial.println("PID\tOFF");
  Serial.println("------------");
}
#endif
void config() {
#ifdef SERIAL_DEBUG
  Serial.println("Config");
  Serial.println("a) Standby");
  Serial.println("d) Start");
  Serial.println("w) Increase Target");
  Serial.println("s) Decrease Target");
#endif
  if(buttons & BUTTON_LEFT)
    mode = STANDBY;
  if(buttons & BUTTON_RIGHT)
    mode = COOK;
  if(buttons & BUTTON_UP)
    target += 1;
  if(buttons & BUTTON_DOWN)
    target -= 1;
}

void save() {
  settings.Kp = myPID.GetKp();
  settings.Ki = myPID.GetKi();
  settings.Kd = myPID.GetKd();
  eeprom_write_block((const void *)&settings, 0, sizeof(settings));
  mode = STANDBY;
}

void autoTune() {
  //LCD clear
  // Autotuning... Temp may be erratic. Do not run unless equilibrium.
  PID_ATune autoTuner(&temp, &output);
  autoTuner.SetNoiseBand(0.5);
  autoTuner.SetOutputStep(1);
  autoTuner.SetLookbackSec(10);
  autoTuner.SetControlType(1); // PID
  do {
    getTemp();
    //delay(200);
  } while(0 == autoTuner.Runtime());
  myPID.SetTunings(autoTuner.GetKp(), autoTuner.GetKp(), autoTuner.GetKd());
  mode = STANDBY;
}

void cook() {
  #ifdef SERIAL_DEBUG
  Serial.println("Cook");
  Serial.println("a) Config");
  #endif
  enablePID();
  // print display
  if(buttons & BUTTON_LEFT)
    mode = CONFIG;
}

void loop() {
  getTemp();
  if (settings.running)
    myPID.Compute();
  getButtons();
  switch (mode) {
    case STANDBY:
      standby();
      break;
    case SAVE:
      save();
      break;
    case AUTO:
      autoTune();
      break;
    case CONFIG:
      config();
      break;
    case COOK:
      cook();
      break;
    default:
      err();
  }
  delay(200);
}
