#include <avr/eeprom.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define STANDBY  0
#define COOK  1
#define CONFIG  2
#define ERR 3
#define SAVE  4
#define AUTO  5

#define BUTTON_LEFT 1<<0
#define BUTTON_RIGHT  1<<1
#define BUTTON_UP 1<<2
#define BUTTON_DOWN 1<<3

#define BUTTON_LEFT_PIN   4
#define BUTTON_RIGHT_PIN  5
#define BUTTON_UP_PIN   6
#define BUTTON_DOWN_PIN   7 

// A0 is 14.
#define TEMP_PIN  A0
#define PID_PIN   3

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
ISR(TIMER2_OVF_vect) {
  // Will have to be a TPC https://playground.arduino.cc/Code/PIDLibraryRelayOutputExample
  // To enable slow relay or SSR (zero-crossings) to function.
  static unsigned long startTime;
  unsigned long now = millis();
  if (TPC_WINDOW < now - startTime) {
    startTime = now;
  }
  digitalWrite(PID_PIN, (now - startTime < output));
  TCNT2 = 0;
  TIFR2 = 0;
}

void setup() {
//  noInterrupts();
  TIMSK2 = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TIFR2 = 0;
  TCCR2A = 0; // COUNTER "NORMAL" mode.
  TCCR2B = (1 << CS22) + (1 << CS21) + (1 << CS20); // 255 * 1024/16MHz = 16ms
  TIMSK2 = (1 << TOIE2);
//  interrupts();
  
  eeprom_read_block((void *) &settings, (void *) 0, sizeof(settings));

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
  output = 0;
  TIMSK2 = 0;
}

void enablePID() {
  myPID.SetMode(AUTOMATIC);
  TIMSK2 = (1 << TOIE2);
}

void getTemp() {
  // Prolly will end up with a onewire temp probe, will be asynchronous readings.
  // if temp ready read and start a new conversion,
  // otherwise return.
  int i = analogRead(TEMP_PIN);
  temp = i/60 *32; // or some shit.
}


void getButtons() {
  buttons = 0;
  buttons &= digitalRead(BUTTON_LEFT_PIN);
  buttons &= digitalRead(BUTTON_RIGHT_PIN);
  buttons &= digitalRead(BUTTON_UP_PIN);
  buttons &= digitalRead(BUTTON_DOWN_PIN);
}

void err() {
  disablePID();
}

void standby() {
  disablePID();
  getTemp();
  if(buttons & BUTTON_RIGHT)
    mode = CONFIG;
  if(buttons & BUTTON_DOWN)
    mode = SAVE;
  if(buttons & BUTTON_RIGHT)
    mode = AUTO;
}

void config() {
  // do display
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
  eeprom_write_block((const void *)&settings, (void *) 0, sizeof(settings));
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
  } while(0 == autoTuner.Runtime());
  myPID.SetTunings(autoTuner.GetKp(), autoTuner.GetKp(), autoTuner.GetKd());
  mode = STANDBY;
}

void cook() {
  enablePID();
  // print display
  if(buttons & BUTTON_LEFT)
    mode = CONFIG;
}

void loop() {
  getTemp();
  if (AUTOMATIC == myPID.GetMode()) {
    myPID.Compute();
  }
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
