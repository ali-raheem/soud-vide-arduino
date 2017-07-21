// Debug info over Serial
#define SERIAL_DEBUG

#include "sous_vide.h"
#include "modes.h"

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Sous Vide Cooker");
#endif
  
  pinMode(BUTTON_UP_PIN, INPUT);
  pinMode(BUTTON_DOWN_PIN, INPUT);
  pinMode(BUTTON_RIGHT_PIN, INPUT);
  pinMode(BUTTON_LEFT_PIN, INPUT);
  digitalWrite(BUTTON_LEFT_PIN, HIGH);
  digitalWrite(BUTTON_RIGHT_PIN, HIGH);
  digitalWrite(BUTTON_DOWN_PIN, HIGH);
  digitalWrite(BUTTON_UP_PIN, HIGH);
  pinMode(PID_PIN, OUTPUT);
  
  EEPROM.setMaxAddress(sizeof(settings));
  eeprom_read_block((void *) &settings, 0, sizeof(settings));
  settings.running = false;
  sensors.begin();
  sensors.setWaitForConversion(false);
  int rv = sensors.getDeviceCount();
#ifdef SERIAL_DEBUG
  if(1 > rv)
    Serial.println("Fatal error: No temperature sensor.");
  else
    Serial.println("Found temperature sensor");
#endif
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.requestTemperaturesByAddress(tempDeviceAddress);
  myPID.SetTunings(50, 50, 50);
  myPID.SetOutputLimits(0, TPC_WINDOW);
  disablePID();
  rv = attachCoreTimerService(doOutput);
#ifdef SERIAL_DEBUG
  if(0 == rv)
    Serial.println("Fatal error: No ISR slot available for PID");
  else
    Serial.println("PID registered with ISR");
#endif
  changeMode(STANDBY);
#ifdef SERIAL_DEBUG
  Serial.println("Setup done.");
#endif
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
  delay(100);
}

