#include "modes.h"
#define SERIAL_DEBUG
void getButtons(void) {
   buttons = 0;
  if(!digitalRead(BUTTON_LEFT_PIN))
    buttons |= BUTTON_LEFT;
  if(!digitalRead(BUTTON_RIGHT_PIN))
    buttons |= BUTTON_RIGHT;
  if(!digitalRead(BUTTON_UP_PIN))
    buttons |= BUTTON_UP;
  if(!digitalRead(BUTTON_DOWN_PIN))
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
      case '?':
        buttons |= BUTTON_HELP;
        break;

    }
   // Serial.print(buttons, BIN);
#endif
}


void cook(void) {
  if(not settings.running)
    enablePID();
  #ifdef SERIAL_DEBUG
  if(buttons & BUTTON_HELP || not serialPrinted) {
    serialPrinted = true;
    Serial.println("Cook");
    printStatusSerial();
    Serial.println("a) Config");
  }
  #endif
  if(buttons & BUTTON_LEFT)
    changeMode(CONFIG);
}

void save(void) {
#ifdef SERIAL_DEBUG
  Serial.println("Saving PID settings");
#endif
  eeprom_write_block((const void *)&settings, 0, sizeof(settings));
  changeMode(STANDBY);
}

void autoTune(void) {
  enablePID();
  PID_ATune autoTuner(&temp, &output);
  autoTuner.SetNoiseBand(0.07);
  autoTuner.SetOutputStep(50);
  autoTuner.SetLookbackSec(10);
  autoTuner.SetControlType(1); // PID
  output = 2500;
#ifdef SERIAL_DEBUG
    Serial.println("Autotuning...");
    Serial.println("a) Cancel");
#endif
  do {
    getTemp();
    getButtons();
    if(buttons & BUTTON_LEFT) {
      autoTuner.Cancel();
      changeMode(STANDBY);
      return;
    }
  } while(0 == autoTuner.Runtime());
  settings.Kp = autoTuner.GetKp();
  settings.Ki = autoTuner.GetKi();
  settings.Kd = autoTuner.GetKd();
#ifdef SERIAL_DEBUG
  Serial.print("Kp: ");
  Serial.print(settings.Kp, DEC);
  Serial.println();
  Serial.print("Ki: ");
  Serial.print(settings.Ki, DEC);
  Serial.println();
  Serial.print("Kd: ");
  Serial.print(settings.Kd, DEC);
  Serial.println();
#endif
  myPID.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  changeMode(STANDBY);
}

void config(void) {
#ifdef SERIAL_DEBUG
  if(buttons & BUTTON_HELP || not serialPrinted) {
    serialPrinted = true;
    Serial.println("Config");
    printStatusSerial();
    Serial.println("a) Standby");
    Serial.println("d) Start");
    Serial.println("w) Increase Target");
    Serial.println("s) Decrease Target");
  }
#endif
  if(buttons & BUTTON_LEFT)
    changeMode(STANDBY);
  if(buttons & BUTTON_RIGHT)
    changeMode(COOK);
  if(buttons & BUTTON_UP) {
    target += 0.25;
#ifdef SERIAL_DEBUG
    serialPrinted = false;
#endif
  }
  if(buttons & BUTTON_DOWN) {
    target -= 0.25;
#ifdef SERIAL_DEBUG
    serialPrinted = false;
#endif
  }
}

void err(void) {
#ifdef SERIAL_DEBUG
  if(buttons & BUTTON_HELP || not serialPrinted) {
    serialPrinted = true;
    Serial.println("ERROR!");
    printStatusSerial();
  }
#endif
  disablePID();
}

void standby(void) {
  if(settings.running)
    disablePID();
#ifdef SERIAL_DEBUG
  if(buttons & BUTTON_HELP || not serialPrinted) {
    serialPrinted = true;
    Serial.println("Standby");
    printStatusSerial();
    Serial.println("d) Configuration");
    Serial.println("w) AutoTune");
    Serial.println("s) Save Config");
  }
#endif
  if(buttons & BUTTON_RIGHT)
    changeMode(CONFIG);
  if(buttons & BUTTON_DOWN)
    changeMode(SAVE);
  if(buttons & BUTTON_UP)
    changeMode(AUTO);
}

void changeMode(const unsigned int newMode) {
  oldMode = mode;
  mode = newMode;
#ifdef SERIAL_DEBUG
    serialPrinted = false;
#endif
}
