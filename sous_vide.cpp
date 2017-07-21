#include "sous_vide.h"

uint32_t doOutput (uint32_t currentTime) {
  static uint32_t startTime = currentTime;
  if (currentTime - startTime > TPC_WINDOW * CORE_TICK_RATE)
    startTime = currentTime;
  digitalWrite(PID_PIN, currentTime - startTime < output * CORE_TICK_RATE);
  return (currentTime + 33 * CORE_TICK_RATE);
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

void disablePID(void) {
  myPID.SetMode(MANUAL);
  settings.running = false;
  output = 0;
  //detachCoreTimerService(doOutput);
  //digitalWrite(PID_PIN, LOW);
}

void enablePID(void) {
  myPID.SetMode(AUTOMATIC);
  //attachCoreTimerService(doOutput);
  settings.running = true;
}

void getTemp(void) {
  // read voltage and convert it to celcius.
  static unsigned long lastTempReq = millis();
  unsigned long now = millis();
  if (now - lastTempReq >= 750) {
    temp = sensors.getTempC(tempDeviceAddress);
    sensors.requestTemperaturesByAddress(tempDeviceAddress);
    lastTempReq = now;
  }
}

#ifdef SERIAL_DEBUG
void printStatusSerial(void) {
  Serial.println("------------");
  Serial.print("Temp\t");
  Serial.print(temp, DEC);
  Serial.println();
  Serial.print("Target\t");
  Serial.print(target, DEC);
  Serial.println();
  if(settings.running)
    Serial.println("PID\tON");
  else
    Serial.println("PID\tOFF");
  Serial.println("------------");
}
#endif


