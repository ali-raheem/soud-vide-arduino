#include <avr/eeprom.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define STANDBY	0
#define COOK	1
#define CONFIG	2
#define ERR	3
#define SAVE	4
#define AUTO	5

#define BUTTON_LEFT	1<<0
#define BUTTON_RIGHT	1<<1
#define BUTTON_UP	1<<2
#define BUTTON_DOWN	1<<3

#define BUTTON_LEFT_PIN		4
#define BUTTON_RIGHT_PIN	5
#define BUTTON_UP_PIN		6
#define BUTTON_DOWN_PIN		7 


#define TEMP_PIN	A0
#define PID_PIN		A1

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

void setup() {
// Serial.
// setup

	eeprom_read_block((void *)&settings, (void *) 0, sizeof(settings));

	pinMode(BUTTON_UP_PIN, INPUT); //etc...
	digitalWrite(BUTTON_UP_PIN, HIGH); // pull up
	pinMode(TEMP_PIN, INPUT);

	myPID.setTunings(settings.Kp, settings.Ki, settings.Kd);
	disablePID();
	mode = STANDBY;
}

void disablePID() {
	myPID.setMode(MANUAL);
	output = 0;
}

void getTemp() {
	int i = analogRead(TEMP_PIN);
	temp = i/60 *32; // or some shit.
}
void doOutput() {
	// This will likely become a Timer ISR using 
	analogWrite(PID_PIN, output);
}

unsigned int getButtons() {
	int buttons = 0;
	buttons &= digitalRead(BUTTON_LEFT_PIN);
	buttons &= digitalRead(BUTTON_RIGHT_PIN);
	buttons &= digitalRead(BUTTON_UP_PIN);
	buttons &= digitalRead(BUTTON_DOWN_PIN);
	return buttons;
}

void err() {
// disable output
// log time **
// await reset
// disable interupts
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
	settings.Kp = myPID.getKp();
	settings.Ki = myPID.getKi();
	settings.Kd = myPID.getKd();
	eeprom_write_block((const void *)&settings, (void *) 0, sizeof(settings));
	mode = STANDBY;
}

void auto() {
	//LCD clear
	// Autotuning... Temp may be erratic. Do not run unless equilibrium.
	PID_ATune autoTuner(&temp, &output);
	autoTuner.setNoiseBand(0.5);
	autoTuner.setOutputStep(1);
	autoTuner.setLookBackSec(10);
	autoTuner.setControlType(1); // PID
	do {
		getTemp();
		doOutput();
	} while(0 == autoTuner.Runtime());
	myPID.setTunings(autoTuner.getKp(), autoTuner.getKp(), autoTuner.getKd());
	mode = STANDBY;
}

void cook() {
	myPID.setMode(AUTOMATIC);
	// print display
	if(buttons & BUTTON_LEFT)
		mode = CONFIG;
}

void loop() {
	getTemp();
	if (AUTOMATIC == myPID.getMode) {
		myPID.Compute();
		doOutput();
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
			autotune();
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
