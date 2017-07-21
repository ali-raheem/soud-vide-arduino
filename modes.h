#ifndef MODE_H
#define MODE_H

#include "sous_vide.h"

static unsigned int buttons = 0;
static bool serialPrinted = false;
void err(void);
void cook(void);
void save(void);
void standby(void);
void autoTune(void);
void config(void);

void getButtons(void);
void changeMode(const unsigned int newMode);
#endif
