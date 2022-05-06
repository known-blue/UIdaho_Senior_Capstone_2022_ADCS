#ifndef __ACTUATORS_H__
#define __ACTUATORS_H__

#include "global_definitions.h"
#include "DRV10970.h"

extern DRV10970 flywhl;

void initFlyWhl(void);

#endif