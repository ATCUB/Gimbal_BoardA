#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define SINGLE_MOVE_MODE

#ifdef SINGLE_MOVE_MODE
#define FRIC_UP_MAX 1650
#define FRIC_UP_UP 1650
#define FRIC_UP 1400
#define FRIC_DOWN 1320
#define FRIC_DOWN_MIN 1180
#define FRIC_OFF 1000

#else
#define FRIC_UP 1400
#define FRIC_DOWN 1320
#define FRIC_OFF 1500

#define FRIC1_UP 1300
#define FRIC1_DOWN 1380
#define FRIC2_UP 1700
#define FRIC2_DOWN 1620
#define FRIC_OFF 1500

#endif

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
