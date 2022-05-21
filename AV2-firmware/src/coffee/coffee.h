/*
 * aps2.h
 *
 * Created: 10/05/2021 11:07:47
 *  Author: RafaelCF4
 */ 


#ifndef COFFEE_H_
#define COFFEE_H_

#include <asf.h>
#include "arm_math.h"

/************************************************************************/
/* DAC                                                                  */
/************************************************************************/
#define DACC_CHANNEL        0 // (PB13)
#define DACC_BASE           DACC
#define DACC_ID             ID_DACC
#define DACC_ANALOG_CONTROL (DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02))

/************************************************************************/
/* COFFEE                                                               */
/************************************************************************/

#define TEMP_MAX 90
#define TEMP_MIN 18

#define TEMP_RAMP_UP 5
#define TEMP_RAMP_DOWN 3

#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

/************************************************************************/
/* FUNCTIONS                                                            */
/************************************************************************/
void task_av2(void *pvParameters);

void coffe_pump_on(void);
void coffe_pump_off(void);
void coffee_heat_on(void);
void coffee_heat_off(void);
int analog_to_temp(int ana);

#endif /* COFFEE_H_ */