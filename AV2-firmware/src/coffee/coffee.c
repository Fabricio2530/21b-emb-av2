/*
 * aps2.c
 *
 * Created: 10/05/2021 11:11:07
 *  Author: RafaelCF4
 */ 

#include "coffee.h"

static int heat_on = 0;
static int pump_on = 0;

static void config_DAC(void) {
  sysclk_enable_peripheral_clock(DACC_ID);
  dacc_reset(DACC_BASE);
  dacc_set_transfer_mode(DACC_BASE, 0);
  dacc_set_osr(DACC_BASE, DACC_CHANNEL, 5 );
  dacc_enable_channel(DACC_BASE, DACC_CHANNEL);
  dacc_set_analog_control(DACC_BASE, DACC_ANALOG_CONTROL);
}

void coffe_pump_on(void) {
	printf("[PUMP] ON \n");
	pio_clear(LED_PIO, LED_IDX_MASK);
	pump_on = 1;
}

void coffe_pump_off(void) {
	printf("[PUMP] OFF \n");
	pio_set(LED_PIO, LED_IDX_MASK);
	pump_on = 0;
}

void coffee_heat_on(void) {
	printf("[HEAT] ON \n");
	heat_on = 1;
}

void coffee_heat_off(void) {
	printf("[HEAT] OFF \n");
	heat_on = 0;
}

int temp_to_analog(int temp) {
	return (temp * 4095 / 100);
}

int analog_to_temp(int ana) {
	return ( ((float) ana) * 100.0 / 4095.0 );
}

void task_av2(void *pvParameters) {
  
  int temperature = TEMP_MIN;
  
  config_DAC();
  
  pmc_enable_periph_clk(LED_PIO_ID);
  pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT);

  for (;;)  {
	  
	 
	if (pump_on) {
		temperature -= TEMP_RAMP_DOWN;
	} else if (heat_on) {
		temperature += TEMP_RAMP_UP;
	} else {
		temperature -= TEMP_RAMP_DOWN;
	}

	if (temperature > TEMP_MAX) {
		temperature = TEMP_MAX;
	}		
	else if (temperature < TEMP_MIN) {
		temperature = TEMP_MIN;
	}
	
	printf("[TEMP] %d, [HEAT] %d, [PUMP] %d \n", temperature, heat_on, pump_on );
   	dacc_write_conversion_data(DACC_BASE, temp_to_analog(temperature) , DACC_CHANNEL);
    vTaskDelay(400);
  }
}

