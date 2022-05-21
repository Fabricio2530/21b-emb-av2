#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// AV2
#include "coffee/coffee.h"
#include "PIO_OLED.h"

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

#define USART_COM_ID ID_USART1
#define USART_COM USART1

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PC31


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_ADC_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY            (tskIDLE_PRIORITY)

/************************************************************************/
/* glboals                                                               */
/************************************************************************/
QueueHandle_t xQueueADC;
QueueHandle_t xQueueOLED;
QueueHandle_t xQueueBUT;
QueueHandle_t xQueueRTT;
QueueHandle_t xQueueTC;

typedef struct {
	uint value;
} adcData;

int doing_coffe = 0;
int rtt_on = 0;
int on = 1;
int flag_rtt = 0;
int but = 0;

/************************************************************************/
/* PROTOtypes                                                           */
/************************************************************************/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

static void USART1_init(void);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);
static void configure_console(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) { configASSERT( ( volatile void * ) NULL ); }

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

void TC1_Handler(void) {
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
	
	int value = 1;
	//ENVIANDO OS DADOS DOS LEDS
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueTC, &value, &xHigherPriorityTaskWoken);
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	 /* IRQ due to Time has changed */
	 if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		
	 }

	 /* IRQ due to Alarm */
	 if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		int signal;
		
		if (flag_rtt) {
			signal = 2;	
		} else {
			signal = 1;
		}
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(xQueueRTT, &signal, &xHigherPriorityTaskWoken);
	 } 
}

static void AFEC_pot_Callback(void) {
	adcData adc;
	adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}

void but1_callback(void){
	//indica qual o botão que foi apertado
	int value = 1;
	
	if (on == 0) {
		on = 1;
		flag_rtt = 0;
	}
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueBUT, &value, &xHigherPriorityTaskWoken);
}

void but2_callback(void){
	int value = 2;
	
	if (on == 0) {
		on = 1;
		flag_rtt = 0;
	}
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueBUT, &value, &xHigherPriorityTaskWoken);
}

void but3_callback(void) {

}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void desenha_rect(int t, int n_max){
	gfx_mono_generic_draw_filled_rect(80, 18, 40, 7, GFX_PIXEL_CLR);
	gfx_mono_generic_draw_rect(80, 18, 40, 7, GFX_PIXEL_SET);
	
	int width = 40;
	double inc = (double) width/n_max;
	gfx_mono_generic_draw_filled_rect(80, 18, inc*t, 7, GFX_PIXEL_SET);	
}

void limpa_visor() {
	gfx_mono_generic_draw_filled_rect(80, 18, 40, 7, GFX_PIXEL_CLR);
	gfx_mono_generic_draw_rect(80, 18, 40, 7, GFX_PIXEL_SET);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	
	int temp;
	int rtt_signal;
	int tc_signal;
	int contador = 0;
	for (;;)  {
		 if (xQueueReceive(xQueueOLED, &(temp), 1000)) {
			
			//iniciando a lógica dos processos da AV2
			if (temp < 85 && !doing_coffe && on) {
				gfx_mono_draw_string("Aquecendo            ", 0, 0, &sysfont);
				gfx_mono_draw_string("           ", 0, 20, &sysfont);
				coffee_heat_on();
				coffe_pump_off();
				limpa_visor();
				
				if (xQueueReceive(xQueueTC, &(tc_signal),0)){
					pin_toggle(LED_PI1, LED_PI1_IDX_MASK);
					pin_toggle(LED_PI2, LED_PI2_IDX_MASK);
					pin_toggle(LED_PI3, LED_PI3_IDX_MASK);
				}
					
			} else if (temp > 80 && !doing_coffe && on) {
				gfx_mono_draw_string("Pronta               ", 0, 0, &sysfont);
				gfx_mono_draw_string("           ", 0, 20, &sysfont);
				coffee_heat_on();
				coffe_pump_off();
				
				if (!flag_rtt){
					RTT_init(4, 80, RTT_MR_ALMIEN);	
				} 
				
				pio_clear(LED_PI1, LED_PI1_IDX_MASK);
				pio_clear(LED_PI2, LED_PI2_IDX_MASK);
				pio_clear(LED_PI3, LED_PI3_IDX_MASK);
				
				flag_rtt = 1;
				
			} else if (on && doing_coffe) {
				gfx_mono_draw_string("Fazendo cafe         ", 0, 0, &sysfont);
				coffee_heat_off();
				coffe_pump_on();
				
				if (xQueueReceive(xQueueTC, &(tc_signal),0)){
					if (but == 1) {
						pin_toggle(LED_PI1, LED_PI1_IDX_MASK);
						pio_clear(LED_PI2, LED_PI2_IDX_MASK);
						pio_clear(LED_PI3, LED_PI3_IDX_MASK);
						desenha_rect(contador, 50);
					} else if (but == 2){
						pin_toggle(LED_PI2, LED_PI2_IDX_MASK);
						pio_clear(LED_PI1, LED_PI1_IDX_MASK);
						pio_clear(LED_PI3, LED_PI3_IDX_MASK);
						desenha_rect(contador, 100);
					}
				}
				
				contador++;
			}
			
			if (temp > 80 && !doing_coffe) {
				if (xQueueReceive(xQueueBUT, &(but), 0)) {
					
					doing_coffe = 1;
					flag_rtt = 0;
					contador = 0;
					
					//dentro de cada um desses ifs, começamos o RTT de alarme
					if (but == 1) {
						gfx_mono_draw_string("Simples    ", 0, 20, &sysfont);
						RTT_init(4, 20, RTT_MR_ALMIEN);
					} else if (but == 2) {
						gfx_mono_draw_string("Duplo      ", 0, 20, &sysfont);
						RTT_init(4, 40, RTT_MR_ALMIEN);
					}
					
				}	
			}
			
			if (xQueueReceive(xQueueRTT, &(rtt_signal),0)){
				printf("Verificando o RTT signal --------------\n");
				if (rtt_signal == 2) {
					on = 0;
					gfx_mono_draw_string("DESLIGADA            ", 0, 0, &sysfont);
					coffe_pump_off();
				} else {
					doing_coffe = 0;
					gfx_mono_draw_string("Cafe PRONTO", 0, 20, &sysfont);
				}
			
			if (!on) {
				coffee_heat_off();
				coffe_pump_off();
				pio_set(LED_PI1, LED_PI1_IDX_MASK);
				pio_set(LED_PI2, LED_PI2_IDX_MASK);
				pio_set(LED_PI3, LED_PI3_IDX_MASK);
			}
			
			}
		 }
	}
}

static void task_adc(void *pvParameters) {
	
	 // configura ADC e TC para controlar a leitura
	 config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
	 TC_init(TC0, ID_TC1, 1, 10);
	 tc_start(TC0, 1);

	 // variável para recever dados da fila
	 adcData adc;

	 while (1) {
		 if (xQueueReceive(xQueueADC, &(adc), 1000)) {
			 int temp_digital = (100 * adc.value)/4095;
			 printf("O valor de temp lido é %d\n", temp_digital);
			 BaseType_t xHigherPriorityTaskWoken = pdTRUE;
			 xQueueSendFromISR(xQueueOLED, &temp_digital, &xHigherPriorityTaskWoken);
		 }
		 
	 }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
  uint32_t ul_div;
  uint32_t ul_tcclks;
  uint32_t ul_sysclk = sysclk_get_cpu_hz();

  pmc_enable_periph_clk(ID_TC);

  tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
  tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
  tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

  NVIC_SetPriority((IRQn_Type)ID_TC, 4);
  NVIC_EnableIRQ((IRQn_Type)ID_TC);
  tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	oled_init();
	
	pio_handler_set(BUT_PI1,
	BUT_PI1_ID,
	BUT_PI1_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but1_callback);
	
	pio_handler_set(BUT_PI2,
	BUT_PI2_ID,
	BUT_PI2_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but2_callback);
	
	pio_handler_set(BUT_PI3,
	BUT_PI3_ID,
	BUT_PI3_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but3_callback);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_PI1, BUT_PI1_IDX_MASK);
	pio_get_interrupt_status(BUT_PI1);
	
	pio_enable_interrupt(BUT_PI2, BUT_PI2_IDX_MASK);
	pio_get_interrupt_status(BUT_PI2);
	
	pio_enable_interrupt(BUT_PI3, BUT_PI3_IDX_MASK);
	pio_get_interrupt_status(BUT_PI3);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PI1_ID);
	NVIC_SetPriority(BUT_PI1_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT_PI2_ID);
	NVIC_SetPriority(BUT_PI2_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT_PI3_ID);
	NVIC_SetPriority(BUT_PI3_ID, 4); // Prioridade 4

	/* Initialize the console uart */
	configure_console();
	
	//CRIA A FILA
	xQueueADC = xQueueCreate(100, sizeof(adcData));
	if (xQueueADC == NULL)
	printf("falha em criar a queue xQueueADC \n");
	
	xQueueOLED = xQueueCreate(100, sizeof(adcData));
	if (xQueueOLED == NULL)
	printf("falha em criar a queue xQueueOLED \n");
	
	xQueueBUT = xQueueCreate(100, sizeof(int));
	if (xQueueOLED == NULL)
	printf("falha em criar a queue xQueueBUT \n");
	
	xQueueRTT = xQueueCreate(100, sizeof(int));
	if (xQueueOLED == NULL)
	printf("falha em criar a queue xQueueRTT \n");
	
	xQueueTC = xQueueCreate(100, sizeof(int));
	if (xQueueOLED == NULL)
	printf("falha em criar a queue xQueueTC \n");
	

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_av2, "av2", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_adc, "ADC", TASK_ADC_STACK_SIZE, NULL,
	TASK_ADC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test ADC task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
