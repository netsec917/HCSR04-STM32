/*
 * application.h
 *
 *  Created on: Aug 31, 2020
 *      Author: Tom
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include "main.h"
#include "stdbool.h"
#include "ring_buffer.h"
#include "string.h"
#include "stdio.h"

#define DISTANCE_CM(time_us) ((time_us / 2) / 29)

#define SLEEP_MODE() {\
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;\
	FLASH->ACR &= ~FLASH_ACR_SLEEP_PD; \
	__WFI(); \
}

// Enum for FSM representing HCSR04 measurement cycle
typedef enum {
	TRIG_BEGIN,
	TRIG_END,
	SAMPLE_BEGIN,
	SAMPLE_END,
	SAMPLE_COMPLETE
} Measurement_state;

extern volatile Measurement_state state;
extern volatile uint16_t timer2_ic_rising_cnt;
extern volatile uint16_t timer2_ic_falling_cnt;
extern ring_buffer usart_tx_buf;

void start_timer2(void);
void output_reading(char* str_buf, uint16_t time_us);
void usart_log(char* str);
#endif /* INC_APPLICATION_H_ */
