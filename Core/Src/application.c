/*
 * demo.c
 *
 *  Created on: Aug 31, 2020
 *      Author: Tom
 */

#include "application.h"

volatile Measurement_state state;
volatile uint16_t timer2_ic_rising_cnt = 0;
volatile uint16_t timer2_ic_falling_cnt = 0;

ring_buffer usart_tx_buf;

void start_timer2(void)
{
	// Enable timebase generation, output compare on channel 1 & input capture on channel 2 w/interrupts
	LL_TIM_SetUpdateSource(TIM2, LL_TIM_UPDATESOURCE_COUNTER);
	LL_TIM_EnableUpdateEvent(TIM2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableIT_CC1(TIM2);
	LL_TIM_EnableIT_CC2(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM2);
}

void usart_log(char* str)
{
	while (*str != '\0') {
		if (!ring_buffer_full(&usart_tx_buf)) {
			ring_buffer_enqueue(&usart_tx_buf, *str);
			++str;
		} else {
			break;
		}
	}
	LL_USART_EnableIT_TXE(USART2);
}

void output_reading(char* str_buf, uint16_t time_us)
{
	uint16_t distance_cm = DISTANCE_CM(time_us);
	sprintf(str_buf, "Distance: %dcm\n\r", distance_cm);
	usart_log(str_buf);
}
