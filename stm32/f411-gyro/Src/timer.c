/*
 * timer.c
 *
 *  Created on: Oct 2, 2018
 *      Author: henorvell
 */

#include "timer.h"
#include "RingBuffer.h"

static RINGBUFFER(int_timestamp_buffer, 256, uint32_t);
static void (*sInterrupt_cb)(int int_num);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3) {
		uint32_t read_value = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_3);
		if(!RINGBUFFER_FULL(&int_timestamp_buffer))
			RINGBUFFER_PUSH(&int_timestamp_buffer, &read_value);
		if(sInterrupt_cb)
			sInterrupt_cb(1); //1 for first sensor flag
	}
}

uint64_t timer_get_irq_timestamp()
{
	uint32_t timestamp = 0;
	//disable_irq();
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	if(!RINGBUFFER_EMPTY(&int_timestamp_buffer))
		RINGBUFFER_POP(&int_timestamp_buffer, &timestamp);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	//enable_irq();
	return (uint64_t)timestamp;
}

int timer_clear_irq_timestamp()
{
	//disable_irq();
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	RINGBUFFER_CLEAR(&int_timestamp_buffer);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	//enable_irq();
	return 0;
}

int timer_configure_timebase(TIM_HandleTypeDef* timer_num, uint32_t frequency)
{
	uint32_t prescaler;
	/* Compute the prescaler value for the requested frequency */
	prescaler = (SystemCoreClock / frequency) - 1;
	if(prescaler > UINT16_MAX)
		return -1;
	__HAL_TIM_SET_PRESCALER(timer_num, prescaler);
	__HAL_TIM_SET_AUTORELOAD(timer_num, UINT32_MAX);
	return 0;
}

void timer_configure_callback(void (*interrupt_cb)(int int_num))
{
	sInterrupt_cb = interrupt_cb;
}

