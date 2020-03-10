/*
 * timer.h
 *
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

uint64_t timer_get_irq_timestamp();
int timer_clear_irq_timestamp();
int timer_configure_timebase(TIM_HandleTypeDef* timer_num, uint32_t frequency);
void timer_configure_callback(void (*interrupt_cb)(int int_num));

#endif /* INC_TIMER_H_ */
