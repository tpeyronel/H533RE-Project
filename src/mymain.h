#pragma once
#include "FreeRTOS.h"
#include "stm32h5xx_hal.h"

void mymain();
void hal_tim_period_elapsed_callback(TIM_HandleTypeDef* htim, BaseType_t* xHigherPriorityTaskWoken);