/**
	*	Cortex-M0 Processor Interruption and Exception Handlers
	*
	* @file  : stm32f0xx_it.c
	* @author: Kris Wolff
	*
	* @attention Copyright (c) 2020 STMicroelectronics. All rights reserved.
	* This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
	*/
	
#include "main.h"
#include "stm32f0xx_it.h"


extern volatile uint8_t lidar_timer;


/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {}

	
/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) { while (1); }


/**
  * Handles System service call via SWI instruction.
	* @return None
  */
void SVC_Handler(void) {}

	
/**
  * Handles Pendable request for system service.
  * @return None
	*/
void PendSV_Handler(void) {}

	
/**
  * Handles System tick timer.
	* @return None
  */
void SysTick_Handler(void) { 
	HAL_IncTick();
	if (lidar_timer > 0)
		lidar_timer--;
}
