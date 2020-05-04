/**
	* MSP Initialization and de-Initialization codes
	*
	* @file  : stm32f0xx_hal_msp.c
	* @author: Kris Wolff
	*
	*	@attention Copyright (c) 2020 STMicroelectronics. All rights reserved.
	* This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
	*/
	
#include "main.h"


/**
  * Initializes the Global MSP.
	* @return None
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}
