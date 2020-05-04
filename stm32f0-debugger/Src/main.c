/**
	* Entry point for Application
	*
	*	@file  : main.c
	* @author: Kris Wolff
	*/

#include <string.h>
#include "main.h"
#include "servo.h"
#include "usart.h"
#include "usb_debug.h"
#include "lidar.h"


void SystemClock_Config(void);


/**
  * Application entry point.
  * @return int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

	LIDAR_init(); /* RPLIDAR A2M8 -- PB3 (PWM), PB10 (TX), PB11 (RX) */
	SERVO_init(); /* SPT5435LV-180 servo -- PB4 (PWM)) */
  USB_init(); 	/* COM3 @ BR: 115200 -- PB6 (TX), PB7 (RX) */
	
	while (1)
  {
		USB_process();
		LIDAR_process();
  }
}



/**
  * System Clock Configuration
  * @return None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * Executed in case of error occurrence.
  * @return None
  */
void Error_Handler(void) {}

#ifdef  USE_FULL_ASSERT
/**
  * Reports the name of the source file and the source line number where the 
	* assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @return None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif
	
