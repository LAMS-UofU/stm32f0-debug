#include "servo.h"


#define FREQUENCY 330
#define MINIMUM_us 500
#define MAXIMUM_us 2500
#define SERVO_PSC 8
#define SERVO_ARR 3030


void SERVO_init(void) 
{
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	TIM3->PSC = SERVO_PSC - 1;	
	TIM3->ARR = SERVO_ARR;

	/** Configure timer to use the PWM mode */
	/* Set CC1S to output (00) */
	TIM3->CCMR1 &= ~(0x3 << TIM_CCMR1_CC1S_Pos);
	/* Set OC1M to PWM Mode 1 (110) */
	TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);
	TIM3->CCMR1 &= ~(0x1 << TIM_CCMR1_OC1M_Pos);
	/* Enable output compare preload in channel 1 */
	TIM3->CCMR1 |= (0x1 << TIM_CCMR1_OC1PE_Pos);
	/* Enable channel 1 */
	TIM3->CCER |= (0x1 << TIM_CCER_CC1E_Pos);
	
	/** Enable/start timer */
	/* Set ARPE */
	TIM3->CR1 |= (0x1 << TIM_CR1_ARPE_Pos);
	/* Set CEN */
	TIM3->CR1 |= (0x1 << TIM_CR1_CEN_Pos);
	
	GPIO_InitTypeDef servo_pb4 = {
		GPIO_PIN_4,
		GPIO_MODE_AF_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_LOW,
		GPIO_AF1_TIM3
	};
	
	HAL_GPIO_Init(GPIOB, &servo_pb4);
	
	SERVO_set_angle(0);
}

void SERVO_set_angle(int angle)
{
	TIM3->CCR1 = MINIMUM_us + (angle * (MAXIMUM_us - MINIMUM_us)) / 180.0;
}
