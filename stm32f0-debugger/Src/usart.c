/**
	*	USART1 and USART3 initialization, and trasmission
	*
	* @file  : usart.c
	* @author: Kris Wolff
	*/
#include "usart.h"


volatile char usb_received_value;
volatile int usb_newData_flag;

volatile char lidar_received_value;
volatile int lidar_newData_flag;


/**
	*	Handles USART1 interrupt
	* @return None
	*/
void USART1_IRQHandler(void)
{
	if ((USART1->ISR >> USART_ISR_RXNE_Pos) & 0x1) {
		usb_received_value = USART1->RDR;
		usb_newData_flag = 1;
	}
}



/**
	*	Handles USART3 or 4 interrupt
	* @return None
	*/
void USART3_4_IRQHandler(void)
{
	if ((USART3->ISR >> USART_ISR_RXNE_Pos) & 0x1) {
		lidar_received_value = USART3->RDR;
		lidar_newData_flag = 1;
	} 
}



/**
	*	Setting up USART1 for GPIO pins PB6 (TX) and PB7 (RX)
	* @return None
	*
*/
void USART1_init(int baud_rate)
{
	/* Enable the system clocks for USART1 in the RCC */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/* Setup PB6 and PB7 to use AF4 alternate function for USART1 TX/RX communications 
		 - PB6 (TX): USART1_TX (AF0)
		 - PB7 (RX): USART1_RX (AF0)	*/
	GPIO_InitTypeDef uart1 = {
		GPIO_PIN_6 | GPIO_PIN_7,
		GPIO_MODE_AF_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_LOW,
		GPIO_AF0_USART1
	};
	
	/* Initialize PB6 and PB7 */
	HAL_GPIO_Init(GPIOB, &uart1);
	
	/* Set baud rate */
	USART1->BRR = (unsigned int)(HAL_RCC_GetHCLKFreq() / baud_rate);
	
	/* Enable transmitter */
	USART1->CR1 |= USART_CR1_TE;
	
	/* Enable receiver	*/
	USART1->CR1 |= USART_CR1_RE;
	
	/* Enable USART1 receive register not emtpy interrupt */
	USART1->CR1 |= USART_CR1_RXNEIE;
	
	/* Setup USART1 interrupt handler and set priority in NVIC */
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 2);
	
	/* Enable USART
		 NOTE: enable after everything, otherwise register become read-only */
	USART1->CR1 |= USART_CR1_UE;
}



/**
	* Places inputted character into TDR register and checks whether the 
	* register is empty or not before inputting
	* @return None
	*/
void USART1_transmit_single_character(char ch)
{
	/* Check and wait on the USART status flag that indicates the transmit 
		 register is empty. Write the character into the transmit data register.
		 No need to manually clear the status bit, automatically modified by the 
		 peripheral.	*/
	while (((USART1->ISR >> USART_ISR_TXE_Pos) & 0x1) != 0x1);
	USART1->TDR = ch;
}



/**
	* Sends inputted string using USART1_Transmit_Single_Character
	* @return None
	*/
void USART1_transmit_string(char* string)
{
	int idx = 0;
	while (string[idx] != '\0') {
		USART1_transmit_single_character(string[idx]);
		idx++;
	}
}



/**
	*	Setting up USART3 for GPIO pins PB10 (TX) and PB11 (RX)
	* @return None
	*/
void USART3_init(int baud_rate)
{
	/* Enable the system clocks for USART3 in the RCC */
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/* Setup PB10 and PB11 to use AF4 alternate functions for USART3 TX/RX 
	   communications */
	/* Configure pin alternate functions
		 - PB10 (TX): USART3_TX (AF4)
		 - PB11 (RX): USART3_RX (AF4) */
	GPIO_InitTypeDef uart3 = {
		GPIO_PIN_10 | GPIO_PIN_11,
		GPIO_MODE_AF_PP,
		GPIO_PULLDOWN,
		GPIO_SPEED_FREQ_LOW,
		GPIO_AF4_USART3
	};
	
	/* Initialize PB10 and PB11 */
	HAL_GPIO_Init(GPIOB, &uart3);
	
	/* Set baud rate */
	USART3->BRR = (unsigned int)(HAL_RCC_GetHCLKFreq() / baud_rate);
	
	/* Enable transmitter: USART_CR1 bit 3 (TE) */
	USART3->CR1 |= USART_CR1_TE;
	
	/* Enable receiver: USART_CR1 bit 2 (RE) */
	USART3->CR1 |= USART_CR1_RE;
	
	/* Enable USART3 receive register not empty interrupt: USART_CR1 bit 5 (RXNEIE) */
	USART3->CR1 |= USART_CR1_RXNEIE;
	
	/* Setup USART3 interrupt handler and set priority in NVIC */
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
	
	/*Enable USART: USART_CR1 bit 0 (UE) 
		NOTE: enable after everything, otherwise registers become read-only */
	USART3->CR1 |= USART_CR1_UE;
}



/**
	* Place inputted character into TDR register and checks whether the register 
	* is empty or not before inputting
	* @return None
	*/
void USART3_transmit_single_character(char ch)
{
	/* Check and wait on the USART status flag that indicates the transmit 
		 register is empty. Write the character into the transmit data register.
		 No need to manually clear the status bit, automatically modified by the 
		 peripheral.	*/
	while (((USART3->ISR >> USART_ISR_TXE_Pos) & 0x1) != 0x1);
	USART3->TDR = ch;
}



/**
	* Sends inputted string using USART3_Transmit_Single_Character
	* @return None
	*/
void USART3_transmit_string(char* string)
{
	int idx = 0;
	while (string[idx] != '\0') {
		USART3_transmit_single_character(string[idx]);
		idx++;
	}
}
