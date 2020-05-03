#include "main.h"

void USART1_IRQHandler(void);
void USART3_4_IRQHandler(void);

void USART1_init(int baud_rate);
void USART1_transmit_single_character(char ch);
void USART1_transmit_string(char* string);

void USART3_init(int baud_rate);
void USART3_transmit_single_character(char ch);
void USART3_transmit_string(char* string);
