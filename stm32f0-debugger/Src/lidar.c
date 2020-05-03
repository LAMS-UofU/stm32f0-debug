/**	NOTE: requests are in little endian order
	*
	*/
	

#include <stdio.h>
#include "lidar.h"
#include "usb_debug.h"


#define MAX_PRINT_BUFFER_SIZE		256
#define MAX_RESPONSE_SIZE				1080

#define LIDAR_FREQUENCY 	25000 // Hz
#define LIDAR_PSC 				16
#define LIDAR_ARR 				20
#define LIDAR_DUTY_CYCLE 	60   	// %

#define LIDAR_BAUD_RATE 115200

#define LIDAR_REQ_START_BIT				0xA5
#define LIDAR_REQ_STOP 				 		0x25
#define LIDAR_REQ_RESET 			 		0x40
#define LIDAR_REQ_SCAN				 		0x20
#define LIDAR_REQ_EXPRESS_SCAN 		0x82
#define LIDAR_REQ_FORCE_SCAN			0x21
#define LIDAR_REQ_GET_INFO				0x50
#define LIDAR_REQ_GET_HEALTH			0x52
#define LIDAR_REQ_GET_SAMPLERATE	0x59

#define LIDAR_SEND_MODE_SINGLE_RES	0x0
#define LIDAR_SEND_MODE_MULTI_RES		0x1



extern volatile char lidar_received_value;
extern volatile int lidar_newData_flag;

volatile uint8_t lidar_timer;
volatile uint8_t lidar_timing;
volatile uint8_t lidar_scanning;

volatile unsigned char lidar_request;
uint32_t byte_count;

static char print_buffer[MAX_PRINT_BUFFER_SIZE];

#define RESP_DESC_SIZE 7
/**	start1        - 1 byte (0xA5)
	* start2        - 1 byte (0x5A)
	*	response_info - 32 bits ([32:30] send_mode, [29:0] data_length)
	*	data_type 		- 1 byte	*/
struct response_descriptor {
	uint8_t start1;
	uint8_t start2;
	uint32_t response_info;
	uint8_t data_type;
};
struct response_descriptor resp_desc;

char DATA_RESPONSE[MAX_RESPONSE_SIZE];



void LIDAR_init(void)
{
	USART3_init(LIDAR_BAUD_RATE);
	LIDAR_PWM_init();
}



void LIDAR_send(char* message)
{
	USART3_transmit_string(message);
}




void LIDAR_process(void)
{
	unsigned data_idx;
	
	if (!lidar_timer && lidar_timing) {
		switch(lidar_request) {
			case LIDAR_REQ_STOP:
				LIDAR_RES_stop();
				break;
			case LIDAR_REQ_RESET:
				LIDAR_RES_reset();
				break; 
		};
		return;
	}
			
	// No new data
	if (!lidar_newData_flag)
		return;
	
	lidar_newData_flag = 0;
	
	/** Process response descriptor */
	switch (byte_count) {
		case 0:
			resp_desc.start1 = lidar_received_value;
			byte_count++;
			return;
		case 1:
			resp_desc.start2 = lidar_received_value;
			byte_count++;
			return;
		case 2:
			resp_desc.response_info = lidar_received_value;
			byte_count++;
			return;
		case 3:
			resp_desc.response_info |= ((unsigned)lidar_received_value << 8);
			byte_count++;
			return;
		case 4:
			resp_desc.response_info |= ((unsigned)lidar_received_value << 16);
			byte_count++;
			return;
		case 5:
			resp_desc.response_info |= ((unsigned)lidar_received_value << 24);
			byte_count++;
			return;
		case 6:
			resp_desc.data_type = lidar_received_value;
			byte_count++;
			return;
		
		default:		 
			data_idx = byte_count - RESP_DESC_SIZE;
			DATA_RESPONSE[data_idx] = lidar_received_value; 
			byte_count++;
	};
	
	if (byte_count == (resp_desc.response_info & 0x3FFFFFFF) + RESP_DESC_SIZE) {
		switch(lidar_request) {
			case  LIDAR_REQ_SCAN:
				LIDAR_RES_scan();
				return;
			case  LIDAR_REQ_EXPRESS_SCAN:
				LIDAR_RES_express_scan();
				return;
			case  LIDAR_REQ_FORCE_SCAN:
				LIDAR_RES_force_scan();
				return;
			case  LIDAR_REQ_GET_INFO:
				LIDAR_RES_get_info();
				break;
			case  LIDAR_REQ_GET_HEALTH:
				LIDAR_RES_get_health();
				break;
			case  LIDAR_REQ_GET_SAMPLERATE:
				LIDAR_RES_get_samplerate();
				break;
			default:
				return;
		};
		byte_count = 0;
		USB_send("CMD?");
		USB_reset_command();
		return;
	}
}



void LIDAR_reset_print_buffer(void)
{
	int i;
	for (i=0; i<MAX_PRINT_BUFFER_SIZE; i++)
		print_buffer[i] = '\0';
}



void LIDAR_reset_response_descriptor(void)
{
	resp_desc.start1 = 0;
	resp_desc.start2 = 0;
	resp_desc.response_info = 0;
	resp_desc.data_type = 0;
}



// channel 2
void LIDAR_PWM_init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	TIM2->PSC = LIDAR_PSC - 1;
	TIM2->ARR = LIDAR_ARR;
	
	/** Configure timer to use the PWM mode */
	/* Set CC2S to output (00) */
	TIM2->CCMR1 &= ~(0x3 << TIM_CCMR1_CC2S_Pos);
	/* Set OC2M to PWM Mode 1 (110) */
	TIM2->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos);
	TIM2->CCMR1 &= ~(0x1 << TIM_CCMR1_OC2M_Pos);
	/* Enable output preload in channel 2 */
	TIM2->CCMR1 |= (0x1 << TIM_CCMR1_OC2PE_Pos);
	/* Enable channel 2 */
	TIM2->CCER |= (0x1 << TIM_CCER_CC2E_Pos);
	
	/** Enable/start timer */
	/* Set ARPE */
	TIM2->CR1 |= (0x1 << TIM_CR1_ARPE_Pos);
	/* Set CEN */
	TIM2->CR1 |= (0x1 << TIM_CR1_CEN_Pos);
	
	GPIO_InitTypeDef lidar_pwm_pb3 = {
		GPIO_PIN_3,
		GPIO_MODE_AF_PP,
		GPIO_PULLDOWN,
		GPIO_SPEED_FREQ_LOW,
		GPIO_AF2_TIM2
	};

	HAL_GPIO_Init(GPIOB, &lidar_pwm_pb3);
}



void LIDAR_PWM_start(void)
{
	/* Set duty cycle */
	TIM2->CCR2 = (LIDAR_DUTY_CYCLE * LIDAR_ARR) / 100.0;
}



void LIDAR_PWM_stop(void)
{
	/* Set duty cycle */
	TIM2->CCR2 = 0;
}


/**	Request LiDAR to exit scanning state and enter idle state. No response!
	* Need to wait at least 1 millisecond before sending another request.
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_STOP (0x25)				*/
void LIDAR_REQ_stop(void)
{
	lidar_request = LIDAR_REQ_STOP;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	USB_reset_command();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_STOP);
	LIDAR_send(print_buffer);
	
	/** Wait 1 ms. Decrementing in @ref SysTick_Handler every 1 ms */
	lidar_timer = 1;
	lidar_timing = 1;
}



/** Request LiDAR to reset (reboot) itself by sending this request. State 
	* similar to as it LiDAR had just powered up. No response! Need to wait at
	* least 2 milliseconds before sending another request.
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_RESET (0x40)				*/
void LIDAR_REQ_reset(void)
{
	lidar_request = LIDAR_REQ_RESET;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	USB_reset_command();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_RESET);
	LIDAR_send(print_buffer);
	
	/** Wait 2 ms. Decrementing in @ref SysTick_Handler every 1 ms */
	lidar_timer = 2;
	lidar_timing = 1;
}


/** Request LiDAR to enter scanning state and start receiving measurements.
	* Note: RPLIDAR A2 and other device models support 4khz sampling rate will 
	* lower the sampling rate when processing this request. Please use EXPRESS_SCAN 
	* for the best performance.
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_SCAN (0x20)				*/
void LIDAR_REQ_scan(void)
{
	// Send GET_SAMPLERATE to get info about single measurement sampling time  
	
	lidar_request = LIDAR_REQ_SCAN;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_SCAN);
	LIDAR_send(print_buffer);
	
	lidar_scanning = 1;
}



/** Request LiDAR to enter scanning state and start receiving measurements.
	* Different from SCAN request as this will make LiDAR work at the sampling 
	* rate as high as it can be.  For RPLIDAR A2 and the device models support 
	* 4khz sampling rate, the host system is required to send this request to 
	* let the RPLIDAR work at 4khz sampling rate and output measurement sample 
	* data accordingly; for the device models with the 2khz sample rate (such as 
	* RPLIDAR A1 series), this request will implement the same sampling rate as 
	* the scan(SCAN) request.
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_EXPRESS_SCAN (0x82)		
	*									+2		working_mode (set to 0)
	*									+3		reserved_field (set to 0)
	*									+4		reserved_field (set to 0)
	*									+5		reserved_field (set to 0)
	*									+6		reserved_field (set to 0)		*/
void LIDAR_REQ_express_scan(void)
{
	// Send GET_SAMPLERATE to get info about single measurement sampling time  
	
	lidar_request = LIDAR_REQ_EXPRESS_SCAN;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_EXPRESS_SCAN);
	LIDAR_send(print_buffer);
	
	lidar_scanning = 1;
}



/** Request LiDAR to forcefully start measurement sampling and send out the 
	* results immediately. Useful for device debugging.	
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_FORCE_SCAN (0x21)		*/
void LIDAR_REQ_force_scan(void)
{
	lidar_request = LIDAR_REQ_FORCE_SCAN;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_FORCE_SCAN);
	LIDAR_send(print_buffer);
	
	lidar_scanning = 1;
}



/** Request LiDAR to send device information.
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_GET_INFO (0x50)		*/
void LIDAR_REQ_get_info(void)
{
	lidar_request = LIDAR_REQ_GET_INFO;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_GET_INFO);
	LIDAR_send(print_buffer);
}



/** Request LiDAR health state. If it has entered the Protection Stop state 
	* caused by hardware failure, relate code of the failure will be sent out.
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_GET_HEALTH (0x52)		*/
void LIDAR_REQ_get_health(void)
{
	lidar_request = LIDAR_REQ_GET_HEALTH;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_GET_HEALTH);
	LIDAR_send(print_buffer);
}



/** Request LiDAR to send single measurement duration for scan mode and express 
	*	scan mode. 
	*		Byte Order:		+0		Request Start Bit (0xA5)
	*									+1		LIDAR_REQ_GET_SAMPLERATE (0x59)		*/
void LIDAR_REQ_get_samplerate(void)
{
	lidar_request = LIDAR_REQ_GET_SAMPLERATE;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_REQ_START_BIT, LIDAR_REQ_GET_SAMPLERATE);
	LIDAR_send(print_buffer);
}



/** "STOP" request has no response. Instead using @ref SysTick_Handler to wait 
	* 1 ms with precision. */
void LIDAR_RES_stop(void) 
{
	if (!lidar_timer) {
		byte_count = 0;
		lidar_timing = 0;
		USB_send("DONE\r\nCMD?");
	}
}
	
/** "RESET" request has no response. Instead using @ref SysTick_Handler to wait 
	* 2 ms with precision. */
void LIDAR_RES_reset(void) 
{
	if (!lidar_timer) {
		byte_count = 0;
		lidar_timing = 0;
		USB_send("DONE\r\nCMD?");
	}
}
	


/** Process "SCAN" request's response
	*		Byte Offset:	+0		quality, ~start, start
	*		Order 8->0		+1		angle[6:0], check
	*									+2		angle_q6[14:7]
	*									+3		distance_q2[7:0]
	*									+4		distance_q2[15:8]		*/
void LIDAR_RES_scan(void) 
{
	LIDAR_reset_print_buffer();
	
	uint16_t angle = ((uint16_t)DATA_RESPONSE[1] >> 1) + 
											((uint16_t)DATA_RESPONSE[2] << 7);
	uint16_t distance = DATA_RESPONSE[3] + ((uint16_t)DATA_RESPONSE[4] << 8);
	/** check[0] - start
		* check[1] - ~start
		* check[2] - check		*/
	uint8_t check = ((DATA_RESPONSE[0] & 0x3) << 1) | (DATA_RESPONSE[1] & 0x1);
	
	/* Rewrite these specific bytes */
	byte_count -= 5;
	
	/* Checking: check=1, ~start=0, start=1 */
	if (check == 0x5 || check == 0x6) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"{\"Q\":%u,\"A\":%u,\"D\":%u}\r\n",
							DATA_RESPONSE[0] >> 2, angle, distance);
	}
	else {
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Invalid response: C=%u, !S=%u, S=%u\r\n",
							(check >> 3), ((check >> 2) & 0x1), (check & 0x1));
	}
		
	USB_send(print_buffer);
}



void LIDAR_RES_express_scan(void) {}



void LIDAR_RES_force_scan(void) {}



/**	Process "GET_INFO" request's response
	*		Byte Offset:	+0		model
	*		Order 8->0		+1		firmware_minor
	*									+2		firware_major
	*									+3		hardware
	*									+4		serial_number[0]
	*														 ...
	*									+19 	serial_number[15]		
	*		NOTE: when converting serial_number to text from hex, the least significant byte 
	*		prints first*/
void LIDAR_RES_get_info(void) 
{
	USB_send("DONE\r\n");
	
	uint8_t model_id 							 = DATA_RESPONSE[0];
	uint8_t firmware_version[2] 	 = { DATA_RESPONSE[1], DATA_RESPONSE[2] };
	uint8_t hardware_version 			 = DATA_RESPONSE[3];
	char serial_number[16] = {0};
	
	/** Get hexadecimal string output */
	int i, j=0;
	for (i=15; i>=0; i--) {
		sprintf(&serial_number[j++], "%02X", DATA_RESPONSE[i+4]);
	}
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : RPLiDAR Model ID: %u\r\n", model_id);
	USB_send(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : Firmware Version: %u.%u\r\n", 
						firmware_version[0], firmware_version[1]);
	USB_send(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : Hardware Version: %u\r\n", hardware_version);
	USB_send(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : Serial Number: 0x%s\r\n", serial_number);
	USB_send(print_buffer);
}



/**	Process "GET_HEALTH" request's response
	*		Byte Offset:	+0		status
	*		Order 8->0		+1		error_code[7:0]
	*									+2		error_code[15:8]	*/
void LIDAR_RES_get_health(void) 
{
	USB_send("DONE\r\n");
	
	char* status;
	uint16_t error_code;  
	
	switch(DATA_RESPONSE[0]) {
		case 0: status = "GOOD"; break;
		case 1: status = "WARNING"; break;
		case 2: status = "ERROR"; break;
		default: status = "UNKNOWN"; break;
	}
	
	error_code = DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8);
	
	LIDAR_reset_print_buffer();
	
	if (error_code == 0)
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							" : LiDAR Health is %s!\r\n", status);
	else
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							" : LiDAR Health is %s!\r\n : Error code: %u\r\n", 
							status, error_code);
	
	USB_send(print_buffer);
}


/**	Process "GET_SAMPLERATE" request's response
	*		Byte Offset:	+0		Tstandard[7:0]
	*		Order 8->0		+1		Tstandard[15:8]
	*									+2		Texpress[7:0]
	*									+3		Texpress[15:8]	*/
void LIDAR_RES_get_samplerate(void)
{
	USB_send("DONE\r\n");
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE, 
						" : Standard Scan Samplerate: %u\r\n", 
						DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8));
	USB_send(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE, 
						" : Express Scan Samplerate: %u\r\n", 
						DATA_RESPONSE[2] + ((unsigned)DATA_RESPONSE[3] << 8));
	USB_send(print_buffer);
}


