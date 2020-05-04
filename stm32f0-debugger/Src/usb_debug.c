/**
	*	USB handling from a UART communicating with a serial connection for 
	* debugging specific components. The messages that are received from console
	* are formatted as <command> (optional)<payload>. Minimal error correcting, 
	* however commands and payloads are checked for what to do. Processing to
	* determine sent commands and payloads from messages are determined in 
	* @see USB_process which should be called in the infinite loop in main.
	*	
	* @file  : usb_debug.c
	* @author: Kris Wolff
	*/
	
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usb_debug.h"
#include "servo.h"
#include "lidar.h"



#define USB_BAUD_RATE 115200

#define MAX_PRINT_BUFFER_SIZE		256
#define BUFFER_SIZE 32

#define COMMAND_COUNT 5
#define LIDAR_PAYLOAD_COUNT 10



/* Structure of incoming messages */
struct usb_message {
	char cmd[BUFFER_SIZE];
	char payload[BUFFER_SIZE];
	int sel;
	int space_idx;
} command;

/* Available commands to be processed */
static char* COMMAND_LIST[COMMAND_COUNT] = {
	"help", "servo", "lidar", "stop", "reset"
};

/* Available payloads for a "lidar" command */
static char* LIDAR_LIST[LIDAR_PAYLOAD_COUNT] = {
	"pwm_start", "pwm_stop",
	"stop", "reset", 
	"scan", "express_scan", "force_scan",
	"get_info", "get_health", "get_samplerate"
};

/* Buffer for printing */
static char print_buffer[MAX_PRINT_BUFFER_SIZE];

/* Populted from USART1_IRQHandler, checked in USB_check */
extern volatile char usb_received_value;
extern volatile int usb_newData_flag;

/* Size of buffer for bytes reeived processing USB messages */
static int buffer_current_size;



/** 
	* Initialize USB connection through USART1 on GPIO pins PB6 (TX) and PB7 (RX)
	* @see USART1_init	
	* @see USART1_IRQHandler
	* @return None
	*/
void USB_init(void)
{
	USART1_init(USB_BAUD_RATE);
}



/**
	* Sends message to USB through USART connection
	* @param char*: string to send
	* @see USART1_transmit_string
	* @return None
	*/
void USB_send(char* message)
{
	USART1_transmit_string(message);
}



/** 
	*	Resets local print buffer.
	*	@return None
	*/
void USB_reset_print_buffer(void)
{
	int i;
	for (i=0; i<MAX_PRINT_BUFFER_SIZE; i++)
		print_buffer[i] = '\0';
}



/**
	*	Resets command for new messages sent
	* @return None
	*/
void USB_reset_command(void) 
{
	int i;
	for (i=0; i<BUFFER_SIZE; i++) {
		command.cmd[i] = '\0';
		command.payload[i] = '\0';
	}
	command.sel = 0;
	command.space_idx = 0;
	buffer_current_size = 0;
}



/** 
	* Checks flags of any incoming bytes from USB
	*	@return int: code based on flags
	* 		-2 : incorrect format
	*	 		-1 : buffer full
	*		 	 0 : no new data
	*		 	 1 : sent back character and added to buffer (do nothing)
	*		 	 2 : ready to process!
	* 	 	 3 : sending debug info (do nothing)
	*/
int USB_check(void)
{
	/* No new data */
	if (!usb_newData_flag) 
		return 0;
	
	usb_newData_flag = 0;
	
	/** Weird fix for @bug where there is a newline sent at the beginning of a 
			request for a new CMD */
	if (usb_received_value == '\n' && buffer_current_size == 0)
		return 0;
	
	/* Process if newline sent */
	if (usb_received_value == 0x0D) {
		USB_send("\r\n");
		return 2;
	}
	
	/* Buffer full, cannot fill more */
	if (buffer_current_size == (BUFFER_SIZE*2)) 
		return -1;
	
	/* Select if writing to command or writing to payload */
	if (buffer_current_size == 0) {
		command.sel = 0;
		command.space_idx = 0;
	} else if (usb_received_value == ' ') {
		if (command.space_idx > 0)
			return -2;
		command.sel = 1;
		command.space_idx = buffer_current_size;
	}
	
	/* Write to command or payload */
	if (!command.sel)
		command.cmd[buffer_current_size-command.space_idx] = usb_received_value;
	else {
		command.payload[buffer_current_size-command.space_idx-1] = usb_received_value;
	}
	
	buffer_current_size++;

	/* Send back character for acknowledgement */
	USB_reset_print_buffer();
	sprintf(print_buffer, "%c", usb_received_value);
	USB_send(print_buffer);

	return 1;
}



/**	
	* Processes code returned from @see USB_check and will either print specific
	* error message or will send finished message to be processed.
	* @return None
	*/
void USB_process(void)
{
	int code = USB_check();
	switch(code) {
		case -2 :
			USB_send("\r\nUse format: \"<cmd> (optional)<payload>\"\r\n");
			USB_reset_command();
			USB_send("CMD?");
			usb_received_value = '\0';
			break;
		
		case -1 :
			USB_send("\r\nToo many characters! Try again!\r\n");
			USB_reset_command();
			USB_send("CMD?");
			usb_received_value = '\0';
			break;
		
		case 0 : 
			/* Do nothing. No new data. */
			break;
		
		case 1 : 
			/* Do nothing. Added new character to command input */
			break;
		
		case 2 : 
			USB_reset_print_buffer();
			snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
							 "CMD: [\"%s\"], [\"%s\"]\r\n", command.cmd, command.payload);
			USB_send(print_buffer);
		
			USB_process_message();
		
			usb_received_value = '\0';
			break;
	}
}




/**
	*	Process full message from USB to determine command to use
	* @return None
	*/
void USB_process_message(void)
{
	USB_reset_print_buffer();
	
	int i, steps = 0, value = 0, length = 0;
	char cmd[50]; 
	
	while (command.payload[++length] != '\0');
	
	for (i=length-1; i >= 0; i--) {
		value += (command.payload[i] - '0') * pow(10, steps++);
	}
	
	/* Set command.cmd to lowercase */
	for (i=0; command.cmd[i]; i++)
		command.cmd[i] = tolower(command.cmd[i]);
	
	if (!strcmp(command.cmd, COMMAND_LIST[0])) {
		USB_COMMAND_help();
	}
	else if (!strcmp(command.cmd, COMMAND_LIST[1])) {
		USB_COMMAND_servo(value);
	}
	else if (!strcmp(command.cmd, COMMAND_LIST[2])) {
		USB_COMMAND_lidar();
	}
	else if (!strcmp(command.cmd, COMMAND_LIST[3])) {
		LIDAR_REQ_stop();
	}
	else if (!strcmp(command.cmd, COMMAND_LIST[3])) {
		LIDAR_REQ_reset();
	}
	else {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"%s%s", 
							"Invalid command! Use format <cmd> <payload>:\r\n",
							"<cmd> options:\r\n");
		for (i=0; i<COMMAND_COUNT; i++) {
			snprintf(cmd, 50, " : %s\r\n", COMMAND_LIST[i]);
			strncat(print_buffer, cmd, MAX_PRINT_BUFFER_SIZE);
		}
		strncat(print_buffer, "CMD?", MAX_PRINT_BUFFER_SIZE);
		USB_send(print_buffer);
		USB_reset_command();
	}
}



/**
	*	Sends output for "help" command
	*	@return None
	*/
void USB_COMMAND_help(void) 
{
	USB_reset_print_buffer();
	
	int i;
	const unsigned BUF_SIZE = 64;
	char cmd[BUF_SIZE]; 
	
	snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
						"%s%s", 
						"Format: <cmd> <payload>:\r\n",
						"<cmd> options:\r\n");
	
	for (i=0; i<COMMAND_COUNT; i++) {
		snprintf(cmd, BUF_SIZE, " : %s\r\n", COMMAND_LIST[i]);
		strncat(print_buffer, cmd, MAX_PRINT_BUFFER_SIZE);
	}
	
	strncat(print_buffer, "CMD?", MAX_PRINT_BUFFER_SIZE);
	USB_send(print_buffer);
	USB_reset_command();
}



/**
	*	Processes command to adjust servo angle and checks if angle between 0 and 180 
	* @param int: servo angle
	* @return None
	*/
void USB_COMMAND_servo(int payload) 
{
	USB_reset_print_buffer();
	
	if (payload < 0 || payload > 180)
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Invalid value! Value must be between 0 and 180.\r\n");
	else {
		SERVO_set_angle(payload);
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Setting servo angle to %u-degree(s)...DONE\r\n", payload);
	}
	
	strncat(print_buffer, "CMD?", MAX_PRINT_BUFFER_SIZE);
	USB_send(print_buffer);
	USB_reset_command();
}



/**
	* Process command for lidar and calls corresponding LiDAR request function
	* @return None
	* @bug if you send too long of a payload (~21), then nothing is getting 
	* written to command.cmd. Tested by sending "lidar get_samplerates" from 
	* console. Afterwards, wasn't able to input any other commmand until I 
	* I typed enter a couple times which seemed to clear it.
	*/
void USB_COMMAND_lidar(void)
{
	USB_reset_print_buffer();
	
	int i;
	char cmd_payload[BUFFER_SIZE];
	
	/* Set payload to lowercase */
	for (i=0; command.payload[i]; i++)
		command.payload[i] = tolower(command.payload[i]);
	
	if (!strcmp(command.payload, LIDAR_LIST[0])) {
		USB_send("Starting LiDAR motor...\r\n");
		LIDAR_PWM_start();
		USB_send("CMD?");
		USB_reset_command();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[1])) {
		USB_send("Stopping LiDAR motor...\r\n");
		LIDAR_PWM_stop();
		USB_send("CMD?");
		USB_reset_command();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[2])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Stopping LiDAR...");
		USB_send(print_buffer);
		LIDAR_REQ_stop();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[3])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Resetting LiDAR...");
		USB_send(print_buffer);
		LIDAR_REQ_reset();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[4])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Requesting LiDAR scan...\r\n");
		USB_send(print_buffer);
		LIDAR_REQ_scan();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[5])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Requesting LiDAR express scan...\r\n");
		USB_send(print_buffer);
		LIDAR_REQ_express_scan();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[6])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Requesting LiDAR force scan...\r\n");
		USB_send(print_buffer);
		LIDAR_REQ_force_scan();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[7])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Retrieving LiDAR information...");
		USB_send(print_buffer);
		LIDAR_REQ_get_info();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[8])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Retrieving LiDAR health...");
		USB_send(print_buffer);
		LIDAR_REQ_get_health();
	}
	else if (!strcmp(command.payload, LIDAR_LIST[9])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Retrieving LiDAR samplerates...");
		USB_send(print_buffer);
		LIDAR_REQ_get_samplerate();
	}
	else {
		USB_reset_command();
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"%s%s", 
							"Invalid LiDAR request! Use format \"lidar <payload>\":\r\n",
							"<payload> options:\r\n");
		for (i=0; i<LIDAR_PAYLOAD_COUNT; i++) {
			snprintf(cmd_payload, BUFFER_SIZE, " : %s\r\n", LIDAR_LIST[i]);
			strncat(print_buffer, cmd_payload, MAX_PRINT_BUFFER_SIZE);
		}
		strncat(print_buffer, "CMD?", MAX_PRINT_BUFFER_SIZE);
		USB_send(print_buffer);
	}
}
