#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usb_debug.h"


#define MAX_PRINT_BUFFER_SIZE		64
static char print_buffer[MAX_PRINT_BUFFER_SIZE];

#define USB_BAUD_RATE 115200
#define buffer_size 64

int buffer_current_size = 0;
struct usb_message command;


extern volatile char usb_received_value;
extern volatile int usb_newData_flag;
extern const char* COMMAND_LIST[];



void USB_init(void)
{
	// PA0 (TX) and PA1 (RX) for USART4 connection
	USART1_init(USB_BAUD_RATE);
	
	/** Send initial transmission to begin receiving */
	USB_send("CMD?");
}



void USB_send(char* message)
{
	USART1_transmit_string(message);
}


void USB_reset_print_buffer(void)
{
	int i;
	for (i=0; i<MAX_PRINT_BUFFER_SIZE; i++)
		print_buffer[i] = '\0';
}



int USB_check(void)
{
	// no new data
	if (!usb_newData_flag) 
		return 0;
	
	usb_newData_flag = 0;
	
	if (usb_received_value == '\n' && buffer_current_size == 0)
		return 0;
	
	// process if newline sent
	if (usb_received_value == 0x0D) {
		USB_send("\r\n");
		return 2;
	}
	
	// buffer full, cannot fill more
	if (buffer_current_size == 64) 
		return -1;
	
	// select buffer
	if (buffer_current_size == 0) {
		command.sel = 0;
		command.space_idx = 0;
	} else if (usb_received_value == ' ') {
		if (command.space_idx > 0)
			return -2;
		command.sel = 1;
		command.space_idx = buffer_current_size;
	}
	
	// write to first or second buffer
	if (!command.sel)
		command.cmd[buffer_current_size-command.space_idx] = usb_received_value;
	else {
		command.val[buffer_current_size-command.space_idx-1] = usb_received_value;
	}
	
	buffer_current_size++;

	// send back character for acknowledgement
	USB_reset_print_buffer();
	sprintf(print_buffer, "%c", usb_received_value);
	USB_send(print_buffer);

	return 1;
}



/**		-2 : incorrect format
	* 	-1 : buffer full
	*		 0 : no new data
	*		 1 : sent back character and added to buffer (do nothing)
	*		 2 : ready to process!
	* 	 3 : sending debug info (do nothing)
	*/
void USB_process(int check_code)
{
	switch(check_code) {
		case -2 :
			USB_send("\r\nUse format: \"<cmd> (optional)<value>\"\r\n");
			USB_reset_command();
			// Wait for new command
			USB_send("CMD?");
			usb_received_value = '\0';
			break;
		
		case -1 :
			USB_send("\r\nToo many characters! Try again!\r\n");
			USB_reset_command();
			// Wait for new command
			USB_send("CMD?");
			usb_received_value = '\0';
			break;
		
		case 0 : 
			// Do nothing. No new data. 
			break;
		
		case 1 : 
			// Do nothing. Added new character to command input
			break;
		
		case 2 : 
			// View command message
			USB_reset_print_buffer();
			snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
							 "CMD: [\"%s\"], [\"%s\"]\r\n", command.cmd, command.val);
			USB_send(print_buffer);
		
			USB_process_message();
		
			usb_received_value = '\0';
			break;
	}
}



void USB_reset_command(void) 
{
	int i;
	for (i=0; i<32; i++) {
		command.cmd[i] = '\0';
		command.val[i] = '\0';
	}
	command.sel = 0;
	command.space_idx = 0;
	buffer_current_size = 0;
}



void USB_process_message(void)
{
	USB_reset_print_buffer();
	
	int i, steps = 0, value = 0, length = 0;
	char cmd[50]; 
	
	while (command.val[++length] != '\0');
	
	for (i=length-1; i >= 0; i--) {
		value += (command.val[i] - '0') * pow(10, steps++);
	}
	
	/** Set command.cmd to lowercase */
	for (i=0; command.cmd[i]; i++)
		command.cmd[i] = tolower(command.cmd[i]);
	
	if (!strcmp(command.cmd, COMMAND_LIST[0])) {
		COMMAND_help();
		return;
	}
	else if (!strcmp(command.cmd, COMMAND_LIST[1])) {
		COMMAND_servo(value);
		return;
	}
	else if (!strcmp(command.cmd, COMMAND_LIST[2])) {
		COMMAND_lidar(command.val);
		return;
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
