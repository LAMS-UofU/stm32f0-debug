#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "commands.h"
#include "usb_debug.h"
#include "servo.h"
#include "lidar.h"



#define MAX_PRINT_BUFFER_SIZE		256
static char print_buffer[MAX_PRINT_BUFFER_SIZE];

const char* COMMAND_LIST[COMMAND_COUNT] = {
	"help", "servo", "lidar"
};

const char* LIDAR_LIST[LIDAR_PAYLOAD_COUNT] = {
	"pwm_start", "pwm_stop",
	"stop", "reset", 
	"scan", "express_scan", "force_scan",
	"get_info", "get_health", "get_samplerate"
};



void COMMAND_reset_print_buffer(void)
{
	int i;
	for (i=0; i<MAX_PRINT_BUFFER_SIZE; i++)
		print_buffer[i] = '\0';
}



void COMMAND_servo(int value) 
{
	COMMAND_reset_print_buffer();
	
	if (value < 0 || value > 180)
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Invalid value! Value must be between 0 and 180.\r\n");
	else {
		SERVO_set_angle(value);
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Setting servo angle to %u-degree(s)...DONE\r\n", value);
	}
	
	strncat(print_buffer, "CMD?", MAX_PRINT_BUFFER_SIZE);
	USB_send(print_buffer);
}



void COMMAND_lidar(char* value)
{
	COMMAND_reset_print_buffer();
	
	int i;
	char payload[20];
	
	/** Set value to lowercase */
	for (i=0; value[i]; i++)
		value[i] = tolower(value[i]);
	
	if (!strcmp(value, LIDAR_LIST[0])) {
		USB_send("Starting LiDAR motor...\r\n");
		LIDAR_PWM_start();
		USB_send("CMD?");
		USB_reset_command();
	}
	else if (!strcmp(value, LIDAR_LIST[1])) {
		USB_send("Stopping LiDAR motor...\r\n");
		LIDAR_PWM_stop();
		USB_send("CMD?");
		USB_reset_command();
	}
	else if (!strcmp(value, LIDAR_LIST[2])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Stopping LiDAR...");
		USB_send(print_buffer);
		LIDAR_REQ_stop();
	}
	else if (!strcmp(value, LIDAR_LIST[3])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Resetting LiDAR...");
		USB_send(print_buffer);
		LIDAR_REQ_reset();
	}
	else if (!strcmp(value, LIDAR_LIST[4])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Requesting LiDAR scan...\r\n");
		USB_send(print_buffer);
		LIDAR_REQ_scan();
	}
	else if (!strcmp(value, LIDAR_LIST[5])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Requesting LiDAR express scan...\r\n");
		USB_send(print_buffer);
		LIDAR_REQ_express_scan();
	}
	else if (!strcmp(value, LIDAR_LIST[6])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Requesting LiDAR force scan...\r\n");
		USB_send(print_buffer);
		LIDAR_REQ_force_scan();
	}
	else if (!strcmp(value, LIDAR_LIST[7])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Retrieving LiDAR information...");
		USB_send(print_buffer);
		LIDAR_REQ_get_info();
	}
	else if (!strcmp(value, LIDAR_LIST[8])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Retrieving LiDAR health...");
		USB_send(print_buffer);
		LIDAR_REQ_get_health();
	}
	else if (!strcmp(value, LIDAR_LIST[9])) {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Retrieving LiDAR samplerates...");
		USB_send(print_buffer);
		LIDAR_REQ_get_samplerate();
	}
	else {
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"%s%s", 
							"Invalid LiDAR request! Use format \"lidar <payload>\":\r\n",
							"<payload> options:\r\n");
		for (i=0; i<LIDAR_PAYLOAD_COUNT; i++) {
			snprintf(payload, 20, " : %s\r\n", LIDAR_LIST[i]);
			strncat(print_buffer, payload, MAX_PRINT_BUFFER_SIZE);
		}
		strncat(print_buffer, "CMD?", MAX_PRINT_BUFFER_SIZE);
		USB_send(print_buffer);
	}
}


void COMMAND_help(void) 
{
	COMMAND_reset_print_buffer();
	
	int i;
	char cmd[50]; 
	
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
}
