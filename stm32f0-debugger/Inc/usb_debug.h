#include "usart.h"

void USB_init(void);
void USB_send(char* message);
int USB_check(void);
void USB_process(void);
void USB_reset_command(void);
void USB_process_message(void);

void USB_COMMAND_help(void);
void USB_COMMAND_servo(int payload);
void USB_COMMAND_lidar(void);
