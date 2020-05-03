#include "usart.h"
#include "commands.h"

struct usb_message {
	char cmd[32];
	char val[32];
	int sel;
	int space_idx;
};

void USB_init(void);
void USB_send(char* message);
int USB_check(void);
void USB_process(int check_code);
void USB_reset_command(void);
void USB_process_message(void);
