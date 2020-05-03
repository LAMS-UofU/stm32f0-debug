#include "main.h"
#include "usart.h"

void LIDAR_init(void);
void LIDAR_process(void);

void LIDAR_reset_print_buffer(void);
void LIDAR_reset_response_descriptor(void);

void LIDAR_PWM_init(void);
void LIDAR_PWM_start(void);
void LIDAR_PWM_stop(void);

void LIDAR_REQ_stop(void);
void LIDAR_REQ_reset(void);
void LIDAR_REQ_scan(void);
void LIDAR_REQ_express_scan(void);
void LIDAR_REQ_force_scan(void);
void LIDAR_REQ_get_info(void);
void LIDAR_REQ_get_health(void);
void LIDAR_REQ_get_samplerate(void);

void LIDAR_RES_stop(void);
void LIDAR_RES_reset(void);
void LIDAR_RES_scan(void);
void LIDAR_RES_express_scan(void);
void LIDAR_RES_force_scan(void);
void LIDAR_RES_get_info(void);
void LIDAR_RES_get_health(void);
void LIDAR_RES_get_samplerate(void);
