#ifndef INTERFACE_COM_
#define INTERFACE_COM_

#include <stdint.h>

extern uint8_t FLAG_ENVIO_ON;
extern uint8_t FLAG_CALIB_ON;

extern uint8_t FLAG_ENVIO_INIT;


//void INIT_STATUS_LED();
void INICIAR_UART(void);
void SEND_DATA(uint8_t data,uint8_t pooling);
void PRINT_VAL(char *BUFF, uint8_t BUFF_CNT);
void ANALIZAR_TRAMA (uint8_t *MODO, char *BUFF, uint8_t BUFF_CNT);
void print_serial(char *s);
void print_dmp(float qw, float qx, float qy, float qz, float roll, float pitch, float yaw);
void print_raw(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
void print_calib(int16_t x, int16_t y, int16_t z);
void print_uint8(uint8_t num);
void print_uint16(uint16_t num);
void print_double(double num);
void print_float(float num);
void print2comp(float qw, float qx, float qy,float qz, float qw2, float qx2, float qy2,float qz2, float qw3, float qx3, float qy3,float qz3);
void print2RPY(float roll, float pitch, float roll2, float pitch2, float roll3, float pitch3);
void print_uint32(uint32_t num);
void print_ticks(unsigned long num);
#endif /* INTERFACE_COM */
