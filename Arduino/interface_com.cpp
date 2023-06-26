/*****************************
 *  Funciones de UART 
 ****************************/
#ifndef  F_CPU
#define F_CPU  16000000UL
#endif

#define TRUE 0x01
#define FALSE 0x00
#include <stdio.h>
#include <string.h>

#ifndef AVR_IO
	#include <avr/io.h>
#endif 

#include "interface_com.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#ifndef MODOS
	#define MODOS
	#define INIT 0x00
	#define CALIB 0x01
	#define ENVIO 0x02
#endif


/**********************************
 *  Funciones de Configuraci�n LED
 *********************************/

unsigned long ticks;
uint16_t tiempo_led = 0;
//
//void //INIT_STATUS_LED(){
//	TCCR1A = 0b00000000;
//	TCCR1B = 0b00001011;
//	OCR1A = 248;
//	TIMSK1 = 0x02;
//}

/**********************************
 *  Funciones de Comunicaci�n UART
 *********************************/
uint8_t FLAG_ENVIO_ON;
uint8_t FLAG_ENVIO_INIT;
uint8_t FLAG_CALIB_ON;

void print_serial(char *s);

void INICIAR_UART(void){
  UCSR0B |= (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
  UCSR0B &= (~1<< UCSZ02) | (~1<< RXB80) | (~1<< TXB80);
  
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00); // 8 - bits
  UCSR0C &= (~1<< UMSEL01) | (~1<< UMSEL00) | (~1<< UPM01) | (~1<< UPM00) | (~1<<USBS0) | (~1<<UCPOL0) ;
  
  UBRR0L = 8; // 25-38400 //  8 - 115200 ///103 - 9600  // 51 - 19200  / No parity / 1-bit Stop 
  UBRR0H = 0;
}

void SEND_DATA(uint8_t data,uint8_t pooling){
	
	if (pooling){
		while (!(UCSR0A & (1<<UDRE0)));
	}
	else{
		if (UCSR0A &  (1<<UDRE0));
	}
	UDR0 = data;

}

void PRINT_VAL(char *BUFF, uint8_t BUFF_CNT){ 
	uint8_t count = 0;
	do{
		SEND_DATA(BUFF[count],1);
		count++;
		
	}while(BUFF[count] !='\0' || count >= BUFF_CNT);
}

/*****************************
 *  Funciones de interfaz 
 ****************************/
void ANALIZAR_TRAMA(uint8_t *MODO, char *BUFF, uint8_t BUFF_CNT){
  if (*MODO == INIT && BUFF_CNT == 3){ // Solo hay 3 caracteres: Inicio, Modo, Final
	if (BUFF[1] == '2'){
		FLAG_ENVIO_INIT = FALSE;
		*MODO = ENVIO; // Modo Env�o
		tiempo_led = 2;
		////INIT_STATUS_LED();
		print_serial("Modo ENVIO\n");
	}
	else if (BUFF[1] == '1'){
		*MODO = CALIB; // Modo Calibraci�n 
		tiempo_led = 1;
		//////INIT_STATUS_LED();
		print_serial("Modo CALIB\n"); 
		FLAG_CALIB_ON = FALSE;
	}
	else if (BUFF[1] == '0'){
		*MODO = INIT; // Modo Normal 
		tiempo_led = 0;
		PORTB |= 0x01;
		////INIT_STATUS_LED();
		print_serial("Modo INIT\n"); 
	}

  }
  else if (*MODO == ENVIO ){
	  // Cambio de modos
	  	if (BUFF[1] == '1'){
			*MODO = CALIB; // Modo Calibraci�n
			tiempo_led = 1;
			////INIT_STATUS_LED();
			print_serial("Modo CALIB\n");
			FLAG_ENVIO_ON = FALSE;	
			FLAG_CALIB_ON = FALSE;
	  	}
		else if (BUFF[1] == '0'){
			*MODO = INIT; // Modo Normal
			tiempo_led = 0;
			PORTB |= 0x01;
			////INIT_STATUS_LED();
			print_serial("Modo INIT\n");

			FLAG_ENVIO_ON = FALSE;	
		}
	  
	  // Sub-modos
	  if (BUFF_CNT == 4 && BUFF[1] == '2'){
		  if (BUFF[2] == '0'){
			FLAG_ENVIO_ON = FALSE;	
			print_serial("ENVIO:PAUSA\n");
		  }
		  else if (BUFF[2] == '1'){
			FLAG_ENVIO_ON = TRUE;
			print_serial("ENVIO:ACITVAR\n");
		  }
		  else if (BUFF[2] == '2'){
			print_serial("ENVIO:\n");
			FLAG_ENVIO_ON = 2;
		  }
	  }  
  }
  
  else if(*MODO == CALIB){
	if (BUFF[1] == '0'){
		tiempo_led = 0;
		PORTB |= 0x01;
		*MODO = INIT; // Modo Normal
		////INIT_STATUS_LED();
		print_serial("Modo INIT\n");
	}
	else if (BUFF[1] == '2'){
		FLAG_ENVIO_INIT = FALSE;
		*MODO = ENVIO; // Modo Env�o
		tiempo_led = 2;
		////INIT_STATUS_LED();
		print_serial("Modo ENVIO\n");
	}
	
	if (BUFF_CNT == 4 && BUFF[1] == '1'){
		if (BUFF[2] == '0'){
			FLAG_CALIB_ON = FALSE;
			print_serial("CALIBRACION:OFF\n");
		}
		else if (BUFF[2] == '1'){
			FLAG_CALIB_ON = TRUE;
			print_serial("CALIB:ON\n");
		}
		else if (BUFF[2] == '2'){
			FLAG_CALIB_ON = 2;
			print_serial("CALIB:RAW\n");
		}
	}
  
  }
}

void print_serial(char *s){
	char buf_data[100] = "";
	sprintf(buf_data,s);
	PRINT_VAL(buf_data,100);
}

void print_dmp(float qw, float qx, float qy, float qz, float roll, float pitch, float yaw){
	char buf_data[100] = "";
  
  char qw_s[8];
  dtostrf(qw  , 7, 2, qw_s);
  char qx_s[8];
  dtostrf(qx  , 7, 2, qx_s);
  char qy_s[8];
  dtostrf(qy  , 7, 2, qy_s);
  char qz_s[8];
  dtostrf(qz  , 7, 2, qz_s);
  
  char roll_s[8];
  dtostrf(roll  , 7, 2, roll_s);
  char pitch_s[8];
  dtostrf(pitch  , 7, 2, pitch_s);
  char yaw_s[8];
  dtostrf(yaw  , 7, 2, yaw_s);
  strcpy(buf_data,qw_s);
  strcat(buf_data," ");
  strcat(buf_data,qx_s );
  strcat(buf_data," ");
  strcat(buf_data,qy_s );
  strcat(buf_data," ");
  strcat(buf_data,qz_s );
  strcat(buf_data," ");
  strcat(buf_data,roll_s );
  strcat(buf_data," ");
  strcat(buf_data,pitch_s );
  strcat(buf_data," ");
  strcat(buf_data,yaw_s );
  strcat(buf_data,"\n");
  
	// sprintf(buf_data, "q: %5.3f x:%5.3f y:%5.3f z:%5.3f / r:%5.2f p:%5.2f y:%5.2f \n ",qw, qx, qy, qz, roll, pitch, yaw);
	PRINT_VAL(buf_data,100);
}

void print_raw(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz){
	char buf_data[100] = "";
	sprintf(buf_data, "ax:%d ay:%d az:%d gx:%d gy:%d gz:%d\n",ax, ay, az, gx, gy, gz);
	PRINT_VAL(buf_data,100);
}

void print_calib(int16_t x, int16_t y, int16_t z){
	char buf_data[100] = "";
	sprintf(buf_data, "x:%d y:%d z:%d \n",x, y, z);
	PRINT_VAL(buf_data,100);
}

void print_ticks(unsigned long num){
	char buf_data[100] = "";
	sprintf(buf_data,"%lu\n",num);
	PRINT_VAL(buf_data,100);
}

void print_uint8(uint8_t num){
	char buf_data[100] = "";
	sprintf(buf_data,"%u\n",num);
	PRINT_VAL(buf_data,100);
}

void print_uint16(uint16_t num){
	char buf_data[100] = "";
	sprintf(buf_data,"%u\n",num);
	PRINT_VAL(buf_data,100);
}

void print_uint32(uint32_t num){
	char buf_data[100] = "";
	sprintf(buf_data,"%lu \n",num);
	PRINT_VAL(buf_data,100);
}

void print_double(double num){
  char buf_data[100] = "";
  char num_s[8];
  dtostrf((float) num  , 7, 2, num_s);
  strcat(buf_data,num_s);
  PRINT_VAL(buf_data,100);
}

void print_float(float num){
  char buf_data[100] = "";
  char num_s[8];
  dtostrf((float) num  , 7, 2, num_s);
  strcat(buf_data,num_s);
  PRINT_VAL(buf_data,100);
}


void print2comp(float qw, float qx, float qy,float qz, float qw2, float qx2, float qy2,float qz2, float qw3, float qx3, float qy3,float qz3){
  char buf_data[100] = "";
  char data_num[8];

  // Añadir del primer IMU (w,x,y,z)
  dtostrf(qw, 5, 2, data_num);
  strcpy(buf_data,data_num);
  strcat(buf_data,",");
  
  dtostrf(qx, 5, 2, data_num);
  strcat(buf_data,data_num);
  strcat(buf_data,",");
  
  dtostrf(qy, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");

  dtostrf(qz, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");

  // Añadir del primer IMU 2 (w,x,y,z)
  dtostrf(qw2, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
  dtostrf(qx2, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
  dtostrf(qy2 , 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
  dtostrf(qz2, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");

  // Añadir del primer IMU 3(w,x,y,z)
  dtostrf(qw3, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
  dtostrf(qx3 , 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");

  dtostrf(qy3 , 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");

  dtostrf(qz3 , 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
	PRINT_VAL(buf_data,100);
}

void print2RPY(float roll, float pitch, float roll2, float pitch2, float roll3, float pitch3){
  char buf_data[100] = "";
  char data_num[8];

  // Añadir del primer IMU 1 (Roll and Pitch)
  dtostrf(roll, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
  dtostrf(pitch , 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");

  // Añadir del primer IMU 2 (Roll and Pitch)
  dtostrf(roll2, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
  dtostrf(pitch2 , 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");

  // Añadir del primer IMU 3 (Roll and Pitch)
  dtostrf(roll3, 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,",");
  
  dtostrf(pitch3 , 5, 2, data_num);
  strcat(buf_data,data_num );
  strcat(buf_data,"\n");

	PRINT_VAL(buf_data,100);
}
