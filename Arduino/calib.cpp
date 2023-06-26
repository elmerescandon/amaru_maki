#ifndef  F_CPU
	#define F_CPU  16000000UL
#endif

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include "mpu_utils.h"
#include "interface_com.h"



MPU6050* mpu_v;
/******************************
 *  Variables de Calibraci�n
 *****************************/
// Par�metros
uint16_t buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int16_t discardfirstmeas = 100;  // Amount of initial measurements to be discarded
int16_t acel_deadzone = 10;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int16_t giro_deadzone = 5;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t accel_offset_divisor = 8; //8;
int16_t gyro_offset_divisor = 4; //4;

// Variables de medici�n
int16_t ax, ay, az, gx, gy, gz; // Mediciones de sensores
int16_t mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
int16_t ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset; // Offset de medici�n
int16_t ax_initoffset ,ay_initoffset,az_initoffset,gx_initoffset,gy_initoffset,gz_initoffset; // Valores iniciales
int16_t *OFF_IMU_temp;
// Banderas
uint8_t state; // Estado de calibraci�n
uint8_t CalibResult = 0; // false
uint8_t loopcount=0;


void promediarsensores();
uint8_t calibracion();
void begin_calibracion();

void EXEC_CALIB(int16_t *OFFS_IMU_t, MPU6050 *mpu){
  mpu_v = mpu;
	OFF_IMU_temp = OFFS_IMU_t;
	ax_initoffset = OFFS_IMU_t[0];
	ay_initoffset = OFFS_IMU_t[1];
	az_initoffset = OFFS_IMU_t[2];
	gx_initoffset = OFFS_IMU_t[3];
	gy_initoffset = OFFS_IMU_t[4];
	gz_initoffset = OFFS_IMU_t[5];
		
	begin_calibracion();
}

void begin_calibracion(){
	
	if (state==0){
		// Leer los sensores por primera vez
//		print_serial("Calibracion: Primera lectura\n");
		promediarsensores(); // Incluir sensor
//		print_serial("Mediciones medias obtenidas aceleracion (x,y,z)");
//		print_calib(mean_ax,mean_ay,mean_az);
//		print_serial("Mediciones obtenidos giroscopio (x,y,z)");
//		print_calib(mean_gx,mean_gy,mean_gz);
		state++;
	}
	
	if (state==1) {
		// Calcular los offsets
//		print_serial("Ponderacion...\n");
		CalibResult = calibracion();
//		if (CalibResult) {
//			print_serial("Calibracion Terminada: Y\n");
//		}
//		else {
//			print_serial("Calibracion Terminada: N\n");
//		}
		state++;
		delay(200);
	}

	if (state==2) {
		promediarsensores();
//		print_serial("Retornar offsets nuevos ========================== \n");

		print_serial("Offsets obtenidos aceleracion (x,y,z)");
		print_calib(ax_offset + ax_initoffset,ay_offset + ay_initoffset ,az_offset + az_initoffset);
		print_serial("Offsets obtenidos giroscopio (x,y,z)");
		print_calib(gx_offset + gx_initoffset, gy_offset + gy_initoffset, gz_offset + gz_initoffset);;
		OFF_IMU_temp[0] = ax_offset+ax_initoffset;
		OFF_IMU_temp[1] = ay_offset+ay_initoffset;
		OFF_IMU_temp[2] = az_offset+az_initoffset;
		OFF_IMU_temp[3] = gx_offset+gx_initoffset;
		OFF_IMU_temp[4] = gy_offset+gy_initoffset;
		OFF_IMU_temp[5] = gz_offset+gz_initoffset;
		
		state = 0;
		loopcount = 0;
		return; 

	}
}

void promediarsensores(){
	long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
	mean_ax= 0;
	mean_ay = 0;
	mean_az = 0;
	mean_gx = 0;
	mean_gy = 0;
	mean_gz = 0;
	ax = 0;
	ay = 0;
	az = 0;
	gx = 0;
	gy = 0;
	gz = 0;
	

	while (i<(buffersize+discardfirstmeas+1)){
		// read raw accel/gyro measurements from device
		mpu_v -> getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		if (i>discardfirstmeas && i<=(buffersize+discardfirstmeas)){ //First 100 measures are discarded
			buff_ax=buff_ax+ax;
			buff_ay=buff_ay+ay;
			buff_az=buff_az+az;
			buff_gx=buff_gx+gx;
			buff_gy=buff_gy+gy;
			buff_gz=buff_gz+gz;
		}
		if (i==(buffersize+100)){
			mean_ax=buff_ax/buffersize;
			mean_ay=buff_ay/buffersize;
			mean_az=buff_az/buffersize;
			mean_gx=buff_gx/buffersize;
			mean_gy=buff_gy/buffersize;
			mean_gz=buff_gz/buffersize;
		}
		i++;
		delay(2); //Needed so we don't get repeated measures
	}
	
}

uint8_t calibracion(){
	ax_offset=-mean_ax/accel_offset_divisor;
	ay_offset=-mean_ay/accel_offset_divisor;
	az_offset=(16384-mean_az)/accel_offset_divisor;

	gx_offset=-mean_gx/gyro_offset_divisor;
	gy_offset=-mean_gy/gyro_offset_divisor;
	gz_offset=-mean_gz/gyro_offset_divisor;
	
	
	while (1){
		int ready=0;
		
		mpu_v -> setXAccelOffset(ax_offset+ax_initoffset); 
		mpu_v -> setYAccelOffset(ay_offset+ay_initoffset); 
		mpu_v -> setZAccelOffset(az_offset+az_initoffset); 

		mpu_v -> setXGyroOffset(gx_offset+gx_initoffset);
		mpu_v -> setYGyroOffset(gy_offset+gy_initoffset);
		mpu_v -> setZGyroOffset(gz_offset+gz_initoffset);

		promediarsensores();
		
		// Cargando
		if (abs(mean_ax)<=acel_deadzone) ready++;
		else ax_offset=ax_offset-mean_ax/acel_deadzone;

		if (abs(mean_ay)<=acel_deadzone) ready++;
		else ay_offset=ay_offset-mean_ay/acel_deadzone;

		if (abs(16384-mean_az)<=acel_deadzone) ready++;
		else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

		if (abs(mean_gx)<=giro_deadzone) ready++;
		else gx_offset=gx_offset-mean_gx/(giro_deadzone + 1 );

		if (abs(mean_gy)<=giro_deadzone) ready++;
		else gy_offset=gy_offset-mean_gy/(giro_deadzone + 1);

		if (abs(mean_gz)<=giro_deadzone) ready++;
		else gz_offset= gz_offset-mean_gz/(giro_deadzone + 1);

		if (ready==6) {
			return 1;
			break;
		}


//		print_serial("=======================================================\n");
//		print_serial("Mediciones medias obtenidas aceleracion (x,y,z)");
//		print_calib(mean_ax,mean_ay,mean_az);
//		print_serial("Offsets obtenidos aceleracion (x,y,z)");
//		print_calib(ax_offset + ax_initoffset,ay_offset + ay_initoffset ,az_offset + az_initoffset);
		
//		print_serial("Mediciones obtenidos giroscopio (x,y,z)");
//		print_calib(mean_gx,mean_gy,mean_gz);
//		print_serial("Offsets obtenidos giroscopio (x,y,z)");
//		print_calib(gx_offset + gx_initoffset, gy_offset + gy_initoffset, gz_offset + gz_initoffset);
		print_serial("Cuenta:");
		print_ticks(loopcount);

		loopcount=loopcount+1;
		
		if (loopcount==5) {
			return 0;
			break; // exit the calibration routine if no stable results can be obtained after 20 calibration loops
		}
	}
}
