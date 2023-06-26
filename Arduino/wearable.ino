
/********************************
   Definiciones y Librerías
 ********************************/
#include "one_euro.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "interface_com.h"
#include "mpu_utils.h"
#include "calib.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "Kalman.h" 

#define TRUE 0x01
#define FALSE 0x00
#define MAX_BUF_SIZE 50

#ifndef MODOS
#define MODOS
#define INIT 0x00
#define CALIB 0x01
#define ENVIO 0x02
#endif
// Definición de pines
#define INTERRUPT_PIN 2


/*************************
   Variables IMUs
 *************************/
// Parametros IMU1
int16_t OFFS_IMU1[6] = { -2109, -1094, 1534, -576, 63, -10}; //{-2396, -2952, 3229, -547, 36, -26};
int16_t OFFS_IMU2[6] = { -3477, 1026, 1565, 79, 41, 17}; //{-3687, -805, 3110, 33, 26, 5};
int16_t OFFS_IMU3[6] = { -5122, 1339, 381, 51, 36, 15}; //{-4888,-363, 2298, 62, 21, 8};

uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[28]; // FIFO storage buffer

/* IMU Data */
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 acc;
VectorInt16 gyro;

/*****************/ 
/*    IMU 1      */ 
/*****************/ 

// Create the Kalman instances
Kalman kalmanX; 
Kalman kalmanY;

// Vectores de Filtro 1E
float quat[4] = {0};
float quat_filt[4] = {0};
float quat_prev[4] = {0};
float dquat_prev[4] = {0};

// Vector de Filtro de Kalman
float RPYK[2] = {0};
float RPYK_filt[2] = {0};
float RPYK_prev[2] = {0};
float dRPYK_prev[2] = {0};
float roll = 0.0, pre_roll = 0.0;
float pitch = 0.0;

// Variables del sensor 
float acc_rate[3] = {0};
float gyro_rate[3] = {0};
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

// Variables de t_detal
float time_delta;


/*****************/ 
/*    IMU 2      */ 
/*****************/ 

// // Create the Kalman instances
Kalman kalmanX2; 
Kalman kalmanY2;

// Vectores de Filtro 1E
float quat_2[4] = {0};
float quat_filt_2[4] = {0};
float quat_prev_2[4] = {0};
float dquat_prev_2[4] = {0};

// Vector de Filtro de Kalman
float RPYK_2[2] = {0};
float RPYK_filt_2[2] = {0};
float RPYK_prev_2[2] = {0};
float dRPYK_prev_2[2] = {0};

// Variables del sensor 
float acc_rate_2[3] = {0};
float gyro_rate_2[3] = {0};
float kalAngleX_2, kalAngleY_2; // Calculated angle using a Kalman filter

/*****************/ 
/*    IMU 3      */ 
/*****************/ 

// Create the Kalman instances
Kalman kalmanX3; 
Kalman kalmanY3;

// Vectores de Filtro 1E
float quat_3[4] = {0};
float quat_filt_3[4] = {0};
float quat_prev_3[4] = {0};
float dquat_prev_3[4] = {0};

// Vector de Filtro de Kalman
float RPYK_3[2] = {0};
float RPYK_filt_3[2] = {0};
float RPYK_prev_3[2] = {0};
float dRPYK_prev_3[2] = {0};

// Variables del sensor 
float acc_rate_3[3] = {0};
float gyro_rate_3[3] = {0};
float kalAngleX_3, kalAngleY_3; // Calculated angle using a Kalman filter

Kalman* dir_kalman[6] = {&kalmanX, &kalmanY, &kalmanX2, &kalmanY2, &kalmanX3, &kalmanY3};
float* angle_kalman[6] = {&kalAngleX, &kalAngleY, &kalAngleX_2, &kalAngleY_2, &kalAngleX_3, &kalAngleY_3};
float* acc_vector[3] = {acc_rate, acc_rate_2, acc_rate_3};
float* gyro_vector[3] = {gyro_rate, gyro_rate_2, gyro_rate_3};
float* RPYK_vector[3] = {RPYK, RPYK_2, RPYK_3};
float* RPYK_prev_vector[3] = {RPYK_prev, RPYK_prev_2, RPYK_prev_3};

/******************************
    Variables de uso Interfaz
 *****************************/
uint8_t VERBOSE = FALSE;
uint8_t MODO = INIT;
uint8_t FLAG_ERROR = FALSE;

/******************************
    Variables de UART
 *****************************/
uint8_t FLAG_PRIMERCHAR = TRUE; // Por defecto
uint8_t DATO;
char BUFF[MAX_BUF_SIZE];
uint8_t BUF_LIMIT;
uint8_t BUFF_CNT = 0;
uint8_t FLAG_INITRAMA = FALSE;
uint8_t FLAG_CHECK_TRAMA = FALSE;

uint8_t FLAG_DMP_ON = FALSE;


/****************************
        Setup Inicial
*****************************/
void setup() {
  PORTB |= 0x01;
  // Inicar UART
  INICIAR_UART();
  // Iniciar I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(1000, true); //timeout value in uSec - SBWire uses 100 uSec, so 1000 should be OK
  delay(200);
  // Inicializar dispositivos
  INIT_MPUS(OFFS_IMU1,  OFFS_IMU2,  OFFS_IMU3);

  // kalmanX.setAngle(0.0);
  delay(200);
}

/****************************
        Loop
*****************************/

void loop() {
  if (FLAG_CHECK_TRAMA) {
    ANALIZAR_TRAMA (&MODO, BUFF, BUFF_CNT);
    FLAG_CHECK_TRAMA = FALSE;
    BUFF_CNT = 0;
    if (MODO != ENVIO || FLAG_ENVIO_ON == 0) {
      tcaselect(0);
      mpu.setDMPEnabled(false);
      FLAG_DMP_ON = FALSE;
    }
  }
  if (MODO == ENVIO) {
    if (FLAG_ENVIO_ON == 1) {

      if (!FLAG_DMP_ON) {
        tcaselect(0);
        mpu.setDMPEnabled(true);
        tcaselect(1);
        mpu.setDMPEnabled(true);
        tcaselect(2);
        mpu.setDMPEnabled(true);
        FLAG_DMP_ON = TRUE;
        timer = micros();
        delay(200);
        kalmanX3.setAngle(0.0);
        kalmanY3.setAngle(0.0);
        kalmanX2.setAngle(0.0);
        kalmanY2.setAngle(0.0);
        kalmanX.setAngle(0.0);
        kalmanY.setAngle(0.0);
      }

      tcaselect(0);
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&acc, fifoBuffer);
        mpu.dmpGetGyro(&gyro, fifoBuffer);
      };

      quat[0] = q.w;     quat[1] = q.x;     quat[2] = q.y;     quat[3] = q.z;
      acc_rate[0] = acc.x*9.81/16384.0; acc_rate[1] = acc.y*9.81/ 16384.0; acc_rate[2] = acc.z*9.81 / 16384.0;
      gyro_rate[0] = gyro.x / 131.0; gyro_rate[1] = gyro.y / 131.0; gyro_rate[2] = gyro.z / 131.0;


      tcaselect(1);
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&acc, fifoBuffer);
        mpu.dmpGetGyro(&gyro, fifoBuffer);
      };

      quat_2[0] = q.w;  quat_2[1] = q.x;  quat_2[2] = q.y;  quat_2[3] = q.z;
      acc_rate_2[0] = acc.x*9.81/16384.0; acc_rate_2[1] = acc.y*9.81/ 16384.0; acc_rate_2[2] = acc.z*9.81 / 16384.0;
      gyro_rate_2[0] = gyro.x / 131.0; gyro_rate_2[1] = gyro.y / 131.0; gyro_rate_2[2] = gyro.z / 131.0;

      tcaselect(2);
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&acc, fifoBuffer);
        mpu.dmpGetGyro(&gyro, fifoBuffer);
      };

      quat_3[0] = q.w;  quat_3[1] = q.x;  quat_3[2] = q.y;  quat_3[3] = q.z;
      acc_rate_3[0] = acc.x*9.81/16384.0; acc_rate_3[1] = acc.y*9.81/ 16384.0; acc_rate_3[2] = acc.z*9.81 / 16384.0;
      gyro_rate_3[0] = gyro.x / 131.0; gyro_rate_3[1] = gyro.y / 131.0; gyro_rate_3[2] = gyro.z / 131.0;


      time_delta = (float) (micros() - timer) / 1000000;
      timer = micros();

      /****************************
          Kalman Filter
      *****************************/
      
      for (uint8_t n = 0; n < 3; n++){

        float* acc_vector_n = acc_vector[n];
        float* gyro_vector_n = gyro_vector[n];
        Kalman* dir_kalman_y = dir_kalman[n*2+1];

        pre_roll  = atan(acc_vector[n][1] / sqrt(acc_vector_n[0] * acc_vector_n[0] + acc_vector_n[2] * acc_vector_n[2])) * RAD_TO_DEG;
        if (!isnan(pre_roll))
          roll = pre_roll;

        pitch = atan2(-acc_vector_n[0], acc_vector_n[2]) * RAD_TO_DEG;


        if ((pitch < -90 && *angle_kalman[n*2+1] > 90) || (pitch > 90 && *angle_kalman[n*2+1] < -90)) {
          // kalmanY.setAngle(pitch);
          *angle_kalman[n*2+1] = pitch;
        } else{
          if (abs(pitch - RPYK_vector[n][1]) > 45){
              *angle_kalman[n*2+1] = RPYK_vector[n][1];
          }else{
            *angle_kalman[n*2+1] = dir_kalman[n*2+1] -> getAngle(pitch, gyro_vector_n[1], time_delta); // Calculate the angle using a Kalman filter
          }
        }

        if (abs(*angle_kalman[n*2+1]) > 90)
          gyro_vector_n[1] = -gyro_vector_n[1]; // Invert rate, so it fits the restriced accelerometer reading


        if (abs(roll - RPYK_prev_vector[n][0]) > 45){
            *angle_kalman[n*2] = RPYK_vector[n][0];
        }else{
          *angle_kalman[n*2] = dir_kalman[n*2] -> getAngle(roll, gyro_vector_n[0], time_delta); // Calculate the angle using a Kalman filter
        }

        RPYK_vector[n][0] = *angle_kalman[n*2];
        RPYK_vector[n][1] = *angle_kalman[n*2+1];

        // RPYK_vector[n][0] = roll;
        // RPYK_vector[n][1] = pitch;
      }


      /****************************
          One Euro
      *****************************/
      OneEuro_RPY(quat, quat_prev, dquat_prev, quat_filt, time_delta, 0, 10, 0.01);
      OneEuro_RPY(quat_2, quat_prev_2, dquat_prev_2, quat_filt_2, time_delta, 0, 10, 0.01);
      OneEuro_RPY(quat_3, quat_prev_3, dquat_prev_3, quat_filt_3, time_delta, 0, 10 ,0.01);


      OneEuro_RPY(RPYK, RPYK_prev, dRPYK_prev, RPYK_filt, time_delta, 1, 10, 0.01);
      OneEuro_RPY(RPYK_2, RPYK_prev_2, dRPYK_prev_2, RPYK_filt_2, time_delta, 1, 10, 0.01);
      OneEuro_RPY(RPYK_3, RPYK_prev_3, dRPYK_prev_3, RPYK_filt_3, time_delta, 1, 10, 0.01);
      print2comp(quat_filt[0], quat_filt[1], quat_filt[2], quat_filt[3],   quat_filt_2[0], quat_filt_2[1], quat_filt_2[2], quat_filt_2[3], quat_filt_3[0], quat_filt_3[1], quat_filt_3[2], quat_filt_3[3]);
      print2RPY(RPYK_filt[0], RPYK_filt[1], RPYK_filt_2[0], RPYK_filt_2[1], RPYK_filt_3[0], RPYK_filt_3[1]);
      // print2comp(quat_filt[0], quat_filt[1], quat_filt[2], quat_filt[3], acc_rate[0], acc_rate[1], acc_rate[2], gyro_rate[0], gyro_rate[1], gyro_rate[2], RPYK_filt[0], RPYK_filt[1]);
      delay(2);
    }
  }
  else if (MODO == CALIB) {
    if (FLAG_CALIB_ON == 1) {
      tcaselect(0);
      print_serial("Calibracion Primer Sensor: Codo\n");
      EXEC_CALIB(OFFS_IMU1, &mpu);
      tcaselect(1);
      print_serial("Calibracion Segundo Sensor: Muneca\n");
      EXEC_CALIB(OFFS_IMU2, &mpu);
      tcaselect(2);
      print_serial("Calibracion Tercer Sensor: Palma\n");
      EXEC_CALIB(OFFS_IMU3, &mpu);
      FLAG_CALIB_ON = 0;
      print_serial("CALIB:LISTO\n");
    }
  }
}


/******************************
    Funciones de Interrupcion
 ******************************/
ISR(USART_RX_vect) {
  uint8_t NUEVO_DATO = UDR0;
  if (FLAG_PRIMERCHAR) {
    if (NUEVO_DATO == 's')
      FLAG_PRIMERCHAR = FALSE;
    BUFF[BUFF_CNT] = NUEVO_DATO;
    BUFF_CNT++;
  }
  else {
    BUFF[BUFF_CNT] = NUEVO_DATO;
    BUFF_CNT++;
    if (NUEVO_DATO == 'e') {
      FLAG_PRIMERCHAR = TRUE;
      FLAG_CHECK_TRAMA = TRUE;
    }
    else if (BUFF_CNT >= MAX_BUF_SIZE) { // Limpiar B?fer - Lleno
      FLAG_PRIMERCHAR = TRUE;
      BUFF_CNT = 0;
      FLAG_ERROR = TRUE;
    }

  }
}










