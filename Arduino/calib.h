#ifndef CALIB_H_
#define CALIB_H_




void EXEC_CALIB(int16_t *OFFS_IMU_t, MPU6050 *mpu);
void begin_calibracion();
void promediarsensores();
uint8_t calibracion();



#endif /* CALIB_H_ */
