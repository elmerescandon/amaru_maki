#ifndef MPU_UTILS_H_
#define MPU_UTILS_H_

extern MPU6050 mpu;
extern uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
void tcaselect(uint8_t i);
uint8_t INIT_MPUS(uint16_t* OFFS_IMU1, uint16_t* OFFS_IMU2, uint16_t* OFFS_IMU3);
#endif
