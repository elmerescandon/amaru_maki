#include "MPU6050_6Axis_MotionApps612.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define TCAADDR 0x70
#include "interface_com.h"
// Creación de componente del sensor
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


uint8_t INIT_MPUS(uint16_t* OFFS_IMU1, uint16_t* OFFS_IMU2, uint16_t* OFFS_IMU3 ){
    // initialize device
    tcaselect(0);
    mpu.initialize();
    delay(100);
    devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(OFFS_IMU1[0]); 
    mpu.setYAccelOffset(OFFS_IMU1[1]); 
    mpu.setZAccelOffset(OFFS_IMU1[2]); 
    mpu.setXGyroOffset(OFFS_IMU1[3]);
    mpu.setYGyroOffset(OFFS_IMU1[4]);
    mpu.setZGyroOffset(OFFS_IMU1[5]);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.setDMPEnabled(false);
    } 
    else{
      while(true){
        print_serial("Error - IMU1\n");  
        delay(1000);
      }
    }

   
    // initialize device
    tcaselect(1);
    mpu.initialize();
    delay(100);
    devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(OFFS_IMU2[0]); 
    mpu.setYAccelOffset(OFFS_IMU2[1]); 
    mpu.setZAccelOffset(OFFS_IMU2[2]); 
    mpu.setXGyroOffset(OFFS_IMU2[3]);
    mpu.setYGyroOffset(OFFS_IMU2[4]);
    mpu.setZGyroOffset(OFFS_IMU2[5]);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.setDMPEnabled(false);
    } 
    else{
      while(true){
        print_serial("Error - IMU2\n");  
        delay(1000);
      }
    }

    tcaselect(2);
    mpu.initialize();
    delay(100);
    devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(OFFS_IMU3[0]); 
    mpu.setYAccelOffset(OFFS_IMU3[1]); 
    mpu.setZAccelOffset(OFFS_IMU3[2]); 
    mpu.setXGyroOffset(OFFS_IMU3[3]);
    mpu.setYGyroOffset(OFFS_IMU3[4]);
    mpu.setZGyroOffset(OFFS_IMU3[5]);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.setDMPEnabled(false);

    } 
    else{
      while(true){
        print_serial("Error - IMU3\n");  
        delay(1000);
      }
    }

  print_serial("Todo bien!");


}
