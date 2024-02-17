#include <Arduino.h> 
#include "common.h"
#include "bmi2.h"
#include "imu.hpp"
#include <bmi270.h>

float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
struct bmi2_sens_data imu_data;

void imu_init(void)
{
  USBSerial.printf("Start IMU Initialize!\n\r");
  pinMode(46, OUTPUT);//CSを設定
  digitalWrite(46, 0);//CSをHIGH
  bmi270_dev_init();
  USBSerial.printf("SPI Initilize status:%d\n\r",spi_init());
  usleep(1000*10);
  uint8_t data=0;

  //BMI270 Init
  USBSerial.printf("#INIT Status:%d\n\r", bmi270_init(pBmi270));
  USBSerial.printf("#Chip ID DEV:%02X\n\r", Bmi270.chip_id);
  USBSerial.printf("#APP_STATUS:%02X\n\r", Bmi270.aps_status);
  
  USBSerial.printf("#INIT_STATUS Read:%d\n\r",bmi2_get_regs(0x21, &data, 1, pBmi270));  
  USBSerial.printf("#INIT_STATUS:%02X\n\r", data);
  //IMU Config
  USBSerial.printf("#Config Status:%d\n\r", set_accel_gyro_config(pBmi270));
  uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  USBSerial.printf("#Sensor enable Status:%d\n\r", bmi2_sensor_enable(sensor_list, 2, pBmi270));
}

void imu_update(void)
{
    bmi2_get_sensor_data(&imu_data, pBmi270);
}

float imu_get_acc_x(void)
{
    return lsb_to_mps2(imu_data.acc.y, 8.0, 16)/GRAVITY_EARTH;
}

float imu_get_acc_y(void)
{
    return lsb_to_mps2(imu_data.acc.x, 8.0, 16)/GRAVITY_EARTH;
}

float imu_get_acc_z(void)
{
    return -lsb_to_mps2(imu_data.acc.z, 8.0, 16)/GRAVITY_EARTH;
}

float imu_get_gyro_x(void)
{
    return lsb_to_rps(imu_data.gyr.y, DPS20002RAD, 16);
}

float imu_get_gyro_y(void)
{
    return lsb_to_rps(imu_data.gyr.x, DPS20002RAD, 16);
}

float imu_get_gyro_z(void)
{
    return -lsb_to_rps(imu_data.gyr.z, DPS20002RAD, 16);
}