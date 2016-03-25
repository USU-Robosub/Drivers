/*
 * MPU6050.cpp
 *
 *  Created on: Feb 2, 2015
 * Last Edited: Feb 7, 2016
 *      Author: TekuConcept
 */

#include "MPU6050.h"


MPU6050::MPU6050() : g_full(FS_SEL::FSG_250), a_full(AFS_SEL::FSA_2) {}



MPU6050::~MPU6050() {}



int MPU6050::WhoAmI() {
    return static_cast<short>(I2C::readShort(MPU6050_ADDR_A, MPU6050_WHO_AM_I));
}



float MPU6050::accel_X() {
    return scaleAccel(I2C::readShort(MPU6050_ADDR_A, MPU6050_ACCEL_XOUT_H));
}



float MPU6050::accel_Y() {
    return scaleAccel(I2C::readShort(MPU6050_ADDR_A, MPU6050_ACCEL_YOUT_H));
}



float MPU6050::accel_Z() {
    return scaleAccel(I2C::readShort(MPU6050_ADDR_A, MPU6050_ACCEL_ZOUT_H));
}



float MPU6050::gyro_X(){
    return scaleGyro(I2C::readShort(MPU6050_ADDR_A, MPU6050_GYRO_XOUT_H));
}



float MPU6050::gyro_Y() {
    return scaleGyro(I2C::readShort(MPU6050_ADDR_A, MPU6050_GYRO_YOUT_H));
}



float MPU6050::gyro_Z() {
    return scaleGyro(I2C::readShort(MPU6050_ADDR_A, MPU6050_GYRO_ZOUT_H));
}



float MPU6050::temperature() {
    int16_t v = 0;
    v = I2C::readShort(MPU6050_ADDR_A, MPU6050_TEMP_OUT_H);
    return (v / 340) + 36.53F;
}



float MPU6050::scaleGyro(short value){
    return value * (1.0 / GYRO_SCALER[static_cast<int>(g_full)]);
}



float MPU6050::scaleAccel(short value) {
    return value * (1.0 / ACCEL_SCALER[static_cast<int>(a_full)]);
}



void MPU6050::setGyroFullScale(FS_SEL select) {
    g_full = select;
    I2C::writeByte(MPU6050_ADDR_A, MPU6050_GYRO_CONFIG,
        static_cast<int>(select) << 3);
}



void MPU6050::setAcclFullScale(AFS_SEL select) {
    a_full = select;
    I2C::writeByte(MPU6050_ADDR_A, MPU6050_ACCEL_CONFIG,
        static_cast<int>(select) << 3);
}



void MPU6050::sleep() {
    I2C::writeByte(MPU6050_ADDR_A, MPU6050_PWR_MGMT_1, 0x40);
}



void MPU6050::awake() {
    I2C::writeByte(MPU6050_ADDR_A, MPU6050_PWR_MGMT_1, 0x00);
}
