/*
 * HMC5883L.cpp
 *
 *  Created on: Jan 31, 2015
 * Last Edited: Feb  7, 2016
 *      Author: TekuConcept
 */

#include "HMC5883L.h"


HMC5883L::HMC5883L() : currentGain(Gain::G1_3) {}



HMC5883L::~HMC5883L() {}



int HMC5883L::WhoAmI() {
    return static_cast<short>(I2C::readShort(HMC5883L_ADDR, HMC5883L_ID_A));
}



float HMC5883L::X() {
    return scaleWithGain((short)I2C::readShort(HMC5883L_ADDR, HMC5883L_X_H));
}



float HMC5883L::Y() {
    return scaleWithGain((short)I2C::readShort(HMC5883L_ADDR, HMC5883L_Y_H));
}



float HMC5883L::Z() {
    return scaleWithGain((short)I2C::readShort(HMC5883L_ADDR, HMC5883L_Z_H));
}



float HMC5883L::scaleWithGain(short value) {
    return (float)(value * GAIN_SCALER[static_cast<int>(currentGain)]);
}



bool HMC5883L::isLocked() {
    return (I2C::readByte(HMC5883L_ADDR, HMC5883L_STATUS) & 2) >> 1;
}



bool HMC5883L::isReady() {
    return (I2C::readByte(HMC5883L_ADDR, HMC5883L_STATUS) & 1);
}



void HMC5883L::setSampleAverage(Sample ma) {
    uint cra = I2C::readByte(HMC5883L_ADDR, HMC5883L_CONFIG_A) & 0x9F;
    cra |= static_cast<int>(ma) << 5;
    I2C::writeByte(HMC5883L_ADDR, HMC5883L_CONFIG_A, cra);
}



void HMC5883L::setOutputRate(Rate dor) {
    uint cra = I2C::readByte(HMC5883L_ADDR, HMC5883L_CONFIG_A) & 0xE3;
    cra |= static_cast<int>(dor) << 2;
    I2C::writeByte(HMC5883L_ADDR, HMC5883L_CONFIG_A, cra);
}



void HMC5883L::setGain(Gain gn) {
    currentGain = gn;
    uint crb = I2C::readByte(HMC5883L_ADDR, HMC5883L_CONFIG_B) & 0x1F;
    crb |= static_cast<int>(gn) << 5;
    I2C::writeByte(HMC5883L_ADDR, HMC5883L_CONFIG_B, crb);
}



void HMC5883L::setMode(Mode md) {
    int _m_ = static_cast<int>(md);
    I2C::writeByte(HMC5883L_ADDR, HMC5883L_MODE, _m_);
}



int HMC5883L::getSampleAverage() {
    return ((I2C::readByte(HMC5883L_ADDR, HMC5883L_CONFIG_A) & 0x60) >> 5);
}



int HMC5883L::getOutputRate() {
    return ((I2C::readByte(HMC5883L_ADDR, HMC5883L_CONFIG_A) & 0x1C) >> 2);
}



int HMC5883L::getGain() {
    int val = ((I2C::readByte(HMC5883L_ADDR, HMC5883L_CONFIG_B) & 0xE0) >> 5);
    currentGain = static_cast<Gain>(val);
    return val;
}



int HMC5883L::getMode() {
    return ((I2C::readByte(HMC5883L_ADDR, HMC5883L_MODE) & 0x03));
}
