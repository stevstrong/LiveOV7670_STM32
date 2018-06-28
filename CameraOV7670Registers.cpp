//
// Created by indrek on 30.10.2016.
//

#include "CameraOV7670Registers.h"
#include <Wire.h>

HardWire mWire(2);

void CameraOV7670Registers::init()
{
  mWire.begin();
}


bool CameraOV7670Registers::resetSettings()
{
  bool isSuccessful = setRegister(REG_COM7, COM7_RESET);
  delay(500);
  return isSuccessful;
}


void CameraOV7670Registers::setRegisters(const RegisterData *programMemPointer)
{
  while (true) {
    RegisterData regData = {
        addr: (programMemPointer->addr),
        val: (programMemPointer->val)
    };
    if (regData.addr == 0xFF) {
      break;
    } else {
      setRegister(regData.addr, regData.val);
      programMemPointer++;
    }
  }
}


bool CameraOV7670Registers::setRegister(uint8_t addr, uint8_t val)
{
  mWire.beginTransmission(i2cAddress);
  mWire.write(addr);
  mWire.write(val);
  return mWire.endTransmission() == 0;
}


uint8_t CameraOV7670Registers::readRegister(uint8_t addr)
{
  mWire.beginTransmission(i2cAddress);
  mWire.write(addr);
  mWire.endTransmission();

  mWire.requestFrom(i2cAddress, (uint8_t)1);
  return mWire.read();
}


void CameraOV7670Registers::setRegisterBitsOR(uint8_t addr, uint8_t bits)
{
  uint8_t val = readRegister(addr);
  setRegister(addr, val | bits);
}


void CameraOV7670Registers::setRegisterBitsAND(uint8_t addr, uint8_t bits)
{
  uint8_t val = readRegister(addr);
  setRegister(addr, val & bits);
}


void CameraOV7670Registers::setDisablePixelClockDuringBlankLines()
{
  setRegisterBitsOR(REG_COM10, COM10_PCLK_HB);
}


void CameraOV7670Registers::setInternalClockPreScaler(int preScaler)
{
  setRegister(REG_CLKRC, 0x80 | ((preScaler)&0x1F)); // f = input / (val + 1)
  //setRegister(DBLV, 0x4A); // PLL control[7-6]: 01: Input clock x4
}


void CameraOV7670Registers::setManualContrastCenter(uint8_t contrastCenter)
{
  setRegisterBitsAND(MTXS, 0x7F); // disable auto contrast
  setRegister(REG_CONTRAST_CENTER, contrastCenter);
}


void CameraOV7670Registers::setContrast(uint8_t contrast)
{
  // default 0x40
  setRegister(REG_CONTRAS, contrast);
}


void CameraOV7670Registers::setBrightness(uint8_t birghtness)
{
  setRegister(REG_BRIGHT, birghtness);
}


void CameraOV7670Registers::reversePixelBits()
{
  setRegisterBitsOR(REG_COM3, COM3_SWAP);
}

