
#include "CameraOV7670.h"


bool CameraOV7670::init()
{
  registers.init(); // init SCCB
  initIO();
  delay(100); // give camera some time to run before starting setup
  return setUpCamera();
}

void CameraOV7670::initIO()
{
  // configure input pins
  for(int i=0; i<8; i++) {
    pinMode(inputPins[i],INPUT);
  }
// init timer for XCLK generation
  pinMode(OV7670_XCLK_PIN, PWM);
  pwmWrite(OV7670_XCLK_PIN, 1);
  // change here for different PWM frequency output:
  timer_set_reload(TIMER1, 1);
}

bool CameraOV7670::setUpCamera()
{
  if (registers.resetSettings()) {
    registers.setRegisters(CameraOV7670Registers::regsDefault);

    switch (pixelFormat) {
      default:
      case PIXEL_RGB565:
        registers.setRegisters(CameraOV7670Registers::regsRGB565);
        break;
      case PIXEL_BAYERRGB:
        registers.setRegisters(CameraOV7670Registers::regsBayerRGB);
        break;
      case PIXEL_YUV422:
        registers.setRegisters(CameraOV7670Registers::regsYUV422);
        break;
    }

    switch (resolution) {
      case RESOLUTION_VGA_640x480:
        registers.setRegisters(CameraOV7670Registers::regsVGA);
        break;
      case RESOLUTION_QVGA_320x240:
        registers.setRegisters(CameraOV7670Registers::regsQVGA);
        break;
      default:
      case RESOLUTION_QQVGA_160x120:
        registers.setRegisters(CameraOV7670Registers::regsQQVGA);
        break;
    }

    registers.setDisablePixelClockDuringBlankLines();
    registers.setInternalClockPreScaler(internalClockPreScaler);

    return true;
  } else {
    return false;
  }
}
