
#ifndef _CAMERA_OV7670_H_
#define _CAMERA_OV7670_H_


#include "CameraOV7670Registers.h"


// STM32
#define OV7670_XCLK_PIN PA8

// vsync - PB5
#ifndef OV7670_VSYNC
#define OV7670_VSYNC_PIN PB5
#define OV7670_VSYNC ((*idr) & BIT5) //#define OV7670_VSYNC ((*GPIOB_BASE).IDR & BIT5)
#endif

// href - PB3
#ifndef OV7670_HREF
#define OV7670_HREF_PIN PB3
#define OV7670_HREF ((*idr) & BIT3) //(*GPIOB_BASE).IDR & BIT3)
#endif

// pixel byte - PB8..PB15
#ifndef OV7670_PIXEL_BYTE
#define OV7670_PIXEL_BYTE ((uint8_t*)(&(*idr)))[1] //((uint8_t*)(&(*GPIOB_BASE).IDR))[1]
#endif


class CameraOV7670
{
public:

  volatile uint32_t * idr = portInputRegister(GPIOB);

  enum PixelFormat {
    PIXEL_RGB565,
    PIXEL_BAYERRGB,
    PIXEL_YUV422
  };

  enum Resolution {
    RESOLUTION_VGA_640x480,
    RESOLUTION_QVGA_320x240,
    RESOLUTION_QQVGA_160x120
  };

private:

  Resolution resolution;
  PixelFormat pixelFormat;
  uint8_t internalClockPreScaler;

  CameraOV7670Registers registers;

  void initIO();
  bool setUpCamera();

public:

  CameraOV7670(Resolution _resolution, PixelFormat _format, uint8_t _internalClockPreScaler) :
      resolution(_resolution),
      pixelFormat(_format),
      internalClockPreScaler(_internalClockPreScaler) {};

  bool init();
  inline void setManualContrastCenter(uint8_t center) { registers.setManualContrastCenter(center); }
  inline void setContrast(uint8_t contrast)           { registers.setContrast(contrast); }
  inline void setBrightness(uint8_t brightness)       { registers.setBrightness(brightness); }
  inline void reversePixelBits()                      { registers.reversePixelBits(); }

  inline uint8_t getPrescaler(void)             { return internalClockPreScaler; }

  inline uint16_t isVsyncOn(void)               { return(OV7670_VSYNC); }
  inline void waitForVsync(void)                { while(!OV7670_VSYNC); }
  inline void waitForVsyncEnd(void)             { while( OV7670_VSYNC); }
  inline uint16_t isHrefOn(void)                { return(OV7670_HREF); }
  inline void waitForHref(void)                 { while(!OV7670_HREF); }
  inline void waitForHrefEnd(void)              { while( OV7670_HREF); }
  inline uint8_t readPixelByte(void)            { return OV7670_PIXEL_BYTE; }
};


#endif // _CAMERA_OV7670_h_
