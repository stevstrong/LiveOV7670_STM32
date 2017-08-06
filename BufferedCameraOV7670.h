//
// Created by indrek on 30.04.2016.
//

#ifndef BUFFERED_CAMERA_OV7670_H
#define BUFFERED_CAMERA_OV7670_H

#include "CameraOV7670.h"


// Tx type for line length. If line length is smaller than 256 then uin8_t can be used otherwise use uin16_t
// Ty type for line count. If line length is smaller than 256 then uin8_t can be used otherwise use uin16_t
template <uint16_t lineLength, uint16_t lineCount>

class BufferedCameraOV7670 : public CameraOV7670
{
public:
  BufferedCameraOV7670(Resolution resolution, PixelFormat format, uint8_t internalClockPreScaler) :
      CameraOV7670(resolution, format, internalClockPreScaler) {};

  inline static uint16_t getLineLength() { return lineLength; }
  inline static uint16_t getLineCount()  { return lineCount; }
};

//-----------------------------------------------------------------------------
class BufferedCameraOV7670_QVGA : public BufferedCameraOV7670<320, 240>
{
public:
  enum Prescaler {
	  FPS_14p6_Hz = 2,
	  FPS_11p_Hz  = 3,
	  FPS_9p_Hz   = 4,
      FPS_7p5_Hz  = 5,
      FPS_6p4_Hz  = 6,
      FPS_5p6_Hz  = 7,
      FPS_5_Hz    = 8,
      FPS_4p5_Hz  = 9
  };

  BufferedCameraOV7670_QVGA(PixelFormat format, Prescaler fps) :
      BufferedCameraOV7670(Resolution::RESOLUTION_QVGA_320x240, format, fps) {};

};

//-----------------------------------------------------------------------------
class BufferedCameraOV7670_QQVGA : public BufferedCameraOV7670<160, 120>
{
public:
  enum Prescaler {
      FPS_XX_Hz  = 1,
      FPS_15_Hz  = 2,
      FPS_12_Hz  = 3,
      FPS_9_Hz   = 4,
      FPS_7p2_Hz = 5
  };

public:
  BufferedCameraOV7670_QQVGA(PixelFormat format, Prescaler fps) :
      BufferedCameraOV7670(Resolution::RESOLUTION_QQVGA_160x120, format, fps) {};

};


#endif //BUFFERED_CAMERA_OV7670_H
