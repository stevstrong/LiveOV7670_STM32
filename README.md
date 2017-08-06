

Live images from OV7670 camera on ILI9341 LCD display
---

Resolution: 320x240

Frame rate: 15fps



Code inspired from: https://github.com/indrekluuk/LiveOV7670_stm32-arduino


TFT connection:

PA2 - TFT reset

PA3 - TFT D/C (data/command)

PA4 - TFT chip select

PA5 - TFT SPI CLK

PA6 - TFT SPI MISO

PA7 - TFT SPI MOSI

Camera connections:

PA8 - XCLCK (camera clock)

PA1 - PCLCK (pixel clock)

PB3 - HREF (horizontal reference signal)

PB5 - VSYNC (vertical sync)

PB6 - i2c Clock (10K resistor to 3.3V)

PB7 - i2c data (10K resistor to 3.3V)

PB8..PB15 - D0..D7 (pixel byte)
