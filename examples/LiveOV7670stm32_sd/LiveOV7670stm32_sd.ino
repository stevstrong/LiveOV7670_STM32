//
// Created by indrek on 4.12.2016.
//
// Adapted for ILI9341 by stevestrong on 29.07.2017
//

#include <LiveOV7670_stm32.h>
#include <Adafruit_ILI9341_STM.h>
#include "Streaming.h"

#include <SdFat.h>

const uint8_t SD_CS = PA3;
SdFat sd;


// TFT connection:
// PA2 - TFT reset
// PA3 - TFT D/C (data/command)
// PA4 - TFT chip select
// PA5 - TFT SPI CLK
// PA6 - TFT SPI MISO
// PA7 - TFT SPI MOSI

// Camera connection:
// PA8 - camera clock
// PB3 - HREF
// PA1 - pixel clock PCLK
// PB5 - VSYNC
// PB6 - I2C Clock
// PB7 - I2C data
// PB8..PB15 - pixel data

//-----------------------------------------------------------------------------
#define LED_PIN PC13
#define DBG_PIN PA3
#define SET_DBG digitalWrite(DBG_PIN, HIGH)
#define CLR_DBG digitalWrite(DBG_PIN, LOW)

//-----------------------------------------------------------------------------
// CPU@72MHZ -> 11fps, CPU@80MHZ-> 12.5fps
//BufferedCameraOV7670_QVGA camera(CameraOV7670::PIXEL_RGB565, BufferedCameraOV7670_QVGA::FPS_11p_Hz);
BufferedCameraOV7670_QVGA camera(CameraOV7670::PIXEL_RGB565, (BufferedCameraOV7670_QVGA::Prescaler)8);

//BufferedCameraOV7670<uint16_t, 320, uint8_t, 160, uint8_t, 120> camera(
//    CameraOV7670::RESOLUTION_QQVGA_160x120,
//    CameraOV7670::PIXEL_RGB565,
//    4);

//Adafruit_ILI9341_STM tft(PA4, PA3, PA2); // SS, DC, RST
Adafruit_ILI9341_STM tft(PA2, PA0, PB4); // SS, DC, RST

// Normally it is a portrait screen. Use it as landscape
uint16_t screen_w = ILI9341_TFTHEIGHT;
uint16_t screen_h = ILI9341_TFTWIDTH;
uint16_t screenLineIndex;
uint8_t bufIndex;

#define PIXELS_PER_LINE ILI9341_TFTHEIGHT

// The GPIO.IDR register can be only accessed in word (32 bit) mode.
// Therefore a 4 times larger buffer should be allocated to one line.
// But because for each pixel we need 2 reads, the buffer should be doubled again.
// The useful bytes are packed in the same buffer after reading is finished.
// This is a lot of wasted memory, but it runs with DMA, which is a huge benefit!
#define PIXEL_BUFFER_LENGTH (2*PIXELS_PER_LINE+1)

uint8_t pixBuf[2][PIXEL_BUFFER_LENGTH];

//-----------------------------------------------------------------------------
void TIMER_Setup(void)
{
	gpio_set_mode(GPIOA, 1, GPIO_INPUT_FLOATING);

// Slave mode: Reset mode (see RM0008, chap. 15.3.14, page 394)
// ------------------------
// The counter and its prescaler can be reinitialized in response to an event on a trigger input.
// Moreover, if the URS bit from the TIMx_CR1 register is low, an update event UEV is generated.
// Then all the preloaded registers (TIMx_ARR, TIMx_CCRx) are updated.
//
// In the following example, the upcounter is cleared in response to a rising edge on TI2 input:
// • Configure the channel 2 to detect rising edges on TI2.
// - Configure the input filter duration (in this example, we don’t need any filter, so we keep IC1F=0000).
// - The capture prescaler is not used for triggering, so you don’t need to configure it.
// - The CC2S bits select the input capture source only, CC2S = 01 in the TIMx_CCMR1 register.
// - Write CC2P=0 in TIMx_CCER register to validate the polarity (and detect rising edges only).
// • Configure the timer in reset mode by writing SMS=100 in TIMx_SMCR register.
// - Select TI2 as the input source by writing TS=101 in TIMx_SMCR register.
// • Start the counter by writing CEN=1 in the TIMx_CR1 register.
//
// The counter starts counting on the internal clock, then behaves normally until TI2 rising edge.
// When TI2 rises, the counter is cleared and restarts from 0.
// In the meantime, the trigger flag is set (TIF bit in the TIMx_SR register) and an interrupt request,
// or a DMA request can be sent if enabled (depending on the TIE and TDE bits in TIMx_DIER register).
//
// This event will trigger the DMA to save the content of GPIOB.IDR[1] to memory.

//	timer_pause(TIMER2); // stop timer
	timer_init(TIMER2);  // turn timer RCC on
	// configure PA2 = timer 2 channel 2 == input TI2

	// as this mode is not supported by the core lib, we have to set up the registers manually.
	(TIMER2->regs).gen->CR1 = 0; // stop the timer
	(TIMER2->regs).gen->CR2 = 0;
	(TIMER2->regs).gen->SMCR = (TIMER_SMCR_TS_TI2FP2 | TIMER_SMCR_SMS_RESET);//TIMER_SMCR_SMS_TRIGGER);
	(TIMER2->regs).gen->DIER = (TIMER_DIER_UDE); // enable DMA request on TIM2 update
	(TIMER2->regs).gen->SR = 0;
	(TIMER2->regs).gen->EGR = 0;
	(TIMER2->regs).gen->CCMR1 = TIMER_CCMR1_CC2S_INPUT_TI2; // IC2F='0000', IC2PSC='0', CC2S='01' 
	(TIMER2->regs).gen->CCMR2 = 0;
	(TIMER2->regs).gen->CCER = (TIMER_CCER_CC2P); // inverse polarity, active low
	(TIMER2->regs).gen->CNT = 0;
	(TIMER2->regs).gen->PSC = 0;
	(TIMER2->regs).gen->ARR = 0;
	(TIMER2->regs).gen->CCR1 = 0;
	(TIMER2->regs).gen->CCR2 = 0;
	(TIMER2->regs).gen->CCR3 = 0;
	(TIMER2->regs).gen->CCR4 = 0;
	(TIMER2->regs).gen->DCR = 0;			// don't need DMA for timer
	(TIMER2->regs).gen->DMAR = 0;
	// don't forget to set the DMA trigger source to TIM2-UP
	//timer_resume(TIMER2); // start timer
}
//-----------------------------------------------------------------------------
void DMA_Setup(void)
{
	dma_init(DMA1);
	uint32_t pmem = ((uint32_t)(&(GPIOB->regs->IDR)) + 1); // use GPIOB high byte as source
	dma_setup_transfer(DMA1, DMA_CH2,
		(uint8_t *)pmem, DMA_SIZE_8BITS,
		(uint8_t *)&pixBuf, DMA_SIZE_8BITS,
		(DMA_MINC_MODE));//| DMA_CIRC_MODE));
	dma_set_priority(DMA1, DMA_CH2, DMA_PRIORITY_VERY_HIGH);
	//dma_set_num_transfers(DMA1, DMA_CH2, 2*PIXELS_PER_LINE); // 2 readings for each pixel
	//dma_enable(DMA1, DMA_CH2);
}

//-----------------------------------------------------------------------------
void setup()
{
  //while(!Serial);
  delay(1000);
  Serial << "ILI9341 LCD and OV7670 camera application\n";
  
  pinMode(LED_PIN, OUTPUT);
#ifdef DBG_PIN
  pinMode(DBG_PIN, OUTPUT);
  CLR_DBG;
#endif
  // initialise the LCD
  tft.begin(); // use standard SPI port

  tft.setRotation(1); // rotate 90° for landscape
  tft.setAddrWindow(0, 0, screen_h, screen_w);
  tft.fillScreen(ILI9341_BLUE);

  tft.setTextSize(2);
  tft.print("\n\nScreen size: "); tft.print(screen_w); tft.write('x'); tft.print(screen_h);
  tft.println("\n\nSetting up the camera...");
  Serial.print("Setting up the camera...");

  // initialise the camera
  if ( !camera.init() ) {
    tft.fillScreen(ILI9341_RED);
    tft.print("\n   Camera init failure!");
    Serial.println("failed!");
    blink(0);
  } else {
    tft.print("done.");
    Serial.println("done.\n");
  }

  Serial.print("Starting SD card...");
  if (!sd.begin(SD_CS, SD_SCK_MHZ(50)))
  {
    sd.initErrorHalt();
  }
  Serial.println("done.\n");

  bufIndex = 0;
  BMP_Setup();
  // wait two frames to stabilize the image
  camera.waitForVsync();  camera.waitForVsyncEnd();
  tft.setAddrWindow(0, 0, screen_w, screen_h);
  camera.waitForVsync();  camera.waitForVsyncEnd();
  // enable the timer and corresponding DMA request
  DMA_Setup();
  TIMER_Setup();
	SPI.setDataSize(DATA_SIZE_8BIT); // set to 8 bit mode
}
//-----------------------------------------------------------------------------
void blink(uint8_t br)
{
	while(1)
	{
		digitalWrite(LED_PIN, LOW);
		delay(125);
		digitalWrite(LED_PIN, HIGH);
		delay(125);
		if (br) break;
	}
}
uint32_t loop_counter, line_counter;
//-----------------------------------------------------------------------------
void loop()
{
  uint8_t * ptr = (uint8_t*)&pixBuf;
  uint32_t * ptr32 = (uint32_t*)&pixBuf+1;

  // start of frame
  camera.waitForVsync();
	digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  camera.waitForVsyncEnd();

  if ( loop_counter && (loop_counter&(BIT2-1))==0 )
  {
    if  ( (loop_counter&(BIT7-1))==0 ) {
        Serial.write('\n');
    }
    Serial.write('.');
    // write here to SD card
    LCD2SD();
    // wait one more frame
    camera.waitForVsync();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    camera.waitForVsyncEnd();
  }
  loop_counter ++;

  line_counter = screen_h;

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // the timing within the following while loops is very code sensitive, so
  // DON'T CHNAGE ANYTHING THERE!
  // working conditions: build with optimize option "-Os"

noInterrupts();
  // receive lines
  while ( (line_counter--) ) {
    // activate here the DMA and the timer
    dma_set_num_transfers(DMA1, DMA_CH2, 2*PIXELS_PER_LINE+1); // 2 readings for each pixel
    dma_clear_isr_bits(DMA1, DMA_CH2);
    dma_enable(DMA1, DMA_CH2);
    timer_resume(TIMER2); // start timer
    // one byte will be read instantly - TODO: find out why ?
    camera.waitForHref(); // wait for line start
    uint32_t t0 = millis();
    // monitor the DMA channel transfer end, just loop waiting for the flag to be set.
    while ( !(dma_get_isr_bits(DMA1, DMA_CH2) & DMA_ISR_TCIF) ) {
      //Serial.print("DMA status: "); Serial.println(status, HEX);
      if ((millis() - t0) > 1000) { Serial.println("DMA timeout!"); blink(0); }
      if ( !camera.isHrefOn() ) break; // stop if over end of line
    }

    timer_pause(TIMER2); // stop timer
    dma_disable(DMA1, DMA_CH2);
    dma_clear_isr_bits(DMA1, DMA_CH2);

    if ( dma_get_isr_bits(DMA1, DMA_CH2) & DMA_ISR_TCIF ) {
        Serial.print("2. DMA ISR: "); Serial.print(DMA1->regs->ISR, HEX); Serial.print(", CNT: "); Serial.println(DMA1CH2_BASE->CNDTR);
        Serial.println("DMA ISR bits reset error!"); blink(0);
    }
    // clear some disturbing pixels
    ptr32 = (uint32_t*)&pixBuf;
    *ptr32 = 0; // clear disturbing first pixels

    // send pixel data to LCD
    tft.pushColors((uint16_t*)ptr, 2*PIXELS_PER_LINE, 0); // 3rd parameter: send async
  }
interrupts();
}

//-----------------------------------------------------------------------------
unsigned char bmpFileHeader[14]; // = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0 };
unsigned char bmpInfoHeader[40]; // = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0 };  
char name[12];
SdFile file;
//-----------------------------------------------------------------------------
const int w = 320;
const int h = 240;
  // create image data
const int filesize = 54 + 4 * w * h;      //  w is image width, h is image height
//-----------------------------------------------------------------------------
void BMP_Setup(void)
{
  bmpFileHeader[ 0] = 'B';
  bmpFileHeader[ 1] = 'M';
  bmpFileHeader[ 2] = (unsigned char)(filesize    );
  bmpFileHeader[ 3] = (unsigned char)(filesize >> 8);
  bmpFileHeader[ 4] = (unsigned char)(filesize >> 16);
  bmpFileHeader[ 5] = (unsigned char)(filesize >> 24);
  bmpFileHeader[10] = 54;

  bmpInfoHeader[ 0] = 40;
  bmpInfoHeader[ 4] = (unsigned char)(       w    );
  bmpInfoHeader[ 5] = (unsigned char)(       w >> 8);
  bmpInfoHeader[ 6] = (unsigned char)(       w >> 16);
  bmpInfoHeader[ 7] = (unsigned char)(       w >> 24);
  bmpInfoHeader[ 8] = (unsigned char)(       h    );
  bmpInfoHeader[ 9] = (unsigned char)(       h >> 8);
  bmpInfoHeader[10] = (unsigned char)(       h >> 16);
  bmpInfoHeader[11] = (unsigned char)(       h >> 24);
  bmpInfoHeader[12] = 1;
  bmpInfoHeader[14] = 24;
}
//-----------------------------------------------------------------------------
void LCD2SD()
{
#define NUM_LINES_BUFFERED 12
uint8_t lineBufSD[w*3*NUM_LINES_BUFFERED];

  unsigned long m;

  // if name exists, create new filename
  for (int i = 0; i < 100; i++)
  {
    name[4] = i / 10 + '0';
    name[5] = i % 10 + '0';
    if (file.open(name, O_CREAT | O_EXCL | O_WRITE))
    {
      break;
    }
  }

  //BMP_Setup();
  file.write(bmpFileHeader, sizeof(bmpFileHeader));    // write file header
  file.write(bmpInfoHeader, sizeof(bmpInfoHeader));    // " info header

  m=millis();
  uint8_t t;
  int lineOffset;
  for (int y = 0 ; y < h ; y++) 
  {

    lineOffset = y%NUM_LINES_BUFFERED *3 * w;
    tft.readPixels24(0,(h-1)-y,320,(h-1)-y, lineBufSD + lineOffset);
    if ((y+1)%NUM_LINES_BUFFERED==0)
    {
      // swap colour channels from  RGB to BGR
      for (int x = 0; x < w * 3 * NUM_LINES_BUFFERED; x+=3) 
      {
        t=lineBufSD[x+2];
        lineBufSD[x+2]=lineBufSD[x];
        lineBufSD[x]=t;
      }
       file.write(lineBufSD, 3 * w * NUM_LINES_BUFFERED);
    }
  }
  Serial.print("Saved in ");Serial.print(millis()-m);Serial.println(" mS ");
  file.close();
}
