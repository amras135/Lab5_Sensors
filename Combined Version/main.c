#include "stm32l476xx.h"
#include "lcd.h"

/*************************************  32L476GDISCOVERY ***************************************************************************
// STM32L4:  STM32L476VGT6 MCU = ARM Cortex-M4 + FPU + DSP, 
//           LQFP100, 1 MB of Flash, 128 KB of SRAM
//           Instruction cache = 32 lines of 4x64 bits (1KB)
//           Data cache = 8 lines of 4x64 bits (256 B)
//
// Joystick (MT-008A): 
//   Right = PA2        Up   = PA3         Center = PA0
//   Left  = PA1        Down = PA5
//
// User LEDs: 
//   LD4 Red   = PB2    LD5 Green = PE8
//   
// CS43L22 Audio DAC Stereo (I2C address 0x94):  
//   SAI1_MCK = PE2     SAI1_SD  = PE6    I2C1_SDA = PB7    Audio_RST = PE3    
//   SAI1_SCK = PE5     SAI1_FS  = PE4    I2C1_SCL = PB6                                           
//
// MP34DT01 Digital MEMS microphone 
//    Audio_CLK = PE9   Audio_DIN = PE7
//
// LSM303C eCompass (a 3D accelerometer and 3D magnetometer module): 
//   MEMS_SCK  = PD1    MAG_DRDY = PC2    XL_CS  = PE0             
//   MEMS_MOSI = PD4    MAG_CS  = PC0     XL_INT = PE1       
//                      MAG_INT = PC1 
//
// L3GD20 Gyro (three-axis digital output): 
//   MEMS_SCK  = PD1    GYRO_CS   = PD7
//   MEMS_MOSI = PD4    GYRO_INT1 = PD2
//   MEMS_MISO = PD3    GYRO_INT2 = PB8
//
// ST-Link V2 (Virtual com port, Mass Storage, Debug port): 
//   USART_TX = PD5     SWCLK = PA14      MFX_USART3_TX   MCO
//   USART_RX = PD6     SWDIO = PA13      MFX_USART3_RX   NRST
//   PB3 = 3V3_REG_ON   SWO = PB5      
//
// Quad SPI Flash Memory (128 Mbit)
//   QSPI_CS  = PE11    QSPI_D0 = PE12    QSPI_D2 = PE14
//   QSPI_CLK = PE10    QSPI_D1 = PE13    QSPI_D3 = PE15
//
// LCD (24 segments, 4 commons, multiplexed 1/4 duty, 1/3 bias) on DIP28 connector
//   VLCD = PC3
//   COM0 = PA8     COM1  = PA9      COM2  = PA10    COM3  = PB9
//   SEG0 = PA7     SEG6  = PD11     SEG12 = PB5     SEG18 = PD8
//   SEG1 = PC5     SEG7  = PD13     SEG13 = PC8     SEG19 = PB14
//   SEG2 = PB1     SEG8  = PD15     SEG14 = PC6     SEG20 = PB12
//   SEG3 = PB13    SEG9  = PC7      SEG15 = PD14    SEG21 = PB0
//   SEG4 = PB15    SEG10 = PA15     SEG16 = PD12    SEG22 = PC4
//   SEG5 = PD9     SEG11 = PB4      SEG17 = PD10    SEG23 = PA6
// 
// USB OTG
//   OTG_FS_PowerSwitchOn = PC9    OTG_FS_VBUS = PC11    OTG_FS_DM = PA11  
//   OTG_FS_OverCurrent   = PC10   OTG_FS_ID   = PC12    OTG_FS_DP = PA12  
//
// PC14 = OSC32_IN      PC15 = OSC32_OUT
// PH0  = OSC_IN        PH1  = OSC_OUT 
// 
// PA4  = DAC1_OUT1 (NLMFX0 WAKEUP)   PA5 = DAC1_OUT2 (Joy Down)
// PA3  = OPAMP1_VOUT (Joy Up)        PB0 = OPAMP2_VOUT (LCD SEG21)
//
//****************************************************************************************************************

//volatile uint8_t hour,min,sec; 

*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*

//volatile uint32_t digoutput;
//volatile double dist_check[100];
//volatile double distance_chk;
//volatile double dec_output;
//volatile double voltage;
//volatile double distance; */
uint8_t bar[2];
uint8_t distance = 70;
uint8_t distanceOnes;
uint8_t distanceTens;	
uint8_t sensor[4] = {0,1,0,1};
uint8_t display[6];

int i;

void LCD_Convert(void);
void BarDisplay(void);

int main(void){

	
	LCD_Initialization();
	
	

	
	while(1) {
		LCD_Convert();
		LCD_Display(display, bar[0], bar[1]);
	}
		
	
}

void LCD_Convert() 
{
	for(i = 0; i < 6; i++) {
		display[i+2] = 'W';
		if(sensor[i] == 1) display[i+2] = 'B';
	}
	
	distanceTens = distance/10;
	distanceOnes = distance - (distanceTens*10);
	
	display[0] = distanceTens + 48;
	display[1] = distanceOnes + 48;
	
	BarDisplay();
}

void BarDisplay()
{
	if((distance >= 10) && (distance <= 27.5))
	{
		bar[0] = 0xA; bar[1] = 0xA;
	}
	else if((distance >= 27.5) && (distance <= 45))
	{
		bar[0] = 0x8; bar[1] = 0xA;
	}
	else if((distance >= 45) && (distance <= 52.5))
	{
		bar[0] = 0x8; bar[1] = 0x8;
	}
	else if((distance >= 52.5) && (distance <= 80))
	{
		bar[0] = 0x0; bar[1] = 0x8;
	}
	else {
		bar[0] = 0x0; bar[1] = 0x0;
	}
}