#include "stm32l476xx.h"
#include "SysClock.h"
#include "ADC.h"

volatile uint32_t digoutput;
//volatile uint32_t x;
//volatile uint32_t y;
volatile double voltage;
volatile double distance;

//volatile double result;

int main(void){
	//double result = 0;
	
	//double z = 0;
	System_Clock_Init(); // Switch System Clock = 80 MHz
	
	// Analog Inputs: 
	//  PA1 (ADC12_IN6), PA2 (ADC12_IN7)
	//  These pins are not used: PA0 (ADC12_IN5, PA3 (ADC12_IN8) 
	//  0V <=> 0, 3.3V <=> 4095
	ADC_Init();
	

	
	while(1){
		ADC1->CR |= ADC_CR_ADSTART;			
		while ( (ADC123_COMMON->CSR | ADC_CSR_EOC_MST) == 0);
		digoutput = ADC1->DR;
		
		z = (double)(digoutput)/(double)(1241);

		
	}
}



