#include "timers.h"
#include "MKL25Z4.h"
#include "ADC.h"
#include "LEDs.h"
#include "GPIO_defs.h"


volatile unsigned char pwr_mode = 0x00;
uint8_t  flash_counter = 0;

void Init_LPTMR(void) {
	SIM->SCGC5 |=  SIM_SCGC5_LPTMR_MASK;

	// Configure LPTMR
	// select 1 kHz LPO clock with prescale factor 0, dividing clock by 2
	// resulting in 500 Hz clock
	LPTMR0->PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PRESCALE(0); 
	LPTMR0->CSR = LPTMR_CSR_TIE_MASK;
	LPTMR0->CMR = 50; // Generate interrupt every 50 clock ticks or 100 ms

	// Configure NVIC 
	NVIC_SetPriority(LPTimer_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(LPTimer_IRQn); 
	NVIC_EnableIRQ(LPTimer_IRQn);	

}

void Start_LPTMR(void) {
	LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
}

void Stop_LPTMR(void) {
	LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK;
}

void LPTimer_IRQHandler(void) {
	float v_rail;
	static uint8_t n=LED_PERIOD;

	PTE->PSOR |= MASK(DEBUG_RUNNING_POS);
	NVIC_ClearPendingIRQ(LPTimer_IRQn);
	LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;

  switch(pwr_mode)
  {
    case GREEN_MODE:
      Control_RGB_LEDs(0,1,0);
      break;
    
    case YELLOW_MODE:
      Control_RGB_LEDs(0,1,1);
      break;
    
    case RED_MODE:
      Control_RGB_LEDs(1,0,0);
      break;
  }
  
      
    n--;
    if(n == 0){
      n=LED_PERIOD;
      Control_RGB_LEDs(0,0,0);
    }
}
