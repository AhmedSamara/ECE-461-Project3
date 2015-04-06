#include <MKL25Z4.H>
#include <stdlib.h>
#include "mma8451.h"
#include "i2c.h"
#include "delay.h"
#include <math.h>

int16_t acc_X=0, acc_Y=0, acc_Z=0;
//last values of acc when roll/pitch calculated.
int16_t p_acc_X=0, p_acc_Y=0, p_acc_Z=0;
int16_t mag_X=0, mag_Y=0, msg_Z=0;
float roll=0.0f, pitch=0.0f;

float  ax = 0.0f,
			 ay = 0.0f,
			 az = 0.0f;

//mma data ready
extern uint32_t DATA_READY;

//initializes mma8451 sensor
//i2c has to already be enabled
int init_mma()
{
	  //check for device
		if(i2c_read_byte(MMA_ADDR, REG_WHOAMI) == WHOAMI)	{
			
		  Delay(100);
		  //turn on data ready irq; defaults to int2 (PTA15)
		  i2c_write_byte(MMA_ADDR, REG_CTRL4, 0x01);
		  Delay(100);
		  //set active 14bit mode and 100Hz (0x19)
		  i2c_write_byte(MMA_ADDR, REG_CTRL1, 0x01);
				
		  //enable the irq in the NVIC
		  //NVIC_EnableIRQ(PORTA_IRQn);
		  return 1;
		}
		
		//else error
		return 0;
}

void read_fake_xyz(){
  acc_X = X_ACC_2;
  acc_Y = Y_ACC_2;
  acc_Z = Z_ACC_2;
}

void update_previous_acc(){
  p_acc_X = acc_X;
  p_acc_Y = acc_Y;
  p_acc_Z = acc_Z;
}

void read_full_xyz()
{
	int i;
	uint8_t data[6];
	
	i2c_start();
	i2c_read_setup(MMA_ADDR , REG_XHI);
  
  for(i=0;i<2;i++){
			data[i] = i2c_repeated_read(0);
  }
  
	acc_X = (int16_t)((data[0]<<8) | (data[1]<<2));
  //don't bother reading the rest if acc_X is same.
  if( abs(acc_X - p_acc_X) < THRESH ){
    i2c_repeated_read(1);
    return;
  }
	
	for( ;i<5;i++)	{
			data[i] = i2c_repeated_read(0);
	}
  data[5] = i2c_repeated_read(1);

	acc_X = (int16_t)((data[0]<<8) | (data[1]<<2));
	acc_Y = (int16_t)((data[2]<<8) | (data[3]<<2));
	acc_Z = (int16_t)((data[4]<<8) | (data[5]<<2));

}

void read_xyz(void)
{
	// sign extend byte to 16 bits - need to cast to signed since function
	// returns uint8_t which is unsigned
	acc_X = (int8_t) i2c_read_byte(MMA_ADDR, REG_XHI);
	//Delay(100);
	acc_Y = (int8_t) i2c_read_byte(MMA_ADDR, REG_YHI);
	//Delay(100);
	acc_Z = (int8_t) i2c_read_byte(MMA_ADDR, REG_ZHI);

}

void convert_xyz_to_roll_pitch(void) {
  
  //don't recalculate if accs haven't changed.
  if(  (acc_X-p_acc_X) < THRESH 
    && (acc_Y-p_acc_Y) < THRESH 
    && (acc_Z-p_acc_Z) < THRESH){
    return;
  }
  
  
  ax = acc_X / COUNTS_PER_G,
  ay = acc_Y / COUNTS_PER_G,
  az = acc_Z / COUNTS_PER_G;
  
  /*
  ax = acc_X >> G_SHIFT,
  ay = acc_Y >> G_SHIFT,
  az = acc_Z >> G_SHIFT;
  */
	
  
	roll = atan2_approximation1(ay, az)*RAD_TO_DEGREES;
	pitch = atan2_approximation1(ax, sqrt_approx(ay*ay + az*az))*RAD_TO_DEGREES;
  
  update_previous_acc();
}

float atan2_approximation1(float y, float x)
{
  //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
  //Volkan SALMA
 
  //const float ONEQTR_PI = M_PI / 4.0;
	//const float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition

	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = QTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = QTR_PI;
	}
  angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return(angle );

}

float sqrt_approx(float S){
  //babylonian method.
  //a should be val close to real sqrt.
  float a = 1.0f;
  int i = 0;//number of sig figs
  for(i=0;i<3;i++){
    a = 0.5f*(a+S/a);
  }
  
  return a;
}

float arctan_approx(float x)
{
  return x - x*x*x/3.0f + x*x*x*x*x/5.0f;
}

//mma data ready irq
// void PORTA_IRQHandler()
// {
// 	NVIC_ClearPendingIRQ(PORTA_IRQn);
// 	DATA_READY = 1;	
// }
