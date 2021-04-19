









#include <stdio.h>
#include  "stm32.h"

#define success 1
#define failed	0
#define dev_id    0x7c
#deine slave_addr 0x5a  //assume motion sensor

uint8_t sensor_data[100]= {0};
uint8_t final_data = 0;

typedef enum
{
  Active,
  sleep,
}state_t;

state_t state = Active;

void system_clk_init()
{
    clock_ctrl_reg.HSEBYP = 0;		//disable the external clock
    clock_ctrl_reg.HSION = 1;		//set internal 8mhz rc osc as sys clk
    while(!(clock_ctrl_reg.HSIRDY << 2) && 0xFFFF);	//wait until the system clock set
}

void enter_sleep_mode()
{
     system.control_reg.SLEEPONEXIT = 0;	//by clearing this bit make system wake up from sleep mode when interrupt occur
     wait_for_interrupt();			//To put the system on sleep mode

}

void Timer_init()
{
    RCC_APB1ENR |= (1 >> TIM14_EN)		//To set the Timer 14 interrupt
    TIM14.CR1.UDIS = 0;				//Enable the update event
    TIM14.EVR.UW = 1;				//Enable the even interrupt generation 
    TIM14.PSC = 0x8;				//Prescaler 1:8  (8MHZ/8 = 1MHZ)
    TIM14.CNT = 0x9D;				//Count 157 to make timer value 20.096ms
    TIM14.DIER.UIE = 1;				//enable the interrupt while counter overflow 
    NVIC_EnableIRQ(TMR14_IRQn);			//add timer interrupt into the vector table
    NVIC_SetPriority(TMR14_IRQn,1);		//gives interrupt priority	
}

void I2C_init()
{
   GPIO_PINCONFIG(PIN_9,SDA);
   GPIO_PINCONFIG(PIN10, SCL);
   I2C2.TIMINGR = (uint32_t)0x00B01A4B;		//Fast mode @400khz with I2CCLK= 48MHz,rise time= 140ns, fall time= 40ns
   I2C2.CR1 = I2C_CR1_PE;			//Periph enable
   I2C2.CR2 = I2C_CR2_AUTOEND | (1 << 16) | (I2C1_OWNADD << 1) //assume slave address = 0x5A, write address, 1 byte to tx
}

uint8_t I2C_transmit(uint8_t *data)
{
   if((I2C2.ISR & I2C2_ISR_TXE)== I2C2_ISR_TXE)
     {
	I2C2.TXDR = *data;
	I2C2.CR2 |= I2C_CR2_START;
	return success;
      }
   else
      {
        return failed;
      }
}

uint8_t I2C_receive((uint8_t *rxdata)
{
    if((I2C2.ISR & I2C2_ISR_RXNE)== I2C2_ISR_TXE)
       {
	 *rxdata = I2C2.RXDR;
	 return success;
       }
     else
       {
         return failed;
       }

}

uint8_t sensor_init()
{
	uint8_t dev_id_t = 0;
	while(I2C_transmit(slave_addr)!=1);
	while(I2C_receive(&dev_id_t)== dev_id;		// wait until sensor responds with correct device id
}

void sample_sensor_data()
{
    uint32_t temp = 0;
    for(int i=0; i<100; i++)
     {
     	I2C_receive(sensor_data[i]);
	temp += sensor_data[i];
     }
	
     final_data = temp / 100;

}   

/*Interrupt service rouine for timer interrupt*/ 

void TIM14_ISR()
{
	 state = Active;   
}


void main()
{
	system_clk_init();
        I2C_init();
	Timer_init();
	sensor_init();
	
	while(1)
	{
	  switch(state)
	    {
		case Active:
			{
			  sample_sensor_data();
			   state = sleep;
			}
			  break;

		case sleep:
			{
			   enter_sleep_mode();
			}
			  break;
			  
		default:
			state = sleep;
			break;
	   }


	
