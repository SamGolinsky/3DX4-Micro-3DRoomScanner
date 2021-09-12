//Samuel Golinsky
//400176564
//golinsks
//bus speed 24MHz
//distance status LED PF0

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "functions.h"
#include "vl53l1x_api.h"
#include "uart.h"
#include "onboardLEDs.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

int status=0;
uint16_t	dev=0x52;
volatile int IntCount;
//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void PortL_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;		              // activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	        // allow time for clock to stabilize
  GPIO_PORTL_DIR_R = 0x0;                        		// Enabled as digital input
	GPIO_PORTL_DEN_R = 0x1;
	return;
	}

void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	
	GPIO_PORTF_DIR_R = 0b10001;        							 							
  GPIO_PORTF_DEN_R = 0b10001;
	return;
}

void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;			
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	
	GPIO_PORTM_DIR_R |= 0xFF;        								   								
  GPIO_PORTM_DEN_R |= 0xFF;        																															   									
	return;
}


void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                   // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0
    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}

void aquire_send_data(void){
	uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;
	for(int i = 0; i < 32; i++) { //32 scans for 1 horintal scan
		
		while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);
		status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		sprintf(printf_buffer,"%u, %u\n", Distance, RangeStatus); //sending data using UART
		UART_printf(printf_buffer);
		
		GPIO_PORTF_DATA_R = 0b1;
		
		for(int i=0; i<16; i++){ //16 corresponds to 11.25deg whhere 11.25 * 32 = 360 deg
			GPIO_PORTM_DATA_R = 0b00001100;
			SysTick_Wait10ms(1); 
			GPIO_PORTM_DATA_R = 0b00000110;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b00000011;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b00001001;
			SysTick_Wait10ms(1);}		
		
		GPIO_PORTF_DATA_R = 0b0;
	}
	
  
	for(int i=0; i<=512; i++){
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10ms(1); 
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);}	
	
	GPIO_PORTM_DATA_R = 0b0; //motor off
	GPIO_PORTF_DATA_R = 0b0;
	
}

int main(void){
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortL_Init();
	PortF_Init();

/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);
	
	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	//UART_printf("ToF Chip Booted!\r\n");
 	//UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	//Status_Check("SensorInit", status);

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	//Status_Check("StartRanging", status);
	
	GPIO_PORTF_DATA_R = 0b10000;

	
	int count=0;
	while(1){ //constantly polling for button press
		if((GPIO_PORTL_DATA_R&0b1)==0 && GPIO_PORTM_DATA_R == 0b0){ //only acquires data when active low button is pressed and motor is off
			aquire_send_data();
			count++;
		}
		if(count==10){VL53L1X_StopRanging(dev);} //after 10 button presses turn ToF off
	}
}
