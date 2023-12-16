
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include <stdbool.h>




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
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
   //I2C0_MTPR_R = 0x14;                                        						// 8) configure for 100 kbps clock
        
}

void PortH_Init(void){
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTH_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTH_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}

// This function initializes Port M pins for push button input
void PortM_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; // Activate the clock for Port M
while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){}; // Allow time for clock to stabilize
GPIO_PORTM_DIR_R &= ~0b00000001; // Set PM0 as input, for reading if the button is pressed or not
GPIO_PORTM_DEN_R |= 0b00000011; // Enable PM0 and PM1 for start and stop buttons
return;
}

// This declares an integer variable "count" and initializes it to 0
int count = 0;

// This function spins a motor in a clockwise direction by a specified angle and at a specified speed.
// It loops through the specified angle and sets the GPIO_PORTH_DATA_R value to a binary value that causes the motor to turn in a clockwise direction
// It waits for a specified number of clock cycles using the SysTick_Wait function after each change in the GPIO_PORTH_DATA_R value
// This loop increments the variable "count" after each iteration.
void spinCW(int speed,int angle){
	for(int i=0; i<angle; i++){ 
		GPIO_PORTH_DATA_R = 0b00001100; // Set PH2 and PH3 pins high
		SysTick_Wait(speed); // Wait for specified clock cycles
		GPIO_PORTH_DATA_R = 0b00000110; // Set PH2 and PH3 pins low and PH1 and PH0 pins high
		SysTick_Wait(speed); // Wait for specified clock cycles
		GPIO_PORTH_DATA_R = 0b00000011; // Set PH0 and PH1 pins low and PH2 and PH3 pins high
		SysTick_Wait(speed); // Wait for specified clock cycles
		GPIO_PORTH_DATA_R = 0b00001001; // Set PH1 and PH2 pins high and PH0 and PH3 pins low
		SysTick_Wait(speed); // Wait for specified clock cycles
		count++; // Increment count by 1
	}
}

// This function spins a motor in a counterclockwise direction
// It loops through 512 iterations and sets the GPIO_PORTH_DATA_R value to a binary value that causes the motor to turn in a counterclockwise direction
// It waits for a specified number of clock cycles using the SysTick_Wait function after each change in the GPIO_PORTH_DATA_R value
void spinCCW(int speed){
	for (int i=0; i<512; i++){
		GPIO_PORTH_DATA_R = 0b00001001; // Set PH1 and PH2 pins high and PH0 and PH3 pins low
		SysTick_Wait(speed); // Wait for specified clock cycles
		GPIO_PORTH_DATA_R = 0b00000011; // Set PH0 and PH1 pins low and PH2 and PH3 pins high
		SysTick_Wait(speed); // Wait for specified clock cycles
		GPIO_PORTH_DATA_R = 0b00000110; // Set PH2 and PH3 pins low and PH1 and PH0 pins high
		SysTick_Wait(speed); // Wait for specified clock cycles
		GPIO_PORTH_DATA_R = 0b00001100; // Set PH2 and PH3 pins high
		SysTick_Wait(speed); // Wait for specified clock cycles
	}
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

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}



// This function sets GPIO_PORTH_DATA_R to a binary value that causes pins PH2 and PH3 to turn on and off in a sequence
// It waits for 100000 clock cycles using SysTick_Wait function after each change in the GPIO_PORTH_DATA_R value
// This loop runs 16 times and increments the variable "count" after each iteration.
void startFunction() 
{
    for(int i=0; i<16; i++){ 
        GPIO_PORTH_DATA_R = 0b00001100; // Set PH2 and PH3 pins high
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        GPIO_PORTH_DATA_R = 0b00000110; // Set PH2 and PH3 pins low and PH1 and PH0 pins high
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        GPIO_PORTH_DATA_R = 0b00000011; // Set PH0 and PH1 pins low and PH2 and PH3 pins high
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        GPIO_PORTH_DATA_R = 0b00001001; // Set PH1 and PH2 pins high and PH0 and PH3 pins low
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        count++; // Increment count by 1
    }
}

// This function sets GPIO_PORTH_DATA_R to a binary value that turns off all the pins of the GPIO port
// It waits for 100000 clock cycles using SysTick_Wait function after each change in the GPIO_PORTH_DATA_R value
// This loop runs 16 times and increments the variable "count" after each iteration.
void stopFunction() 
{
    for(int i=0; i<16; i++){ 
        GPIO_PORTH_DATA_R = 0b00000000; // Set all pins low
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        GPIO_PORTH_DATA_R = 0b00000000; // Set all pins low
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        GPIO_PORTH_DATA_R = 0b00000000; // Set all pins low
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        GPIO_PORTH_DATA_R = 0b00000000; // Set all pins low
        SysTick_Wait(100000); // Wait for 100000 clock cycles
        count++; // Increment count by 1
    }
}
//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	PortH_Init();
	PortM_Init();
	I2C_Init();
	UART_Init();
	
	/ A boolean variable to represent whether a button has been pressed or not
bool button_press = false;

// An integer variable to indicate whether the stop indicator is on or off (1 = on, 0 = off)
int stopindicator = 1;

// A boolean variable to indicate whether the program is running or not
bool isRunning;

// Infinite loop to keep the program running
while(1){
	
	// Used to measure the BUS speed on the AD2
	/*while(1){
		GPIO_PORTN_DATA_R=0b00001111;
		SysTick_Wait(50000);
		GPIO_PORTN_DATA_R=0b00000000;
		SysTick_Wait(50000);
	}*/
 		
	// Display a message to indicate that the program is running
	UART_printf("Running Program \r\n");

	// Set an integer variable to hold a number and print a message that includes the number
	int mynumber = 1;
	sprintf(printf_buffer,"2DX3 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);

	// Get the sensor ID and print it
	status = VL53L1X_GetSensorId(dev, &wordData);
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Boot the ToF sensor and wait for it to complete booting
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
    SysTick_Wait10ms(10);
	}

	// Flash all four LEDs to indicate that the ToF chip is booted
	FlashLED1(1);
	FlashLED2(1);
	FlashLED3(1);
	FlashLED4(1);

	// Display a message to indicate that the ToF chip is booted and awaiting input
	UART_printf("ToF Chip is Booted!\r\n Awaiting. . . . \r\n");

	// Clear any interrupt that may have occurred
	status = VL53L1X_ClearInterrupt(dev); 

	// Initialize the sensor and check for any errors
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
}
	

 // Start Ranging by calling VL53L1X_StartRanging function
	status = VL53L1X_StartRanging(dev);

	// Wait for a button press (input on Port M Pin 0) to stop the program and perform measurements
	int x = 0;
	while((GPIO_PORTM_DATA_R&0b00000001) == 0){
	stopindicator = 0;
}

	// If stop button was pressed, enter loop to perform ToF sensor measurements
	if(stopindicator==0){
	// Initialize variables
	double angle = 0;
		// Perform 32 measurements
for(int i = 0; i < 32; i++) {
	// Wait until the ToF sensor's data is ready
	while (dataReady == 0){
		status = VL53L1X_CheckForDataReady(dev, &dataReady);
		VL53L1_WaitMs(dev, 5);
	}
	// Flash LED to indicate data is ready
	FlashLED4(1);
	dataReady = 0;
	
	// Read the data values from ToF sensor
	status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	status = VL53L1X_GetDistance(dev, &Distance);					
	status = VL53L1X_GetSignalRate(dev, &SignalRate);
	status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
	status = VL53L1X_GetSpadNb(dev, &SpadNum);
	
	// Flash LED to indicate measurements are done
	FlashLED4(10);
	
	status = VL53L1X_ClearInterrupt(dev); 
	
	// Print the resulted readings to UART
	sprintf(printf_buffer,"Measure %u %f\r\n",Distance,angle);
	UART_printf(printf_buffer);
	
	// Increase the angle of the motor for each measurement and rotate the motor
	x +=1;
	angle += 11.25;
	spinCW(100000,16);
}

// Rotate the motor back to its initial position
spinCCW(100000);

// Stop ranging and reset stopindicator
VL53L1X_StopRanging(dev);
stopindicator =1;
}
}


