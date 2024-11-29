// ########################################################
// 1. Project Name : JTS & WOWSYS
// 2. Developer : Gyu-Han. Lee
// 5. Development Period : 2022. 04. 25 ~ 2021. 05. 20
// 6. Microcontroller Chip : Atmel ATmega64A-16AU 64pin
// 7. Development Tool : IAR Embedded Workbench 7.20
// ########################################################
#include <ina90.h>
#include <iom64a.h> 
#include <stdio.h>

#define TRUE	1
#define FALSE	0

#define SET	1
#define CLR	0

#define STX	0x02
#define ETX	0x03

#define CR	0x0D
#define LF  	0x0A

#define OS_ENTER_CRITICAL()  __disable_interrupt();
#define OS_EXIT_CRITICAL()   __enable_interrupt();

__flash unsigned int  MaskW[16] = { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
                                    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
__flash unsigned char MaskB[8]  = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

__flash int CompDelay[6] = {30, 60, 90, 120, 150, 180};

__flash unsigned int ManualSpeed[10] = {102, 204, 306, 408, 510, 612, 714, 816, 918, 1023};
//==============================================
#define _BV(bit) 		(1 << (bit))
#define cbi(sfr, bit)	(sfr) &= ~_BV(bit)
#define sbi(sfr, bit)	(sfr) |= _BV(bit)
//==============================================

//######### Key Input Define ###########
#define Kmode			0x01        	// Mode Key   
#define Kup         	0x02         	// Up Key
#define Kdown       	0x04			// Down Key
#define Kenter			0x08			// Enter Key
#define Kreset			0x0C			// Reset Key when Alarm Display
#define Khand            0x0A           // Pump run by Hand control

//######### Time Delay ###########
void Delay_ns(unsigned int time_ns)
{
	register unsigned int i;

     for(i = 0; i < time_ns; i++) {     // 4 cycle +
        asm(" PUSH  R0 ");     		// 2 cycle +
        asm(" POP   R0 ");       		// 2 cycle = 8 cycle = 500 ns for 16MHz
     }
}

void Delay_us(unsigned int time_us)		/* time delay for us */
{
     register unsigned int i;

     for(i = 0; i < time_us; i++) {     // 4 cycle +
        asm(" PUSH  R0 ");     		// 2 cycle +
        asm(" POP   R0 ");       		// 2 cycle +
        asm(" PUSH  R0 ");       		// 2 cycle +
        asm(" POP   R0 ");       		// 2 cycle +
        asm(" PUSH  R0 ");       		// 2 cycle +
        asm(" POP   R0 ");	     	// 2 cycle = 16 cycle = 1 us for 16MHz
     }
}

void Delay_ms(unsigned int time_ms)       	/* time delay for ms */
{
     register unsigned int i;

     for(i = 0; i < time_ms; i++) {
        Delay_us(250);
        Delay_us(250);
        Delay_us(250);
        Delay_us(250);
     }
}

//######### EEPROM Read & Write ###########
void EE_PUT(unsigned int addr, unsigned char data)
{
     unsigned char cSREG;

     while(EECR & 0x02); 		// Poll EEWE
     cSREG = SREG;

     asm (" CLI            ");	// Clear Global Interrupt Flag
     EEAR  = addr;
     EEDR  = data;
     EECR |= 0x04;  			// Set EEMWE
     EECR |= 0x02;  			// Set EEWE
     SREG = cSREG;
}

unsigned char EE_GET(unsigned int addr)
{
     register unsigned char data;
     while(EECR & 0x02); 		// Poll EEWE
     EEAR  = addr;
     EECR |= 0x01;  			// Set EERE
     data  = EEDR;   			// Get 1 Byte
     return data;
}

//######### ADC ###########
int ad_conversion(char ch)   		// Single Ended Input A/D Conversion
{
  	// ADC0 = NTC Temperature Sensor
  	// ADC1 = Pressure Sensor
	ADMUX = (ch & 0x03);

     ADCSRA |= 0x40; 			// ADC Start

     while((ADCSRA & 0x10) != 0x10);
     return(ADC);
}

char boggle = 1;

#include "Initial.h"
#include "Display.h"
#include "Control.h"

void main( void )
{
	int i;	
	char toggle = 1;
	
	OS_ENTER_CRITICAL();			// global interrupt disable
	
	// -------------------------------------------------------------------------------------------
	// MCU initialize
	// -------------------------------------------------------------------------------------------
	SFIOR = 0x04;		// PUD = 1, Pull-up Disable
	
	WDTCR = 0x18;		// Watchdog Change Enable, Watchdog System Reset Enable
	WDTCR = 0x00;		// 
	
	// -------------------------------------------------------------------------------------------
	// Port initialize
	// -------------------------------------------------------------------------------------------
	DDRA  = 0x80;	// Key Input & Digital Input
	PORTA = 0x00;	// PORTA.0(0) : Mode Key
				// PORTA.1(0) : Up Key
				// PORTA.2(0) : Down Key
				// PORTA.3(0) : Enter Key 
				// PORTA.4(0) : Run/Stop Input
				// PORTA.5(0) : Tank Level 1 Input
				// PORTA.6(0) : Tank Level 2 Input
				// PORTA.7(1) : Non Connect
	
	DDRB  = 0xFD;	// SPI Download
	PORTB = 0x00;	// PortB.0(1) : Non Connection
				// PortB.1(0) : SCK Signal
				// PortB.2(1) : Relay 5 Out for Tank Level Alarm 1
				// PortB.3(1) : Relay 4 Out for Solenoid Valve 2    --> for Pump(200W) output
				// PortB.4(1) : Relay 3 Out for Solenoid Valve 1    --> for Heater(200W) output
				// PortB.5(1) : Relay 2 Out for Fan output
				// PortB.6(1) : Relay 1 Out for Pump(200W) output   --> for Solenoid Valve 2
				// PortB.7(1) : Relay 0 Out for Heater(200W) output --> for Solenoid Valve 1
				
	DDRC  = 0xFF;	// FND Data Bus
	PORTC = 0x00;	// PORTC.0(1) : FND-a
				// PORTC.1(1) : FND-b
				// PORTC.2(1) : FND-c
				// PORTC.3(1) : FND-d
				// PORTC.4(1) : FND-e
				// PORTC.5(1) : FND-f 
				// PORTC.6(1) : FND-g
				// PORTA.7(1) : FND-dp
	
	DDRD  = 0xFF;	// FND Digit Select & Digital Input
	PORTD = 0x00;	// PortD.0(1) : FND 0 Selection
				// PortD.1(1) : FND 1 Selection
				// PortD.2(1) : FND 2 Selection
				// PortD.3(1) : FND 3 Selection
				// PortD.4(1) : LED 0 for Run
				// PortD.5(1) : LED 1 for Warning
				// PortD.6(1) : LED 2 for Alarm
				// PortD.7(1) : LED 3 for Reserved
	
	DDRE  = 0xFE;	// SPI Download & PWM Output
	PORTE = 0x00;	// PortE.0(0) : MOSI - Data Input
				// PortE.1(1) : MISO - Data Output
				// PortE.2(1) : Non Connect
				// PortE.3(1) : PWM 1 Out
				// PortE.4(1) : PWM 2 Out
				// PortE.5(1) ~ PortE.7(1) : Reserved
	
	DDRF  = 0xFC;	// Analog Input and Relay Output
	PORTF = 0x00;	// PortF.0(0) : Temperature Sensor
				// PortF.1(0) : Pressure Sensor 
				// PortF.2(1) : Reserved Analog In 2
				// PortF.3(1) : Reserved Analog In 3
				// PortF.4(1) : 
				// PortF.5(1) : Relay 6 Out for Tank Level Alarm 2 
				// PortF.6(1) : Relay 7 Out for Heater Run/Stop 
				// PortF.7(1) : Relay 8 Out for Total Alarm 
	
	DDRG  = 0x1F;	// Buzzer
	PORTG = 0x00;	// PortG.0(1) : Buzzer Output
				// PortG.1(1) ~ PORTG.4 : Non Connect
		
	// -------------------------------------------------------------------------------------------
	// Timer/Count initialize
	// -------------------------------------------------------------------------------------------
	// Timer/Count 0 for FND display
	TCCR0 = 0x06;		// Normal Mode(for FND Drive), 256 / 16MHz = 16us
	TCNT0 = 0;
	OCR0  = 80;		// 16us * 80 = 1.3ms Task
	
	// Timer/Count 1 for Main Control
	TCCR1A = 0x00;		// Normal Mode(for Processing and Control)
	TCCR1B = 0x03;		// 64 / 16MHz = 4us
	TCNT1 = 0;	
	OCR1A = 25000;		// 4us * 25000 =  100ms Task
	
	// Timer/Count 3 for PWM Output
	TCCR3A = 0xA3;		// OC3A & OC3B(PWM Output)
	TCCR3B = 0x0A;		// Fast PWM Mode(10bit), 8 prescale(16MHz / 8 = 2MHz)
	TCNT3 = 0;		
	OCR3A = 0;		// Motor Speed 1 PWM Output
	OCR3B = 0;		// Motor Speed 2 PWM Output
			
	TIMSK = 0x12;		// Timer/Counter 1, 0 Output Compare A 
	
	// -------------------------------------------------------------------------------------------
	// ADC initialize
	// -------------------------------------------------------------------------------------------
	ADCSRA = 0x85; 	// ADC Enable
   					// ADC Prescaler = 32
   	ADCSRB = 0x00;		// Free Running Mode, ADC0~ADC4
	ADMUX = 0x00;  	// Ref. Voltage = AREF pin
                        	// ADLAR = 0
   	
   	i = 0;
	i = ad_conversion(0);	// Temperature
	i = ad_conversion(1);	// Pressure
	i++;
		
	Delay_ms(100);
	
	// -------------------------------------------------------------------------------------------
	// Etc. initialize
	// -------------------------------------------------------------------------------------------
 	if(EE_GET(33) != 0x17){
		EEPROM_Init(); 		// EEPROM, RTC Initialization without Birthday
		EE_PUT(33,0x17);		
	}
	
	for(i=0;i<8;i++){			// All LED Blink during 2 second
		if(toggle) PORTD |=  0xF0;
		else		 PORTD &= ~0xF0;
		Delay_ms(250);
		toggle ^= 1;
	}
	
	Delay_ms(100);
 	VARIABLE_Init();	// Variable iniInitializationtialize
	
	WDTCR = 0x18;		// Watch Dog Enable
	WDTCR = 0x0E;		// Period = 1s
	 
	OS_EXIT_CRITICAL();   			// Global Interrupt Enable
	
	MainMenu();
}
