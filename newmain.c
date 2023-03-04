   /*
 * Project: The Watch Tower
 * Author: Tyler Scotti
 * Purpose: Uses a PIC16F1829 and will output:
    * 1. The surrounding room temperature 
    * 2. The amount of light in the area
    * 3. It will determine how far an object is from the UltraSonic Sensor
    * 4. It will buzz if something is closer than 15cm away from the buzzer    * 
    * all onto an 2x16 LCD Display using the I2C interface on PIC
 *
 * Created on November 28, 2022, 11:45 PM
 */
#include <xc.h>  
#include <math.h>
#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>
#include  "i2c.h"
#include  "i2c_LCD.h"
#include <pic16f1829.h>

// PIC16F1829 Configuration Bit Settings

// 'C' source line config statements

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define BAUD 9600
#define FOSC 16000000L
#define DIVIDER ((int)(FOSC/(16UL * BAUD) -1))  //Should be 25 for 9600/4MhZ
#define NINE_BITS 0
#define SPEED 0x4
#define RX_PIN TRISC5
#define TX_PIN TRISC4
#define _XTAL_FREQ 16000000.0    /*for 16mhz*/
#define I2C_SLAVE 0x27


void setup_comms(void);
void putch(unsigned char);
unsigned char getch(void);
unsigned char getche(void);
void RA5Blink(void);  
void pinConfig(void); 
void I2C_LCD_Command(unsigned char,unsigned char);
void I2C_LCD_SWrite(unsigned char,unsigned char *, char);
void I2C_LCD_Init(unsigned char);
void I2C_LCD_Pos(unsigned char,unsigned char);
unsigned char I2C_LCD_Busy(unsigned char);
int calc_distance(void);


 /*Global Vars ALL MY ADDITIONS */ 
unsigned int Pval, Temp, FVval, Phval, light;  
double Ctemp, FTemp;  
float Temperature, Voltage, distance; //Temp is an unsigned int  

int main(int argc, char** argv) {
    unsigned int Time, j,i;
    int Dist; 
    T1CON =   0X25;     //set up Tmr1 for gate control Pin RA4
    T1GCON =  0XC0;
    TRISA5 = 0;     //LED
    TRISA2 = 0;     //Use this as Ping trigger
    TRISA4 = 1;     //the gate control pin
    RA5 = 1;
    APFCON0 = 0x84; //This sets pins RC5 and RC4 as RX & TX on pic16f1829
    OSCCON  = 0x7A; /* b6..4 = 1101 = 4MHz  1111 7A for 16MHz*/
    ANSELA  = 0;
    ANSELC  = 0x00;
    TRISC   = 0xFF;     //all PortC as input?
    LATC    = 0x00;   /* zero out */
    INTCON  =0;	// purpose of disabling the interrupts.
    char  Sout[16];
	unsigned char * Sptr;
    FVRCON = 0xA2;
    TSEN = 1; //enable module
    TSRNG = 0; //Select lower range that will work with void as low as 1.8V
    //LCD Output Variables    
	Sptr = Sout; 
    i2c_Init();				// Start I2C as Master 100KH
	I2C_LCD_Init(I2C_SLAVE); //pass I2C_SLAVE to the init function to create an instance
    setup_comms();	// set up the USART - settings defined in usart.h 
//************************************ LOOP ************************
    while (1){
//  Start Ping Sensor
        RA2 = 0;
        TMR1H = 0;
        TMR1L = 0;
        RA2 = 1;
        for (i=0; i<3; i++);    //with 16mhz clock this gives 11.68usec delay
        RA2 = 0;
        for(i=0;i<30000; i++) continue; //wait long enough for max range 68msec
        Time = ((TMR1H<<8)| (TMR1L));
//   The Math is based on a system clock of 16MHz prescaled by 1:4 and Fosc/4
//   So:  1/(16/4/4) = 1 usec per tick
        //Ping Sensor 
        Dist =  Time / 58.0;
        
        RA5 ^= 0x01;    //Toggles the LED to help with debugging
        for (j=0; j<0x50; j++);  //  Add a bit of delay here        
             
// Get set up for A2D  
    ADCON1 = 0xC0; //Right justify and Fosc/4 and Vss and Vdd references  
    
/*Pot Read  it is on Chanel AN11  Thus the value in the ADCON0 reg = 0x2D */   
       ADCON0 = 0x2D; // set up for the pot input channel AN11
       __delay_ms(10); //Allow cap to recharge  
       ADGO = 1; // initiate conversion on the selected channel  
       while(ADGO)continue;  
       Pval = (unsigned)((ADRESH<<8)+(ADRESL)); //Store 10 bits into Pval, 8 + 2

    /*Fixed Voltage Reference Read*/     
       ADCON0 = 0x7D; // set up for the FVR input channel = 11111  
       __delay_ms(10); //Allow cap to recharge  
       ADGO = 1; // initiate conversion on the selected channel  
       while(ADGO)continue;  
       FVval = ((ADRESH<<8)+(ADRESL)); //Store 10 bits into Pval, 8 + 2
    
           /*Photo Read & converting to conditions*/     
       ADCON0 = 0x1D; // set up for the FVR input channel = 11111  
       __delay_ms(10); //Allow cap to recharge  
       ADGO = 1; // initiate conversion on the selected channel  
       while(ADGO)continue;  
       Phval = ((ADRESH<<8)+(ADRESL)); //Store 10 bits into Pval, 8 + 2
       if(Phval<200)
       {
           light = 0; //No Light
       }
       else if (Phval>=200&&Phval<500)
       {
           light = 1; //Dim Light
       }
        else if (Phval>=500&&Phval<800)
       {
           light = 2; //Ambient
       }
        else
        {
            light = 3; //Bright
        }
                
    /*Internal Temperature Read*/
       ADCON0 = 0x19; // set up for the temp input channel = 11101  
       __delay_ms(10); //Allow cap to recharge  
       ADGO = 1; // initiate conversion on the selected channel  
       while(ADGO)continue;  
       Temp = ((ADRESH<<8)+(ADRESL)); //Store 10 bits into Pval, 8 + 2
      Voltage = (float)(Temp*5.0)/1024;
      Temperature = (Voltage-.5)/0.01;    
    /*Write to Terminal + Temperature Math*/   
       Ctemp = (((Temp*5)/1024.0)-0.5)/0.1; 
       FTemp = (Temperature*1.8)+32;
        
       //Buzzer will be plugged into RA3 which is defaulted to be ON
       if(Dist>=15){
           RA3 = 0; //Turning off buzzer if Dist is 15 or more cm 
       }
       else if (Dist<15){
           RA3 = 1; // Redundant - but turning OFF buzzer when Dist is less than 15
       }
        
        //LCD Prints
        I2C_LCD_Command(I2C_SLAVE, 0x01); //clear display
      __delay_ms(1000.0);
       I2C_LCD_Pos(I2C_SLAVE, 0x00); //Set position to start bottom line
       sprintf(Sout, "Distance = %dcm",Dist);
       I2C_LCD_SWrite(I2C_SLAVE, Sout, strlen(Sout));
       __delay_ms(1000.0);
       I2C_LCD_Pos(I2C_SLAVE, 0x40); //Set position to start bottom line
       sprintf(Sout, "Temp = %.0fF",FTemp);
       I2C_LCD_SWrite(I2C_SLAVE, Sout, strlen(Sout));
       __delay_ms(1000.0);
       I2C_LCD_Command(I2C_SLAVE, 0x01); //clear display
              sprintf(Sout, "Light = %d", light);
       I2C_LCD_SWrite(I2C_SLAVE, Sout, strlen(Sout));
       __delay_ms(1000.0);
       
    }
    RA5Blink();
    return (EXIT_SUCCESS);
}
void setup_comms(void){
	RX_PIN = 1;
	TX_PIN = 1;
	SPBRG = DIVIDER;
	RCSTA = (NINE_BITS|0x90);
	TXSTA = (SPEED|NINE_BITS|0x20);
        TXEN = 1;
        SYNC = 0;
        SPEN = 1;
        BRGH = 1;
        CREN = 1;
}

void putch(unsigned char byte)
{
	/* output one byte */
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;
}

unsigned char
getch() {
	/* retrieve one byte */
	while(!RCIF)	/* set when register is not empty */
		continue;
	return RCREG;
}

unsigned char
getche(void)
{
	unsigned char c;
	putch(c = getch());
	return c;
}


void pinConfig(void)  {  
 OSCCON = 0x6A; /* b6..4 = 1101 = 4MHz */  
 TXCKSEL = 1; // both bits in APFCON0 MUST BE 1 for 1829 0 for 1825  
 RXDTSEL = 1; /* makes RC4 & 5 TX & RX for USART (Allows ICSP)*/  
  
 TRISA = 0x10;  
 ANSELA =0x10;  
 TRISC = 0x0C; /* set as output */  
    ANSELC =0x0C; /* all ADC pins to digital I/O */  
    TRISB = 0xF0;
    ANSELB =0xF0;
    INTCON = 0; // purpose of disabling the interrupts.  
  }  

void RA5Blink(void)  {  
    RA5 ^= 0x01; // Toggles the LED to help with debugging  
    __delay_ms(200);   
}  
