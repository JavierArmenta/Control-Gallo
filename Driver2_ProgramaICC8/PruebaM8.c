//!Programa del sistema de control de los 2 motores ppales del gallo UAA
// Elaboro Dr. Luis Antonio Raygoza Pérez
// Fuse bits: high = 1101 1001 = D9
//			  low  = 1110 1111 = EF

#include <iom8v.h>
#include <macros.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>

//!---------------------------- CONEXIONES -------------------------------------------------
//                       ---------------\/---------------
//			RESET SPI  =|PC6(in)	      PC5(?) 	     |= SCL I2C
//			RX UART    =|PD0(in)	      PC4(?)		 |= SDA I2C
//			TX UART    =|PD1(out)	      PC3(out) 		 |= Led verde ?
//			Encoder M2 =|PD2(in INT0)     PC2(in) 		 |= Error o sobrecorriente drivers M1 y M2
//			Encoder M1 =|PD3(in INT1)     PC1(in ADC) 	 |= Corriente IM2
//			RPWM M1    =|PD4(out) 	      PC0(in ADC) 	 |= Corriente IM1
//			5v         =|VCC              GND	 		 |= GND
//			GND        =|GND              AREF	 		 |= 5V
//			XTAL 16Mhz =|PB6(in)	      AVCC	 		 |= 5V
//			XTAL 16Mhz =|PB7(in)	      PB5(in)		 |= SCK  SPI
//			LPWM M1    =|PD5(out)	      PB4(out)		 |= MISO SPI
//			Enable M1  =|PD6(out)	      PB3(in)	   	 |= MOSI SPI
//			RPWM M2    =|PD7(out)	      PB2(in pull up)|= SS   SPI
//			LPWM M2    =|PB0(out) 	      PB1(out) 	   	 |= Enable M2
//				 	     --------------------------------

//!---------------------------- DEFINICIONES ------------------------------------------
// ***** Declaracion de pines de control ***** //
#define LPWM1 (1 << 5)	//PD5	(salida)		   LPWM M1
#define RPWM1 (1 << 4)	//PD4	(salida) 		   RPWM M1
#define EN_M1 (1 << 6)	//PD6	(salida)		   Enable M1
#define LPWM2 (1 << 0)	//PB0	(salida) 		   LPWM M2
#define RPWM2 (1 << 7)	//PD7	(salida)		   RPWM M2
#define EN_M2 (1 << 1)	//PB1	(salida) 	   	   Enable M2
#define LEDV  (1 << 3)	//PC3	(salida) 		   Led verde ?

#define IM1   (1 << 0)  //PC0	(entrada ADC) 	   Corriente IM1
#define IM2   (1 << 1)  //PC1	(entrada ADC) 	   Corriente IM2
#define ERROR (1 << 2)  //PC2	(entrada) 		   Error o sobrecorriente drivers M1 y M2
#define ENCM2 (1 << 2)  //PD2	(entrada INT0)	   Encoder M2
#define ENCM1 (1 << 3)  //PD3	(entrada INT1)	   Encoder M1

#define SSSPI   (1 << 2)  //PB2	(entrada pull up)  SS   SPI
#define MOSISPI (1 << 3)  //PB3	(entrada)	   	   MOSI SPI
#define MISOSPI (1 << 4)  //PB4	(salida)		   MISO SPI
#define SCKSPI  (1 << 5)  //PB5	(entrada)		   SCK  SPI
#define SDAI2C  (1 << 4)  //PC4	(?)				   SDA I2C
#define SCLI2C  (1 << 5)  //PC5	(?)				   SCL I2C
#define RXUART  (1 << 0)  //PD0	(entrada)		   RX UART
#define TXUART  (1 << 1)  //PD1	(salida)		   TX UART

// ***** Declaracion de macros de control ***** //
//SALIDAS
#define SET_LPWM1	 	PORTD |=  LPWM1		  //prender LPWM1
#define CLR_LPWM1	 	PORTD &= ~LPWM1		  //apagar  LPWM1
#define SET_RPWM1	 	PORTD |=  RPWM1		  //prender RPWM1
#define CLR_RPWM1	 	PORTD &= ~RPWM1		  //apagar  RPWM1
#define SET_EN_M1	 	PORTD |=  EN_M1		  //prender EN_M1
#define CLR_EN_M1	 	PORTD &= ~EN_M1		  //apagar  EN_M1
#define SET_LPWM2	 	PORTB |=  LPWM2		  //prender LPWM2
#define CLR_LPWM2	 	PORTB &= ~LPWM2		  //apagar  LPWM2
#define SET_RPWM2	 	PORTD |=  RPWM2		  //prender RPWM2
#define CLR_RPWM2	 	PORTD &= ~RPWM2		  //apagar  RPWM2
#define SET_EN_M2	 	PORTB |=  EN_M2		  //prender EN_M2
#define CLR_EN_M2	 	PORTB &= ~EN_M2		  //apagar  EN_M2
#define SET_LEDV	 	PORTC |=  LEDV		  //prender LEDV
#define CLR_LEDV	 	PORTC &= ~LEDV		  //apagar  LEDV
//ENTRADAS
#define IF_ERROR		(PINC&ERROR)==ERROR
#define NO_ERROR		(PINC&ERROR)!=ERROR


//!---------------------------- DECLARACION DE VARIABLES GLOBALES ---------------------
int16_t  i=0;					    //contador
uint8_t  VMF=0;		                //indicador de direccion del motor fijo
uint8_t  VMD=0;		                //indicador de direccion del motor de giro
long     L=0;				   	    //temporizacion
uint8_t  Esclavo_ID=1, DatoSPI;
char     Haciendo=0;            //parte del ciclo que se encuentra haciendo con el Master
uint16_t idx=0;                 //numero de byte para transmision o recepcion

//!---------------------------- DECLARACION DE FUNCIONES DE USUARIO -------------------
void     Config(void);			    //configuracion inicial de perifericos
int16_t  ADCCH(char ch); 		    //realizar una conversion en el canal ch
void     VelMF(int vel); 		    //establecer la velocidad del motor fijo
void     VelMD(int vel); 		    //establecer la velocidad del motor direccion
uint8_t  SPI_SlaveTransfer(uint8_t Dato, char llamada);
void     delayms(long msec);		    //rutina de retardos

//!---------------------------- PROGRAMA PRINCIPAL ------------------------------------
void main(void)
{
 Config();

 while (1)
 {
  if(NO_ERROR)
  {
   SET_LEDV;
   SET_EN_M1;
   SET_EN_M2;
   delayms(1000);

   CLR_LEDV;
   delayms(1000);
   SET_LEDV;
   SET_LPWM1;
   CLR_RPWM1;
   CLR_LPWM2;
   CLR_RPWM2;
   delayms(1000);

   CLR_LEDV;
   delayms(1000);
   SET_LEDV;
   CLR_LPWM1;
   SET_RPWM1;
   CLR_LPWM2;
   CLR_RPWM2;
   delayms(1000);

   CLR_LEDV;
   delayms(1000);
   SET_LEDV;
   CLR_LPWM1;
   CLR_RPWM1;
   SET_LPWM2;
   CLR_RPWM2;
   delayms(1000);

   CLR_LEDV;
   delayms(1000);
   SET_LEDV;
   CLR_LPWM1;
   CLR_RPWM1;
   CLR_LPWM2;
   SET_RPWM2;
   delayms(1000);

   CLR_LEDV;
   delayms(1000);
   SET_LEDV;
   SET_LPWM1;
   CLR_RPWM1;
   SET_LPWM2;
   CLR_RPWM2;
   delayms(1000);

   CLR_LEDV;
   delayms(1000);
   SET_LEDV;
   CLR_LPWM1;
   SET_RPWM1;
   CLR_LPWM2;
   SET_RPWM2;
   delayms(1000);

   CLR_LEDV;
   CLR_EN_M1;
   CLR_EN_M2;
   CLR_LPWM1;
   CLR_RPWM1;
   CLR_LPWM2;
   CLR_RPWM2;
   delayms(1000);
  }


 }
}

//!------------------------------ FUNCIONES DEL USUARIO -------------------------------
void Config(void)
{
 //CLI();

 //PB0	(salida) 		   LPWM M2
 //PB1	(salida) 	   	   Enable M2
 //PB2	(entrada pull up)  SS   SPI
 //PB3	(entrada)	   	   MOSI SPI
 //PB4	(salida)		   MISO SPI
 //PB5	(entrada)		   SCK  SPI
 //PB6	XTAL1
 //PB7	XTAL2
 PORTB = 0b00000100;
 DDRB  = 0b00010011;

 //PC0	(entrada ADC) 	   Corriente IM1
 //PC1	(entrada ADC) 	   Corriente IM2
 //PC2	(entrada) 		   Error o sobrecorriente drivers M1 y M2
 //PC3	(salida) 		   Led verde ?
 //PC4	(?)				   SDA I2C
 //PC5	(?)				   SCL I2C
 //PC6	(entrada)		   RESET SPI
 PORTD = 0b00000000;
 DDRC  = 0b00001000;

 //PD0	(entrada)		   RX UART
 //PD1	(salida)		   TX UART
 //PD2	(entrada INT0)	   Encoder M2
 //PD3	(entrada INT1)	   Encoder M1
 //PD4	(salida) 		   RPWM M1
 //PD5	(salida)		   LPWM M1
 //PD6	(salida)		   Enable M1
 //PD7	(salida)		   RPWM M2
 PORTD = 0b00000000;
 DDRD  = 0b11110010;

 //------------- SPI
 SPCR = (1<<SPE);           // Enable SPI


 //-------------PWM 1 y 2 usando Timer1
 //TCNT1=0xFE00;
 //TCCR1A=0x00;
 //TCCR1B=0x02;  		  	  		//clk/8 para usar 9bits
 //TIMSK=0x1C;					//habilitar interrupciones compare A, B y overflow
 								//Si conteo desde TIMER1 hasta OVF
								//                                  clk/1	 clk/8     clk/64
								//14 bits TIM1=65536-16384=C000       976Hz
								//12 bits TIM1=65536-4096 =F000      3906Hz
								//10 bits TIM1=65536-1024 =FC00 	15625Hz  1953Hz    244Hz
								//9  bits TIM1=65536-512  =FE00	    31250Hz  3906Hz    448Hz
								//8  bits TIM1=65536-256  =FF00	    62500Hz  7812Hz    976Hz
								//y asi OCR=TIM1+vel
 //SEI();
}

//!-------------------------------
int16_t ADCCH(char ch)		    	//realizar una conversion en el canal ch
{
 int ADCres;
 ADMUX=0x40|ch;
 ADCSRA  = 0xD7;		   		//ADC enable, ADC start convertion, No conv por interrupcion, apagar bandera conv. completa
 		   				   		//No int enable, presc en 64 siguientes conversiones
 while((ADCSRA&0x10)!=0x10){}   //Polling conversion completa
 ADCres=ADC;            		//Read 8 low bits first (important)
 return ADCres;
}
//!--------------------------------
void VelMF(int vel) 		   //establecer la velocidad del motor
{
 if(vel!=0)
 {
  VMF=1;
  OCR1A=vel+0xFE00;	   				   //Actualizar timer1A
 }
 else	   	   				   //freno
 {
  PORTD|=0x05;		  	 		//motorI en stop PD2=PD0=1
  VMF=0;					   //indicador de freno
 }
}
//!--------------------------------
void VelMD(int vel) 		   //establecer la velocidad del motor de giro
{
 if(vel!=0)
 {
  if(vel<0)	   				   //en reversa
  {
   vel=-vel;
   VMD=2;
  }
  else	 					   //hacia adelante
  {
   VMD=1;
  }
  OCR1B=vel+0xFE00;			   //Actualizar timer1B
 }
 else	   	   				   //freno
 {
  PORTD|=0x18;				   //motor en freno PD4=PD3=1
  VMD=0;					   //indicador de freno
 }
}

//!------------------------------------------------ Transferencia SPI -----------------------------------------------------
uint8_t SPI_SlaveTransfer(uint8_t Dato, char llamada)
{
    //dato a Transferir
    SPDR=Dato;
    /*if(llamada==1)
    {
        //falta esperar a que el pin no sea cero debido a otro dispositivo en la red (checar, no creo que funcione)
        //Indicar Ready
        DDRB |= 0x04;  //poner como salida PB2 (INT0)
        PORTB&=~0x04;  //PB2=0 (INT0)
    }*/
    //Limpiar bandera counter overflow y contador para comenzar envio
	SPSR = (1<<SPIF);//USISR = (1<<USIOIF);

    //while((USISR&(1<<USIOIF))!=(1<<USIOIF)){}; //Esperar a que se termine la transaccion
    //while(!(SPSR & (1<<SPIF))); // Wait for reception complete
    while((SPSR&(1<<SPIF))!=(1<<SPIF)){}; //Esperar a que se termine la transaccion

    /*if(llamada==1)
    {
        //Indicar busy
        DDRB &= ~0x04; //poner como entrada PB2 (INT0)
        PORTB|= 0x04;  //Pull up PB2 (INT0)
    }*/
    //Leer dato recibido
    Dato=SPDR;
    return Dato;
}

//!--------------------------------
void delayms(long msec)
{
    long j;
    for(j=0;j<(msec*77);j++);

}
//------------------------------- FUNCIONES DE INTERRUPCION --------------------------
//interrupcion externa 0
#pragma interrupt_handler int0_isr:2
void int0_isr(void)
{ }
//interrupcion externa 1
#pragma interrupt_handler int1_isr:3
void int1_isr(void)
{ }
//timer 2 comparacion igual
#pragma interrupt_handler timer2_comp_isr:4
void timer2_comp_isr(void)
{ }
//timer 2 overflow
#pragma interrupt_handler timer2_ovf_isr:5
void timer2_ovf_isr(void)
{ }
//timer 1 input capture
#pragma interrupt_handler timer1_capt_isr:6
void timer1_capt_isr(void)
{ }
//timer 1 comparacion igual A
#pragma interrupt_handler timer1_compa_isr:7
void timer1_compa_isr(void)
{ }
//timer 1 comparacion igual B
#pragma interrupt_handler timer1_compb_isr:8
void timer1_compb_isr(void)
{ }
//timer 1 overflow
#pragma interrupt_handler timer1_ovf_isr:9
void timer1_ovf_isr(void)
{ }
//timer 0 overflow
#pragma interrupt_handler timer0_ovf_isr:10
void timer0_ovf_isr(void)
{ }
//SPI completo
#pragma interrupt_handler spi_stc_isr:11
void spi_stc_isr(void)
{ }
//UART byte recibido
#pragma interrupt_handler uart0_rx_isr:12
void uart0_rx_isr(void)
{ }
//UART registro de datos vacio
#pragma interrupt_handler uart0_udre_isr:13
void uart0_udre_isr(void)
{ }
//UART byte transmitido
#pragma interrupt_handler uart0_tx_isr:14
void uart0_tx_isr(void)
{ }
//ADC conversion completa
#pragma interrupt_handler adc_isr:15
void adc_isr(void)
{ }
//EEPROM listo
#pragma interrupt_handler eeprom_ready_isr:16
void eeprom_ready_isr(void)
{ }
//comparador analogico
#pragma interrupt_handler ana_comp_isr:17
void ana_comp_isr(void)
{ }
//twi completo
#pragma interrupt_handler twi_isr:18
void twi_isr(void)
{ }
//FLASH guardado listo
#pragma interrupt_handler spm_rdy_isr:19
void spm_rdy_isr(void)
{ }
