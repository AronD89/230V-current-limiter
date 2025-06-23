#include <avr/io.h>
#define F_CPU 16000000UL //16MHz - a delay fuggvenyeknek
#include <util/delay.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stddef.h> //NULL

#define COM0_GND DDRD |=  (1<<PD0); PORTD &= ~(1<<PD0); //COM0 - GND
#define COM0_1P2 DDRD &= ~(1<<PD0); PORTD &= ~(1<<PD0); //COM0 - 1/2 VCC
#define COM0_VCC DDRD |=  (1<<PD0); PORTD |=  (1<<PD0); //COM0 - VCC
#define COM1_GND DDRD |=  (1<<PD1); PORTD &= ~(1<<PD1);
#define COM1_1P2 DDRD &= ~(1<<PD1); PORTD &= ~(1<<PD1);
#define COM1_VCC DDRD |=  (1<<PD1); PORTD |=  (1<<PD1);
#define COM2_GND DDRD |=  (1<<PD2); PORTD &= ~(1<<PD2);
#define COM2_1P2 DDRD &= ~(1<<PD2); PORTD &= ~(1<<PD2);
#define COM2_VCC DDRD |=  (1<<PD2); PORTD |=  (1<<PD2);
#define COM3_GND DDRD |=  (1<<PD3); PORTD &= ~(1<<PD3);
#define COM3_1P2 DDRD &= ~(1<<PD3); PORTD &= ~(1<<PD3);
#define COM3_VCC DDRD |=  (1<<PD3); PORTD |=  (1<<PD3);

#define SEG26_ON  PORTD |= (1<<PD4); //1. kis szegmenscsoport VCC
#define SEG25_ON  PORTD |= (1<<PD5);  
#define SEG24_ON  PORTD |= (1<<PD6); 
#define SEG23_ON  PORTD |= (1<<PD7);  
#define SEG22_ON  PORTC |= (1<<PC0); 
#define SEG21_ON  PORTC |= (1<<PC1); 
#define SEG20_ON  PORTC |= (1<<PC2); 
#define SEG19_ON  PORTC |= (1<<PC3); 

#define SEG26_OFF PORTD &= ~(1<<PD4); //1. kis szegmenscsoport GND
#define SEG25_OFF PORTD &= ~(1<<PD5);
#define SEG24_OFF PORTD &= ~(1<<PD6);
#define SEG23_OFF PORTD &= ~(1<<PD7);
#define SEG22_OFF PORTC &= ~(1<<PC0);
#define SEG21_OFF PORTC &= ~(1<<PC1);
#define SEG20_OFF PORTC &= ~(1<<PC2);
#define SEG19_OFF PORTC &= ~(1<<PC3); 

#define SEG1_ON  PORTA |= (1<<PA4); //26. nagy szegmenscsoport VCC
#define SEG2_ON  PORTA |= (1<<PA5);  
#define SEG3_ON  PORTA |= (1<<PA6); 
#define SEG4_ON  PORTA |= (1<<PA7);  
#define SEG5_ON  PORTC |= (1<<PC7); 
#define SEG6_ON  PORTC |= (1<<PC6); 
#define SEG7_ON  PORTC |= (1<<PC5); 
#define SEG8_ON  PORTC |= (1<<PC4); 

#define SEG1_OFF PORTA &= ~(1<<PA4); //26. nagy szegmenscsoport GND
#define SEG2_OFF PORTA &= ~(1<<PA5);  
#define SEG3_OFF PORTA &= ~(1<<PA6); 
#define SEG4_OFF PORTA &= ~(1<<PA7);  
#define SEG5_OFF PORTC &= ~(1<<PC7); 
#define SEG6_OFF PORTC &= ~(1<<PC6); 
#define SEG7_OFF PORTC &= ~(1<<PC5); 
#define SEG8_OFF PORTC &= ~(1<<PC4); 

#define RELAY_ON  PORTB |= (1<<PB0); 
#define RELAY_OFF PORTB &= ~(1<<PB0); 

#define DOWN PA1
#define UP PA2
#define RESET PA3

#define N 150 //mintak szama
#define Shunt 206 //sont ellenallas erteke (ohm)
#define VCC 3.3  //tapfeszultseg (V)
#define CT_ratio 1000 //CT szekunder menetszam
#define delay 2

uint8_t Segments[4][4] = {{0,0,0,0},{0,0,0,0}};

void setSegments (uint8_t s00, uint8_t s01, uint8_t s02, uint8_t s03, uint8_t s10, uint8_t s11, uint8_t s12, uint8_t s13) 
{    
	Segments[0][0] = s00; Segments[0][1] = s01; Segments[0][2] = s02; Segments[0][3] = s03;
    Segments[1][0] = s10; Segments[1][1] = s11; Segments[1][2] = s12; Segments[1][3] = s13;
	
}

void SEGs0() { setSegments(1,0,1,1, 1,1,1,0); } //A "0" szegmensei COM0..3 fuggvenyeben
void SEGs1() { setSegments(0,0,0,0, 0,1,1,0); }
void SEGs2() { setSegments(0,1,1,1, 1,1,0,0); }
void SEGs3() { setSegments(0,1,0,1, 1,1,1,0); }
void SEGs4() { setSegments(1,1,0,0, 0,1,1,0); }
void SEGs5() { setSegments(1,1,0,1, 1,0,1,0); }
void SEGs6() { setSegments(1,1,1,1, 1,0,1,0); }
void SEGs7() { setSegments(0,0,0,0, 1,1,1,0); }
void SEGs8() { setSegments(1,1,1,1, 1,1,1,0); }
void SEGs9() { setSegments(1,1,0,1, 1,1,1,0); }
void OVERLOAD() { setSegments(0,0,0,0, 0,0,0,1); }
void decimal() { setSegments(0,0,0,0, 0,0,0,1); }

uint16_t set_value=100;//kezdeti aramkorlat
uint16_t overflows=0;  //idozito lejarasanak szama, a hosszan nyomvatartott UP es DOWN gombokbak
uint16_t display=0;    //idozito lejarasanak szama, a kijelzo frissitesi rataja
uint16_t meas_value=0, adc_value, meas_avg=0;
uint8_t converted;
float avg_voltage = 0;
float rms_voltage = 0;
float rms_current = 0;
float adc_voltage=0;
uint8_t button_action = 0, overload=0, i=0; //flag, hogy egyszerre csak egy gomb mukodjon
uint8_t (*(LoadSEGs[10]))() = {SEGs0, SEGs1, SEGs2, SEGs3, SEGs4, SEGs5, SEGs6, SEGs7, SEGs8, SEGs9}; //index = fuggvenynev
xSemaphoreHandle binaris;
uint16_t set1, set2, set3, set4, meas1, meas2, meas3, meas4;

void LCD_print(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t overload); //m - measured, c - configured

uint8_t UP_pressed()
{
	if (!(PINA & (1<<UP))) //ha le van nyomva
	{
		return 1; 
	}
	return 0; 
}

uint8_t DOWN_pressed()
{
	if (!(PINA & (1<<DOWN))) //ha le van nyomva
	{
		return 1;
	}
	return 0;
}

uint8_t RESET_pressed()
{
	if (!(PINA & (1<<RESET))) //ha le van nyomva
	{
		return 1;
	}
	return 0;
}

void start_timer0()
{
	TCCR0B |= (1<< CS02); //CLK/256, start
	TIMSK0= (1<<TOIE0);  //enable overflow interrupt
	sei(); //set the I-bit in SREG
}

void stop_timer0()
{
	TCCR0B &= ~(1<< CS02); //nincs orajel => az idozito megall
}

void initPORTs()
{
	DDRA |= (1<<PA4) | (1<<PA5) | (1<<PA6) | (1<<PA7); //kimenet szegmenscsoportoknak
	DDRA &= ~((1<<UP) | (1<<DOWN) | (1<<RESET));//bemenet a gomboknak
	PORTA |= (1<<UP)|(1<<DOWN)|(1<<RESET); //felhuzo ellenallasok a gomboknak
	DDRB |= (1<<PB0); //kimenet a relenek
	DDRC = 0xff; //kimenet szegmenscsoportoknak
	DDRD |= (1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7);  //kimenet szegmens-csoportoknak 
}

void initADC()
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //16MHz/32=500kHz)
	ADMUX |= (1 << REFS0); //ADC referencia = AVCC
	//Mux4..0=0 - ADC0 csatorna
	ADCSRA |= (1<<ADIE); //Enable ADC interrupt
	ADCSRA |= (1<<ADATE); //Enable auto-triggering
	ADCSRA |= (1<<ADEN); //Enable ADC
	sei();
	ADCSRA |= (1<<ADSC); //start first ADC conversion
	converted = 1;
}

void adc_read()
{
	ADCSRA |= (1<<ADSC); //start ADC
	ADCSRA |= (1<<ADIE); //Enable ADC interrupt
}

void ADC_conversion()
{
	while(1)
	{
		xSemaphoreTake(binaris, (portTickType) 1 );
		if(i<N && converted) //N darab minta osszegzese
		{
			adc_read(); converted=0;			   //1 ADC minta
			adc_voltage = (adc_value * VCC)/1024; //minta feszultsegge alakitasa
			avg_voltage += pow(adc_voltage,2);    //feszultsegek negyzetosszege
			i++;
		} 
		if (i==N)
		{
			i=0;
			avg_voltage = avg_voltage/N;		//kozepertek
			rms_voltage = sqrt(avg_voltage);	//negyzetes kozep
			rms_current = rms_voltage / Shunt;  //I=U/R
			rms_current = rms_current*CT_ratio; //mA -> A
			meas_value = rms_current*100;       //kijelzohoz valo igazitas (2 tizedes)
		}
		if(meas_value > set_value) {overload=1; RELAY_OFF;}
		xSemaphoreGive(binaris);
	}
}

void LCD_Update()
{
	while(1)
	{
		xSemaphoreTake(binaris, (portTickType) 1 );
		if(meas_value > set_value) {overload=1; RELAY_OFF;}
		
		//a beallitott ertek szamjegyekre bontasa
		set4 = (set_value / 1) % 10;
		set3 = (set_value / 10) % 10;
		set2 = (set_value / 100) % 10;
		set1 = (set_value / 1000) % 10;
		
		//szamtani kozepertek
		if(display<16){meas_avg+=meas_value;}
		if (display==15)
		{	
			meas_avg=meas_avg>>4; //=osztva 16-tal
			if(meas_avg > set_value) {overload=1; RELAY_OFF;}
			//szamjegyekre bontas
			meas4 = (meas_avg / 1) % 10;
			meas3 = (meas_avg / 10) % 10;
			meas2 = (meas_avg / 100) % 10;
			meas1 = (meas_avg / 1000) % 10;
			display=0;
		} else {display++;}
		
		LCD_print(meas1,meas2,meas3,meas4, set1,set2,set3,set4, overload);
		xSemaphoreGive(binaris);
	}
}
void Cehck_buttons()
{
	while(1)
	{
		xSemaphoreTake(binaris, (portTickType) 1 );
		if(meas_value > set_value) {overload=1; RELAY_OFF;}
		
		if (UP_pressed() && !button_action)
		{
			button_action=1; 
			if (set_value<1600) {set_value++;}
			start_timer0();  
		}
				
		if (DOWN_pressed() && !button_action)
		{
			button_action=1;
			if (set_value>0) {set_value--;}
			start_timer0();  
		}
		
		if (RESET_pressed() && !button_action && meas_value < set_value)
		{
			button_action=1;
			overload=0;
			RELAY_ON;
			_delay_ms(100); //bekapcsolas utani varakozas az elso meres elott
		}
		
		if (!DOWN_pressed() && !UP_pressed() && button_action)
		{
			button_action=0;
			overflows=0;
			stop_timer0();
		}	
		
		xSemaphoreGive(binaris);
	}
}

int main(void)
{
	initPORTs();
	initADC();
	vSemaphoreCreateBinary(binaris);
	RELAY_ON; 
	_delay_ms(100); //bekapcsolas utani varakozas az elso meres elott
	xTaskCreate(ADC_conversion,(signed char*)"ADC",80,NULL,1,NULL);
	xTaskCreate(LCD_Update,(signed char*)"LCD",80,NULL,1,NULL);
	xTaskCreate(Cehck_buttons,(signed char*)"Button",80,NULL,1,NULL);
	vTaskStartScheduler();
	return 0;

}

ISR (TIMER0_OVF_vect) // timer0 overflow interrupt
{	
	if (UP_pressed())
	{
		if (set_value<1600 && overflows == 0) {overflows++; return;}
		if (set_value<=1599 && overflows >= 120) {set_value+=1; overflows++; return;}
	}
	
	if (DOWN_pressed())
	{
		if (set_value>0 && overflows == 0) {overflows++; return;}
		if (set_value>=1 && overflows >= 120) {set_value-=1; overflows++; return;}
	} 
	
	overflows++;
}

ISR(ADC_vect)
{
  adc_value = ADC;
  converted = 1;
}

//A COM-ok egymas utan kapcsolnak be es ki. A koztes idoben 1/2 feszultsegen pihennek.
//Mindenik COM allapotnal bekapcsolnak a szamjegyhez tartozo szegmenscsoportok
//A ket felperiodus egymas ellentete
void LCD_print(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t overload)
{	
	//pozitiv felperiodus
	COM3_1P2; COM0_GND; //COM0-hoz tartozo szegmensek: a Segments tomb 0. oszlopa
	LoadSEGs[m1](); if (Segments[0][0]) {SEG1_ON} else {SEG1_OFF} if (Segments[1][0]) {SEG2_ON} else {SEG2_OFF} 
	LoadSEGs[m2](); if (Segments[0][0]) {SEG3_ON} else {SEG3_OFF} if (Segments[1][0]) {SEG4_ON} else {SEG4_OFF} 
	LoadSEGs[m3](); if (Segments[0][0]) {SEG5_ON} else {SEG5_OFF} if (Segments[1][0]) {SEG6_ON} else {SEG6_OFF}
	LoadSEGs[m4](); if (Segments[0][0]) {SEG7_ON} else {SEG7_OFF} if (Segments[1][0]) {SEG8_ON} else {SEG8_OFF}
	LoadSEGs[c1](); if (Segments[0][0]) {SEG26_ON} else {SEG26_OFF} if (Segments[1][0]) {SEG25_ON} else {SEG25_OFF} 
	LoadSEGs[c2](); if (Segments[0][0]) {SEG24_ON} else {SEG24_OFF} if (Segments[1][0]) {SEG23_ON} else {SEG23_OFF} 
	LoadSEGs[c3](); if (Segments[0][0]) {SEG22_ON} else {SEG22_OFF} if (Segments[1][0]) {SEG21_ON} else {SEG21_OFF}
	LoadSEGs[c4](); if (Segments[0][0]) {SEG20_ON} else {SEG20_OFF} if (Segments[1][0]) {SEG19_ON} else {SEG19_OFF}
	_delay_ms(delay);
	
	COM0_1P2; COM1_GND; 
	LoadSEGs[m1](); if (Segments[0][1]) {SEG1_ON} else {SEG1_OFF} if (Segments[1][1]) {SEG2_ON} else {SEG2_OFF} 
	LoadSEGs[m2](); if (Segments[0][1]) {SEG3_ON} else {SEG3_OFF} if (Segments[1][1]) {SEG4_ON} else {SEG4_OFF} 
	LoadSEGs[m3](); if (Segments[0][1]) {SEG5_ON} else {SEG5_OFF} if (Segments[1][1]) {SEG6_ON} else {SEG6_OFF} 
	LoadSEGs[m4](); if (Segments[0][1]) {SEG7_ON} else {SEG7_OFF} if (Segments[1][1]) {SEG8_ON} else {SEG8_OFF} 
	LoadSEGs[c1](); if (Segments[0][1]) {SEG26_ON} else {SEG26_OFF} if (Segments[1][1]) {SEG25_ON} else {SEG25_OFF} 
	LoadSEGs[c2](); if (Segments[0][1]) {SEG24_ON} else {SEG24_OFF} if (Segments[1][1]) {SEG23_ON} else {SEG23_OFF} 
	LoadSEGs[c3](); if (Segments[0][1]) {SEG22_ON} else {SEG22_OFF} if (Segments[1][1]) {SEG21_ON} else {SEG21_OFF}
	LoadSEGs[c4](); if (Segments[0][1]) {SEG20_ON} else {SEG20_OFF} if (Segments[1][1]) {SEG19_ON} else {SEG19_OFF}
	_delay_ms(delay);
	
	COM1_1P2; COM2_GND; 
	LoadSEGs[m1](); if (Segments[0][2]) {SEG1_ON} else {SEG1_OFF} if (Segments[1][2]) {SEG2_ON} else {SEG2_OFF} 
	LoadSEGs[m2](); if (Segments[0][2]) {SEG3_ON} else {SEG3_OFF} if (Segments[1][2]) {SEG4_ON} else {SEG4_OFF}  
	LoadSEGs[m3](); if (Segments[0][2]) {SEG5_ON} else {SEG5_OFF} if (Segments[1][2]) {SEG6_ON} else {SEG6_OFF} 
	LoadSEGs[m4](); if (Segments[0][2]) {SEG7_ON} else {SEG7_OFF} if (Segments[1][2]) {SEG8_ON} else {SEG8_OFF} 
	LoadSEGs[c1](); if (Segments[0][2]) {SEG26_ON} else {SEG26_OFF} if (Segments[1][2]) {SEG25_ON} else {SEG25_OFF} 
	LoadSEGs[c2](); if (Segments[0][2]) {SEG24_ON} else {SEG24_OFF} if (Segments[1][2]) {SEG23_ON} else {SEG23_OFF} 
	LoadSEGs[c3](); if (Segments[0][2]) {SEG22_ON} else {SEG22_OFF} if (Segments[1][2]) {SEG21_ON} else {SEG21_OFF}
	LoadSEGs[c4](); if (Segments[0][2]) {SEG20_ON} else {SEG20_OFF} if (Segments[1][2]) {SEG19_ON} else {SEG19_OFF}
	_delay_ms(delay);
	
	COM2_1P2; COM3_GND; 
	LoadSEGs[m1](); if (Segments[0][3]) {SEG1_ON} else {SEG1_OFF} if (Segments[1][3]) {SEG2_ON} else {SEG2_OFF} 
	LoadSEGs[m2](); if (Segments[0][3]) {SEG3_ON} else {SEG3_OFF} if (Segments[1][3]) {SEG4_ON} else {SEG4_OFF} 
	LoadSEGs[m3](); if (Segments[0][3]) {SEG5_ON} else {SEG5_OFF} if (Segments[1][3]) {SEG6_ON} else {SEG6_OFF} 
	LoadSEGs[m4](); if (Segments[0][3]) {SEG7_ON} else {SEG7_OFF} if (Segments[1][3]) {SEG8_ON} else {SEG8_OFF} 
	LoadSEGs[c1](); if (Segments[0][3]) {SEG26_ON} else {SEG26_OFF} if (Segments[1][3]) {SEG25_ON} else {SEG25_OFF} 
	LoadSEGs[c2](); if (Segments[0][3]) {SEG24_ON} else {SEG24_OFF} if (Segments[1][3]) {SEG23_ON} else {SEG23_OFF} 
	LoadSEGs[c3](); if (Segments[0][3]) {SEG22_ON} else {SEG22_OFF} if (Segments[1][3]) {SEG21_ON} else {SEG21_OFF}
	LoadSEGs[c4](); if (Segments[0][3]) {SEG20_ON} else {SEG20_OFF} if (Segments[1][3]) {SEG19_ON} else {SEG19_OFF}
	if (overload) {OVERLOAD(); if (Segments[1][3]) {SEG8_ON} else {SEG8_OFF}}
	decimal(); if (Segments[1][3]) {SEG4_ON; SEG23_ON} else {SEG4_OFF; SEG23_OFF}
	_delay_ms(delay); 
	
	//negativ felperiodus
	COM3_1P2; COM0_VCC; 
	LoadSEGs[m1](); if (Segments[0][0]) {SEG1_OFF} else {SEG1_ON} if (Segments[1][0]) {SEG2_OFF} else {SEG2_ON}
	LoadSEGs[m2](); if (Segments[0][0]) {SEG3_OFF} else {SEG3_ON} if (Segments[1][0]) {SEG4_OFF} else {SEG4_ON} 
	LoadSEGs[m3](); if (Segments[0][0]) {SEG5_OFF} else {SEG5_ON} if (Segments[1][0]) {SEG6_OFF} else {SEG6_ON} 
	LoadSEGs[m4](); if (Segments[0][0]) {SEG7_OFF} else {SEG7_ON} if (Segments[1][0]) {SEG8_OFF} else {SEG8_ON} 
	LoadSEGs[c1](); if (Segments[0][0]) {SEG26_OFF} else {SEG26_ON} if (Segments[1][0]) {SEG25_OFF} else {SEG25_ON} 
	LoadSEGs[c2](); if (Segments[0][0]) {SEG24_OFF} else {SEG24_ON} if (Segments[1][0]) {SEG23_OFF} else {SEG23_ON} 
	LoadSEGs[c3](); if (Segments[0][0]) {SEG22_OFF} else {SEG22_ON} if (Segments[1][0]) {SEG21_OFF} else {SEG21_ON}
	LoadSEGs[c4](); if (Segments[0][0]) {SEG20_OFF} else {SEG20_ON} if (Segments[1][0]) {SEG19_OFF} else {SEG19_ON}
	_delay_ms(delay);
	
	COM0_1P2; COM1_VCC; 
	LoadSEGs[m1](); if (Segments[0][1]) {SEG1_OFF} else {SEG1_ON} if (Segments[1][1]) {SEG2_OFF} else {SEG2_ON} 
	LoadSEGs[m2](); if (Segments[0][1]) {SEG3_OFF} else {SEG3_ON} if (Segments[1][1]) {SEG4_OFF} else {SEG4_ON}  
	LoadSEGs[m3](); if (Segments[0][1]) {SEG5_OFF} else {SEG5_ON} if (Segments[1][1]) {SEG6_OFF} else {SEG6_ON}  
	LoadSEGs[m4](); if (Segments[0][1]) {SEG7_OFF} else {SEG7_ON} if (Segments[1][1]) {SEG8_OFF} else {SEG8_ON}  
	LoadSEGs[c1](); if (Segments[0][1]) {SEG26_OFF} else {SEG26_ON} if (Segments[1][1]) {SEG25_OFF} else {SEG25_ON} 
	LoadSEGs[c2](); if (Segments[0][1]) {SEG24_OFF} else {SEG24_ON} if (Segments[1][1]) {SEG23_OFF} else {SEG23_ON} 
	LoadSEGs[c3](); if (Segments[0][1]) {SEG22_OFF} else {SEG22_ON} if (Segments[1][1]) {SEG21_OFF} else {SEG21_ON}
	LoadSEGs[c4](); if (Segments[0][1]) {SEG20_OFF} else {SEG20_ON} if (Segments[1][1]) {SEG19_OFF} else {SEG19_ON}
	_delay_ms(delay);
	
	COM1_1P2; COM2_VCC; 
	LoadSEGs[m1](); if (Segments[0][2]) {SEG1_OFF} else {SEG1_ON} if (Segments[1][2]) {SEG2_OFF} else {SEG2_ON} 
	LoadSEGs[m2](); if (Segments[0][2]) {SEG3_OFF} else {SEG3_ON} if (Segments[1][2]) {SEG4_OFF} else {SEG4_ON}  
	LoadSEGs[m3](); if (Segments[0][2]) {SEG5_OFF} else {SEG5_ON} if (Segments[1][2]) {SEG6_OFF} else {SEG6_ON} 
	LoadSEGs[m4](); if (Segments[0][2]) {SEG7_OFF} else {SEG7_ON} if (Segments[1][2]) {SEG8_OFF} else {SEG8_ON} 
	LoadSEGs[c1](); if (Segments[0][2]) {SEG26_OFF} else {SEG26_ON} if (Segments[1][2]) {SEG25_OFF} else {SEG25_ON} 
	LoadSEGs[c2](); if (Segments[0][2]) {SEG24_OFF} else {SEG24_ON} if (Segments[1][2]) {SEG23_OFF} else {SEG23_ON} 
	LoadSEGs[c3](); if (Segments[0][2]) {SEG22_OFF} else {SEG22_ON} if (Segments[1][2]) {SEG21_OFF} else {SEG21_ON}
	LoadSEGs[c4](); if (Segments[0][2]) {SEG20_OFF} else {SEG20_ON} if (Segments[1][2]) {SEG19_OFF} else {SEG19_ON}
	_delay_ms(delay);
	
	COM2_1P2; COM3_VCC; 
	LoadSEGs[m1](); if (Segments[0][3]) {SEG1_OFF} else {SEG1_ON} if (Segments[1][3]) {SEG2_OFF} else {SEG2_ON} 
	LoadSEGs[m2](); if (Segments[0][3]) {SEG3_OFF} else {SEG3_ON} if (Segments[1][3]) {SEG4_OFF} else {SEG4_ON}   
	LoadSEGs[m3](); if (Segments[0][3]) {SEG5_OFF} else {SEG5_ON} if (Segments[1][3]) {SEG6_OFF} else {SEG6_ON} 
	LoadSEGs[m4](); if (Segments[0][3]) {SEG7_OFF} else {SEG7_ON} if (Segments[1][3]) {SEG8_OFF} else {SEG8_ON} 
	LoadSEGs[c1](); if (Segments[0][3]) {SEG26_OFF} else {SEG26_ON} if (Segments[1][3]) {SEG25_OFF} else {SEG25_ON} 
	LoadSEGs[c2](); if (Segments[0][3]) {SEG24_OFF} else {SEG24_ON} if (Segments[1][3]) {SEG23_OFF} else {SEG23_ON} 
	LoadSEGs[c3](); if (Segments[0][3]) {SEG22_OFF} else {SEG22_ON} if (Segments[1][3]) {SEG21_OFF} else {SEG21_ON}
	LoadSEGs[c4](); if (Segments[0][3]) {SEG20_OFF} else {SEG20_ON} if (Segments[1][3]) {SEG19_OFF} else {SEG19_ON}
	if (overload) {OVERLOAD(); if (Segments[1][3]) {SEG8_OFF} else {SEG8_ON}}
	decimal(); if (Segments[1][3]) {SEG4_OFF; SEG23_OFF} else {SEG4_ON; SEG23_ON}
	_delay_ms(delay);
}
