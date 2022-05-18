/*
 * ATMega32_utility_bib.c
 *Created 18.05.2022
 * Author: Jan-Hendrik Aschen
 */


 #include "ATMega32_utility_bib.h"

 ADC_read::ADC_read(uint8_t _channel):channel(_channel){}

 uint16_t ADC_read::adc_value(void)
 {

   //Pullup setzen
   SET_BIT(PORTA,channel);
   uint16_t adc_value=0;
   // ADC init
   // REFS1:0 = 00 => AREF externe Referenzspannung (=5V beim RNCTRL1.4)
   // ADMUX Kanal 7 PINA7 = > MUX0:1:2 == 1
   uint8_t ADCChan= channel;
   ADMUX =(ADMUX & 0b11100000) | ( ADCChan & 0b00011111) ;// AD Multiplexer


   // ADEN (ADC Enable )  = 1 => AD-Wandler freigeben
   // ADSC (ADC Start Conversion)= 1 => AD-Wandlung starten
   //ADPS ()ADC Prescaler) 0-2 = 111 => Taktvorteiler 128
   // Muss so eingestellt werden, dass der Wandlertakt des ADC 50...200kHz
   ADCSRA = (1<<ADEN) | ( 1<<ADSC) | (1<< ADPS2) | (1<< ADPS1) | (1<< ADPS0) ;  // Statusregister A

   // AD-Wandlung starten
   SET_BIT(ADCSRA,ADSC);

  // Warten bis die AD-Wandlung abgeschlossen ist
    while(BIT_IS_CLR(ADCSRA,ADIF)){}    // ADIF (ADC Interrupt Flag) wird gesetzt, wenn Wandlung angechlossen ist.

     adc_value=ADCW;

     return adc_value;
 }


Button::Button(){}

int8_t Button::Button_read(void)
{
 int8_t button_pressed=-1;

 ADC_read pin(7);
 uint16_t analog7 = pin.adc_value(); //ADC Wert lesen und zwischenspeichern

 // Prüfe, welcher Taster gedrückt wurde (Spannungsteiler)
   if((analog7>=405) && (analog7<=406)) {button_pressed = 1;}
   else if((analog7>=336) && (analog7<=338)) {button_pressed = 2;}
   else if((analog7>=264) && (analog7<=266)) {button_pressed = 3;}
   else if((analog7>=187) && (analog7<=189)) {button_pressed= 4;}
   else if((analog7>=104) && (analog7<=106)) {button_pressed= 5;}
   else     {button_pressed=-1;}

   return button_pressed;
}
