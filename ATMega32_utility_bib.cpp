/*
 * ATMega32_utility_bib.c
 *Created 18.05.2022
 * Author: Jan-Hendrik Aschen
 */


#include <stdarg.h>
#include "ATMega32_utility_bib.h"
#include "user_button_values.h"


uint8_t ADC_read::channel; 

void ADC_read::init(uint8_t _channel){
   channel=_channel;
 }

 uint16_t ADC_read::adc_value(void)
 {
   // PIN 7 als Eingang
   CLR_BIT(DDRA,channel);
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

int8_t Button::Button_read(void)
{
 int8_t button_pressed=-1;

 ADC_read::init(7);
 uint16_t analog7 = ADC_read::adc_value(); //ADC Wert lesen und zwischenspeichern

 // Prüfe, welcher Taster gedrückt wurde (Spannungsteiler)
   if      ( (analog7 >= ATMEGA32_USER_BUTTON1_LOW) && (analog7 <= ATMEGA32_USER_BUTTON1_HIGH ) ) {button_pressed = 1;}
   else if ( (analog7 >= ATMEGA32_USER_BUTTON2_LOW) && (analog7 <= ATMEGA32_USER_BUTTON2_HIGH ) ) {button_pressed = 2;}
   else if ( (analog7 >= ATMEGA32_USER_BUTTON3_LOW) && (analog7 <= ATMEGA32_USER_BUTTON3_HIGH) ) {button_pressed = 3;}
   else if ( (analog7 >= ATMEGA32_USER_BUTTON4_LOW) && (analog7 <= ATMEGA32_USER_BUTTON4_HIGH) ) {button_pressed= 4;}
   else if ( (analog7 >= ATMEGA32_USER_BUTTON5_LOW) && (analog7 <= ATMEGA32_USER_BUTTON5_HIGH) ) {button_pressed= 5;}
   else {button_pressed=-1;}

   return button_pressed;
}

//UART Schnittstelle
void UART::init(uint32_t Baudrate, uint8_t CharBits, uint8_t ParBit, uint8_t StopBits)
{
  // Vorhandensein und Art des Paritäts-Bits festlegen:
  // Gerade   Parität: Anzahl der '1' wird auf gerade Anzahl ergänzt
  // Ungerade Parität: Anzahl der '1' wird auf ungerade Anzahl ergänzt
  // Keine    Parität: Paritäts-Bit entfällt
  if (ParBit == USART_EVEN_PARITY)
  UCSRC |= (1 << UPM1) | (0 << UPM0);  // äquivalent: UCSRC |= (1 << UPM1);
  else
  if (ParBit == USART_ODD_PARITY)
  UCSRC |= (1 << UPM1) | (1 << UPM0);
  else
  UCSRC |= (0 << UPM1) | (0 << UPM0);  // brauchen wir eigentlich nicht...

  // Anzahl der Stop-Bits festlegen: 1 oder 2
  if (StopBits == 1)
  UCSRC |= (0 << USBS);
  else
  if (StopBits == 2)
  UCSRC |= (1 << USBS);

  // Anzahl der Zeichenbits: 5..9
  // Üblich sind 8 Bits / Zeichen.
  // Bei 9 Zeichenbits muss man die besondere Behandlung des 9.Bits beim
  // Lesen und Schreiben beachten. Diese ist in unseren Lese- und Schreibroutinen
  // nicht berücksichtigt.
  switch (CharBits)
  {
  case 5: // 5 Zeichenbits
    break;

  case 6: // 6 Zeichenbits
    UCSRC |= (1 << UCSZ0);
    break;

  case 7: // 7 Zeichenbits
    UCSRC |= (1 << UCSZ1);
    break;

  case 8: // 8 Zeichenbits
    UCSRC |= (1 << UCSZ1) | (1 << UCSZ0);
    break;

  case 9: // 9 Zeichenbits
    UCSRB |= (1 << UCSZ2);
    UCSRC |= (1 << UCSZ1) | (1 << UCSZ0);
    break;
  }

  // Einstellen der Bitrate:
  // Bei der Wahl der Bitrate muss der relative Bitratenfehler
  // aufgrund der Taktfrequenz des uC beachtet werden.
  // Dieser wirkt sich bei höheren Bitraten stärker aus, daher
  // darf bei großem Bitratenfehler die Bitrate nicht zu groß
  // gewählt werden.
  UBRRL = (F_CPU/(16*Baudrate)-1) % 256;
  UBRRH = (F_CPU/(16*Baudrate)-1) / 256;

  // Freigabe der Sende-/Empfangs-Kanäle und uC-Pins
  UCSRB |= (1 << RXEN) | (1 << TXEN);
}


void UART::uart_putc(char data)
{
  // Warten, dass Senderegister frei ist
while (BIT_IS_CLR(UCSRA,UDRE));
// Jetzt ist das Senderegister frei, also
  // senden wir ein Zeichen einfach durch Schreiben
  // in das Senderegister UDR
UDR = data;
}

void UART::uart_printf(const char* format, ...)
{
  char buf[30];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
}



void UART::uart_puts(const char* pstring)
{
 while (*pstring !=0) {
   uart_putc(*pstring);
   pstring++;
 }
}

void UART::uart_println(const char* pstring)
 {
   uart_puts(pstring);
   uart_puts("\r\n");
 }

 char UART::uart_getc(void)
 {
 char data;
 // Prüfe ob das Empfangsregister gefüllt ist
 while (BIT_IS_CLR(UCSRA,RXC)) {
   /* code */
 }
 //Daten einlesen
 data=UDR;

 return(data);
 }
