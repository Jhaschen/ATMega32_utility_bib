/*
 * ATMega32_utility_bib.c
 *Created 18.05.2022
 * Author: Jan-Hendrik Aschen
 */


#include <stdarg.h>
#include "ATMega32_utility_bib.h"
#include "user_button_values.h"

void ADC_read::init(uint8_t _channel){
  // PIN 7 als Eingang
  CLR_BIT(DDRA, _channel);
  //Pullup setzen
  SET_BIT(PORTA, _channel);

  // ADC init
  // REFS1:0 = 00 => AREF externe Referenzspannung (=5V beim RNCTRL1.4)
  // ADMUX Kanal 7 PINA7 = > MUX0:1:2 == 1
  uint8_t ADCChan= _channel;
  ADMUX =(ADMUX & 0b11100000) | ( ADCChan & 0b00011111) ;// AD Multiplexer

  // ADEN (ADC Enable )  = 1 => AD-Wandler freigeben
  // ADPS ()ADC Prescaler) 0-2 = 111 => Taktvorteiler 128
  // Muss so eingestellt werden, dass der Wandlertakt des ADC 50...200kHz
  ADCSRA = (1<<ADEN) | (1<< ADPS2) | (1<< ADPS1) | (1<< ADPS0) ;  // Statusregister A
}

uint16_t ADC_read::adc_value(uint8_t channel)
{
  uint16_t adc_value=0;
  // ADSC (ADC Start Conversion)= 1 => AD-Wandlung starten
  // AD-Wandlung starten
  SET_BIT(ADCSRA, ADSC);

  // Warten bis die AD-Wandlung abgeschlossen ist
  while(BIT_IS_CLR(ADCSRA,ADIF)){}    // ADIF (ADC Interrupt Flag) wird gesetzt, wenn Wandlung angechlossen ist.

  adc_value=ADCW;

  return adc_value;
}


void Button::init()
{
  ADC_read::init(7); // ADC mit channel 7 initialisieren
}


int8_t Button::Button_read(void)
{
  int8_t button_pressed=-1;

  uint16_t analog7 = ADC_read::adc_value(7); //ADC Wert lesen und zwischenspeichern

  // Prüfe, welcher Taster gedrückt wurde (Spannungsteiler)
  if      ( (analog7 >= ATMEGA32_USER_BUTTON1_LOW) && (analog7 <= ATMEGA32_USER_BUTTON1_HIGH ) ) {button_pressed = 1;}
  else if ( (analog7 >= ATMEGA32_USER_BUTTON2_LOW) && (analog7 <= ATMEGA32_USER_BUTTON2_HIGH ) ) {button_pressed = 2;}
  else if ( (analog7 >= ATMEGA32_USER_BUTTON3_LOW) && (analog7 <= ATMEGA32_USER_BUTTON3_HIGH) ) {button_pressed = 3;}
  else if ( (analog7 >= ATMEGA32_USER_BUTTON4_LOW) && (analog7 <= ATMEGA32_USER_BUTTON4_HIGH) ) {button_pressed= 4;}
  else if ( (analog7 >= ATMEGA32_USER_BUTTON5_LOW) && (analog7 <= ATMEGA32_USER_BUTTON5_HIGH) ) {button_pressed= 5;}
  else {button_pressed=-1;}

  return button_pressed;
}


int8_t Button::Button_read_debounced(void)
{ 
  int button_value[3] = {};
  //in case the button does not give a consistent value,
  //do max 65535 cycles, then return -2
  //by the way: gen3 rulez
  //credits to Christian Werner
  uint16_t max_cycles_fallback = __UINT16_MAX__;
  do {
    button_value[2]=button_value[1];
    button_value[1]=button_value[0];
    button_value[0]=Button::Button_read();
    if (max_cycles_fallback-- == 0) {
      return -2;
    }
  } while (button_value[0] != button_value[1] || button_value[1] != button_value[2]);
  return button_value[0];
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
  char buf[50];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  uart_puts(buf);
}

void UART::uart_printf_P(const char* format, ...)
{
  char buf[50];
  va_list args;
  va_start(args, format);
  vsnprintf_P(buf, sizeof(buf), format, args);
  va_end(args);
  uart_puts(buf);
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
  }
  //Daten einlesen
  data=UDR;

  return(data);
}

 volatile unsigned long Timer0::_overflow_count = 0;
 volatile unsigned long Timer0::_millis = 0;
 volatile unsigned char Timer0::_fract = 0;

void Timer0::init() {
  // Prescaler 64  
  SET_BIT(TCCR0, CS01);
	SET_BIT(TCCR0, CS00);
  // Timer overflow interrupt
  SET_BIT(TIMSK, TOIE0);
  sei();
}

/************************ ISR timer0 *****************/
ISR(TIMER0_OVF_vect) {
  Timer0::_millis = Timer0::_millis + MILLISECONDS_PER_TIMER0_OVERFLOW;
  Timer0::_fract = Timer0::_fract + FRACT_INC;

  if (Timer0::_fract >= FRACT_MAX) {
    Timer0::_fract = Timer0::_fract - FRACT_MAX;
    Timer0::_millis++;
  }

  Timer0::_overflow_count++;
}

void Timer0::delay(unsigned long ms) {
  uint32_t start = micros();

	while (ms > 0) {
		while ( ms > 0 && (micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}

void Timer0::delayMicroseconds(unsigned int us) {
  if (us <= 1) return;
  us <<= 2; // convert us into cycles: 1 us == 4 cycles
  // busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}

unsigned long Timer0::millis() {
  unsigned long m;
	uint8_t oldSREG = SREG;
	cli();
	m = _millis;
	SREG = oldSREG;
	return m;
}

unsigned long Timer0::micros() {
  unsigned long m;
	uint8_t oldSREG = SREG, t;
	cli();
	m = _overflow_count;
	t = TCNT0;
	if ((TIFR & _BV(TOV0)) && (t < 255)) m++;
	SREG = oldSREG;
	return ((m << 8) + t) * 4;
}


const uint16_t Beeper::_prescalers[7]     = {1,     8,  32,  64, 128, 256, 1024};
const uint8_t  Beeper::_prescaler_bits[7] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6,  0x7};

void Beeper::init() {
  IO::pinMode(PIN_PD7, OUTPUT);

  //Wave Form Generation Mode -> CTC
  CLR_BIT(TCCR2, WGM20);
  SET_BIT(TCCR2, WGM21);

  //Toggle OC2 on Compare Match
  SET_BIT(TCCR2, COM20);
  CLR_BIT(TCCR2, COM21);
}

void Beeper::beep(uint16_t freq) {
  // min frequency is 31 Hz!
  if (freq < 32) freq = 31;
  uint32_t temp_OCR2 = 0;

  for (uint8_t i=0; i < sizeof(_prescalers); i++)
  {
    temp_OCR2 = F_CPU / ((uint32_t) _prescalers[i] * freq * 2);
    if (temp_OCR2 < 256) {
      //UART::uart_printf("Freq: %u, Prescaler: %u\r\n", freq, i);
      TCCR2= (TCCR2 & ~(0x7)) | _prescaler_bits[i];
      OCR2 = (uint8_t) temp_OCR2;
      break;
    }       
  }
}
  
void Beeper::noBeep() {
  TCCR2 = (TCCR2 & ~(0x7)); // keine Taktquelle ausgewählt!
}

uint8_t IO::digitalPinToInterrupt(uint8_t interruptPin) {
  //UART::uart_printf_P(PSTR("called: digitalPinToInterrupt\r\n"));
  switch (interruptPin) {
    case PIN_PD2:
      return 0;
    case PIN_PD3:
      return 1;
    case PIN_PB2:
      return 2;
  }
  return 0xFF;
}

uint8_t IO::_pinToPortAddress(uint8_t pin) {
  switch (pin>>4) {
    case 0x1:
    return 0x1B; // PORTA
    case 0x2:
    return 0x18; // PORTB
    case 0x4:
    return 0x15; // PORTC
    case 0x8:
    return 0x12; // PORTD
  }
  return 0xFF;
}

uint8_t IO::_pinToDDRAddress(uint8_t pin) {
  switch (pin>>4) {
    case 0x1:
    return 0x1A; // DDRA
    case 0x2:
    return 0x17; // DDRB
    case 0x4:
    return 0x14; // DDRC
    case 0x8:
    return 0x11; // DDRD
  }
  return 0xFF;
}

void IO::pinMode(uint8_t pin, uint8_t mode) {
  //UART::uart_printf_P(PSTR("called: pinMode(pin==%d, port==%d, mode==%d)\r\n"), pin&0x7, pin>>4, mode);
  if (mode==INPUT || mode==INPUT_PULLUP) {
    CLR_BIT(_SFR_IO8(_pinToDDRAddress(pin)), pin & 0x07);
    if (mode==INPUT_PULLUP) SET_BIT(_SFR_IO8(_pinToPortAddress(pin)), pin & 0x07);
  } else if (mode==OUTPUT) SET_BIT(_SFR_IO8(_pinToDDRAddress(pin)), pin & 0x07);
}

void IO::digitalWrite(uint8_t pin, uint8_t value) {
  //UART::uart_printf_P(PSTR("called: digitalWrite(pin==%d, port==%d, val==%d)\r\n"), pin&0x7, pin>>4, value);
  if (value) SET_BIT(_SFR_IO8(_pinToPortAddress(pin)), pin & 0x7); else CLR_BIT(_SFR_IO8(_pinToPortAddress(pin)), pin & 0x7);
}

uint8_t IO::digitalRead(uint8_t pin) {
  //UART::uart_printf_P(PSTR("called: digitalRead(pin==%d, port==%d, mode==%d)\r\n"), pin&0x7, pin>>4);
  switch (pin>>4) {
    case 0x1:
      return (BIT_IS_SET(PINA, pin & 0x7));
    case 0x2:
      return (BIT_IS_SET(PINB, pin & 0x7));
    case 0x4:
      return (BIT_IS_SET(PINC, pin & 0x7));
    case 0x8:
      return (BIT_IS_SET(PIND, pin & 0x7));
  }
  return 0;
}

void IO::tone(uint8_t pin, unsigned int frequency, unsigned long duration) {
  // not required!
  UART::uart_printf_P(PSTR("called: tone\r\n"));
}

void IO::noTone(uint8_t pin) {
  // not required!
  UART::uart_printf_P(PSTR("called: noTone\r\n"));
}

void IO::attachInterrupt(uint8_t interrupt, void(*userFunction)(void),  uint8_t mode) {
  //UART::uart_printf_P(PSTR("called: attachInterrup\r\n"));
  if (interrupt < 3 && userFunction && (mode==FALLING || mode==RISING || mode==CHANGE)) {
    IO::intFunc[interrupt] = userFunction;
    switch (interrupt) {
      case (0):
        MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
        GICR |= (1 << INT0);
        break;
      case (1):
        MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
        GICR |= (1 << INT1);
        break;
      case (2):
        CLR_BIT(GICR, INT2);
        if (mode==RISING) SET_BIT(MCUCSR, ISC2);
        if (mode==FALLING) CLR_BIT(MCUCSR, ISC2);
        SET_BIT(GICR, INT2);
        break;
    }
  }
}

void IO::detachInterrupt(uint8_t interrupt) {
  if (interrupt < 3) {
    switch (interrupt) {
      case (0):
        GICR &= ~(1 << INT0);
        break;
      case (1):
        GICR &= ~(1 << INT1);
        break;
      case (2):
        GICR &= ~(1 << INT2);
        break;
    }
    IO::intFunc[interrupt] = IO::nothing;
  }
}

void IO::nothing(void) {
  // do nothing
}

volatile voidFuncPtr IO::intFunc[3] = {IO::nothing,IO::nothing,IO::nothing};

ISR(INT0_vect) {
  IO::intFunc[0]();
}

ISR(INT1_vect) {
  IO::intFunc[1]();
}

ISR(INT2_vect) {
  IO::intFunc[2]();
}

void SPI::SPIbegin() {
  // not required?
  //UART::uart_printf_P(PSTR("called: SPIbegin\r\n"));
}

void SPI::SPIbeginTransaction() {
  // MOSI, SCK, and NSS: output
  DDRB |= ((1<<DDB7)|(1<<DDB5)|(1<<DDB4));
  SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); // 0=SPI no interrupt, 1=SPI enabled, 0=MSB-Fist, 1=Master, 00=SPI Mode 0, 01=div:8 (with SPI2X=1)
  SET_BIT(SPSR, SPI2X);
}

// Write to the SPI bus (MOSI pin) and also receive (MISO pin)
uint8_t SPI::SPItransfer(uint8_t data) {
  //UART::uart_printf_P(PSTR("called: SPItransfer\r\n"));
  SPDR = data;
  /*
    * The following NOP introduces a small delay that can prevent the wait
    * loop form iterating when running at the maximum speed. This gives
    * about 10% more speed, even if it seems counter-intuitive. At lower
    * speeds it is unnoticed.
    */
  asm volatile("nop");
  while (!(SPSR & _BV(SPIF))) ; // wait
  //UART::uart_printf_P(PSTR("SPItransfer returns: %x\r\n"),SPDR);
  return SPDR;
}


// After performing a group of transfers and releasing the chip select
// signal, this function allows others to access the SPI bus
void SPI::SPIendTransaction(void) {
  //UART::uart_printf_P(PSTR("called: SPIendTransaction\r\n"));
}

void SPI::SPIend() {
  // not required?
  //UART::uart_printf_P(PSTR("called: SPIend\r\n"));
}


//static void yield() {
  // not required!
  // does nothing opn this platform
//}


void Random::init() {
  ADC_read::init(0);
  unsigned int seed = 0;
  for (int i = 0; i < 10000; ++i) seed += ADC_read::adc_value(0);
  srand(seed);
}

int Random::getRandom(int min, int max) {
  int range = max - min + 1;
  return (min + (rand() % range));
}