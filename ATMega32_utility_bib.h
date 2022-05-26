/*
 * ATMega32_utility_bib.h
 *Created 18.05.2022
 * Author: Jan-Hendrik Aschen
 */

#ifndef ATMEGA32_UTILITY_BIB_H
#define ATMEGA32_UTILITY_BIB_H

#ifndef F_CPU
#define F_CPU 16000000 // CPU Taktfrequenz
#endif
#ifndef __AVR_ATmega32__ // ATMega32 Bibliothek
#define __AVR_ATmega32__ // ATMega32 Bibliothek
#endif

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLISECONDS_PER_TIMER0_OVERFLOW (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// From: https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring.c
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

// Bibliotheken einbinden
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

// Makro definitionen
#define SET_BIT(REG, BIT) REG |= (1 << BIT)            // Makro setzen des Bit BIT in Register REG
#define CLR_BIT(REG, BIT) REG &= ~(1 << BIT)           // Makro löschen des Bit BIT in Register REG
#define TGL_BIT(REG, BIT) REG ^= (1 << BIT)            // Makro komplemntieren des Bit BIT in Register REG
#define CLR_LED(REG, BIT) REG |= (1 << BIT)            // Makro LED ausschalten (umgekehrte Logik)
#define SET_LED(REG, BIT) REG &= ~(1 << BIT)           // Makro LED einschalten
#define BIT_IS_SET(REG, BIT) ((REG & (1 << BIT)) != 0) // Testen, ob Bit BIT im Register REG gesetzt ist (1)
#define BIT_IS_CLR(REG, BIT) ((REG & (1 << BIT)) == 0) // Testen, ob Bit BIT im Register REG  nicht gesetzt ist (0)

#define LED_PORT PORTC

#define LED_0 0
#define LED_1 1
#define LED_2 2
#define LED_3 3
#define LED_4 4
#define LED_5 5
#define LED_6 6
#define LED_7 7

#define USART_EVEN_PARITY 1
#define USART_ODD_PARITY 2

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
//#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define HIGH (1)
#define LOW (0)

// decimal
#define DEC    (10)

// I/O
#define INPUT (0)
#define OUTPUT (1)
#define INPUT_PULLUP (2)

// Taktflanken:
#define CHANGE  (1)
#define FALLING (2)
#define RISING  (3)

// ADC_init und ADC_read
class ADC_read
{
public:
  static void init(uint8_t _channel);
  static uint16_t adc_value(void);

private:
  static uint8_t channel;
};

// Button einlesen
class Button
{
public:
  static void init();
  static int8_t Button_read(void);
};

// UART Schnittstelle
class UART
{
public:
  static void init(uint32_t _Baudrate, uint8_t _CharBits=8, uint8_t _ParBit=0, uint8_t _StopBits=1);
  static void uart_putc(char data);
  static void uart_puts(const char *pstring);
  static void uart_printf(const char* format, ...);
  static void uart_printf_P(const char* format, ...);
  static void uart_println(const char *pstring);
  static char uart_getc(void);
};


// Timer0,
class Timer0
{
public:
  static void init();
  static unsigned long millis();
  static unsigned long micros();
  static void delay(unsigned long ms);
  static void delayMicroseconds(unsigned int us);
};

// Arduino compatibility functions:
uint8_t digitalPinToInterrupt(uint8_t interruptPin);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
uint8_t digitalRead(uint8_t pin);
void tone(uint8_t pin, unsigned int frequency, unsigned long duration);
void noTone(uint8_t pin);
void attachInterrupt(uint8_t interrupt, void(*userFunction)(void),  uint8_t mode);
void detachInterrupt(uint8_t intterupt);
void yield();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long millis();
unsigned long micros();
void SPIbegin();
void SPIbeginTransaction();
uint8_t SPItransfer (uint8_t b);
void SPIendTransaction();
void SPIend();
#endif // ATMEGA32_UTILITY_BIB
