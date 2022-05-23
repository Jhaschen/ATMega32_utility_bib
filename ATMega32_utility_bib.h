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

// Bibliotheken einbinden
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
  static void uart_println(const char *pstring);
  static char uart_getc(void);
};

#endif // ATMEGA32_UTILITY_BIB
