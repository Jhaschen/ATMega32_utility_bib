#include "ATMega32_utility_bib.h"


#define debug // Ausgabe ADC-Werte für die Taster

#define delay 250








void uart_init(uint8_t CharBits, uint8_t ParBit, uint8_t StopBits, uint32_t Baudrate)
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

void uart_putc(char data)
{
  // Warten, dass Senderegister frei ist
while (BIT_IS_CLR(UCSRA,UDRE));
// Jetzt ist das Senderegister frei, also
  // senden wir ein Zeichen einfach durch Schreiben
  // in das Senderegister UDR
UDR = data;

}

void uart_puts(char* pstring)
{
 char* pdata = pstring;

 while (*pdata !=0) {
   uart_putc(*pdata);
   pdata++;
 }

}


char uart_getc(void)
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




int main(void) {

  // Richtungsregister für die LEDs auf Ausgang
   DDRC = 0b11111111;

   // Alle LEDs aus
   LED_PORT = 0b11111111;

   uart_init(8,0,1,9600) ; // Init 8 Zeichenbits, keine Parität, 1 StoppBit, 9600 Bit/s
   // Led 0 an zur Kontrolle, dass wir bis hier im Programm gekommen sind.
   Button B;
   CLR_BIT(LED_PORT,0);
   // Ausgabe einer Meldung zur Aufgabeeinforderung

   char buffer[50]; //Buffer Array für Zeichenkette (50 Zeichen)

  int8_t Taster;
   while (1)
   {

     Taster =B.Button_read();
     sprintf(buffer," Taster %d gedrückt",Taster); // String Printf: schreibt eine Zeichenkette in den Buffer
     uart_puts(buffer);
     uart_puts("\n \r");

     LED_PORT= ~Taster ;

    _delay_ms(delay);

   }


return 0;
}
