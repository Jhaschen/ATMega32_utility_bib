#include "ATMega32_utility_bib.h"


#define delay 250


int main(void) {

  // Richtungsregister für die LEDs auf Ausgang
   DDRC = 0b11111111;
   // Alle LEDs aus
   LED_PORT = 0b11111111;

   UART Seriell(8,0,1,9600) ; // Init 8 Zeichenbits, keine Parität, 1 StoppBit, 9600 Bit/s
   Button B; // Neue Instanz der Klasse Button anlegen

  char buffer[50]; //Buffer Array für Zeichenkette (50 Zeichen)
  int8_t Taster;
   while (1)
   {

     Taster =B.Button_read();
     sprintf(buffer," Taster %d gedrückt",Taster); // String Printf: schreibt eine Zeichenkette in den Buffer
     Seriell.uart_puts(buffer);
     Seriell.uart_puts("\n \r");

     LED_PORT= ~Taster ;

    //_delay_ms(delay);

   }


return 0;
}
