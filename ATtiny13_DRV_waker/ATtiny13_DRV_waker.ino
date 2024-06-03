/* ----------- RMXII -----------
 * Use for wake up DRV8245, supervisor concept
 * Toggle 30uS falling edge to nSLEEP pin of DRV8245
 * Reference usage: https://www.allnewstep.com/article/380/%E0%B8%81%E0%B8%B2%E0%B8%A3%E0%B9%80%E0%B8%82%E0%B8%B5%E0%B8%A2%E0%B8%99%E0%B9%82%E0%B8%9B%E0%B8%A3%E0%B9%81%E0%B8%81%E0%B8%A3%E0%B8%A1-arduino-attiny13-arduino-stand-alone
 * Click "Sketch/Upload using Programmer" to uploadcode to ATtiny13
 */

/* ATtiny13A PINOUT
    PB5 |1 U 8| VCC
    PB3 |2   7| PB2
    PB4 |3   6| PB1
    GND |4   5| PB0
*/

uint8_t LED_pin = 3;
uint8_t DRV_start = 4;

uint16_t dela_1 = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_pin, OUTPUT);
  pinMode(DRV_start, OUTPUT);

  //// delay for DRV init
  digitalWrite(DRV_start, HIGH);
  delay(250);

  //// 30uS falling toggle
  digitalWrite(DRV_start, LOW);
  delayMicroseconds(30);
  digitalWrite(DRV_start, HIGH);

}

void loop() {
  //// simulate task, dummy
  // digitalWrite(LED_pin, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(dela_1);                   // wait for a second
  // digitalWrite(LED_pin, LOW);   // turn the LED off by making the voltage LOW
  // delay(dela_1);
}
