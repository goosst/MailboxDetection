/*/
 * sg: created 25 july 2020
 * arduino pro mini is used to work on two aa-batteries up to a very low voltage
 * functions: 
 * - enable and disable the dc-converter which powers ESP and VL53
 * - communicates with ESP on status and goes into sleep mode
 * to be able to work up to a low voltage of 1.8V on the atmega238p, 
 * a different bootloader needs to be flashed using the 1MHz internal oscillation: https://mcudude.github.io/MiniCore/
 * functions like delay(...), don't really have the correct timing (e.g. factor 16 wrong)
 */
 
#include <LowPower.h>

const bool debug = false;

const byte D_DCDC = 6; //pin D6 enables and disables dc-converter, pulled to ground with external resistor
const byte pin_informarduino = 2; //connected to D3 on wemos D1 (pull up resistor inside wemos)
const byte pin_reset_esp = 5; //to wake up wemos out of deep sleep (pull up resistor inside wemos)

volatile int pwm_value = 0;
volatile bool gotosleep = false;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D_DCDC, OUTPUT);
  pinMode(pin_reset_esp, OUTPUT);
  pinMode(pin_informarduino, INPUT_PULLUP);
}


void loop()
{

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(D_DCDC, HIGH); //turn dc-converter on

  digitalWrite(pin_reset_esp, LOW); //in case the esp was still sleeping
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(pin_reset_esp, HIGH); //in case the esp was still sleeping
  digitalWrite(LED_BUILTIN, HIGH);

  for (int ii = 0; ii < 20; ii++)
  {
    pwm_value = digitalRead(pin_informarduino);

    if (pwm_value != 0)
    {
      gotosleep = false;
    }
    else
    {
      gotosleep = true;
    }


    if (gotosleep)
    {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(D_DCDC, LOW); // turn off dc-dc converter
      digitalWrite(pin_reset_esp, HIGH);
      delay(100);

      //go to sleep
      for (int i = 1; i < 20; i++)
      {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      }

    }

    delay(300);

  }

  delay(500);

}


//void hightolow()
//{
//  timestamp_changingedge=millis();
//  attachInterrupt(digitalPinToInterrupt(pin_informarduino), lowtohigh, FALLING);
//}
//
//
//void lowtohigh()
//{
//
//  timestamp_changingedge=millis();
//}
