#include <ManchesterInterrupts.h>

void setup ()
{
 // incoming on pin D2
 ManchesterInterrupts::dataPin = 2;
 // need a change interrupt to detect incoming data
 attachInterrupt (0, ManchesterInterrupts::isr, CHANGE);

 Serial.begin (115200);
}  // end of setup

void loop ()
{
 if ( ManchesterInterrupts::available ())
   Serial.print ((char) ManchesterInterrupts::read ());
}  // end of loop


