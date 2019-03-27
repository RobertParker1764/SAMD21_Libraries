//This example shows how to use the SAMD21RTC Library to create and
//use a 32-bit counter using the SAMD21 RTC peripheral (RTC mode 0). Uses interrupts
//and callback functions to turn LED13 on and off. Prints the current
//value of the count register each time through the loop. The LED blinking
//period will be about 90 seconds using the 48MHz clock.

#include <Arduino.h>
//Include the SAMD21RTC Library file
#include "SAMD21RTC_Library.hpp"

//Declare and initialize a pointer variable that will point to the timer object
//It needs to be declared here so that it has global scope
//Note that we used the scope resolution operator ("SAMD21RTC::") because the 
//RTC library class and other names are all declared within the SAMD21RTC namespace.
//You may prefer to just use a namespace "using directive" instead at the beginning
//of the file following the include lines (e.g. "using namespace SAMD21RTC")
SAMD21RTC::Counter_32Bit * myTimer = nullptr;

//Define the interrupt handler. The interrupt handler is a standard handler
//name defined in the file samd21g18a.h provided by the Arduino application.
//The interrupt handler just needs to call the class specific interrupt
//handler.
void RTC_Handler()
{
  myTimer->handleInterrupt();
}

//Interrupt callback function definitions
//For every interrupt you plan to use, you must provide a callback funtion
//definition to carry out the required actions. The callback functions
//will always have the same prototype ( void funcName(void) )
//In this case the compare interrupt will turn LED13 on and the overflow 
//interrupt will turn LED13 off.
void compareInterrupt()
{
  digitalWrite(13, HIGH);
}

void overflowInterrupt()
{
  digitalWrite(13, LOW);
}


void setup() {
  //Initialize the serial port Serial Monitor support of debugging.
  //Leave this out if you don't need it
  SerialUSB.begin(9600);  //9600 baud
  while(!SerialUSB);      //Wait for the Serial Monitor
  SerialUSB.println("RTC 32-Bit Counter Example");

  //Configure the generic clock for the RTC peripheral
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure generic clock GENDIV register
  //Set DIV bits to 0 (no division of source clock), Set ID bits to generic clock generator #3
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(0x00) | GCLK_GENDIV_ID(0x3);
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Cofigure the generic clock GENCTRL register
  //Set clock source bits (SRC) to DFLL48M (48MHz clock), Set ID bits to generic clock generator #3
  //Set the GENEN (generator enable) bit
  GCLK->GENCTRL.reg = (uint32_t)(GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(0x03) | GCLK_GENCTRL_GENEN);
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure the generic clock control (CLKCTRL) register
  //Set the CLKEN (clock enable) bit, Set the GEN bits (generator ID) to generic clock generator #3
  //Set the ID bits (peripheral module ID) to real time clock
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID(GCM_RTC));

  //Configure the digital output that will be used for the interrupt routines
  pinMode(13, OUTPUT);

  //Instantiate the real time clock
  myTimer = SAMD21RTC::RTC_Base_Class::get32BitRTC();
  //Always check to make sure we got a good pointer and not just nullptr
  if (!myTimer) {
    //Do some sort of error recovery here. We will just call exit() to terminate the program
    exit(5);
  }
  //Configure the 32 bit counter
  //Notice that we need to use the pointer membership operator (->) instead of the object membership
  //operator (.)
  //Here we will not divide the clock source any further and use the default behavior of not clearing
  //on count match to the compare register
  myTimer->configureRTC(SAMD21RTC::DIVIDE_BY_1);
  //Set the compare register to a value about half way to the max value
  myTimer->setCompareValue(0x7FFFFFFF);
  //Enable the interrupts. Pass the callback funtions as parameters
  myTimer->enableInterrupt(SAMD21RTC::RTC_OVERFLOW, overflowInterrupt);
  myTimer->enableInterrupt(SAMD21RTC::RTC_COMPARE_0, compareInterrupt);
  //Enable the 32 bit counter;
  myTimer->enableRTC();
}

void loop() {
  SerialUSB.print("Count: ");
  SerialUSB.println(myTimer->getCount());
  delay(100); //Arbitrary delay insterted here. You will put your loop code here

}
