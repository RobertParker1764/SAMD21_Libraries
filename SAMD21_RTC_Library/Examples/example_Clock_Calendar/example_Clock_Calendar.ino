//This example shows how to use the SAMD21RTC Library to create and
//use a real time clock/calendar using the SAMD21 RTC peripheral (RTC
//peripheral mode 2). Prints the current time/date to the serial monitor

#include <Arduino.h>
//Include the SAMD21RTC Library file
#include "SAMD21RTC_Library.hpp"

//Declare and initialize a pointer variable that will point to the timer object
//It needs to be declared here so that it has global scope
//Note that we used the scope resolution operator ("SAMD21RTC::") because the 
//RTC library class and other names are all declared within the SAMD21RTC namespace.
//You may prefer to just use a namespace "using directive" instead at the beginning
//of the file following the include lines (e.g. "using namespace SAMD21RTC")
SAMD21RTC::ClockCalendar * myTimer = nullptr;

//Define the interrupt handler. The interrupt handler is a standard handler
//name defined in the file samd21g18a.h provided by the Arduino application.
//The interrupt handler just needs to call the class specific interrupt
//handler.
void RTC_Handler()
{
  myTimer->handleInterrupt();
}

//Interrupt callback function definition
//For every interrupt you plan to use, you must provide a callback funtion
//definition to carry out the required actions. The callback functions
//will always have the same prototype ( void funcName(void) )
//In this case the alarm interrupt will turn LED13 on and send a message
//to the serial monitor
void alarmInterrupt()
{
  digitalWrite(13, HIGH);
  SerialUSB.println("Alarm");
}


void setup() {
  //Initialize the serial port Serial Monitor support of debugging.
  //Leave this out if you don't need it
  SerialUSB.begin(9600);  //9600 baud
  while(!SerialUSB);      //Wait for the Serial Monitor
  SerialUSB.println("Real Time Clock/Calendar Example");

  //Configure the generic clock for the RTC peripheral
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure generic clock GENDIV register
  //Set DIV bits to 0x20 (divide by 32), Set ID bits to generic clock generator #3
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(0x20) | GCLK_GENDIV_ID(0x3);
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure the generic clock GENCTRL register
  //Set clock source bits (SRC) to XOSC32K (32KHz clock), Set ID bits to generic clock generator #3
  //Set the GENEN (generator enable) bit
  GCLK->GENCTRL.reg = (uint32_t)(GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(0x03) | GCLK_GENCTRL_GENEN);
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure the generic clock control (CLKCTRL) register
  //Set the CLKEN (clock enable) bit, Set the GEN bits (generator ID) to generic clock generator #3
  //Set the ID bits (peripheral module ID) to real time clock
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID(GCM_RTC));

  //Configure the digital output that will be used for the interrupt routines
  pinMode(13, OUTPUT);

  //Instantiate the real time clock
  myTimer = SAMD21RTC::RTC_Base_Class::getRTCClock();
  //Always check to make sure we got a good pointer and not just nullptr
  if (!myTimer) {
    //Do some sort of error recovery here. We will just call exit() to terminate the program
    exit(5);
  }
  //Configure the clock/calendar
  //Notice that we need to use the pointer membership operator (->) instead of the object membership
  //operator (.)
  //We need to divide the clock source by 1,024 to get the 1Hz clock required by the RTC
  //Also set the mode to 12 hour time
  myTimer->configureRTC(SAMD21RTC::DIVIDE_BY_1024, SAMD21RTC::_12HOUR);
  //Set the clock/calendar to the desired date and time (March 26, 2019, 7:23:0 PM)
  myTimer->setClockCalendar(2019, SAMD21RTC::MARCH, 26, 7, 23, 0, SAMD21RTC::_PM);
  //Set the alarm register to a value a minute ahead of the initial clock setting 
  myTimer->setAlarm(2019, SAMD21RTC::MARCH, 26, 7, 24, 30, SAMD21RTC::HHMMSS, SAMD21RTC::_PM);
  //Enable the alarm interrupt. Pass the callback funtions as parameters
  myTimer->enableInterrupt(SAMD21RTC::RTC_ALARM, alarmInterrupt);
  //Enable the real time clock/calendar bit counter;
  myTimer->enableRTC();
}

void loop() {
  SerialUSB.println(myTimer->getDate());
  SerialUSB.println(myTimer->getTime());
  delay(1000); //Arbitrary delay insterted here. You will put your loop code here

}
