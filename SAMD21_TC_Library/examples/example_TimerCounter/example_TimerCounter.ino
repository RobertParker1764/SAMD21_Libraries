//This example shows how to use the SAMD21TC Library to create and use the SAMD21
//TC peripheral. This example shows how to create and use two timer instances, an
//8-bit timer/counter and a 16-bit timer counter. Interrupts and callback functions
//will be used to turn LEDs on and off in the 8-bit timer/counter. The compare channels
//of the 16-bit time/counter will be used to turn LEDs on and off directly without
//using interrupts.

#include <Arduino.h>
//Include the SAMD21TC Library file
#include "SAMD21TC_Library.h"

//This example uses a "using directive" to bring all of the library classes and names
//into scope. Alternatively, you can use the scope resolution operator 
//(SAMD21TC::) preceeding each name defined within the SAMD21TC namespace
using namespace SAMD21TC;

//Declare pointers to 8-bit and 16-bit timer/counter objects. They need to be declared 
//here so that they have global scope
TC8Bit * testTimer8 = nullptr;
TC16Bit * testTimer16 = nullptr;


//Define the interrupt handler. The interrupt handler is a standard handler
//name defined in the file samd21g18a.h provided by the Arduino application.
//The interrupt handler just needs to call the class specific interrupt
//handler.
void TC3_Handler(){
  TCBaseClass::tcInterruptHandler(0);
}

void TC4_Handler(){
  TCBaseClass::tcInterruptHandler(1);
}

void TC5_Handler(){
  TCBaseClass::tcInterruptHandler(2);
}

// Timer callbacks
void testTimer8Match0()
{
   digitalWrite(2, LOW);
}

void testTimer8Match1()
{
  digitalWrite(2, HIGH); 
}


void setup() {

  //Initialize the serial port for Serial Monitor
  SerialUSB.begin(9600);  //9600 baud
  while(!SerialUSB);      //Wait for the Serial Monitor
  SerialUSB.println("TC Library Example");

  //Configure the digital outputs used by the interrupt routines
  pinMode(2, OUTPUT);
  //pinMode(3, OUTPUT);

  //Configure the generic clock for the RTC peripheral
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure the generic clock GENCTRL register
  //Set clock source bits (SRC) to XOSC32K (32kHz clock), Set ID bits to generic clock generator #1
  //Set the GENEN (generator enable) bit
  GCLK->GENCTRL.reg = (uint32_t)(GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(0x01) | GCLK_GENCTRL_GENEN);
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure the generic clock control (CLKCTRL) register
  //Set the CLKEN (clock enable) bit, Set the GEN bits (generator ID) to generic clock generator #1
  //Set the ID bits (peripheral module ID) to TC3
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
  //Block until sync is complete
  while (GCLK->STATUS.bit.SYNCBUSY == 1);
  //Configure the generic clock control (CLKCTRL) register again, this time for the TC4/TC5 instance
  //Set the CLKEN (clock enable) bit, Set the GEN bits (generator ID) to generic clock generator #1
  //Set the ID bits (peripheral module ID) to TC4/TC5
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));

  //Instantiate the time/counter instances. Always use the factory methods (e.g. get8BitTC()) to instantiate
  //a new timer rather than calling the class constructors directly. The factory methods perform
  //error checking to ensure that the objects are correctly created and that the TC peripheral
  //constraints are met.
  testTimer8 = TCBaseClass::get8BitTC(_TC3);
  //Always check to make sure we get a good pointer and not just nullptr
  if (!testTimer8) {
    //Do some sort of error recovery here. We will just call exit() to terminate the program
    exit(5);
  }
  
  testTimer16 = TCBaseClass::get16BitTC(_TC4);
  if (!testTimer16) {
    exit(6);
  }

  //8-Bit Timer Configuration
  //The 8-bit timer/counter will be configured as follows:
  //Input clock divided by 64. Since the input clock is 32,768Hz, the clock to the timer will be 512Hz
  //Frequency generation mode set to Normal Frequency Generation (NFRQ)
  bool result = testTimer8->configureTimer(DIVIDE_BY_64, NORMAL_FREQUENCY);
  //Always check the result to make sure the instance was configured properly
  if (!result) {
    //Do some error recovery here. We will just print an erro message to the serial monitor
    SerialUSB.println("Configuration of testTimer8 failed.");
  }
  //Set the period (PER) register to the maximum count value
  //In NFRQ mode the counter period is determined by the value in this register
  testTimer8->setPeriod(0xFF);

  //Set the compare channel registers (CC0 and CC1). We will use the count value in these
  //registers to trigger interrupts that will blink an LED connected to D2
  testTimer8->setCompareChannel0(0x2F); //Match will turn LED on via interrupt
  testTimer8->setCompareChannel1(0xA0); //Match will turn LED off via interrupt

  //Enable interrupts for testTimer8
  testTimer8->enableInterrupt(TC_CHANNEL_0_MATCH, testTimer8Match0);
  testTimer8->enableInterrupt(TC_CHANNEL_1_MATCH, testTimer8Match1);

  //16-Bit Timer configuration
  //The 16-Bit timer/counter will be configured as follows:
  //Input clock undevided so the input clock will be 32768KHz
  //Frequency generation mode set to Normal PWM (NPWM)
  //Waveform outputs are connected to D20 (SDA) and D21 (SCL)
  result = testTimer16->configureTimer(DIVIDE_BY_1, NORMAL_PWM, 20, 21);
  if (!result) {
    //Do some error recovery here. We will just print an erro message to the serial monitor
    SerialUSB.println("Configuration of testTimer16 failed.");
  }

  //Set the compare channel registers (CC0 and CC1). We will use the count value in these
  //registers to trigger the waveform output pins (WO0 and WO1) which will blink
  //LEDS connected to these pins
  testTimer16->setCompareChannel0(0x7F00);   
  testTimer16->setCompareChannel1(0x6000);   

  //The following code prints messages to the serial monitor to confirm
  //the basic timer/counter setups. Remove for your program
  SerialUSB.print("testTimer8 Size: ");
  SAMD21TC::counterSize_t TC_Size = testTimer8->getSize();
  switch (TC_Size)
  {
    case _8BIT:   SerialUSB.println("8 bit");
                  break;
    case _16BIT:  SerialUSB.println("16 bit");
                  break;
    case _32BIT:  SerialUSB.println("32 bit");
                  break;
    default :     SerialUSB.println("invalid size");
  }

  SerialUSB.print("testTimer16 Size: ");
  TC_Size = testTimer16->getSize();
  switch (TC_Size)
  {
    case _8BIT:   SerialUSB.println("8 bit");
                  break;
    case _16BIT:  SerialUSB.println("16 bit");
                  break;
    case _32BIT:  SerialUSB.println("32 bit");
                  break;
    default :     SerialUSB.println("invalid size");
  }

  SerialUSB.print("testTimer8 instance: ");
  uint8_t testTimerInstance = testTimer8->getCounterInstance();
  switch (testTimerInstance)
  {
    case 0:   SerialUSB.println("TC3");
              break;
    case 1:   SerialUSB.println("TC4");
              break;
    case 2:   SerialUSB.println("TC5");
              break;
    default:  SerialUSB.println("Invalid TC instance");
  }

  SerialUSB.print("testTimer16 instance: ");
  testTimerInstance = testTimer16->getCounterInstance();
  switch (testTimerInstance)
  {
    case 0:   SerialUSB.println("TC3");
              break;
    case 1:   SerialUSB.println("TC4");
              break;
    case 2:   SerialUSB.println("TC5");
              break;
    default:  SerialUSB.println("Invalid TC instance");
  }

  //Enable timer/counters. Both timer/counters will start counting now.
  SerialUSB.println("Enabling counter...");
  testTimer8->enableTC();
  testTimer16->enableTC();

  delay(5000);

  //The timer/counters can be stopped and started as needed
  SerialUSB.print("Timer is ");
  if (testTimer8->isStopped()) {
    SerialUSB.println("stopped.");
  } else {
    SerialUSB.println("running.");
  }

  delay(5000);
  SerialUSB.println("Stopping timer ...");
  testTimer8->stopTC();
  
  SerialUSB.print("Timer is ");
  if (testTimer8->isStopped()) {
    SerialUSB.println("stopped.");
  } else {
    SerialUSB.println("running.");
  }

  //Send Retrigger command
  delay(5000);
  SerialUSB.println("Restarting timer ...");
  testTimer8->retriggerTC();
  SerialUSB.print("Timer is ");
  if (testTimer8->isStopped()) {
    SerialUSB.println("stopped.");
  } else {
    SerialUSB.println("running.");
  }

}

void loop() 
{
  //We can print the current count values to the serial monitor
  SerialUSB.print("testTimer8 count = ");
  SerialUSB.println(testTimer8->getCount());
  SerialUSB.print("testTimer16 count = ");
  SerialUSB.println(testTimer16->getCount());
  delay(750);
}
