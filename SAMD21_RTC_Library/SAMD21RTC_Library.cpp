//SAMD21G18 Real-Timer Counter (RTC) Peripheral Driver
//Version 1.0
//March 24, 2019
//By Robert Parker

/*
 Copyright (c) 2019 Robert Parker. All right reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include "SAMD21RTC_Library.hpp"

namespace SAMD21RTC {
    
    
    /******************** Static Class Member Initialization *****************************/
    
    //This static variable is initially set to false. When an RTC instance is created
    //the variable is set to true, preventing more than one instance from being created.
    bool RTC_Base_Class::RTC_Allocated = false;
    
    /******************** Static Member Function Definition ******************************/
    
    //=====================================================================================
    //get32BitRTC()
    //User interface to allocate new 32-bit RTC driver object. All 32-bit RTC objects
    //should be instantiated using this method to ensure valid objects are created.
    //Parameters: None
    //Return:   A pointer to the newly created obect or nullptr if an object could
    //          not be created. User should ALWAYS check for nullptr to make sure
    //          a valid object was created before trying to use it.
    //=====================================================================================
    
    Counter_32Bit * RTC_Base_Class::get32BitRTC()
    {
        Counter_32Bit * rtc;
        
        //Make sure no other rtc instance has been created. If it has retrn the nullptr
        if (RTC_Allocated) {
            return nullptr;
        }
        
        //Instantiate the requested timer/counter instance, mark the timer number as used
        //and return a pointer to the timer/counter instance
        rtc = new Counter_32Bit();
        RTC_Allocated = true;
        
        return rtc;
    }
    
    //=====================================================================================
    //get16BitRTC()
    //User interface to allocate new 16-bit RTC driver object. All 16-bit RTC objects
    //should be instantiated using this method to ensure valid objects are created.
    //Parameters: None
    //Return:   A pointer to the newly created obect or nullptr if an object could
    //          not be created. User should ALWAYS check for nullptr to make sure
    //          a valid object was created before trying to use it.
    //=====================================================================================
    
    Counter_16Bit * RTC_Base_Class::get16BitRTC()
    {
        Counter_16Bit * rtc;
        
        //Make sure no other rtc instance has been created. If it has retrn the nullptr
        if (RTC_Allocated) {
            return nullptr;
        }
        
        //Instantiate the requested timer/counter instance, mark the timer number as used
        //and return a pointer to the timer/counter instance
        rtc = new Counter_16Bit();
        RTC_Allocated = true;
        
        return rtc;
    }
    
    //=====================================================================================
    //getRTCClock()
    //User interface to allocate new RTC Clock driver object. All RTC Clock objects
    //should be instantiated using this method to ensure valid objects are created.
    //Parameters: None
    //Return:   A pointer to the newly created obect or nullptr if an object could
    //          not be created. User should ALWAYS check for nullptr to make sure
    //          a valid object was created before trying to use it.
    //=====================================================================================
    
    ClockCalendar * RTC_Base_Class::getRTCClock()
    {
        ClockCalendar * rtc;
        
        //Make sure no other rtc instance has been created. If it has retrn the nullptr
        if (RTC_Allocated) {
            return nullptr;
        }
        
        //Instantiate the requested timer/counter instance, mark the timer number as used
        //and return a pointer to the timer/counter instance
        rtc = new ClockCalendar();
        RTC_Allocated = true;
        
        return rtc;
    }

    
    //=========================================================================================
    //******************* RTCBaseClass Member Function Definitions ****************************
    //=========================================================================================
    
    //Constructor
    RTC_Base_Class::RTC_Base_Class()
    {
        pRegisterBaseAddress = RTC;     //RTC is a constant defined in include/samd21g18a.h
        
        //Initialize the array of callback function pointers to all nullptr
        for (int index = 0; index < 8; index++) {
            callBackFuncPtrs[index] = nullptr;
        }
        
        //Enable the user interface clock for the RTC peripheral
        PM->APBAMASK.reg |= PM_APBAMASK_RTC;
    }
    
    //Public Interface
    
    //=====================================================================================
    //enableRTC()
    //Enables operation of the SAMD21 hardware RTC instance. Sets the ENABLE bit
    //in the control register.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void RTC_Base_Class::enableRTC()
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
    }
    
    //=====================================================================================
    //disableRTC()
    //Disables operation of the SAMD21 hardware RTC instance. Clears the ENABLE
    //bit in the control register.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void RTC_Base_Class::disableRTC()
    {
        //TODO: Need to disable interrupts here
        
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE;
    }
    
    //=====================================================================================
    //resetRTC()
    //Issues a software reset to the SAMD21 hardware RTC instance. Resets all RTC
    //registers (except DBGCTRL) to their initial state. The RTC will be disabled.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void RTC_Base_Class::resetRTC()
    {
        disableRTC();
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.CTRL.reg = 0x1;   //Set the SWRST bit
        //Block until software reset is complete (SWRST bit is cleared)
        while (pRegisterBaseAddress->MODE0.CTRL.bit.SWRST == 1);
    }
    
    //=====================================================================================
    //requestClockCountSync()
    //Initiates a request for read synchronization of the COUNT or CLOCK registers. If
    //continuous is true then the RCONT bit will also be set and continuous synchronization
    //of the COUNT or CLOCK registers will be enabled.
    //Parameters:
    //  continuous: If true then continuous synchronization will be enabled.
    //Return: None
    //=====================================================================================
    void RTC_Base_Class::requestClockCountSync(bool continuous)
    {
        uint16_t value = 0x8000;
        if (continuous) {
            value |= 0x4000;
        }
        pRegisterBaseAddress->MODE0.READREQ.reg = value;
    }
    
    //=====================================================================================
    //setFrequencyCorrection()
    //Loads the FREQCORR register with the desired prescaler correction factor to adjust
    //the output frequency in approximate 1ppm steps. If speedUp is true then the prescaler
    //output frequency is increased; it is slowed otherwise.
    //Parameters:
    //  correction: The correction value in approx. 1ppm steps
    //  speedUp:    If true the frequency is increased. If false it is decreased.
    //Return: None
    //=====================================================================================
    void RTC_Base_Class::setFrequencyCorrection(uint8_t correction, bool speedUp)
    {
        uint8_t value = correction;
        if (!speedUp) {
            value |= 0x80;
        }
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.FREQCORR.reg = value;
    }
    
    //=====================================================================================
    //enableEvents()
    //Enables the requested events. This function must be called before the RTC is enabled.
    //Calling this fuction when the RTC is enabled will have no effect. Note that the
    //SAMD21G18 Event System peripheral must be setup before RTC events can be used to
    //signal other peripherals.
    //Parameters:
    //  events: The events to be enabled (bits set)
    //Return: None
    //=====================================================================================
    void RTC_Base_Class::enableEvents(rtcEvent_t events)
    {
        if (pRegisterBaseAddress->MODE0.CTRL.bit.ENABLE) {
            return;
        }
        pRegisterBaseAddress->MODE0.EVCTRL.reg = events;
    }
    
    
    //Low Level Public Interface
    
    //=====================================================================================
    //setControlRegister()
    //Writes the supplied value to the CTRL register. The RTC must be disabled before
    //writing to the control register. If the RTC is enabled then writes to the control
    //register are ignored.
    //Parameters:
    //  value: New 16-bit value for CTRL register
    //Return: None
    //=====================================================================================
    void RTC_Base_Class::setControlRegister(uint16_t value)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.CTRL.reg = value;
    }
    
    //=====================================================================================
    //getControlRegister()
    //Reads the current CTRL register value and returns it.
    //Parameters: None
    //Return: The 16-bit CTRL register value
    //=====================================================================================
    uint16_t RTC_Base_Class::getControlRegister() const
    {
        return pRegisterBaseAddress->MODE0.CTRL.reg;
    }
    
    
    //=========================================================================================
    //****************** Counter_32Bit Member Function Definitions ****************************
    //=========================================================================================
    
    //=====================================================================================
    //configureRTC()
    //Sets the PRESCALER, MODE, and MATCHCLR bits in the CTRL register
    //Parameters:
    //  prescale:   A prescaler division factor of type rtcClockPrescaler_t
    //  clearOnMatch:   If true the MATCHCLR bit in the CTRL register will be set. It will
    //                  be cleared otherwise.
    //Return: None
    //=====================================================================================

    void Counter_32Bit::configureRTC(rtcClockPrescaler_t prescale, bool clearOnMatch)
    {
        //Make sure we are disabled first
        disableRTC();
        
        uint16_t value = prescale | RTC_MODE0_CTRL_MODE_COUNT32;
        if (clearOnMatch) {
            value |= RTC_MODE0_CTRL_MATCHCLR;
        }
        
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.CTRL.reg = value;
    }
    
    //=====================================================================================
    //setCount()
    //Sets the COUNT register to the specified value
    //Parameters:
    //  count:   The value to be loaded into the COUNT register
    //Return: None
    //=====================================================================================
    void Counter_32Bit::setCount(uint32_t count)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.COUNT.reg = count;
    }
    
    //=====================================================================================
    //getCount()
    //Gets the current value of the COUNT register
    //Parameters: None
    //Return: The current 32-bit value of the COUNT register
    //=====================================================================================
    uint32_t Counter_32Bit::getCount() const
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        return pRegisterBaseAddress->MODE0.COUNT.reg;
    }
    
    //=====================================================================================
    //setCompareValue()
    //Sets the COMP register to the specified value
    //Parameters:
    //  compareValue:   The value to be loaded into the COMP register
    //Return: None
    //=====================================================================================
    void Counter_32Bit::setCompareValue(uint32_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE0.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE0.COMP[0].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareValue()
    //Gets the current value of the COMP register
    //Parameters: None
    //Return: The current 32-bit value of the COMP register
    //=====================================================================================
    uint32_t Counter_32Bit::getCompareValue() const
    {
        return pRegisterBaseAddress->MODE0.COMP[0].reg;
    }
    
    //Interrupt Management
    
    //=====================================================================================
    //getInterruptFlags()
    //Returns the current value of the Interrupt Flag Status and Clear register (INTFLAG)
    //Parameters: None
    //Return: Current value of INTFLAG register
    //=====================================================================================
    uint8_t Counter_32Bit::getInterruptFlags() const
    {
        return (pRegisterBaseAddress->MODE0.INTFLAG.reg) & (MODE0_INTR_MASK);
    }
    
    //=====================================================================================
    //clearInterruptFlags()
    //Clears the specified interrupt flags
    //Parameters:
    //  flags:  A value specifying one or more interrupt flags to clear. Valid values are
    //          RTC_OVERFLOW RTC_SYNCRDY, RTC_COMPARE_0, or any combination of these
    //Return: None
    //=====================================================================================
    void Counter_32Bit::clearInterruptFlags(uint8_t flags)
    {
        pRegisterBaseAddress->MODE0.INTFLAG.reg = flags;
    }
    
    //=====================================================================================
    //getInterruptEnableState()
    //Returns the current value of the Interrupt Enable register (INTENCLR)
    //Parameters: None
    //Return: Current value of INTENCLR register
    //=====================================================================================
    uint8_t Counter_32Bit::getInterruptEnableState()
    {
        return pRegisterBaseAddress->MODE0.INTENCLR.reg & MODE0_INTR_MASK;
    }
    
    //=====================================================================================
    //enableInterrupt()
    //Enables the specified interrupt and registers the callback function for later use.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. RTC_OVERFLOW)
    //  callbackFunc:   Pointer to the interrupt callback function
    //Return: None
    //=====================================================================================
    void Counter_32Bit::enableInterrupt(rtcInterrupt_t interuptName, void(* callbackFunc)(void))
    {
        //Save the callback function pointer
        switch (interuptName) {
            case RTC_OVERFLOW:
                callBackFuncPtrs[7] = callbackFunc;
                break;
            case RTC_SYNCRDY:
                callBackFuncPtrs[6] = callbackFunc;
                break;
            case RTC_COMPARE_0:
                callBackFuncPtrs[0] = callbackFunc;
                break;
            default:
                return;
        }
        
        //Enable RTC interrupts in the Nested Vector Interrupt Controller (NVIC)
        //RTC_IRQn is defined in the file samd21g18a.h
        //Reference Table 11-3 in the SAMD21G user manual/data sheet. The
        //NVIC_ClearPendingIRQ() and NVIC_EnableIRQ() are inline functions provided
        //via the Atmel CMSIS software and are contained in file core_cm0plus.h
        //Clear any pending interrupts for the associated IRQ Number
        NVIC_ClearPendingIRQ(RTC_IRQn);
        //Enable interrupts for the associated IRQ Number
        NVIC_EnableIRQ(RTC_IRQn);
        
        /* Enable TC interrupt */
        pRegisterBaseAddress->MODE0.INTENSET.reg = interuptName;
    }
    
    //=====================================================================================
    //disableInterrupt()
    //Disables the specified interrupt and clears the registered callback function.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. RTC_OVERFLOW)
    //Return: None
    //=====================================================================================
    void Counter_32Bit::disableInterrupt(rtcInterrupt_t interuptName)
    {
        //Set the corresponding callback function to nullptr
        switch (interuptName) {
            case RTC_OVERFLOW:
                callBackFuncPtrs[7] = nullptr;
                break;
            case RTC_SYNCRDY:
                callBackFuncPtrs[6] = nullptr;
                break;
            case RTC_COMPARE_0:
                callBackFuncPtrs[0] = nullptr;
                break;
            default:
                return;
                
        //Disable the indicated interrupt
        pRegisterBaseAddress->MODE0.INTENCLR.reg = interuptName;
        
        }
    }
    
    //=====================================================================================
    //handleInterrupt()
    //Examines the interrupt flag register (INTFLAG) and handles any enabled interrupts
    //by calling the associated user provided callback functions.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void Counter_32Bit::handleInterrupt()
    {
        //Retrieve the interrupt flag state
        uint8_t interruptFlags = pRegisterBaseAddress->MODE0.INTFLAG.reg;
        //Clear the INTFLAG register
        pRegisterBaseAddress->MODE0.INTFLAG.reg = MODE0_INTR_MASK;
        
        
        //For each set interrupt flag check for a callback function (enabled interrupts
        //will always have a callback function) and call it if it exists
        uint8_t index = 0;
        uint8_t flag = 0x01;
        while (interruptFlags) {
            if ((index >= 1) && (index <= 5)) {
                index++;
                flag <<= 1;
                continue;
            }
            
            if ((interruptFlags & flag) && callBackFuncPtrs[index]) {
                //Call the interrupt callback function
                callBackFuncPtrs[index]();
            }
            
            //Clear the flag that we just examined
            interruptFlags &= ~flag;
            
            //Increment for next loop
            flag <<= 1;
            index++;
            
        }

    }
    
    //=========================================================================================
    //****************** Counter_16Bit Member Function Definitions ****************************
    //=========================================================================================
    
    //=====================================================================================
    //configureRTC()
    //Sets the PRESCALER and MODE bits in CTRL register
    //Parameters:
    //  prescale:   A prescaler division factor of type rtcClockPrescaler_t
    //Return: None
    //=====================================================================================
    void Counter_16Bit::configureRTC(rtcClockPrescaler_t prescale)
    {
        //Make sure we are disabled first
        disableRTC();
        
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE1.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE1.CTRL.reg = prescale | RTC_MODE1_CTRL_MODE_COUNT16;
    }
    
    //=====================================================================================
    //setCount()
    //Sets the COUNT register to the specified value
    //Parameters:
    //  count:   The value to be loaded into the COUNT register
    //Return: None
    //=====================================================================================
    void Counter_16Bit::setCount(uint16_t count)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE1.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE1.COUNT.reg = count;
    }
    
    //=====================================================================================
    //getCount()
    //Gets the current value of the COUNT register
    //Parameters: None
    //Return: The current 16-bit value of the COUNT register
    //=====================================================================================
    uint16_t Counter_16Bit::getCount() const
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE1.STATUS.bit.SYNCBUSY == 1);
        return pRegisterBaseAddress->MODE1.COUNT.reg;
    }
    
    //=====================================================================================
    //setPeriod()
    //Sets the PER register to the specified value
    //Parameters:
    //  period:   The value to be loaded into the PER register
    //Return: None
    //=====================================================================================
    void Counter_16Bit::setPeriod(uint16_t period)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE1.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE1.PER.reg = period;
    }
    
    //=====================================================================================
    //getPeriod()
    //Gets the current value of the PER register
    //Parameters: None
    //Return: The current 16-bit value of the PER register
    //=====================================================================================
    uint16_t Counter_16Bit::getPeriod() const
    {
        return pRegisterBaseAddress->MODE1.PER.reg;
    }
    
    //=====================================================================================
    //setCompareValue0()
    //Sets the COMP[0] register to the specified value
    //Parameters:
    //  compareValue:   The value to be loaded into the COMP[0] register
    //Return: None
    //=====================================================================================
    void Counter_16Bit::setCompareValue0(uint16_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE1.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE1.COMP[0].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareValue0()
    //Gets the current value of the COMP[0] register
    //Parameters: None
    //Return: The current 16-bit value of the COMP[0] register
    //=====================================================================================
    uint16_t Counter_16Bit::getCompareValue0() const
    {
        return pRegisterBaseAddress->MODE1.COMP[0].reg;
    }
    
    //=====================================================================================
    //setCompareValue1()
    //Sets the COMP[1] register to the specified value
    //Parameters:
    //  compareValue:   The value to be loaded into the COMP[1] register
    //Return: None
    //=====================================================================================
    void Counter_16Bit::setCompareValue1(uint16_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE1.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE1.COMP[1].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareValue1()
    //Gets the current value of the COMP[1] register
    //Parameters: None
    //Return: The current 16-bit value of the COMP[1] register
    //=====================================================================================
    uint16_t Counter_16Bit::getCompareValue1() const
    {
        return pRegisterBaseAddress->MODE1.COMP[1].reg;
    }
    
    //=====================================================================================
    //getInterruptFlags()
    //Returns the current value of the Interrupt Flag Status and Clear register (INTFLAG)
    //Parameters: None
    //Return: Current value of INTFLAG register
    //=====================================================================================
    uint8_t Counter_16Bit::getInterruptFlags() const
    {
        return (pRegisterBaseAddress->MODE1.INTFLAG.reg) & (MODE1_INTR_MASK);
    }
    
    //=====================================================================================
    //clearInterruptFlags()
    //Clears the specified interrupt flags
    //Parameters:
    //  flags:  A value specifying one or more interrupt flags to clear. Valid values are
    //          RTC_OVERFLOW RTC_SYNCRDY, RTC_COMPARE_0, RTC_COMPARE_1, or any combination
    //          of these
    //Return: None
    //=====================================================================================
    void Counter_16Bit::clearInterruptFlags(uint8_t flags)
    {
        pRegisterBaseAddress->MODE1.INTFLAG.reg = flags;
    }
    
    //=====================================================================================
    //getInterruptEnableState()
    //Returns the current value of the Interrupt Enable register (INTENCLR)
    //Parameters: None
    //Return: Current value of INTENCLR register
    //=====================================================================================
    uint8_t Counter_16Bit::getInterruptEnableState()
    {
        return pRegisterBaseAddress->MODE1.INTENCLR.reg & MODE1_INTR_MASK;
    }
    
    //=====================================================================================
    //enableInterrupt()
    //Enables the specified interrupt and registers the callback function for later use.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. RTC_OVERFLOW)
    //  callbackFunc:   Pointer to the interrupt callback function
    //Return: None
    //=====================================================================================
    void Counter_16Bit::enableInterrupt(rtcInterrupt_t interuptName, void(* callbackFunc)(void))
    {
        //Save the callback function pointer
        switch (interuptName) {
            case RTC_OVERFLOW:
                callBackFuncPtrs[7] = callbackFunc;
                break;
            case RTC_SYNCRDY:
                callBackFuncPtrs[6] = callbackFunc;
                break;
            case RTC_COMPARE_0:
                callBackFuncPtrs[0] = callbackFunc;
                break;
            case RTC_COMPARE_1:
                callBackFuncPtrs[1] = callbackFunc;
                break;
            default:
                return;
        }
        
        //Enable RTC interrupts in the Nested Vector Interrupt Controller (NVIC)
        //RTC_IRQn is defined in the file samd21g18a.h
        //Reference Table 11-3 in the SAMD21G user manual/data sheet. The
        //NVIC_ClearPendingIRQ() and NVIC_EnableIRQ() are inline functions provided
        //via the Atmel CMSIS software and are contained in file core_cm0plus.h
        //Clear any pending interrupts for the associated IRQ Number
        NVIC_ClearPendingIRQ(RTC_IRQn);
        //Enable interrupts for the associated IRQ Number
        NVIC_EnableIRQ(RTC_IRQn);
        
        /* Enable TC interrupt */
        pRegisterBaseAddress->MODE1.INTENSET.reg = interuptName;
    }
    
    //=====================================================================================
    //disableInterrupt()
    //Disables the specified interrupt and clears the registered callback function.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. RTC_OVERFLOW)
    //Return: None
    //=====================================================================================
    void Counter_16Bit::disableInterrupt(rtcInterrupt_t interuptName)
    {
        //Set the corresponding callback function to nullptr
        switch (interuptName) {
            case RTC_OVERFLOW:
                callBackFuncPtrs[7] = nullptr;
                break;
            case RTC_SYNCRDY:
                callBackFuncPtrs[6] = nullptr;
                break;
            case RTC_COMPARE_0:
                callBackFuncPtrs[0] = nullptr;
                break;
            case RTC_COMPARE_1:
                callBackFuncPtrs[1] = nullptr;
                break;
            default:
                return;
                
                //Disable the indicated interrupt
                pRegisterBaseAddress->MODE1.INTENCLR.reg = interuptName;
                
        }
    }
    
    //=====================================================================================
    //handleInterrupt()
    //Examines the interrupt flag register (INTFLAG) and handles any enabled interrupts
    //by calling the associated user provided callback functions.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void Counter_16Bit::handleInterrupt()
    {
        //Retrieve the interrupt flag state
        uint8_t interruptFlags = pRegisterBaseAddress->MODE1.INTFLAG.reg;
        //Clear the INTFLAG register
        pRegisterBaseAddress->MODE1.INTFLAG.reg = MODE1_INTR_MASK;
        
        
        //For each set interrupt flag check for a callback function (enabled interrupts
        //will always have a callback function) and call it if it exists
        uint8_t index = 0;
        uint8_t flag = 0x01;
        while (interruptFlags) {
            if ((index >= 2) && (index <= 5)) {
                index++;
                flag <<= 1;
                continue;
            }
            
            if ((interruptFlags & flag) && callBackFuncPtrs[index]) {
                //Call the interrupt callback function
                callBackFuncPtrs[index]();
            }
            
            //Clear the flag that we just examined
            interruptFlags &= ~flag;
            
            //Increment for next loop
            flag <<= 1;
            index++;
            
        }
    }

    //=========================================================================================
    //****************** ClockCalendar Member Function Definitions ****************************
    //=========================================================================================

    //=====================================================================================
    //configureRTC()
    //Sets the PRESCALER, MATCHCLR, CLKREP, and MODE bits in CTRL register. Disables the RTC
    //Parameters:
    //  prescale:   A prescaler division factor of type rtcClockPrescaler_t
    //  mode:       _12HOUR or _24HOUR mode
    //  clearOnMatch:   If true the MATCHCLR bit in the CTRL register will be set. It will
    //                  be cleared otherwise.
    //Return: None
    //=====================================================================================
    void ClockCalendar::configureRTC(rtcClockPrescaler_t prescale, rtcClockMode_t mode,
                                     bool clearOnMatch)
    {
        //Make sure we are disabled first
        disableRTC();
        
        uint16_t value = prescale | RTC_MODE2_CTRL_MODE_CLOCK;
        if (clearOnMatch) {
            value |= RTC_MODE2_CTRL_MATCHCLR;
        }
        if (mode == _12HOUR) {
            value |= RTC_MODE2_CTRL_CLKREP;
        }
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE2.CTRL.reg = value;
    }
  
    //=====================================================================================
    //setClockCalendar()
    //Sets the YEAR, MONTH, DAY, HOUR, MINUTE, SECOND bit fields in the CLOCK register.
    //If invalid parameter values are passed then no change is made to the CLOCK register.
    //Parameters:
    //  year:   The calendar year. Valid values range from 2016 to 2079.
    //  month:  One of the valid rtcMonth_t month values (e.g. JANUARY)
    //  day:    The day of the month. The values passed must be logical for the month value.
    //          For example, if the month is JANUARY then valid values for this parameter
    //          are from 1 to 31. However, if the month is APRIL then valid values will be
    //          from 1 to 30. If the year value is divisable by 4 then the number of days
    //          in FEBRUARY is 1 to 29 (leap year). Otherwise, FEBRUARY may only have
    //          1 to 28 days.
    //  hour:   This field is interpreted based on the value of the CLKREP bit in the
    //          CTRL register. If CLKREP is set (12 hour mode) then then valid values for
    //          this parameter are 1 through 12. If CLKREP is cleared (24 hour mode) then
    //          valid values for this parameter are 0 through 23.
    //  minute: Valid values range from 0 to 59
    //  second: Valid values range from 0 to 59
    //  AMorPM: Either _AM or _PM for 12 hour mode or NOT_SPECIFIED for 24 hour mode
    //
    //Return:   True if the clock value was set successfully. False if a parameter was out
    //          of range.
    //=====================================================================================
    bool ClockCalendar::setClockCalendar(uint16_t year, rtcMonth_t month, uint8_t day,
                                         uint8_t hour, uint8_t minute, uint8_t second,
                                         rtcAMPM_t AMorPM)
    {
        uint32_t value = 0;
        
        //Check year parameter
        if (((year - BASE_YEAR) >= 0) && (year - BASE_YEAR) <= 63) {
            value |= (RTC_MODE2_CLOCK_YEAR(year - BASE_YEAR) | RTC_MODE2_CLOCK_MONTH(month));
        } else {
            return false;
        }
        
        //Check day parameter
        if (day == 0) {
            return false;
        }
        switch (month) {
            case JANUARY:
            case MARCH:
            case MAY:
            case JULY:
            case AUGUST:
            case OCTOBER:
            case DECEMBER:
                if (day <= 31) {
                    value |= RTC_MODE2_CLOCK_DAY(day);
                } else {
                    return false;
                }
                break;
            case APRIL:
            case JUNE:
            case SEPTEMBER:
            case NOVEMBER:
                if (day <= 30) {
                    value |= RTC_MODE2_CLOCK_DAY(day);
                } else {
                    return false;
                }
                
            default:
                //Only FEBRUARY left
                if (year % 4) {
                    //Not a leap year
                    if (day <= 28) {
                        value |= RTC_MODE2_CLOCK_DAY(day);
                    } else {
                        return false;
                    }
                } else {
                    //Leap year
                    if (day <= 29) {
                        value |= RTC_MODE2_CLOCK_DAY(day);
                    } else {
                        return false;
                    }
                }
                break;
        }
        
        //Check hour parameter
        if (pRegisterBaseAddress->MODE2.CTRL.bit.CLKREP) {
            //12 hour mode
            if (hour >= 1 && hour <= 12) {
                value |= RTC_MODE2_CLOCK_HOUR(hour);
            } else {
                return false;
            }
            
            if (AMorPM == NOT_SPECIFIED) {
                return false;
            } else if (AMorPM == _PM) {
                value |= RTC_MODE2_CLOCK_HOUR_PM;
            }
        } else {
            //24 hour mode
            if (hour <= 23) {
                value |= RTC_MODE2_CLOCK_HOUR(hour);
            } else {
                return false;
            }
        }
        
        //Check minute parameter
        if (minute <= 59) {
            value |= RTC_MODE2_CLOCK_MINUTE(minute);
        } else {
            return false;
        }
        
        //Check second parameter
        if (second <= 59) {
            value |= RTC_MODE2_CLOCK_SECOND(second);
        } else {
            return false;
        }
        
        //Write the CLOCK register
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE2.CLOCK.reg = value;
        
        return true;
    }
    
    //=====================================================================================
    //getTime()
    //Returns a String object containing a time string formated as "HH:MM:SS" or
    //"HH:MM:SS AM" depending on the setting of the CLKREP bit in the control register
    //Parameters: None
    //Return: A formated String object
    //=====================================================================================
    String ClockCalendar::getTime() const
    {
        String time = String("");
        
        //Read the current time
        //Block until synchronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        uint32_t clockRegister = pRegisterBaseAddress->MODE2.CLOCK.reg;
        
        //Extract hours
        //Check if we are in 12 or 24 hour mode
        bool _12HourMode = false;
        if (pRegisterBaseAddress->MODE2.CTRL.bit.CLKREP) {
            _12HourMode =true;
        }
        
        uint32_t data = clockRegister & RTC_MODE2_CLOCK_HOUR_Msk;
        data >>= RTC_MODE2_CLOCK_HOUR_Pos;
        if (_12HourMode) {
            //Strip off the AM/PM bit
            data &= 0xF;
        }
        time.concat((unsigned int)data);
        time.concat(":");
        
        //Extract minutes
        data = clockRegister & RTC_MODE2_CLOCK_MINUTE_Msk;
        data >>= RTC_MODE2_CLOCK_MINUTE_Pos;
        time.concat((unsigned int)data);
        time.concat(":");
        
        //Extract seconds
        data = clockRegister & RTC_MODE2_CLOCK_SECOND_Msk;
        time.concat((unsigned int)data);
        
        //Need AM/PM indicator?
        if (_12HourMode) {
            time.concat(" ");
            if (clockRegister & RTC_MODE2_CLOCK_HOUR_PM) {
                time.concat("PM");
            } else {
                time.concat("AM");
            }
        }
        return time;
    }
    
    //=====================================================================================
    //getDate()
    //Returns a String object containing a date string formated as "Month Day, Year"
    //Parameters: None
    //Return: A formated String object
    //=====================================================================================
    String ClockCalendar::getDate() const
    {
        String date = String("");
        
        //Read the current date
        //Block until synchronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        uint32_t clockRegister = pRegisterBaseAddress->MODE2.CLOCK.reg;
        
        //Extract the month
        uint32_t data = clockRegister & RTC_MODE2_CLOCK_MONTH_Msk;
        data >>= RTC_MODE2_CLOCK_MONTH_Pos;
        switch (data) {
            case JANUARY:
                date.concat("January ");
                break;
            case FEBRUARY:
                date.concat("February ");
                break;
            case MARCH:
                date.concat("March ");
                break;
            case APRIL:
                date.concat("April ");
                break;
            case MAY:
                date.concat("May ");
                break;
            case JUNE:
                date.concat("June ");
                break;
            case JULY:
                date.concat("July ");
                break;
            case AUGUST:
                date.concat("August ");
                break;
            case SEPTEMBER:
                date.concat("September ");
                break;
            case OCTOBER:
                date.concat("October ");
                break;
            case NOVEMBER:
                date.concat("November ");
                break;
            case DECEMBER:
                date.concat("December ");
                
        }
        
        //Extract the day
        data = clockRegister & RTC_MODE2_CLOCK_DAY_Msk;
        data >>= RTC_MODE2_CLOCK_DAY_Pos;
        date.concat((unsigned int)data);
        date.concat(", ");
        
        //Extract the year
        data = clockRegister & RTC_MODE2_CLOCK_YEAR_Msk;
        data >>= RTC_MODE2_CLOCK_YEAR_Pos;
        date.concat((unsigned int)data + BASE_YEAR);
        
        return date;
    }
    
    dateTime_t ClockCalendar::getDateTime() const
    {
        dateTime_t data;
        
        //Get the current clock data
        //Block until synchronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        uint32_t clockRegister = pRegisterBaseAddress->MODE2.CLOCK.reg;
        
        //Determine clock mode (12 or 24 hour)
        //Block until synchronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        if (pRegisterBaseAddress->MODE2.CTRL.bit.CLKREP) {
            data.mode = _12HOUR;
        } else {
            data.mode = _24HOUR;
        }
        
        //Extract data values
        data.second = (uint8_t)(clockRegister & RTC_MODE2_CLOCK_SECOND_Msk);
        clockRegister >>= RTC_MODE2_CLOCK_MINUTE_Pos;
        data.minute = (uint8_t)(clockRegister & (RTC_MODE2_CLOCK_MINUTE_Msk >> RTC_MODE2_CLOCK_MINUTE_Pos));
        clockRegister >>= (RTC_MODE2_CLOCK_HOUR_Pos - RTC_MODE2_CLOCK_MINUTE_Pos);
        if (data.mode == _12HOUR) {
            data.hour = (uint8_t)clockRegister & 0xF;
            if (clockRegister & 0x10) {
                data.AMorPM = _PM;
            } else {
                data.AMorPM = _AM;
            }
        } else {
            data.hour = (uint8_t)clockRegister & 0x1F;
            data.AMorPM = NOT_SPECIFIED;
        }
        clockRegister >>= RTC_MODE2_CLOCK_DAY_Pos - (RTC_MODE2_CLOCK_HOUR_Pos + RTC_MODE2_CLOCK_MINUTE_Pos);
        data.day = (uint8_t)(clockRegister & (RTC_MODE2_CLOCK_DAY_Msk >> RTC_MODE2_CLOCK_DAY_Pos));
        clockRegister >>= RTC_MODE2_CLOCK_MONTH_Pos - (RTC_MODE2_CLOCK_HOUR_Pos + RTC_MODE2_CLOCK_MINUTE_Pos + RTC_MODE2_CLOCK_DAY_Pos);
        data.month = (rtcMonth_t)(clockRegister & (RTC_MODE2_CLOCK_MONTH_Msk >> RTC_MODE2_CLOCK_MONTH_Pos));
        clockRegister >>= RTC_MODE2_CLOCK_YEAR_Pos - (RTC_MODE2_CLOCK_HOUR_Pos + RTC_MODE2_CLOCK_MINUTE_Pos + RTC_MODE2_CLOCK_DAY_Pos + RTC_MODE2_CLOCK_MONTH_Pos);
        data.year = (uint16_t)(clockRegister & (RTC_MODE2_CLOCK_YEAR_Msk >> RTC_MODE2_CLOCK_YEAR_Pos)) + BASE_YEAR;
        
        return data;
    }
    
    //=====================================================================================
    //getMode()
    //Returns the current clock mode setting
    //Parameters: None
    //Return: Current clock mode setting (12 hour or 24 hour)
    //=====================================================================================
    rtcClockMode_t ClockCalendar::getMode() const
    {
        //Block until synchronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        if (pRegisterBaseAddress->MODE2.CTRL.bit.CLKREP) {
            return _12HOUR;
        } else {
            return _24HOUR;
        }
    }
    
    //=====================================================================================
    //setAlarm()
    //Sets the YEAR, MONTH, DAY, HOUR, MINUTE, SECOND bit fields in the ALARM0 register.
    //Also sets the Alarm Mask value in the MASK register
    //If invalid parameter values are passed then no change is made to the CLOCK register.
    //Parameters:
    //  year:   The calendar year. Valid values range from 2016 to 2079.
    //  month:  One of the valid rtcMonth_t month values (e.g. JANUARY)
    //  day:    The day of the month. The values passed must be logical for the month value.
    //          For example, if the month is JANUARY then valid values for this parameter
    //          are from 1 to 31. However, if the month is APRIL then valid values will be
    //          from 1 to 30. If the year value is divisable by 4 then the number of days
    //          in FEBRUARY is 1 to 29 (leap year). Otherwise, FEBRUARY may only have
    //          1 to 28 days.
    //  hour:   This field is interpreted based on the value of the CLKREP bit in the
    //          CTRL register. If CLKREP is set (12 hour mode) then then valid values for
    //          this parameter are 1 through 12. If CLKREP is cleared (24 hour mode) then
    //          valid values for this parameter are 0 through 23.
    //  minute: Valid values range from 0 to 59
    //  second: Valid values range from 0 to 59
    //  enable: A valid rtcAlarmEnable_t value such as "HHMMSS"
    //
    //Return:   True if the alarm value was loaded successfully. False if a parameter was out
    //          of range
    //=====================================================================================
    bool ClockCalendar::setAlarm(uint16_t year, rtcMonth_t month, uint8_t day, uint8_t hour,
                                 uint8_t minute, uint8_t second, rtcAlarmEnable_t enable,
                                 rtcAMPM_t AMorPM)
    {
        uint32_t value = 0;
        
        //Check year parameter
        if (((year - BASE_YEAR) >= 0) && (year - BASE_YEAR) <= 63) {
            value |= (RTC_MODE2_CLOCK_YEAR(year - BASE_YEAR) | RTC_MODE2_CLOCK_MONTH(month));
        } else {
            return false;
        }
        
        //Check day parameter
        if (day == 0) {
            return false;
        }
        switch (month) {
            case JANUARY:
            case MARCH:
            case MAY:
            case JULY:
            case AUGUST:
            case OCTOBER:
            case DECEMBER:
                if (day <= 31) {
                    value |= RTC_MODE2_CLOCK_DAY(day);
                } else {
                    return false;
                }
                break;
            case APRIL:
            case JUNE:
            case SEPTEMBER:
            case NOVEMBER:
                if (day <= 30) {
                    value |= RTC_MODE2_CLOCK_DAY(day);
                } else {
                    return false;
                }
                
            default:
                //Only FEBRUARY left
                if (year % 4) {
                    //Not a leap year
                    if (day <= 28) {
                        value |= RTC_MODE2_CLOCK_DAY(day);
                    } else {
                        return false;
                    }
                } else {
                    //Leap year
                    if (day <= 29) {
                        value |= RTC_MODE2_CLOCK_DAY(day);
                    } else {
                        return false;
                    }
                }
                break;
        }
        
        //Check hour parameter
        if (pRegisterBaseAddress->MODE2.CTRL.bit.CLKREP) {
            //12 hour mode
            if (hour >= 1 && hour <= 12) {
                value |= RTC_MODE2_CLOCK_HOUR(hour);
            } else {
                return false;
            }
            
            if (AMorPM == NOT_SPECIFIED) {
                return false;
            } else if (AMorPM == _PM) {
                value |= RTC_MODE2_CLOCK_HOUR_PM;
            }
        } else {
            //24 hour mode
            if (hour <= 23) {
                value |= RTC_MODE2_CLOCK_HOUR(hour);
            } else {
                return false;
            }
        }
        
        //Check minute parameter
        if (minute <= 59) {
            value |= RTC_MODE2_CLOCK_MINUTE(minute);
        } else {
            return false;
        }
        
        //Check second parameter
        if (second <= 59) {
            value |= RTC_MODE2_CLOCK_SECOND(second);
        } else {
            return false;
        }
        
        //Write the ALARM0 register
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE2.Mode2Alarm[0].ALARM.reg = value;
        
        //Write the MASK register
        if (enable == 0) {
            return false;
        }
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE2.Mode2Alarm[0].MASK.reg = (uint8_t)enable;

        return true;
    }
    
    //=====================================================================================
    //clearAlarm()
    //Disables the RTC alarm
    //Parameters: None
    //Return: None
    //=====================================================================================
    void ClockCalendar::clearAlarm()
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->MODE2.Mode2Alarm[0].MASK.reg = (uint8_t)OFF;
    }
    
    //=====================================================================================
    //getClockRegister()
    //Returns the value of the CLOCK register as an unsigned 32-bit value
    //Parameters: None
    //Return: CLOCK register value
    //=====================================================================================
    uint32_t ClockCalendar::getClockRegister()
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->MODE2.STATUS.bit.SYNCBUSY == 1);
        return pRegisterBaseAddress->MODE2.CLOCK.reg;
    }
    
    //=====================================================================================
    //getInterruptFlags()
    //Returns the current value of the Interrupt Flag Status and Clear register (INTFLAG)
    //Parameters: None
    //Return: Current value of INTFLAG register
    //=====================================================================================
    uint8_t ClockCalendar::getInterruptFlags() const
    {
        return (pRegisterBaseAddress->MODE2.INTFLAG.reg) & (MODE2_INTR_MASK);
    }
    
    //=====================================================================================
    //clearInterruptFlags()
    //Clears the specified interrupt flags
    //Parameters:
    //  flags:  A value specifying one or more interrupt flags to clear. Valid values are
    //          RTC_OVERFLOW RTC_SYNCRDY, RTC_ALARM, or any combination of these
    //Return: None
    //=====================================================================================
    void ClockCalendar::clearInterruptFlags(uint8_t flags)
    {
        pRegisterBaseAddress->MODE2.INTFLAG.reg = flags;
    }
    
    //=====================================================================================
    //getInterruptEnableState()
    //Returns the current value of the Interrupt Enable register (INTENCLR)
    //Parameters: None
    //Return: Current value of INTENCLR register
    //=====================================================================================
    uint8_t ClockCalendar::getInterruptEnableState()
    {
        return pRegisterBaseAddress->MODE2.INTENCLR.reg & MODE2_INTR_MASK;
    }
    
    //=====================================================================================
    //enableInterrupt()
    //Enables the specified interrupt and registers the callback function for later use.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. RTC_OVERFLOW)
    //  callbackFunc:   Pointer to the interrupt callback function
    //Return: None
    //=====================================================================================
    void ClockCalendar::enableInterrupt(rtcInterrupt_t interuptName, void(* callbackFunc)(void))
    {
        //Save the callback function pointer
        switch (interuptName) {
            case RTC_OVERFLOW:
                callBackFuncPtrs[7] = callbackFunc;
                break;
            case RTC_SYNCRDY:
                callBackFuncPtrs[6] = callbackFunc;
                break;
            case RTC_ALARM:
                callBackFuncPtrs[0] = callbackFunc;
                break;
            default:
                return;
        }
        
        //Enable RTC interrupts in the Nested Vector Interrupt Controller (NVIC)
        //RTC_IRQn is defined in the file samd21g18a.h
        //Reference Table 11-3 in the SAMD21G user manual/data sheet. The
        //NVIC_ClearPendingIRQ() and NVIC_EnableIRQ() are inline functions provided
        //via the Atmel CMSIS software and are contained in file core_cm0plus.h
        //Clear any pending interrupts for the associated IRQ Number
        NVIC_ClearPendingIRQ(RTC_IRQn);
        //Enable interrupts for the associated IRQ Number
        NVIC_EnableIRQ(RTC_IRQn);
        
        /* Enable TC interrupt */
        pRegisterBaseAddress->MODE2.INTENSET.reg = interuptName;
    }
    
    //=====================================================================================
    //disableInterrupt()
    //Disables the specified interrupt and clears the registered callback function.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. RTC_OVERFLOW)
    //Return: None
    //=====================================================================================
    void ClockCalendar::disableInterrupt(rtcInterrupt_t interuptName)
    {
        //Set the corresponding callback function to nullptr
        switch (interuptName) {
            case RTC_OVERFLOW:
                callBackFuncPtrs[7] = nullptr;
                break;
            case RTC_SYNCRDY:
                callBackFuncPtrs[6] = nullptr;
                break;
            case RTC_ALARM:
                callBackFuncPtrs[0] = nullptr;
                break;
            default:
                return;
                
                //Disable the indicated interrupt
                pRegisterBaseAddress->MODE2.INTENCLR.reg = interuptName;
                
        }
    }
    
    //=====================================================================================
    //handleInterrupt()
    //Examines the interrupt flag register (INTFLAG) and handles any enabled interrupts
    //by calling the associated user provided callback functions.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void ClockCalendar::handleInterrupt()
    {
        //Retrieve the interrupt flag state
        uint8_t interruptFlags = pRegisterBaseAddress->MODE2.INTFLAG.reg;
        //Clear the INTFLAG register
        pRegisterBaseAddress->MODE2.INTFLAG.reg = MODE2_INTR_MASK;
        
        
        //For each set interrupt flag check for a callback function (enabled interrupts
        //will always have a callback function) and call it if it exists
        uint8_t index = 0;
        uint8_t flag = 0x01;
        while (interruptFlags) {
            if ((index >= 1) && (index <= 5)) {
                index++;
                flag <<= 1;
                continue;
            }
            
            if ((interruptFlags & flag) && callBackFuncPtrs[index]) {
                //Call the interrupt callback function
                callBackFuncPtrs[index]();
            }
            
            //Clear the flag that we just examined
            interruptFlags &= ~flag;
            
            //Increment for next loop
            flag <<= 1;
            index++;
            
        }
    }
    
    
} //SAMD21RTC namespace
