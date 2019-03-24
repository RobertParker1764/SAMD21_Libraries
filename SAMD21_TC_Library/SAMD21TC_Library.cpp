
//SAMD21G18 Timer/Counter (TC) Peripheral Driver
//Version 1.0
//March 10, 2019
//By Robert Parker

/*
 Copyright (c) 2019 Robert Parker. All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

//See user notes in file SAMD21G18_TC_Driver.h

#include <SAMD21TC_Library.h>
#include "wiring_private.h"

namespace SAMD21TC {
    
    
    /******************** Static Class Member Initialization *****************************/
    
    //This array will hold pointers to the TC class objects (e.g. TC8Bit) that are
    //instantiated. The array is managed by the three static factory methods (get8BitTC,
    //etc.). Initially no TC instances are allocated so the array if filled with nullptr.
    TCBaseClass * TCBaseClass::allocatedTC[3] = {nullptr, nullptr, nullptr};
    
    /******************** Static Member Function Definition ******************************/
    
    //=====================================================================================
    //get8BitTC()
    //User interface to allocate new 8-bit timer/counter driver objects. All 8-bit timer/
    //counter objects should be instantiated using this method to ensure valid objects are
    //created.
    //Parameters:
    //  timerNumber:  _TC3, _TC4, or _TC5
    //Return:   A pointer to the newly created obect or nullptr if an object could
    //          not be created. User should ALWAYS check for nullptr to make sure
    //          a valid object was created before trying to use it.
    //=====================================================================================
    
    TC8Bit * TCBaseClass::get8BitTC(timerInstance_t timerNumber)
    {
        TC8Bit * timer;
        
        //Make sure the requested timer instance is available (not already allocated)
        //Can only allocate one timer object for each instance
        if (allocatedTC[timerNumber]) {
            return nullptr;
        }
        
        //Instantiate the requested timer/counter instance, mark the timer number as used
        //and return a pointer to the timer/counter instance
        timer = new TC8Bit(timerNumber);
        allocatedTC[timerNumber] = timer;
        
        return timer;
    }
    
    //=====================================================================================
    //get16BitTC()
    //User interface to allocate new 16-bit timer/counter driver objects. All 16-bit timer/
    //counter objects should be instantiated using this method to ensure valid objects are
    //created.
    //Parameters:
    //  timerNumber:  _TC3, _TC4, or _TC5
    //Return:   A pointer to the newly created obect or nullptr if an object could
    //          not be created. User should ALWAYS check for nullptr to make sure
    //          a valid object was created before trying to use it.
    //=====================================================================================
    
    TC16Bit * TCBaseClass::get16BitTC(timerInstance_t timerNumber)
    {
        TC16Bit * timer;
        
        //Make sure the requested timer is available (not already allocated)
        //Can only allocate one timer object for each instance
        if (allocatedTC[timerNumber]) {
            return nullptr;
        }
        
        //Instantiate the requested timer/counter instance, mark the timer number as used
        //and return a pointer to the timer/counter instance
        timer = new TC16Bit(timerNumber);
        allocatedTC[timerNumber] = timer;
        
        return timer;
    }
    
    //=====================================================================================
    //get32BitTC()
    //User interface to allocate new 32-bit timer/counter driver object. All 32-bit timer/
    //counter objects should be instantiated using this method to ensure valid objects are
    //created. The SAMD21G18 only supports one 32-bit timer/counter at a time and it uses
    //the TC4 and TC3 TC instances.
    //Parameters:
    //  timerNumber:  _TC4 is the only valid TC instance for 32-bit timer/counters
    //Return:   A pointer to the newly created obect or nullptr if an object could
    //          not be created. User should ALWAYS check for nullptr to make sure
    //          a valid object was created before trying to use it.
    //=====================================================================================
    
    TC32Bit * TCBaseClass::get32BitTC(timerInstance_t timerNumber)
    {
        TC32Bit * timer;
        
        //First check to make sure the requested timer is available (not already allocated
        //and make sure the requested timer size is compatible with the requested
        //timer number.
        //Can only allocate one timer object for each instance
        if (allocatedTC[timerNumber]) {
            return nullptr;
        }
        
        //32-bit timers are only allowed on TC4
        if (timerNumber == _TC3 || timerNumber == _TC5) {
            return nullptr;
        }
        
        //32-bit timers require two TC instances (TC3/TC4) to be free
        if (allocatedTC[_TC3]) {
            return nullptr;
        }
        
        //Instantiate the requested timer/counter instance, mark the timer number as used
        //and return a pointer to the timer/counter instance
        timer = new TC32Bit(timerNumber);
        allocatedTC[timerNumber] = timer;
        allocatedTC[timerNumber - 1] = timer;
        
        return timer;
    }
    

    //=====================================================================================
    //deleteTC()
    //User interface to delete an allocated counter/timer driver object. All timer/counter
    //objects should be deleted using this method when they are no longer needed, to free
    //memory and ensure a new driver object can be instantiated later if needed.
    //Parameters:
    //  timerNumber:  _TC3, _TC4, or _TC5
    //Return:   None
    //=====================================================================================
    void TCBaseClass::deleteTC(timerInstance_t timerNumber)
    {
        //Need to query the timer object to see if it is a 32-bit timer
        if (allocatedTC[timerNumber]->mSize == _32BIT) {
            delete allocatedTC[timerNumber];
            allocatedTC[timerNumber] = nullptr;
            allocatedTC[timerNumber + 1] = nullptr;
        } else {
            delete allocatedTC[timerNumber];
            allocatedTC[timerNumber] = nullptr;
        }
    }
    
    //Interrupt Handler
    //=====================================================================================
    //tcInterruptHandler()
    //Forwards an interrupt request to the appropriate TC instance
    //Parameters:
    //  timerNumber:  _TC3, _TC4, or _TC5
    //Return:   None
    //=====================================================================================
    void TCBaseClass::tcInterruptHandler(uint8_t tcNumber)
    {
        if (allocatedTC[tcNumber]) {
            allocatedTC[tcNumber]->handleInterrupt();
        }
    }
    //=========================================================================================
    //******************* TCBaseClass Member Function Definitions *****************************
    //=========================================================================================
    
    //Class Constructor
    //=====================================================================================
    //TCBaseClass()
    //Instantiates a new TCBaseClass object. This class is only used as a base class for
    //the other classes. This constructor should never be called by the user.
    //Parameters:
    //  timerNumber:  _TC3, _TC4, or _TC5
    //Return:   None
    //=====================================================================================
    TCBaseClass::TCBaseClass(timerInstance_t timerNumber)
    {
        //Create a temporary array of register stack pointers
        Tc *const tc_modules[] = TC_INSTS;       //TC_INSTS defined as {TC3, TC4, TC5}
        pRegisterBaseAddress = tc_modules[timerNumber];
        
        instance = timerNumber;
        
        //Initialize the array of callback function pointers to all nullptr
        for (int index = 0; index < 6; index++) {
            callBackFuncPtrs[index] = nullptr;
        }
        
        //Enable the user interface clock for this instance in the power manager
        //Temporary array of APBC Mask register constants (defined in pm.h)
        uint32_t inst_pm_apbmask[] = {PM_APBCMASK_TC3, PM_APBCMASK_TC4, PM_APBCMASK_TC5};
        /* Enable the user interface clock in the PM */
        PM->APBCMASK.reg |= inst_pm_apbmask[timerNumber];
    }
    
    //=====================================================================================
    //enableTC()
    //Enables operation of the SAMD21 hardware timer counter instance. Sets the ENABLE bit
    //in control register A.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void TCBaseClass::enableTC()
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
        
    }
    
    //=====================================================================================
    //disableTC()
    //Disables operation of the SAMD21 hardware timer counter instance. Clears the ENABLE
    //bit in control register A.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void TCBaseClass::disableTC()
    {
        //TODO: Need to disable interrupts here
        
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    }
    
    //=====================================================================================
    //resetTC()
    //Issues a software reset to the SAMD21 hardware timer instance. Resets all TC
    //registers (except DBGCTRL) to their initial state. The TC will be disabled.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void TCBaseClass::resetTC()
    {
        disableTC();
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CTRLA.reg = 0x1;   //Set the SWRST bit
        //Block until software reset is complete (SWRST bit is cleared)
        while (pRegisterBaseAddress->COUNT8.CTRLA.bit.SWRST == 1);
    }
    
    //=====================================================================================
    //retriggerTC()
    //Issues the RETRIGGER command to the SAMD21 hardware timer instance. Effects the CMD
    //bits in Control Register B
    //Parameters: None
    //Return: None
    //=====================================================================================
    void TCBaseClass::retriggerTC()
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CTRLBSET.reg = RETRIGGER_COMMAND;
    }
    
    //=====================================================================================
    //stopTC()
    //Issues the STOP command to the SAMD21 hardware timer instance. Effects the CMD
    //bits in Control Register B
    //Parameters: None
    //Return: None
    //=====================================================================================
    void TCBaseClass::stopTC()
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CTRLBSET.reg = STOP_COMMAND;
    }
    
    //=====================================================================================
    //isStopped()
    //Tests if the SAMD21 hardware timer instance is stopped.
    //Parameters: None
    //Return:   True if the timer is stopped. False othewise
    //=====================================================================================
    bool TCBaseClass::isStopped() const
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        return (pRegisterBaseAddress->COUNT8.STATUS.bit.STOP == 1);
    }
    
    //=====================================================================================
    //setCountDirection()
    //Sets the SAMD21 hardware timer instance count direction to count-up or count-down.
    //Changes the DIR bit in Control Register B.
    //Parameters:
    //  direction: COUNT_UP or COUNT_DOWN
    //Return: None
    //=====================================================================================
    void TCBaseClass::setCountDirection(countDirection_t direction)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        if (direction == COUNT_UP) {
            pRegisterBaseAddress->COUNT8.CTRLBCLR.reg = TC_CTRLBCLR_DIR;
        } else {
            pRegisterBaseAddress->COUNT8.CTRLBSET.reg = TC_CTRLBSET_DIR;
        }
    }
    
    //=====================================================================================
    //configureTimer()
    //This is the primary method used to configure the TC hardware instance before use in
    //counter/compare operations as described in the SAMD21 data sheet.
    //Configures the timer for basic timer operation. The timer may or may not (default)
    //be enabled when this method completes. The user must separately set TOP (PER or CCO),
    //compare channel 0/1 values, and enable interrupts if this functionality is desired.
    //Parameters:
    //  clockDivision: A valid clockPrescaler_t value (e.g. DIVIDE_BY_1)
    //  waveGenerationMode: NORMAL_FREQUENCY, MATCH_FREQUENCY, NORMAL_PWM, or MATCH_PWM
    //  waveformOutput0Pin: Default is 0 - not output pin assigned. Other valid values are
    //                      TC3: 2/10  TC4: A1/20  TC5: 10
    //  waveformOutput1Pin: Default is 0 - no output pin assigned. Other valid values are
    //                      TC3: 5/12  TC4: A2/21  TC5: 11
    //  enable: If true (default) the timer counter instance will be enabled, it will remain
    //          disabled otherwise
    //  prescalerSyncMode: GENERIC_CLOCK (defalut), PRESCALER_CLOCK, or
    //                     GENERIC_CLOCK_RESET_PRESCALER
    //  countDirection: COUNT_UP (defalult) or COUNT_DOWN
    //  invertOutputx: If false (default) the outputs will not be inverted, if true then
    //                 the outputs will be inverted
    //  runStandby: If false (default) then the timer will not run in standby, if true then
    //              the timer will run in standby.
    //  oneShotTimer: If false (default) the timer will not operate in one-shot mode. If true
    //                it will operate in one-shot mode.
    //Return: False if invalid pin numbers were supplied, True otherwise
    //=====================================================================================
    bool TCBaseClass::configureTimer(clockPrescaler_t clockDivision,
                                     waveGeneration_t waveGenerationMode,
                                     uint8_t waveformOutput0Pin,
                                     uint8_t waveformOutput1Pin,
                                     bool enable,
                                     reloadResetMode_t prescalerSyncMode,
                                     countDirection_t countDirection,
                                     bool invertOutput0,
                                     bool invertOutput1,
                                     bool runStandby,
                                     bool oneShotTimer)
    {
        //Disable the timer before starting configuration
        disableTC();
    
        //Compute the value for control register A
        uint16_t tempCtrlA = clockDivision | waveGenerationMode | prescalerSyncMode;
        if (mSize == _8BIT) {
            tempCtrlA |= TC_CTRLA_MODE_COUNT8;
        } else if (mSize == _16BIT) {
            tempCtrlA |= TC_CTRLA_MODE_COUNT16;
        } else {
            tempCtrlA |= TC_CTRLA_MODE_COUNT32;
        }
        if (runStandby) {
            tempCtrlA |= TC_CTRLA_RUNSTDBY;
        }
        
        //Block until synchronization is complete then write register CTRLA
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CTRLA.reg = tempCtrlA;
        
        //Modify control register B if needed
        uint8_t tempCtrlReg = 0;
        if (countDirection || oneShotTimer) {
            if (countDirection) {
                tempCtrlReg |= TC_CTRLBSET_DIR;
            }
            if (oneShotTimer) {
                tempCtrlReg |= TC_CTRLBSET_ONESHOT;
            }
            
            //Block until synchronization is complete then write register CTRLB
            while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
            pRegisterBaseAddress->COUNT8.CTRLBSET.reg = tempCtrlReg;
        }
        
        //Modify control register C if needed
        if (invertOutput0 || invertOutput1) {
            tempCtrlReg = 0;
            if (invertOutput0) {
                tempCtrlReg |= TC_CTRLC_INVEN0;
            }
            if (invertOutput1) {
                tempCtrlReg |= TC_CTRLC_INVEN1;
            }
            
            //Block until synchronization is complete then write register CTRLC
            while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
            pRegisterBaseAddress->COUNT8.CTRLC.reg = tempCtrlReg;
        }
        
        //Enable the waveform output pins if used
        //Channel 0
        if (waveformOutput0Pin) {
            if (pRegisterBaseAddress == TC3) {
                if (waveformOutput0Pin == 2) {
                    pinPeripheral(waveformOutput0Pin, (EPioType)MUX_PA14E_TC3_WO0);
                } else if (waveformOutput0Pin == 10) {
                    pinPeripheral(waveformOutput0Pin, (EPioType)MUX_PA18E_TC3_WO0);
                } else {
                    //Invalid pin
                    return false;
                }
            } else if (pRegisterBaseAddress == TC4) {
                if (waveformOutput0Pin == 20) {
                    pinPeripheral(waveformOutput0Pin, (EPioType)MUX_PA22E_TC4_WO0);
                } else if (waveformOutput0Pin == A1) {
                    pinPeripheral(waveformOutput0Pin, (EPioType)MUX_PB08E_TC4_WO0);
                } else {
                    //Invalid pin
                    return false;
                }
            } else {        //TC5
                if (waveformOutput0Pin == MOSI) {
                    pinPeripheral(waveformOutput0Pin, (EPioType)MUX_PB10E_TC5_WO0);
                } else {
                    //Invalid pin
                    return false;
                }
            }
        }
        
        //Channel 1
        if (waveformOutput1Pin) {
            if (pRegisterBaseAddress == TC3) {
                if (waveformOutput1Pin == 5) {
                    pinPeripheral(waveformOutput1Pin, (EPioType)MUX_PA15E_TC3_WO1);
                } else if (waveformOutput1Pin == 12) {
                    pinPeripheral(waveformOutput1Pin, (EPioType)MUX_PA19E_TC3_WO1);
                } else {
                    //Invalid pin
                    return false;
                }
            } else if (pRegisterBaseAddress == TC4) {
                if (waveformOutput1Pin == 21) {
                    pinPeripheral(waveformOutput1Pin, (EPioType)MUX_PA23E_TC4_WO1);
                } else if (waveformOutput1Pin == A2) {
                    pinPeripheral(waveformOutput1Pin, (EPioType)MUX_PB09E_TC4_WO1);
                } else {
                    //Invalid piin
                    return false;
                }
            } else {        //TC5
                if (waveformOutput1Pin == SCK) {
                    pinPeripheral(waveformOutput1Pin, (EPioType)MUX_PB11E_TC5_WO1);
                } else {
                    //Invalid pin
                    return false;
                }
            }
        }
        
        
        
        if (enable) {
            enableTC();
        }
        
        return true;
    }
    
    bool TCBaseClass::configureCaptureOperation()
    {
        //TODO: Implement configureCounter()
        return false;
    }
    
    //=====================================================================================
    //handleInterrupt()
    //Examines the interrupt flag register (INTFLAG) and handles any enabled interrupts
    //by calling the associated user provided callback functions.
    //Parameters: None
    //Return: None
    //=====================================================================================
    void TCBaseClass::handleInterrupt()
    {
        //Retrieve the interrupt flag state
        uint8_t interruptFlags = pRegisterBaseAddress->COUNT8.INTFLAG.reg;
        //Clear the INTFLAG register
        pRegisterBaseAddress->COUNT8.INTFLAG.reg = INTFLAG_CLEAR;
        
        
        //For each set interrupt flag check for a callback function (enabled interrupts
        //will always have a callback function) and call it if it exists
        uint8_t index = 0;
        uint8_t flag = 0x01;
        while (interruptFlags) {
            if (index == 2) {
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
    
    //=====================================================================================
    //enableInterrupt()
    //Enables the specified interrupt and registers the callback function for later use.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. TC_OVERFLOW)
    //  callbackFunc:   Pointer to the interrupt callback function
    //Return: None
    //=====================================================================================
    void TCBaseClass::enableInterrupt(tcInterrupt_t interuptName, void(* callbackFunc)(void))
    {
        //Save the callback function pointer
        switch (interuptName) {
            case TC_OVERFLOW:
                callBackFuncPtrs[0] = callbackFunc;
                break;
            case TC_ERROR:
                callBackFuncPtrs[1] = callbackFunc;
                break;
            case TC_SYNCRDY:
                callBackFuncPtrs[3] = callbackFunc;
                break;
            case TC_CHANNEL_0_MATCH:
                callBackFuncPtrs[4] = callbackFunc;
                break;
            case TC_CHANNEL_1_MATCH:
                callBackFuncPtrs[5] = callbackFunc;
                
        }
        
        //Enable timer interrupts in the Nested Vector Interrupt Controller (NVIC)
        //The elements of IRQNumbers[] (TC3_IRQn, etc) are defined in the file samd21g18a.h
        //Reference Table 11-3 in the SAMD21G user manual/data sheet. The
        //NVIC_ClearPendingIRQ() and NVIC_EnableIRQ() are inline functions provided
        //via the Atmel CMSIS software and are contained in file core_cm0plus.h
        IRQn_Type IRQNumbers[] = { TC3_IRQn, TC4_IRQn, TC5_IRQn };
        //Clear any pending interrupts for the associated IRQ Number
        NVIC_ClearPendingIRQ(IRQNumbers[instance]);
        //Enable interrupts for the associated IRQ Number
        NVIC_EnableIRQ(IRQNumbers[instance]);
        
        /* Enable TC interrupt */
        pRegisterBaseAddress->COUNT8.INTENSET.reg = interuptName;
    }
    
    //=====================================================================================
    //disableInterrupt()
    //Disables the specified interrupt and clears the registered callback function.
    //Parameters:
    //  interruptName:  The interrupt type (e.g. TC_OVERFLOW)
    //Return: None
    //=====================================================================================
    void TCBaseClass::disableInterrupt(tcInterrupt_t interuptName)
    {
        //Disable the indicated interrupt
        pRegisterBaseAddress->COUNT8.INTENCLR.reg = interuptName;
        
        //Set the corresponding callback function to nullptr
        switch (interuptName) {
            case TC_OVERFLOW:
                callBackFuncPtrs[0] = nullptr;
                break;
            case TC_ERROR:
                callBackFuncPtrs[1] = nullptr;
                break;
            case TC_SYNCRDY:
                callBackFuncPtrs[3] = nullptr;
                break;
            case TC_CHANNEL_0_MATCH:
                callBackFuncPtrs[4] = nullptr;
                break;
            case TC_CHANNEL_1_MATCH:
                callBackFuncPtrs[5] = nullptr;
                break;
                
        }
    }
    
    //=====================================================================================
    //setControlRegisterA()
    //Sets the value of control register A (CTRLA) to the specified value
    //Parameters:
    //  value: The value to be loaded into control register A
    //Return: None
    //=====================================================================================
    void TCBaseClass::setControlRegisterA(uint16_t value)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CTRLA.reg = value;
    }
    
    //=====================================================================================
    //getControlRegisterA()
    //Returns the current value of control register A (CTRLA)
    //Parameters: None
    //Return: Current value of control register A
    //=====================================================================================
    uint16_t TCBaseClass::getControlRegisterA(void)
    {
        return pRegisterBaseAddress->COUNT8.CTRLA.reg;
    }
    
    //=====================================================================================
    //setOneShotMode()
    //Sets the TC instance in oneShot or continuous mode depending on the value of oneShot
    //Parameters:
    //  oneShot: If true the TC instance is put in oneShot mode. If false the TC instance
    //           is put in continuous count mode
    //Return: None
    //=====================================================================================
    void TCBaseClass::setOneShotMode(bool oneShot)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        if (oneShot) {
            pRegisterBaseAddress->COUNT8.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
        } else {
            pRegisterBaseAddress->COUNT8.CTRLBCLR.reg = TC_CTRLBCLR_ONESHOT;
        }
    }
    
    
    //=====================================================================================
    //getControlRegisterB()
    //Returns the current value of control register B (CTRLB)
    //Parameters: None
    //Return: Current value of control register B
    //=====================================================================================
    uint8_t TCBaseClass::getControlRegisterB()
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        return pRegisterBaseAddress->COUNT8.CTRLBSET.reg;
    }
    
    
    //=====================================================================================
    //getInterruptEnableState()
    //Returns the current value of the Interrupt Enable Clear register (INTENCLR)
    //Parameters: None
    //Return: Current value of INTENCLR register
    //=====================================================================================
    uint8_t TCBaseClass::getInterruptEnableState()
    {
        return pRegisterBaseAddress->COUNT8.INTENCLR.reg & INTFLAG_CLEAR;
    }
    
    //=====================================================================================
    //getInterruptFlags()
    //Returns the current value of the Interrupt Flag Status and Clear register (INTFLAG)
    //Parameters: None
    //Return: Current value of INTFLAG register
    //=====================================================================================
    uint8_t TCBaseClass::getInterruptFlags()
    {
        return pRegisterBaseAddress->COUNT8.INTFLAG.reg & INTFLAG_CLEAR;
    }
    
    
    //=====================================================================================
    //clearInterruptFlags()
    //Clears the interrupt flags passed in flags
    //Parameters:
    //  flags:  A value specifying one or more interrupt flags to clear. Valid values are
    //          TC_OVERFLOW, TC_ERROR, TC_SYNCRDY, TC_CHANNEL_0_MATCH, TC_CHANNEL_1_MATCH,
    //          or any combination of these
    //Return: None
    //=====================================================================================
    void TCBaseClass::clearInterruptFlags(uint8_t flags)
    {
        pRegisterBaseAddress->COUNT8.INTFLAG.reg = flags;
    }
    
    //=========================================================================================
    //******************** TC8Bit Member Function Definitions *********************************
    //=========================================================================================
    
    //Class Constructor
    //=====================================================================================
    //TC8Bit()
    //The constructor is called by the factory method get8BitTC(). The user should never
    //directly call this constructor.
    //Parameters:
    //  timerNumber:    _TC3, _TC4, or _TC5
    //Return: A pointer to the TC8Bit object
    //=====================================================================================
    TC8Bit::TC8Bit(timerInstance_t timerNumber) : TCBaseClass(timerNumber)
    {
        mSize = _8BIT;
        
    }
    
    //=====================================================================================
    //setPeriod()
    //Sets the SAMD21 hardware timer instance PER register value.
    //Changes the PER register.
    //Parameters:
    //  period: new 8-bit value to be loaded into PER register
    //Return: None
    //=====================================================================================
    void TC8Bit::setPeriod(uint8_t period)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.PER.reg = period;
    }
    
    //=====================================================================================
    //getPeriod()
    //Returns the SAMD21 hardware timer instance PER register contents.
    //Parameters: None
    //Return: PER register 8-bit value
    //=====================================================================================
    uint8_t TC8Bit::getPeriod() const
    {
        return pRegisterBaseAddress->COUNT8.PER.reg;
    }
    
    //=====================================================================================
    //setCompareChannel0()
    //Sets the SAMD21 hardware timer instance CC0 register value.
    //Changes the CC0 register.
    //Parameters:
    //  compareValue: new 8-bit value to be loaded into CC0 register
    //Return: None
    //=====================================================================================
    void TC8Bit::setCompareChannel0(uint8_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CC[0].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareChannel0()
    //Returns the SAMD21 hardware timer instance CC0 register contents.
    //Parameters: None
    //Return: CC0 register 8-bit value
    //=====================================================================================
    uint8_t TC8Bit::getCompareChannel0() const
    {
        return pRegisterBaseAddress->COUNT8.CC[0].reg;
    }
    
    //=====================================================================================
    //setCompareChannel1()
    //Sets the SAMD21 hardware timer instance CC1 register value.
    //Changes the CC1 register.
    //Parameters:
    //  compareValue: new 8-bit value to be loaded into CC1 register
    //Return: None
    //=====================================================================================
    void TC8Bit::setCompareChannel1(uint8_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.CC[1].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareChannel1()
    //Returns the SAMD21 hardware timer instance CC1 register contents.
    //Parameters: None
    //Return: CC1 register 8-bit value
    //=====================================================================================
    uint8_t TC8Bit::getCompareChannel1() const
    {
        return pRegisterBaseAddress->COUNT8.CC[1].reg;
    }
    
    //=====================================================================================
    //setCount()
    //Sets the SAMD21 hardware timer instance COUNT register value.
    //Changes the COUNT register.
    //Parameters:
    //  count: new 8-bit value to be loaded into COUNT register
    //Return: None
    //=====================================================================================
    void TC8Bit::setCount(uint8_t count)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT8.COUNT.reg = count;
    }
    
    //=====================================================================================
    //getCount()
    //Returns the SAMD21 hardware timer instance COUNT register contents.
    //Parameters: None
    //Return: COUNT register 8-bit value
    //=====================================================================================
    uint8_t TC8Bit::getCount() const
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        return pRegisterBaseAddress->COUNT8.COUNT.reg;
    }
    
    //=========================================================================================
    //******************** TC16Bit Member Function Definitions *********************************
    //=========================================================================================
    
    //Class Constructor
    //=====================================================================================
    //TC16Bit()
    //The constructor is called by the factory method get16BitTC(). The user should never
    //directly call this constructor.
    //Parameters:
    //  timerNumber:    _TC3, _TC4, or _TC5
    //Return: A pointer to the TC16Bit object
    //=====================================================================================
    TC16Bit::TC16Bit(timerInstance_t timerNumber) : TCBaseClass(timerNumber)
    {
        mSize = _16BIT;
        
    }
    
    //=====================================================================================
    //setCompareChannel0()
    //Sets the SAMD21 hardware timer instance CC0 register value.
    //Changes the CC0 register.
    //Parameters:
    //  compareValue: new 16-bit value to be loaded into CC0 register
    //Return: None
    //=====================================================================================
    void TC16Bit::setCompareChannel0(uint16_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT16.CC[0].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareChannel0()
    //Returns the SAMD21 hardware timer instance CC0 register contents.
    //Parameters: None
    //Return: CC0 register 16-bit value
    //=====================================================================================
    uint16_t TC16Bit::getCompareChannel0() const
    {
        return pRegisterBaseAddress->COUNT16.CC[0].reg;
    }
    
    //=====================================================================================
    //setCompareChannel1()
    //Sets the SAMD21 hardware timer instance CC1 register value.
    //Changes the CC1 register.
    //Parameters:
    //  compareValue: new 16-bit value to be loaded into CC1 register
    //Return: None
    //=====================================================================================
    void TC16Bit::setCompareChannel1(uint16_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT16.CC[1].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareChannel1()
    //Returns the SAMD21 hardware timer instance CC1 register contents.
    //Parameters: None
    //Return: CC0 register 16-bit value
    //=====================================================================================
    uint16_t TC16Bit::getCompareChannel1() const
    {
        return pRegisterBaseAddress->COUNT16.CC[1].reg;
    }
    
    //=====================================================================================
    //setCount()
    //Sets the SAMD21 hardware timer instance COUNT register value.
    //Changes the COUNT register.
    //Parameters:
    //  count: new 16-bit value to be loaded into COUNT register
    //Return: None
    //=====================================================================================
    void TC16Bit::setCount(uint16_t count)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT16.COUNT.reg = count;
    }
    
    //=====================================================================================
    //getCount()
    //Returns the SAMD21 hardware timer instance COUNT register contents.
    //Parameters: None
    //Return: COUNT register 16-bit value
    //=====================================================================================
    uint16_t TC16Bit::getCount() const
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        return pRegisterBaseAddress->COUNT16.COUNT.reg;
    }
    
    //=========================================================================================
    //******************** TC32Bit Member Function Definitions *********************************
    //=========================================================================================
    
    //Class Constructor
    //=====================================================================================
    //TC32Bit()
    //The constructor is called by the factory method get32BitTC(). The user should never
    //directly call this constructor.
    //Parameters:
    //  timerNumber:    _TC4 is the only valid value
    //Return: A pointer to the TC32Bit object
    //=====================================================================================
    TC32Bit::TC32Bit(timerInstance_t timerNumber) : TCBaseClass(timerNumber)
    {
        mSize = _32BIT;
        
        
        //Enable the user interface clock for the slave counter/timer instance
        //in the power manager. It can only be the TC4 instance
        PM->APBCMASK.reg |= PM_APBCMASK_TC4;
        
    }
    
    //=====================================================================================
    //setCompareChannel0()
    //Sets the SAMD21 hardware timer instance CC0 register value.
    //Changes the CC0 register.
    //Parameters:
    //  compareValue: new 32-bit value to be loaded into CC0 register
    //Return: None
    //=====================================================================================
    void TC32Bit::setCompareChannel0(uint32_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT32.CC[0].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareChannel0()
    //Returns the SAMD21 hardware timer instance CC0 register contents.
    //Parameters: None
    //Return: CC0 register 32-bit value
    //=====================================================================================
    uint32_t TC32Bit::getCompareChannel0() const
    {
        return pRegisterBaseAddress->COUNT32.CC[0].reg;
    }
    
    //=====================================================================================
    //setCompareChannel1()
    //Sets the SAMD21 hardware timer instance CC1 register value.
    //Changes the CC1 register.
    //Parameters:
    //  compareValue: new 32-bit value to be loaded into CC1 register
    //Return: None
    //=====================================================================================
    void TC32Bit::setCompareChannel1(uint32_t compareValue)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT32.CC[1].reg = compareValue;
    }
    
    //=====================================================================================
    //getCompareChannel1()
    //Returns the SAMD21 hardware timer instance CC1 register contents.
    //Parameters: None
    //Return: CC1 register 32-bit value
    //=====================================================================================
    uint32_t TC32Bit::getCompareChannel1() const
    {
        return pRegisterBaseAddress->COUNT32.CC[1].reg;
    }
    
    //=====================================================================================
    //setCount()
    //Sets the SAMD21 hardware timer instance COUNT register value.
    //Changes the COUNT register.
    //Parameters:
    //  count: new 32-bit value to be loaded into COUNT register
    //Return: None
    //=====================================================================================
    void TC32Bit::setCount(uint32_t count)
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        pRegisterBaseAddress->COUNT32.COUNT.reg = count;
    }
    
    //=====================================================================================
    //getCount()
    //Returns the SAMD21 hardware timer instance COUNT register contents.
    //Parameters: None
    //Return: COUNT register 32-bit value
    //=====================================================================================
    uint32_t TC32Bit::getCount() const
    {
        //Block until syncronization is complete
        while(pRegisterBaseAddress->COUNT8.STATUS.bit.SYNCBUSY == 1);
        return pRegisterBaseAddress->COUNT32.COUNT.reg;
    }
    

    
} //End SAMD21TC namespace
