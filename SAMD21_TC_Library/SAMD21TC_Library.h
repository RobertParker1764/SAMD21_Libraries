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

//This library provides classes which serve as interfaces to the SAMD21G18 TC peripheral.
//Three classes are provided, TC8Bit, TC16Bit, and TC32Bit; each of which inherits from
//the base class TCBaseClass.

//Classes are instantiated in your application by calling the factory methods (static member
//functions of TCBaseClass) get8BitTC, get16BitTC, and get32BitTC. It is important to use
//these factory methods as they provide necessary error checking to ensure that only valid
//TC instances are instantiated. They ensure that a TC instance is only associated with one
//class instantiation and that 32-bit TCs are only associated with the TC4 instance. If
//allocation of a TC instance fails for some reason the factory methods will return nullptr,
//therefore the user should always check for nullptr when instantiating a TC instance.

//All classes and associted constants and code are contained within the namespace "SAMD21TC".
//The user is required to provide a using directive, individual using declarations, or use the
//scope resolution operator (SAMD21TC::) when using names defined within this file.

//Interrupts
//This driver library uses callback functions to handle interrupts. The user is required to
//define/implement a callback function for each type of interrupt that will be used. The
//callback function is provided to the class object at the time of interrupt enabling.

//System Clock Configuration
//System clocks (generic clocks, sources, etc. neet to be configured before configuring
//and running the TC instance. This is accomplished by application code outside the scope
//of this library. Use of clock resources needs to be planned out on a per application basis.

//As of this release the following TC peripheral functionality is not supported
// - Debug control
// - Event system interface
// - DMA Operation

#ifndef SAMD21TC_Library_H_
#define SAMD21TC_Library_H_

#include "Arduino.h"

namespace SAMD21TC {
    
    /******************* Data type definitions ***********************/
    
    //clockPrescaler_t: Enforce valid clock division factors
    //passed to CTRLA register PRESCALER bits. Reference data sheet
    //paragraph 30.8.1
    //TC_CTRLA_PRESCALER(x) macro defined in file tc.h
    enum clockPrescaler_t {
        DIVIDE_BY_1             = TC_CTRLA_PRESCALER(0),
        DIVIDE_BY_2             = TC_CTRLA_PRESCALER(1),
        DIVIDE_BY_4             = TC_CTRLA_PRESCALER(2),
        DIVIDE_BY_8             = TC_CTRLA_PRESCALER(3),
        DIVIDE_BY_16            = TC_CTRLA_PRESCALER(4),
        DIVIDE_BY_64            = TC_CTRLA_PRESCALER(5),
        DIVIDE_BY_256           = TC_CTRLA_PRESCALER(6),
        DIVIDE_BY_1024          = TC_CTRLA_PRESCALER(7)
    };
    
    //counterSize_t: Enforce valid counter size values passed
    //to CTRLA register MODE bits. Reference data sheet paragraph 30.8.1
    //TC_CTRLA_MODE_x literal constants defined in file tc.h
    enum counterSize_t {
        _8BIT                = TC_CTRLA_MODE_COUNT8,
        _16BIT               = TC_CTRLA_MODE_COUNT16,
        _32BIT               = TC_CTRLA_MODE_COUNT32
    };
    
    //waveGeneration_t: Enforce valid wave generation mode settings
    //passed to CTRLA register WAVEGEN bits. Reference data sheet
    //paragraph 30.8.1
    //TC_CTRLA_WAVEGEN_x literal constance defined in file tc.h
    enum waveGeneration_t {
        NORMAL_FREQUENCY    = TC_CTRLA_WAVEGEN_NFRQ,
        MATCH_FREQUENCY     = TC_CTRLA_WAVEGEN_MFRQ,
        NORMAL_PWM          = TC_CTRLA_WAVEGEN_NPWM,
        MATCH_PWM           = TC_CTRLA_WAVEGEN_MPWM
    };
    
    //reloadResetMode_t: Enforce valid reload/reset mode settings
    //passed to CTRLA register PRESCSYNC bit. Reference data sheet
    //paragraph 30.8.1
    //TC_CTRLA_PRESCSYNC_x literal constants defined in file tc.h
    enum reloadResetMode_t {
        GENERIC_CLOCK                   = TC_CTRLA_PRESCSYNC_GCLK,
        PRESCALER_CLOCK                 = TC_CTRLA_PRESCSYNC_PRESC,
        GENERIC_CLOCK_RESET_PRESCALER   = TC_CTRLA_PRESCSYNC_RESYNC
    };
    
    
    //address_t: Enforce valid register address is passed to READREQ
    //register ADDR bits. Reference data sheet paragraph 30.8.2
    //TC_READREQ_ADDR(x) macro is defined in tc.h
    enum address_t {
        COUNT_REGISTER      = TC_READREQ_ADDR(TC_COUNT8_COUNT_OFFSET),
        CC0_REGISTER        = TC_READREQ_ADDR(TC_COUNT8_CC_OFFSET),
        CC1_REGISTER_8BIT   = TC_READREQ_ADDR(TC_COUNT8_CC_OFFSET + 1),
        CC1_REGISTER_16BIT  = TC_READREQ_ADDR(TC_COUNT8_CC_OFFSET + 2),
        CC1_REGISTER_32BIT  = TC_READREQ_ADDR(TC_COUNT8_CC_OFFSET + 4)
    };
    
    //tcCommand_t: Enforce valid command values passed to CTRLBCLR and
    //CTRLBSET register CMD bits. Reference data sheet paragraphs 30.8.3
    //and 30.8.4.
    //TC_CTRLBCLR_CMD_x literal constants defined in file tc.h
    enum tcCommand_t {
        NO_COMMAND          = TC_CTRLBCLR_CMD_NONE,
        RETRIGGER_COMMAND   = TC_CTRLBCLR_CMD_RETRIGGER,
        STOP_COMMAND        = TC_CTRLBCLR_CMD_STOP
    };
    
    //countDirection_t: Enforce valid count direction values are passed
    //to CTRLBCLR and CTRLBSET register DIR bit. Reference data sheet
    //paragraphs 30.8.3 and 30.8.4.
    enum countDirection_t {
        COUNT_UP = 0,
        COUNT_DOWN
    };
    
    //tcInterrupt_t: Interrupt types supported by the TC peripheral and
    //controlled via INTENSET, INTENCLR, and INTFLAG registers
    enum tcInterrupt_t {
        TC_OVERFLOW         = TC_INTFLAG_OVF,       //INTFLAG register bit 0 (OVF)
        TC_ERROR            = TC_INTFLAG_ERR,       //INTFLAG register bit 1 (ERR)
        TC_SYNCRDY          = TC_INTFLAG_SYNCRDY,   //INTFLAG register bit 3 (SYNCRDY)
        TC_CHANNEL_0_MATCH  = TC_INTFLAG_MC0,       //INTFLAG register bit 4 (MC0)
        TC_CHANNEL_1_MATCH  = TC_INTFLAG_MC1        //INTFLAG register bit 5 (MC1)
    };
    
    //ccChannel_t: Compare/Capture channel number definition
    enum ccChannel_t {
        CC_CHANNEL_0    = 0,
        CC_CHANNEL_1
    };
    
    //timerInstance_t: Valid Timer/Counter instances available on SAMD21G18
    enum timerInstance_t {
        _TC3 = 0,
        _TC4,
        _TC5
    };
    
    
    
    /******************************** Literal Constants *******************************/
    
    const uint8_t INTFLAG_CLEAR =   TC_INTFLAG_OVF | TC_INTFLAG_ERR |
                                    TC_INTFLAG_SYNCRDY | TC_INTFLAG_MC0 |
                                    TC_INTFLAG_MC1;
    
    
    /****************************** Forward Declarations ******************************/
    class TC8Bit;
    class TC16Bit;
    class TC32Bit;
    /******************************** Class Declartions *******************************/
    
    //Timer/Counter Base Class
    class TCBaseClass
    {
    private:
        static TCBaseClass * allocatedTC[3]; //Array of pointers to allocated TC objects
    protected:
        counterSize_t mSize;
        uint8_t instance;           //The TC instance (e.g. TC3)
        Tc* pRegisterBaseAddress;   //Pointer to the base address of the TC register stack
        void(* callBackFuncPtrs[6]) (void); //Array of call back function pointers
    public:
        //Factory methods used to allocate and delete timer/counter objects on request
        //These are the only way for a user to safely create and delete timer/counter
        //driver objects
        static TC8Bit * get8BitTC(timerInstance_t timerNumber);
        static TC16Bit * get16BitTC(timerInstance_t timerNumber);
        static TC32Bit * get32BitTC(timerInstance_t timerNumber);
        static void deleteTC(timerInstance_t timerNumber);
        
        //Interrupt Handler
        static void tcInterruptHandler(uint8_t tcNumber);
        
        //Constructor
        TCBaseClass(timerInstance_t timerNumber);
        virtual ~TCBaseClass() {}
        //Public Interface
        counterSize_t getSize() {return mSize;}
        uint8_t getCounterInstance() {return instance;}
        void enableTC();
        void disableTC();
        void resetTC();
        void retriggerTC();
        void stopTC();
        bool isStopped() const;
        void setOneShotMode(bool oneShot);
        void setCountDirection(countDirection_t direction);
        bool configureTimer(clockPrescaler_t clockDivision,
                            waveGeneration_t waveGenerationMode,
                            uint8_t waveformOutput0Pin = 0,
                            uint8_t waveformOutput1Pin = 0,
                            bool enable = false,
                            reloadResetMode_t prescalerSyncMode = GENERIC_CLOCK,
                            countDirection_t countDirection = COUNT_UP,
                            bool invertOutput0 = false,
                            bool invertOutput1 = false,
                            bool runStandby = false,
                            bool oneShotTimer = false
                            );
        bool configureCaptureOperation();
        
        //Interrupt handling functions
        void handleInterrupt();
        void enableInterrupt(tcInterrupt_t interuptName, void(* callbackFunc)(void));
        void disableInterrupt(tcInterrupt_t interuptName);
        uint8_t getInterruptEnableState();
        uint8_t getInterruptFlags();
        void clearInterruptFlags(uint8_t flags);
        
        //Lower level methods (mainly used for debugging)
        void setControlRegisterA(uint16_t value);
        uint16_t getControlRegisterA(void);
        uint8_t getControlRegisterB();
        
    };
    
    //8-Bit Timer/Counter Class
    class TC8Bit : public TCBaseClass
    {
    private:
    public:
        //Constructor
        TC8Bit(timerInstance_t timerNumber);
        //Destructor
        ~TC8Bit() {}
        //Public Interface
        void setPeriod(uint8_t period);
        uint8_t getPeriod() const;
        void setCompareChannel0(uint8_t compareValue);
        uint8_t getCompareChannel0() const;
        void setCompareChannel1(uint8_t compareValue);
        uint8_t getCompareChannel1() const;
        void setCount(uint8_t count);
        uint8_t getCount() const;
        
        
    };
    
    //16-Bit Timer/Counter Class
    class TC16Bit: public TCBaseClass
    {
    private:
    public:
        //Constructor
        TC16Bit(timerInstance_t timerNumber);
        //Destructor
        ~TC16Bit() {}
        //Public Interface
        void setCompareChannel0(uint16_t compareValue);
        uint16_t getCompareChannel0() const;
        void setCompareChannel1(uint16_t compareValue);
        uint16_t getCompareChannel1() const;
        void setCount(uint16_t count);
        uint16_t getCount() const;
        
    };
    
    //32-Bit Timer/Counter Class
    class TC32Bit: public TCBaseClass
    {
    private:
    public:
        //Constructor
        TC32Bit(timerInstance_t timerNumber);
        //Destructor
        ~TC32Bit() {}
        //Public Interface
        void setCompareChannel0(uint32_t compareValue);
        uint32_t getCompareChannel0() const;
        void setCompareChannel1(uint32_t compareValue);
        uint32_t getCompareChannel1() const;
        void setCount(uint32_t count);
        uint32_t getCount() const;
        
    };

    
} //End SAMD21TC namespace

#endif
