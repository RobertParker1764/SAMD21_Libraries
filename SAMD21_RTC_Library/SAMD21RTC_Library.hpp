//SAMD21G18 Real-Timer Counter (RTC) Peripheral Driver
//Version 1.0
//Date
//By Robert Parker

/*
 Copyright (c) 2019 Robert Parker. All right reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

//This library provides classes which serve as interfaces to the SAMD21G18 RTC peripheral.

#ifndef SAMD21RTC_Library_H_
#define SAMD21RTC_Library_H_

#include "Arduino.h"

namespace SAMD21RTC {
    
    /**************************** Data type definitions *******************************/
    
    //clockPrescaler_t: Enforce valid clock division factors
    //passed to CTRL register PRESCALER bits. Reference data sheet
    //paragraph 19.8.1
    //RTC_MODE0_CTRL_PRESCALER_DIVx constants defined in file include/component/rtc.h
    enum rtcClockPrescaler_t {
        DIVIDE_BY_1         = RTC_MODE0_CTRL_PRESCALER_DIV1,
        DIVIDE_BY_2         = RTC_MODE0_CTRL_PRESCALER_DIV2,
        DIVIDE_BY_4         = RTC_MODE0_CTRL_PRESCALER_DIV4,
        DIVIDE_BY_8         = RTC_MODE0_CTRL_PRESCALER_DIV8,
        DIVIDE_BY_16        = RTC_MODE0_CTRL_PRESCALER_DIV16,
        DIVIDE_BY_32        = RTC_MODE0_CTRL_PRESCALER_DIV32,
        DIVIDE_BY_64        = RTC_MODE0_CTRL_PRESCALER_DIV64,
        DIVIDE_BY_128       = RTC_MODE0_CTRL_PRESCALER_DIV128,
        DIVIDE_BY_256       = RTC_MODE0_CTRL_PRESCALER_DIV256,
        DIVIDE_BY_512       = RTC_MODE0_CTRL_PRESCALER_DIV512,
        DIVIDE_BY_1024      = RTC_MODE0_CTRL_PRESCALER_DIV1024
    };
    
    //rtcMode_t: Enforce valid mode values are passed to CTRL register MODE bits.
    //Reference data sheet paragraph 19.8.1
    //RTC_MODE0_CTRL_MODE_x constants defined in file include/component/rtc.h
    enum rtcMode_t {
        COUNTER_32_BIT      = RTC_MODE0_CTRL_MODE_COUNT32,
        COUNTER_16_BIT      = RTC_MODE0_CTRL_MODE_COUNT16,
        REAL_TIME_CLOCK     = RTC_MODE0_CTRL_MODE_CLOCK
    };
    
    enum rtcInterrupt_t {
        RTC_OVERFLOW         = RTC_MODE0_INTENCLR_OVF,       //INTFLAG register bit 7 (OVF)
        RTC_SYNCRDY          = RTC_MODE0_INTENCLR_SYNCRDY,   //INTFLAG register bit 6 (SYNCRDY)
        RTC_COMPARE_0        = RTC_MODE0_INTENCLR_CMP0,      //INTFLAG register bit 0 (CMP0)
        RTC_COMPARE_1        = RTC_MODE1_INTENCLR_CMP1,      //INTFLAG register bit 1 (CMP1)
        RTC_ALARM            = RTC_MODE2_INTENCLR_ALARM0     //INTFLAG register bit 0 (ALARM0)
    };
    
    enum rtcEvent_t {
        RTC_PERIODIC_INTERVAL_0_EVENT     = RTC_MODE0_EVCTRL_PEREO0,
        RTC_PERIODIC_INTERVAL_1_EVENT     = RTC_MODE0_EVCTRL_PEREO1,
        RTC_PERIODIC_INTERVAL_2_EVENT     = RTC_MODE0_EVCTRL_PEREO2,
        RTC_PERIODIC_INTERVAL_3_EVENT     = RTC_MODE0_EVCTRL_PEREO3,
        RTC_PERIODIC_INTERVAL_4_EVENT     = RTC_MODE0_EVCTRL_PEREO4,
        RTC_PERIODIC_INTERVAL_5_EVENT     = RTC_MODE0_EVCTRL_PEREO5,
        RTC_PERIODIC_INTERVAL_6_EVENT     = RTC_MODE0_EVCTRL_PEREO6,
        RTC_PERIODIC_INTERVAL_7_EVENT     = RTC_MODE0_EVCTRL_PEREO7,
        RTC_COMPARE_0_EVENT             = RTC_MODE0_EVCTRL_CMPEO0,
        RTC_CLOCK_ALARM_EVENT           = RTC_MODE2_EVCTRL_ALARMEO0,
        RTC_COMPARE_1_EVENT             = RTC_MODE1_EVCTRL_CMPEO1,
        RTC_OVERFLOW_EVENT              = RTC_MODE0_EVCTRL_OVFEO
    };
    
    enum rtcMonth_t {
        JANUARY = 1,
        FEBRUARY,
        MARCH,
        APRIL,
        MAY,
        JUNE,
        JULY,
        AUGUST,
        SEPTEMBER,
        OCTOBER,
        NOVEMBER,
        DECEMBER,
        NO_MONTH
    };
    
    enum rtcClockMode_t {
        _12HOUR = 1,
        _24HOUR
    };
    
    enum rtcAMPM_t {
        _AM,
        _PM,
        NOT_SPECIFIED
    };
    
    struct dateTime_t {
        rtcClockMode_t mode;
        uint16_t year;
        rtcMonth_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        rtcAMPM_t AMorPM;
    };
    
    enum rtcAlarmEnable_t {
        OFF = 0,
        SS,
        MMSS,
        HHMMSS,
        DDHHMMSS,
        MMDDHHMMSS,
        YYMMDDHHMMSS
    };
    
    
     
    
    /******************************** Literal Constants *******************************/
    
    const uint8_t INTFLAG_CLEAR =   RTC_MODE0_INTENCLR_OVF | RTC_MODE0_INTENCLR_SYNCRDY |
                                    RTC_MODE0_INTENCLR_CMP0 | RTC_MODE1_INTENCLR_CMP1;
    const unsigned int BASE_YEAR = 2016;     //Reference year for mode 2 (Clock/Calendar)
    const uint8_t MODE0_INTR_MASK = (RTC_OVERFLOW | RTC_SYNCRDY | RTC_COMPARE_0);
    const uint8_t MODE1_INTR_MASK = (RTC_OVERFLOW | RTC_SYNCRDY | RTC_COMPARE_0 | RTC_COMPARE_1);
    const uint8_t MODE2_INTR_MASK = (RTC_OVERFLOW | RTC_SYNCRDY | RTC_ALARM);
    
    /****************************** Forward Declarations ******************************/
    class Counter_32Bit;
    class Counter_16Bit;
    class ClockCalendar;
    
    /******************************** Class Declartions *******************************/
    //Real-Timer Conter Base Class
    class RTC_Base_Class {
    private:
        static bool RTC_Allocated; //Prevents more than one RTC instance from being created
        
    protected:
        Rtc * pRegisterBaseAddress;
        void(* callBackFuncPtrs[8]) (void); //Array of call back function pointers
        
    public:
        
        //Factory methods used to allocate and delete RTC objects on request
        //These are the only way for a user to safely create and delete RTC
        //driver objects
        static Counter_32Bit * get32BitRTC();
        static Counter_16Bit * get16BitRTC();
        static ClockCalendar * getRTCClock();
        static void deleteRTC();
        
        //Constructor
        RTC_Base_Class();
        //Destructor
        virtual ~RTC_Base_Class() {}
        
        //Public Interface
        void enableRTC();
        void disableRTC();
        void resetRTC();
        void requestClockCountSync(bool continuous = false);
        void setFrequencyCorrection(uint8_t correction, bool speedUp);
        
        //Event Management
        void enableEvents(rtcEvent_t events);

        //Low Level Public Interface
        void setControlRegister(uint16_t value);
        uint16_t getControlRegister() const;
        
        
    };
    
    class Counter_32Bit : public RTC_Base_Class
    {
    public:
        
        //Constructor
        Counter_32Bit() {}
        //Destructor
        ~Counter_32Bit() {}
        
        //Public Interface
        void configureRTC(rtcClockPrescaler_t prescale, bool clearOnMatch = false);
        void setCount(uint32_t count);
        uint32_t getCount() const;
        void setCompareValue(uint32_t compareValue);
        uint32_t getCompareValue() const;
        
        //Interrupt Management
        uint8_t getInterruptFlags() const;
        void clearInterruptFlags(uint8_t flags);
        uint8_t getInterruptEnableState();
        void enableInterrupt(rtcInterrupt_t interuptName, void(* callbackFunc)(void));
        void disableInterrupt(rtcInterrupt_t interuptName);
        void handleInterrupt();
    };
    
    class Counter_16Bit : public RTC_Base_Class
    {
    public:
        
        //Constructor
        Counter_16Bit() {}
        //Destructor
        ~Counter_16Bit() {}
        
        //Public Interface
        void configureRTC(rtcClockPrescaler_t prescale);
        void setCount(uint16_t count);
        uint16_t getCount() const;
        void setPeriod(uint16_t period);
        uint16_t getPeriod() const;
        void setCompareValue0(uint16_t compareValue);
        uint16_t getCompareValue0() const;
        void setCompareValue1(uint16_t compareValue);
        uint16_t getCompareValue1() const;
        
        //Interrupt Management
        uint8_t getInterruptFlags() const;
        void clearInterruptFlags(uint8_t flags);
        uint8_t getInterruptEnableState();
        void enableInterrupt(rtcInterrupt_t interuptName, void(* callbackFunc)(void));
        void disableInterrupt(rtcInterrupt_t interuptName);
        void handleInterrupt();
        
    };
    
    class ClockCalendar : public RTC_Base_Class
    {
    public:
        
        //Constructor
        ClockCalendar() {}
        //Destructor
        ~ClockCalendar() {}
        
        //Public Interface
        void configureRTC(rtcClockPrescaler_t prescale, rtcClockMode_t mode,
                          bool clearOnMatch = false);
        bool setClockCalendar(uint16_t year, rtcMonth_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second = 0,
                              rtcAMPM_t AMorPM = NOT_SPECIFIED);
        String getTime() const;
        String getDate() const;
        dateTime_t getDateTime() const;
        rtcClockMode_t getMode() const;
        bool setAlarm(uint16_t year, rtcMonth_t month, uint8_t day, uint8_t hour,
                      uint8_t minute, uint8_t second, rtcAlarmEnable_t enable,
                      rtcAMPM_t AMorPM);
        void clearAlarm();
        uint32_t getClockRegister();
        
        //Interrupt Management
        uint8_t getInterruptFlags() const;
        void clearInterruptFlags(uint8_t flags);
        uint8_t getInterruptEnableState();
        void enableInterrupt(rtcInterrupt_t interuptName, void(* callbackFunc)(void));
        void disableInterrupt(rtcInterrupt_t interuptName);
        void handleInterrupt();
    };
    
    
} //End SAMD21RTC namespace

#endif
