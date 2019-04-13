//
// LIS331_Driver.cpp
// Library C++ code
// ----------------------------------
// Developed with embedXcode+
// http://embedXcode.weebly.com
//
// Details	    Classes to manage the ST Microelectronics LIS331 Accelerometer
//
// Created by 	Bob Parker, 3/26/18 8:12 PM
// 				Robert Parker
//
// Copyright 	(c) Bob Parker, 2018
// Licence		<#licence#>
//
// See 			LIS331_Driver.h and ReadMe.txt for references
//


// Library header
#include "LIS331_Driver.h"
#include "Wire.h"
#include "SPI.h"

namespace LIS331

//=========================================================================================
//  Base Class Method Definitions
//=========================================================================================


{
    //Constructor
    LIS331Driver::LIS331Driver()
    {
        _mgPerLSB = 0;
        _ODR = 0;
        
        for (uint8_t index = 0; index < 3; index++) {
            _offset[index] = 0.0;
            _gainCal[index] = 1.00;
        }
    }
    
    //=======================================================================
    //initialize()
    //Initializes Control Regiters 1 and 4 with operating mode, data
    //rate, axis, and full scale range settings.
    //Parameters:
    //mode:     The operating mode. See Table 18 of the data sheet
    //rate:     The output data rate. See Table 19 of the data sheet
    //axis:     The axis that are to be enabled for measurements.
    //range:    The full scale acceleration range. See Table 28 of the data sheet
    //Return: None
    //=======================================================================
    void LIS331Driver::initialize(powerMode_t mode, outputDataRate_t rate, enableAxisSettings_t axis, fullScaleSetting_t range)
    {
        //Initialize the data members
        if (mode == NORMAL_MODE) {
            switch (rate) {
                case ODR_50HZ:
                    _ODR = 50.0;
                    break;
                case ODR_100HZ:
                    _ODR = 100.0;
                    break;
                case ODR_400HZ:
                    _ODR = 400.0;
                    break;
                case ODR_1000HZ:
                    _ODR = 1000.0;
                    break;
            }
        } else {
            switch (mode) {
                case LOW_POWER_0_5HZ:
                    _ODR = 0.5;
                    break;
                case LOW_POWER_1HZ:
                    _ODR = 1.0;
                    break;
                case LOW_POWER_2HZ:
                    _ODR = 2.0;
                    break;
                case LOW_POWER_5HZ:
                    _ODR = 5.0;
                    break;
                case LOW_POWER_10HZ:
                    _ODR = 10.0;
                    break;
                default:
                    _ODR = 0.0;
            }
        }
        
        switch (range) {
            case _6G:
                _mgPerLSB = 3;
                break;
            case _12G:
                _mgPerLSB = 6;
                break;
            case _24G:
                _mgPerLSB = 12;
        }
        
        //Update Control Register 1 with settings
        uint8_t registerValue = mode | rate | axis;
        writeRegister(CTRL_REG1, registerValue);
        
        
        //Read current control register 4 value. We don't want to
        //change anything other than the full scale setting.
        registerValue = readRegister(CTRL_REG4);
        //Clear the FS1/FS0 bits
        registerValue &= ~(FULL_SCALE_MASK);
        //Write the new value with updated full scale range setting
        writeRegister(CTRL_REG4, (registerValue | range));
    }
    
    //=======================================================================
    //initIntrInterface()
    //Initializes control register 3 which configures the systems interface
    //of the LIS331 interrupt hardware.
    //Parameters:
    //intr1Pin:     Configures the internal interrupt source attached to pin 1
    //              Interrupt source 1 is default
    //intr2Pin:     Configures the internal interrupt source attached to pin 2
    //              Interrupt source 2 is default
    //activeLow:    If true the interrupt output is configured as active low.
    //              If false (default it is configured as active high.
    //openDrain:    If true the interrupt output is configured as an open
    //              drain output. If false (defaut it is configures as a push-
    //              pull output.
    //inter1Latched:If true the internal interrupt 1 source is latched until
    //              cleard. If false (default) the interrupt is not latched.
    //inter2Latched:If true the internal interrupt 2 source is latched until
    //              cleard. If false (default) the interrupt is not latched.
    //Return: None
    //=======================================================================
    void LIS331Driver::initIntrInterface(intrPin1Config_t intr1Pin,
                                         intrPin2Config_t intr2Pin,
                                         bool activeLow, bool openDrain,
                                         bool inter1Latched, bool inter2Latched)
    {
        uint8_t registerValue = intr1Pin | intr2Pin ;
        
        if (activeLow) {
            registerValue |= IHL_BIT;
        }
        
        if (openDrain) {
            registerValue |= PP_OD_BIT;
        }
        
        if (inter1Latched) {
            registerValue |= LIR1_BIT;
        }
        
        if (inter2Latched) {
            registerValue |= LIR2_BIT;
        }
        
        //Write the new control register 3 value
        writeRegister(CTRL_REG3, registerValue);
    }
    //=======================================================================
    //setMode()
    //Sets the power mode
    //Parameters:
    //mode: The operating mode. See Table 18 of the data sheet
    //Return: None
    //=======================================================================
    void LIS331Driver::setMode(powerMode_t mode)
    {
        uint8_t registerValue = readRegister(CTRL_REG1);
        registerValue &= ~(POWER_MODE_MASK);
        writeRegister(CTRL_REG1, (registerValue | mode));
    }
    
    //=======================================================================
    //setODR()
    //Sets the output data rate
    //Parameters:
    //rate: The output data rate. See Table 19 of the data sheet
    //Return: None
    //=======================================================================
    void LIS331Driver::setODR(outputDataRate_t rate)
    {
        uint8_t registerValue = readRegister(CTRL_REG1);
        registerValue &= ~(DATA_RATE_MASK);
        writeRegister(CTRL_REG1, (registerValue | rate));
    }
    
    //=======================================================================
    //enableAxis()
    //Enables the specified axis for acceleration measurement
    //Parameters:
    //axis: The axis that are to be enabled
    //Return: None
    //=======================================================================
    void LIS331Driver::enableAxis(enableAxisSettings_t axis)
    {
        uint8_t registerValue = readRegister(CTRL_REG1);
        registerValue &= ~(AXIS_ENABLE_MASK);
        writeRegister(CTRL_REG1, (registerValue | axis));
    }
    
    //=======================================================================
    //refreshInternalRegisters()
    //Initiates and internal register refresh cycle
    //Parameters: None
    //Return: None
    //=======================================================================
    void LIS331Driver::refreshInternalRegisters()
    {
        uint8_t registerValue = readRegister(CTRL_REG2);
        writeRegister(CTRL_REG2, registerValue  | BOOT_BIT);
    }
    
    //=======================================================================
    //setBlockDataUpdate()
    //Sets or clears the block data update bit.
    //Parameters:
    //set:  If true then the bit is set, otherwise it is cleared
    //Return: None
    //=======================================================================
    void LIS331Driver::setBlockDataUpdate(bool set)
    {
        uint8_t registerValue = readRegister(CTRL_REG4);
        if (set) {
            writeRegister(CTRL_REG4, registerValue | BDU_BIT);
        } else {
            writeRegister(CTRL_REG4, registerValue & ~BDU_BIT);
        }
    }
    
    //=======================================================================
    //setBigEndian()
    //Sets or clears the big / little endian bit.
    //Parameters:
    //set:  If true then the bit is set, otherwise it is cleared
    //Return: None
    //=======================================================================
    void LIS331Driver::setBigEndian(bool bigEndian)
    {
        uint8_t registerValue = readRegister(CTRL_REG4);
        if (bigEndian) {
            writeRegister(CTRL_REG4, registerValue | BLE_BIT);
        } else {
            writeRegister(CTRL_REG4, registerValue & ~BLE_BIT);
        }
    }
    
    //=======================================================================
    //setMeasurementRange()
    //Sets the full scale acceleration measurement range
    //Parameters:
    //range:    The full scale measurement range setting
    //Return: None
    //=======================================================================
    void LIS331Driver::setMeasurementRange(fullScaleSetting_t range)
    {
        uint8_t registerValue = readRegister(CTRL_REG4);
        registerValue &= ~(FULL_SCALE_MASK);
        writeRegister(CTRL_REG4, (registerValue | range));
        
        //Update _mgPerLSB
        switch (range) {
            case _6G:
                _mgPerLSB = 3;
                break;
                
            case _12G:
                _mgPerLSB = 6;
                break;
                
            case _24G:
                _mgPerLSB = 12;
                break;
        }
    }
    
    //=======================================================================
    //readXAxisAcceleration()
    //Returns the current value of the x-axis acceleration reading
    //in G's.
    //Parameters: None
    //Return: the 16-bit x-axis acceleration value
    //=======================================================================
    float LIS331Driver::readXAxisAcceleration()
    {
        int16_t registerValue = readAccelRegisterPair(OUT_X_L_REG);
        
        //Right shift to get the 12 bit value. Return the value
        //after scaling for output in Gs and correcting for offset error
        //and measurement gain error.
        registerValue >>= 4;
        return (float)((registerValue * _mgPerLSB * _gainCal[0] / 1000.0) + _offset[0]);
        
    }
    
    //=======================================================================
    //readYAxisAcceleration()
    //Returns the current value of the y-axis acceleration reading
    //in G's.
    //Parameters: None
    //Return: the 16-bit y-axis acceleration value
    //=======================================================================
    float LIS331Driver::readYAxisAcceleration()
    {
        int16_t registerValue = readAccelRegisterPair(OUT_Y_L_REG);
        
        //Right shift to get the 12 bit value. Return the value
        //after scaling for output in Gs and correcting for offset error.
        registerValue >>= 4;
        return (float)((registerValue * _mgPerLSB * _gainCal[1] / 1000.0) + _offset[1]);
    }
    
    //=======================================================================
    //readZAxisAcceleration()
    //Returns the current value of the z-axis acceleration reading
    //in G's.
    //Parameters: None
    //Return: the 16-bit z-axis acceleration value
    //=======================================================================
    float LIS331Driver::readZAxisAcceleration()
    {
        int16_t registerValue = readAccelRegisterPair(OUT_Z_L_REG);
        
        //Right shift to get the 12 bit value. Return the value
        //after scaling for output in Gs and correcting for offset error.
        registerValue >>= 4;
        return (float)((registerValue * _mgPerLSB * _gainCal[2] / 1000.0) + _offset[2]);
    }
    
    //=======================================================================
    //readAcceleration()
    //Reads the 3-channels of acceleration data and returns the
    //results in the caller provided array. Acceleration values are
    //in G's.
    //Parameters:
    //accelArray: An array of 3 int16_t to hold the accel values
    //Return: None
    //=======================================================================
    void LIS331Driver::readAcceleration(float accelArray[])
    {
        int16_t valueArray[3];
        readAccelRegisters(valueArray);
        
        for (uint8_t i = 0; i < 3; i++) {
            valueArray[i] >>= 4;
            accelArray[i] = (float)((valueArray[i] * _mgPerLSB * _gainCal[i]/ 1000.0) + _offset[i]);
        }
        
    }
    
    //=======================================================================
    //isDataReady()
    //Checks the status register for the state of the ZYXDA bit. If
    //set then a new set of data is ready. This method only checks
    //the ZYXDA bit. The user must read the status register and check
    //for bit state to check on the ZDA, YDA, or XDA bits individually.
    //Parameters: None
    //Return: True if the ZYXDA bit is set. False otherwise.
    //=======================================================================
    bool LIS331Driver::isDataReady()
    {
        uint8_t registerValue = readRegister(STATUS_REG);
        if (registerValue & ZYXDA_BIT) {
            return true;
        } else {
            return false;
        }
    }
    
    //=======================================================================
    //isDataOverrun()
    //Checks the status register for the state of the ZYXOR bit. If
    //set then data has been overrun. This method only checks
    //the ZYXOR bit. The user must read the status register and check
    //for bit state to check on the ZOR, YOR, or XOR bits individually.
    //Parameters: None
    //Return: True if the ZYXOR bit is set. False otherwise.
    //=======================================================================
    bool LIS331Driver::isDataOverrun()
    {
        uint8_t registerValue = readRegister(STATUS_REG);
        if (registerValue & ZYXOR_BIT) {
            return true;
        } else {
            return false;
        }
    }
    
    //=======================================================================
    //readStatusRegister()
    //Returns the currrent value of the status register.
    //Parameters: None
    //Return: Status register value.
    //=======================================================================
    uint8_t LIS331Driver::readStatusRegister()
    {
        return readRegister(STATUS_REG);
    }
    
    //=======================================================================
    //Methods for configuring and checking interrupts                       =
    //=======================================================================
    
    //=======================================================================
    //enableInterrupt1()
    //Configures and enables interrupt source 1.
    //Parameters:
    //mode:         The interrupt mode as defined by the AOI and 6D bits in the
    //              INT1_CFG register. Reference Table 38 of the data sheet.
    //enableAxis:   The axis and high/low threshold condition to be enabled.
    //              The user composes this byte by ORing the individual
    //              axis enable bits. Reference Table 37 of the data sheet.
    //threshold:    The detection threshold in Gs.
    //duration:     The detection duration in seconds.
    //Return: None
    //=======================================================================
    void LIS331Driver::enableInterrupt1(interruptModeSetting_t mode, uint8_t enableAxis, float threshold, float duration)
    {
        //Write the threshold and duration values
        //The threshold setting depends on the full scale setting
        if (_mgPerLSB == 3) {
            writeRegister(INT1_THS_REG, (int8_t)(threshold * 1000 / 48));
        } else if (_mgPerLSB == 6) {
            writeRegister(INT1_THS_REG, (int8_t)(threshold * 1000 / 93));
        } else {
            writeRegister(INT1_THS_REG, (int8_t)(threshold * 1000 / 189));
        }
        
        //The duration setting depends on the ODR
        writeRegister(INT1_DURATION_REG, (uint8_t)(duration * _ODR));
        
        //Write the interrupt 1 configuration register
        writeRegister(INT1_CFG_REG, (mode | enableAxis));
        
    }
    
    //=======================================================================
    //enableInterrupt2()
    //Configures and enables interrupt source 2.
    //Parameters:
    //mode:         The interrupt mode as defined by the AOI and 6D bits in the
    //              INT2_CFG register. Reference Table 38 of the data sheet.
    //enableAxis:   The axis and high/low threshold condition to be enabled.
    //              The user composes this byte by ORing the individual
    //              axis enable bits. Reference Table 37 of the data sheet.
    //threshold:    The detection threshold in Gs.
    //duration:     The detection duration in seconds.
    //Return: None
    //=======================================================================
    void LIS331Driver::enableInterrupt2(interruptModeSetting_t mode, uint8_t enableAxis, float threshold, float duration)
    {
        //Write the threshold and duration values
        //The threshold setting depends on the full scale setting
        if (_mgPerLSB == 3) {
            writeRegister(INT2_THS_REG, (int8_t)(threshold * 1000 / 48));
        } else if (_mgPerLSB == 6) {
            writeRegister(INT2_THS_REG, (int8_t)(threshold * 1000 / 93));
        } else {
            writeRegister(INT2_THS_REG, (int8_t)(threshold * 1000 / 189));
        }
        
        //The duration setting depends on the ODR
        writeRegister(INT2_DURATION_REG, (uint8_t)(duration * _ODR));
        
        //Write the interrupt 1 configuration register
        writeRegister(INT2_CFG_REG, (mode | enableAxis));
    }
    
    //=======================================================================
    //getInterrupt1Source()
    //Returns the contents of the interrupt 1 source register so that the
    //source of an interrupt can be determined.
    //Parameters: None
    //Return: The contents of the interrupt 1 source register.
    //=======================================================================
    uint8_t LIS331Driver::getInterrupt1Source()
    {
        return readRegister(INT1_SRC_REG);
    }
    
    //=======================================================================
    //getInterrupt2Source()
    //Returns the contents of the interrupt 2 source register so that the
    //source of an interrupt can be determined.
    //Parameters: None
    //Return: The contents of the interrupt 2 source register.
    //=======================================================================
    uint8_t LIS331Driver::getInterrupt2Source()
    {
        return readRegister(INT2_SRC_REG);
    }
    
    //=======================================================================
    //disableInterrupt1()
    //Disables interrupt detection on all axis
    //Parameters: None
    //Return: None
    //=======================================================================
    void LIS331Driver::disableInterrupt1()
    {
        uint8_t registerValue = readRegister(INT1_CFG_REG);
        registerValue &= ~(AXIS_BITS_MASK);
        writeRegister(INT1_CFG_REG, registerValue);
    }
    
    //=======================================================================
    //disableInterrupt2()
    //Disables interrupt detection on all axis
    //Parameters: None
    //Return: None
    //=======================================================================
    void LIS331Driver::disableInterrupt2()
    {
        int8_t registerValue = readRegister(INT2_CFG_REG);
        registerValue &= ~(AXIS_BITS_MASK);
        writeRegister(INT2_CFG_REG, registerValue);
    }
    
    //=======================================================================
    //Methods for configuring and controlling the high pass filter          =
    //=======================================================================
    
    //=======================================================================
    //resetHPF()
    //Zeros the content of the high pass filter. If the high pass
    //filter is enabled all enabled axis are set to 0g.
    //Parameters: None
    //Return: None
    //=======================================================================
    void LIS331Driver::resetHPF()
    {
        readRegister(HP_FILTER_RESET_REG);
    }
    
    //=======================================================================
    //configureHPF()
    //Configures control register 2 and the reference register as
    //required to configure the internal high pass filter in
    //accordance with the provided parameters.
    //Parameters:
    //bypass:       If true the high pass filter is disabled for data output
    //coef:         The high pass filter coeficients
    //refMode:      If true the high pass filter is in reference mode
    //              If false (default) the high pass filter is in normal mode
    //reference:    The reference value used by the high pass filter (in Gs).
    //Return: None
    //=======================================================================
    void LIS331Driver::configureHPF(bool bypass, hp_Filter_Coeficient_t coef,
                                    bool refMode, float reference)
    {
        //Get the current value of Control Register 2 so we can change
        //only what we want
        uint8_t registerValue = readRegister(CTRL_REG2);
        
        //Clear the HPCF bits
        registerValue &= ~(HPCF_MASK);
        
        //Configure the HPCF bits
        registerValue |= coef;
        
        //Configure the FDS bit
        if (!bypass) {
            registerValue |= FDS_BIT;
        }
        
        //Configure the HPM0 bit
        if (refMode) {
            //Set the HPM0 bit
            registerValue |= HPM0_BIT;
            
            //Convert reference float value in G's to reference register int8_t format
            int8_t refRegValue;
            if (_mgPerLSB == 3) {
                refRegValue = (int8_t)(reference * 48 / 1000);
            } else if (_mgPerLSB == 6) {
                refRegValue = (int8_t)(reference * 93 / 1000);
            } else {
                refRegValue = (int8_t)(reference * 189 / 1000);
            }
            
            writeRegister(REFERENCE_REG, refRegValue);
        }
        
        //Write Control Register 2
        writeRegister(CTRL_REG2, registerValue);
    }
    
    //=======================================================================
    //setHPFMode()
    //Sets the high pass filter mode.
    //Parameters:
    //referenceMode:    If true the high pass filter is configured for
    //                  reference mode. If false normal mode is configured.
    //Return: None
    //=======================================================================
    void LIS331Driver::setHPFMode(bool referenceMode)
    {
        //Read control register 2
        uint8_t registerValue = readRegister(CTRL_REG2);
        
        //Set or clear the HPM0 bit
        if (referenceMode) {
            registerValue |= HPM0_BIT;
        } else {
            registerValue &= ~(HPM0_BIT);
        }
        
        //Write the new control register 2 value
        writeRegister(CTRL_REG2, registerValue);
    }
    
    //=======================================================================
    //setFilterDataSelect()
    //Sets the filter data selection to either bypass the filter or not.
    //Parameters:
    //byPassed: If true the high pass filter data is not used in the output
    //          (bypassed). If false the fiter data is sent to the output.
    //Return: None
    //=======================================================================
    void LIS331Driver::setFilterDataSelect(bool byPassed)
    {
        //Read control register 2
        uint8_t registerValue = readRegister(CTRL_REG2);
        
        //Set or clear the HPM0 bit
        if (byPassed) {
            registerValue &= ~(FDS_BIT);
        } else {
            registerValue |= FDS_BIT;
        }
        
        //Write the new control register 2 value
        writeRegister(CTRL_REG2, registerValue);
    }
    
    //=======================================================================
    //enableHPFInt1()
    //Enables or disables use of the high pass filter data for interrupt 1
    //Parameters:
    //enable:   If true the high pass filter data is used for interrupt 1
    //          source.
    //Return: None
    //=======================================================================
    void LIS331Driver::enableHPFInt1(bool enable)
    {
        //Read control register 2
        uint8_t registerValue = readRegister(CTRL_REG2);
        
        //Set or clear the HPen1 bit
        if (enable) {
            registerValue |= HPEN1_BIT;
        } else {
            registerValue &= ~(HPEN1_BIT);
        }
        
        //Write the new control register 2 value
        writeRegister(CTRL_REG2, registerValue);
    }
    
    //=======================================================================
    //enableHPFInt2()
    //Enables or disables use of the high pass filter data for interrupt 2
    //Parameters:
    //enable:   If true the high pass filter data is used for interrupt 2
    //          source.
    //Return: None
    //=======================================================================
    void LIS331Driver::enableHPFInt2(bool enable)
    {
        //Read control register 2
        uint8_t registerValue = readRegister(CTRL_REG2);
        
        //Set or clear the HPen1 bit
        if (enable) {
            registerValue |= HPEN2_BIT;
        } else {
            registerValue &= ~(HPEN2_BIT);
        }
        
        //Write the new control register 2 value
        writeRegister(CTRL_REG2, registerValue);
    }
    
    //=======================================================================
    //setHPFCoeficients()
    //Configures the HPCF bits according to the user provided parameter
    //Parameters:
    //coef:   The desired setting for the HPCF bits
    //Return: None
    //=======================================================================
    void LIS331Driver::setHPFCoeficients(hp_Filter_Coeficient_t coef)
    {
        //Read control register 2
        uint8_t registerValue =  readRegister(CTRL_REG2);
        
        //Clear the HPCF bits and set the new values
        registerValue &= ~(HPCF_MASK);
        registerValue |= coef;
        
        //Write the new control register 2 value
        writeRegister(CTRL_REG2, registerValue);
    }
    
    //=======================================================================
    //Methods for controlling self test                                     =
    //=======================================================================
    
    //=======================================================================
    //setSelfTestMinus()
    //Sets the self test function to provide a minus/plus input.
    //Parameters:
    //stMinus:  If true then self test is set for minus input. It is set for
    //          positive input otherwise.
    //Return: None
    //=======================================================================
    void LIS331Driver::setSelfTestMinus(bool stMinus)
    {
        //Read the current control register 4 value
        uint8_t registerValue = readRegister(CTRL_REG4);
        
        //Set/clear the STsign bit
        if (stMinus) {
            registerValue |= ST_SIGN_BIT;
        } else {
            registerValue &= ~(ST_SIGN_BIT);
        }
        
        //Write the new control register 4 value
        writeRegister(CTRL_REG4, registerValue);
    }
    
    //=======================================================================
    //enableSelfTest()
    //Enables/disables LIS331 self test
    //Parameters:
    //selfTest: If true then self test is enabled. Otherwise it is disabled.
    //Return: None
    //=======================================================================
    void LIS331Driver::enableSelfTest(bool selfTest)
    {
        //Read the current control register 4 value
        uint8_t registerValue = readRegister(CTRL_REG4);
        
        //Set/clear the Self Test bit
        if (selfTest) {
            registerValue |= ST_BIT;
        } else {
            registerValue &= ~(ST_BIT);
        }
        
        //Write the new control register 4 value
        writeRegister(CTRL_REG4, registerValue);
    }
    
    //=======================================================================
    //Methods supporting calibration                                        =
    //=======================================================================
    
    
    void LIS331Driver::setOffsetCalibration(float xOffset, float yOffset, float zOffset)
    {
        _offset[0] = xOffset;
        _offset[1] = yOffset;
        _offset[2] = zOffset;
    }
    
    void LIS331Driver::setGainCalibration(float xCal, float yCal, float zCal)
    {
        _gainCal[0] = xCal;
        _gainCal[1] = yCal;
        _gainCal[2] = zCal;
    }
    
    
    //=========================================================================================
    //  LIS331_I2C_Driver Class Method Definitions
    //=========================================================================================
    
    //Constructor
    LIS331_I2C_Driver::LIS331_I2C_Driver(uint8_t i2cAddress) : LIS331Driver()
    {
        _I2C_Address = i2cAddress;
        _I2C_Driver = nullptr;
    }
    
    void LIS331_I2C_Driver::begin(TwoWire * i2cDriver)
    {
        _I2C_Driver = i2cDriver;
    }
    
    //=======================================================================
    //readRegister()
    //Reads the specified LIS331 internal register and returns the result
    //Parameters:
    //reg: The register that is to be read.
    //Return: The register value
    //=======================================================================
    uint8_t LIS331_I2C_Driver::readRegister(uint8_t reg) const
    {
        //Configure the LIS331 internal register pointer
        _I2C_Driver->beginTransmission(_I2C_Address);
        _I2C_Driver->write(reg);
        _I2C_Driver->endTransmission();
        
        //Read the LIS331 internal register
        _I2C_Driver->requestFrom(_I2C_Address, 1);
        return _I2C_Driver->read();
        
    }
    
    //=======================================================================
    //readAccelRegisterPair()
    //Reads two contiguous LIS331 internal registers corresponding to one
    //of the accelerometer axis.
    //Parameters:
    //reg: The low byte register of the accelerometer axis that is being read
    //Return: The value of the two registers concatenated into a 16 bit value
    //=======================================================================
    int16_t LIS331_I2C_Driver::readAccelRegisterPair(uint8_t reg) const
    {
        //Configure the LIS331 internal register pointer
        _I2C_Driver->beginTransmission(_I2C_Address);
        _I2C_Driver->write(reg | I2C_MULT_REG_READ);
        _I2C_Driver->endTransmission();
        
        int16_t accelValue = 0;
        int16_t temp = 0;
        
        //Read the LIS331 internal registers and format and return the result
        _I2C_Driver->requestFrom(_I2C_Address, 2);
        temp = _I2C_Driver->read();
        accelValue = _I2C_Driver->read();
        accelValue <<= 8;
        accelValue |= temp;
        return accelValue;
    }
    
    //=======================================================================
    //readAccelRegisters()
    //Reads the  six LIS331 internal registers containing the acceleration
    //data and returns the result in the caller provided array.
    //Parameters:
    //accelArray: The array where the acceleration data will be stored
    //Return: None
    //=======================================================================
    void LIS331_I2C_Driver::readAccelRegisters(int16_t accelArray[]) const
    {
        //Configure the LIS331 internal register pointer
        _I2C_Driver->beginTransmission(_I2C_Address);
        _I2C_Driver->write(OUT_X_L_REG | I2C_MULT_REG_READ);
        _I2C_Driver->endTransmission();
        
        uint8_t temp = 0;
        
        //Read the six internal acceleration registers
        _I2C_Driver->requestFrom(_I2C_Address, 6);
        for (uint8_t index = 0; index < 3; index++) {
            temp = _I2C_Driver->read();
            accelArray[index] = _I2C_Driver->read();
            accelArray[index] <<= 8;
            accelArray[index] |= temp;
        }
        
    }
    
    //=======================================================================
    //writeRegister()
    //Writes a new value to the specified LIS331 internal register
    //Parameters:
    //reg: The internal register that is to be written
    //value: The value to be written
    //Return: None
    //=======================================================================
    void LIS331_I2C_Driver::writeRegister(uint8_t reg, uint8_t value) const
    {
        //Configure the LIS331 internal register pointer and write the data
        _I2C_Driver->beginTransmission(_I2C_Address);
        _I2C_Driver->write(reg);
        _I2C_Driver->write(value);
        _I2C_Driver->endTransmission();
    }
    
    //=========================================================================================
    //  LIS331_SPI_Driver Class Method Definitions
    //=========================================================================================
    
    //Constructor
    LIS331_SPI_Driver::LIS331_SPI_Driver(uint8_t csPin)
    {
        _CS_Pin = csPin;
    }
    
    void LIS331_SPI_Driver::begin(SPIClass * spiDriver)
    {
        mSPI_Driver = spiDriver;
    }
    
    //=======================================================================
    //readRegister()
    //Reads the specified LIS331 8-bit internal register and returns the result
    //Parameters:
    //reg: The register address that is to be read.
    //Return: The register value
    //=======================================================================
    uint8_t LIS331_SPI_Driver::readRegister(uint8_t reg) const
    {
        //Compose the address byte, set the read bit
        uint8_t addressByte = reg | SPI_READ;
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(_CS_Pin, LOW);
        //Send the adddress byte. Ignore the returned value
        mSPI_Driver->transfer(addressByte);
        //Retrieve the register data. The data sent is ignored by the LIS331
        uint8_t regValue = mSPI_Driver->transfer(0x00);
        //Deassert the chip select
        digitalWrite(_CS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
        
        return regValue;
        
    }
    
    //=======================================================================
    //readAccelRegisterPair()
    //Reads the specified LIS331 8-bit internal register pair and returns
    //the result the result as a 16-bit value
    //Parameters:
    //reg: The register address that is to be read.
    //Return: The register value
    //=======================================================================
    int16_t LIS331_SPI_Driver::readAccelRegisterPair(uint8_t reg) const
    {
        //Compose the address byte, set the read bit and set the address increment bit
        uint8_t addressByte = reg | SPI_READ | SPI_MULT_REG_ACCESS;
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(_CS_Pin, LOW);
        //Send the address byte, ignore the returned value
        mSPI_Driver->transfer(addressByte);
        
        int16_t accelValue = 0;
        int16_t temp = 0;
        
        //Retrieve the register data. The data sent is ignored by the LIS331
        temp = mSPI_Driver->transfer(0x00);
        accelValue = mSPI_Driver->transfer(0x00);
        //Deassert the chip select
        digitalWrite(_CS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
        
        //Combine the two register values into a single 16-bit value and return it
        accelValue <<= 8;
        accelValue |= temp;
        return accelValue;
        
    }
    
    //=======================================================================
    //readAccelRegisters()
    //Reads the  six LIS331 internal registers containing the acceleration
    //data and returns the result in the caller provided array.
    //Parameters:
    //accelArray: The array where the acceleration data will be stored
    //Return: None
    //=======================================================================
    void LIS331_SPI_Driver::readAccelRegisters(int16_t accelArray[]) const
    {
        //Compose the address byte, set the read bit
        uint8_t addressByte = OUT_X_L_REG | SPI_READ;
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(_CS_Pin, LOW);
        //Send the address byte, ignore the returned value
        mSPI_Driver->transfer(addressByte);
        
        int16_t accelValue = 0;
        int16_t temp = 0;
        
        //Retrieve the 6 bytes of acceleration data
        for (int index = 0; index < 3; index++) {
            temp = mSPI_Driver->transfer(0x00);
            accelValue = mSPI_Driver->transfer(0x00);
            accelValue <<= 8;
            accelValue |= temp;
            accelArray[index] = accelValue;
        }
        
        //Deassert the chip select
        digitalWrite(_CS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
    }
    
    //=======================================================================
    //writeRegister()
    //Writes a new value to the specified LIS331 internal register
    //Parameters:
    //reg: The internal register that is to be written
    //value: The value to be written
    //Return: None
    //=======================================================================
    void LIS331_SPI_Driver::writeRegister(uint8_t reg, uint8_t value) const
    {
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(_CS_Pin, LOW);
        //Send the address byte
        mSPI_Driver->transfer(reg);
        //Send the data byte
        mSPI_Driver->transfer(value);
        //Deassert the chip select
        digitalWrite(_CS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
        
    }
    
} //End namespace LIS331
