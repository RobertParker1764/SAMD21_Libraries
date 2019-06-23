//L3GD20H MEMS  3-Axis Gyro Driver
//Version 1.0
//April 10, 2019
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

#include "L3GD20H_Driver.hpp"
#include "Wire.h"
#include "SPI.h"

namespace L3GD20H
{
    //=========================================================================================
    //  Base Class Method Definitions
    //=========================================================================================
    
    //=========================================================================================
    //Methods for basic initialization/operation
    //=========================================================================================

    //Constructor
    L3GD20HDriver::L3GD20HDriver()
    {
        for (uint8_t index = 0; index < 3; index++) {
            mCalibration[index] = 0.0;
        }
        
    }
    
    
    //=========================================================================================
    //run()
    //Initializes the L3GD20H and starts measurements. You should configure the FIFO and
    //interrupts before calling run(). The device will be placed in Normal Power Mode unless
    //NO_AXIS is specified for the axis parameter in which case the device will be placed in
    //sleep mode. Block data update will be disabled. The user should call setBDU() before
    //calling run() if block data update is desired. The device will be set to operate in little
    //endian mode. The user should calll setBigEndian() before calling run() if big endian data
    //alignment is needed. The device requires 100ms turn-on delay before measurements will
    //be available.
    //Parameters:
    //  range: The full scale measurement range setting (_245DPS, _500DPS, or _2000DPS)
    //  odr:   The output data rate setting (_12_5HZ, _25HZ, _50HZ, _100HZ, _200HZ, 400HZ, or
    //         _800HZ)
    //  filterMode: The configuration setting for the low pass and high pass filters
    //              (LPF1_ONLY -default, LPF1_AND_LPF2, LPF1_HPF, LPF1_AND_HPF_AND_LPF2)
    //  bw: The bandwidth setting for low pass filter 2 (LOW_BW - default, MEDIUM_LOW_BW,
    //      MEDIUM_HIGH_BW, HIGH_BW)
    //  hpMode: The high pass filter mode setting (NORMAL_MODE_RESET, REFERENCE_SIGNAL,
    //          NORMAL_MODE - default, AUTO_RESET)
    //  cutOff: The high pass filter cutoff frequency setting (HPCF_LVL_0 - default ... HPCF_LVL_9)
    //  referenceSetting:   The high pass filter reference value setting in milli g's per second
    //                      (default is 0). This value is capped at +/- 250, +/-500, or +/- 2000
    //                      milli g's depending on the full scale range setting (range).
    //  axis:   The axis that will be enabled for measurements (NO_AXIS, X, Y, Z, XY, XZ, YZ,
    //          and XYZ - default)
    //Return: None
    //=========================================================================================

    void L3GD20HDriver::run(fullScale_t range,
                            outputDataRate_t odr,
                            filterMode_t filterMode,
                            LPF2_bandwidth_t bw,
                            HP_FiterMode_t hpMode,
                            highPassCutOff_t cutOff,
                            int16_t referenceSetting,
                            axis_t axis)
    {
        setRange(range);
        
        mODR = odr;
        mAxis = axis;
        
        //Control regiser 2 configuration
        if (filterMode == LPF1_HPF || filterMode == LPF1_HPF_LPF2) {
            writeRegister(CTRL_REG2, (hpMode | cutOff));
            
            setHighPassFilterReference(referenceSetting);
        }
        
        //Control register 4 configuration
        uint8_t value = readRegister(CTRL_REG4) & ~FULL_SCALE_MASK;
        writeRegister(CTRL_REG4, (value | range));
        
        //Control register 5 configuration
        value = readRegister(CTRL_REG5) & ~OUTPUT_SOURCE_SEL_MASK;
        value |= filterMode;
        if (filterMode == LPF1_HPF || filterMode == LPF1_HPF_LPF2) {
            //Set the high pass filter enable bit
            value |= HIGHPASS_FILTER_ENABLE;
        } else {
            //Clear the high pass filter enable bit
            value &= ~HIGHPASS_FILTER_ENABLE;
        }
        writeRegister(CTRL_REG5, value);
        
        //LOW ODR register configuration
        if (odr == _12_5HZ || odr == _25HZ || odr == _50HZ) {
            //Set Low_ODR bit
            value = readRegister(LOW_ODR) | LOW_ODR_MODE;
        } else {
            //Clear Low_ODR bit
            value = readRegister(LOW_ODR) & ~LOW_ODR_MODE;
        }
        writeRegister(LOW_ODR, value);
        
        //Control register 1 configuration
        value = 0;
        value |= bw;
        switch (odr) {
            case _12_5HZ:
            case _100HZ:
                value |= ODR_100HZ;
                break;
                
            case _25HZ:
            case _200HZ:
                value |= ODR_200HZ;
                break;
                
            case _50HZ:
            case _400HZ:
                value |= ODR_400HZ;
                break;
                
            case _800HZ:
                value |= ODR_800HZ;
                break;
                
        }
        
        value |= (axis | POWER_MODE_BIT);
        writeRegister(CTRL_REG1, value);
        
    }
    
    //=========================================================================================
    //run()
    //Puts the L3GD20H into NORMAL or Low_ODR mode after having been put into SLEEP mode.
    //Calling this method when the device is not in SLEEP mode will have no efffect. Note that
    //there is a turn-on delay of 1/ODR if low pass filter 2 is disabled and 6/ODR if low pass
    //filter 2 is enabled before measurement data will be available.
    //Parameters: None
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::run()
    {
        uint8_t value = readRegister(CTRL_REG1);
        if (!(value & AXIS_ENABLE_MASK) & (value & POWER_MODE_BIT)) {
            //In sleep mode
            writeRegister(CTRL_REG1, value & mAxis);
        }
    }
    
    
    
    //=========================================================================================
    //sleep()
    //Puts the L3GD20H into sleep mode
    //Parameters: None
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::sleep()
    {
        uint8_t value = readRegister(CTRL_REG1) & ~AXIS_ENABLE_MASK;
        writeRegister(CTRL_REG1, value);
    }
    
    //=========================================================================================
    //setRange()
    //Sets the full scale measurement range. Although not specified, user should allow for a
    //couple of measurement cycles for the output to stabilize at the new measurement range.
    //Parameters:
    //  range:  The desired full scale measurment range
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setRange(fullScale_t range)
    {
        switch (range) {
            case _245DPS:
                mRange = 0;
                break;
                
            case _500DPS:
                mRange = 1;
                break;
                
            case _2000DPS:
                mRange = 2;
                break;
        }
        
        uint8_t value = readRegister(CTRL_REG4) & ~FULL_SCALE_MASK;
        writeRegister(CTRL_REG4, (value | range));
    }
    
    //=========================================================================================
    //setDataRate()
    //Sets the measurement data rate to the specified value.
    //Parameters:
    //  odr:  The desired output data rate
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setDataRate(outputDataRate_t odr)
    {
        if (mODR == odr) {
            //Already set to correct ODR so do nothing
            return;
        }
        
        mODR = odr;
        uint8_t value = readRegister(CTRL_REG1) & ~DATA_RATE_MASK;
        writeRegister(CTRL_REG1, (value | odr));
        
    }
    
    //=========================================================================================
    //setAxis()
    //Sets the enabled measurement axis to the specified value. If the value NO_AXIS is
    //specified then the no change will take effect. Use sleep() to place the device into
    //SLEEP mode.
    //Parameters:
    //  axis:  The desired axis to be enabled for measurement
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setMeasurementAxis(axis_t axis)
    {
        if ((mAxis == axis) || (axis == NO_AXIS)) {
            return;
        }
        
        mAxis = axis;
        uint8_t value = readRegister(CTRL_REG1) & ~AXIS_ENABLE_MASK;
        writeRegister(CTRL_REG1, (value | axis));
    }
    
    //=========================================================================================
    //Methods to retrieve measured data
    //=========================================================================================

    //=========================================================================================
    //isDataReady()
    //Returns true if the data ready bit associated with the requested axis is set. If no
    //axis parameter is provided then this method returns the state of the ZYXDA bit in the
    //Status register. The data ready state of an individual axis may be requested by providing
    //the associated axis in the axis parameter. If the state of two or more axis are requested
    //then this method only returns true if new data is ready for all axis requested.
    //Parameters:
    //  axis: The axis that the ready state is being queried for (XYZ is the default)
    //Return: True if new data is ready, false otherwise
    //=========================================================================================
    bool L3GD20HDriver::isDataReady(axis_t axis)
    {
        uint8_t value = readRegister(STATUS_REG);
        //Default case (and most common case)
        if (axis == XYZ) {
            return (value & XYZ_DATA_READY);
        }
        
        //Go on to check less common cases
        switch (axis) {
            case X:
                return (value & X_DATA_READY);
                break;
                
            case Y:
                return (value & Y_DATA_READY);
                break;
                
            case Z:
                return (value & Z_DATA_READY);
                break;
                
            case XY:
                return ((value & X_DATA_READY) && (value & Y_DATA_READY));
                break;
                
            case XZ:
                return ((value & X_DATA_READY) && (value & Z_DATA_READY));
                break;
                
            case YZ:
                return ((value & Y_DATA_READY) && (value & X_DATA_READY));
                break;
                
        }
    }
    
    //=========================================================================================
    //isDataOverrun()
    //Returns true if the data overrun bit associated with the requested axis is set. If no
    //axis parameter is provided then this method returns the state of the ZYXOR bit in the
    //Status register. The data overrun state of an individual axis may be requested by providing
    //the associated axis in the axis parameter. If the state of two or more axis are requested
    //then this method returns true if either of the requested axis indicates an overrun state.
    //Parameters:
    //  axis: The axis that the data overrun state is being queried for (XYZ is the default)
    //Return: True if data has overrun, false otherwise
    //=========================================================================================
    bool L3GD20HDriver::isDataOverrun(axis_t axis)
    {
        uint8_t value = readRegister(STATUS_REG);
        //Default case (and most common case)
        if (axis == XYZ) {
            return (value & XYZ_DATA_OVERRUN);
        }
        
        //Go on and check less common cases
        switch (axis) {
            case X:
                return (value & X_DATA_OVERRUN);
                break;
                
            case Y:
                return (value & Y_DATA_OVERRUN);
                break;
                
            case Z:
                return (value & Z_DATA_OVERRUN);
                break;
                
            case XY:
                return (value & (X_DATA_OVERRUN | Y_DATA_OVERRUN));
                break;
                
            case XZ:
                return (value & (X_DATA_OVERRUN | Z_DATA_OVERRUN));
                break;
                
            case YZ:
                return (value & (Y_DATA_OVERRUN | Z_DATA_OVERRUN));
                break;
                
        }
    }
    
    //=========================================================================================
    //calibrate()
    //Computes and saves the calibration value for the specified axis. Calibration values are
    //computed by reading the specified number of samples and taking the average value. The
    //calibration() method should be called after setting the various gyro parameters (range,
    //ODR, B/W, etc) and enabling the gyro. If any gyro parameters are changed then the
    //calibration() method should be rerun. The calibration() method should be called during a
    //time when the device is known to be in a static state.
    //Parameters:
    //  samples: The number of samples to be used in the calibration (10 is default)
    //  axis: The axis to be calibrated (X, Y, and Z is default)
    //Return: The X-axis angular rate in degrees per second
    //=========================================================================================
    void L3GD20HDriver::calibrate(uint8_t samples, axis_t axis)
    {
        float rateReading = 0.0;
        uint8_t count = 0;
        if ((axis == XYZ) || (axis == XY) || (axis == XZ) || (axis == X)) {
            //Calibrate the X axis
            for (int8_t index = samples; index > 0 ; index--) {
                rateReading += readXAxisRate();
                count++;
            }
            mCalibration[0] = rateReading / (float)count;
        }
        
        if ((axis == XYZ) || (axis == XY) || (axis == YZ) || (axis == Y)) {
            //Calibrate the Y axis
            rateReading = 0.0;
            count = 0;
            for (int8_t index = samples; index > 0 ; index--) {
                rateReading += readYAxisRate();
                count++;
            }
            mCalibration[1] = rateReading / (float)count;
        }
        
        if ((axis == XYZ) || (axis == XZ) || (axis == YZ) || (axis == Z)) {
            //Calibrate the Z axis
            rateReading = 0.0;
            count = 0;
            for (int8_t index = samples; index > 0 ; index--) {
                rateReading += readZAxisRate();
                count++;
            }
            mCalibration[2] = rateReading / (float)count;
        }


    }
    
    //=========================================================================================
    //readXAxisRate()
    //Returns the X-axis angular rate in degrees per second
    //Parameters: None
    //Return: The X-axis angular rate in degrees per second
    //=========================================================================================
    float L3GD20HDriver::readXAxisRate()
    {
        //Read the current angular rate data
        int16_t registerValue = readDataRegisterPair(OUT_X_L);
        return ((float)registerValue * angularRateScaleFactor[mRange]) - mCalibration[0];
    }
    
    //=========================================================================================
    //readYAxisRate()
    //Returns the Y-axis angular rate in degrees per second
    //Parameters: None
    //Return: The Y-axis angular rate in degrees per second
    //=========================================================================================
    float L3GD20HDriver::readYAxisRate()
    {
        //Read the current angular rate data
        int16_t registerValue = readDataRegisterPair(OUT_Y_L);
        return ((float)registerValue * angularRateScaleFactor[mRange]) - mCalibration[1];
        
    }
    
    //=========================================================================================
    //readZAxisRate()
    //Returns the Z-axis angular rate in degrees per second
    //Parameters: None
    //Return: The Z-axis angular rate in degrees per second
    //=========================================================================================
    float L3GD20HDriver::readZAxisRate()
    {
        //Read the current angular rate data
        int16_t registerValue = readDataRegisterPair(OUT_Z_L);
        return ((float)registerValue * angularRateScaleFactor[mRange]) - mCalibration[2];
        
    }

    //=========================================================================================
    //readAngularRates()
    //Returns the measured angular rate for all three axis in degrees per second
    //Parameters:
    //  angularRates: The array where the measured data is placed
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::readAngularRates(float angularRates[])
    {
        int16_t rawRates[3];
        read3AxisDataRegisters(rawRates);
        
        for (uint8_t index = 0; index < 3; index++) {
            angularRates[index] = ((float)rawRates[index] * angularRateScaleFactor[mRange]) - mCalibration[index];
        }
        
    }
    //=========================================================================================
    //readFIFOData()
    //Retrieves the stored FIFO data and loads it into the caller provided array. Data is
    //passed to the user array in degrees per second
    //Parameters:
    //  rateData: A two dimensional array provided for the passing of the angular rate data
    //  numDataSamples: The number of data sample sets (3 axis measurements per set) to be
    //                  read.
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::readFIFOData(float rateData[][3], uint8_t numDataSamples)
    {
        //First retrieve the raw data from the FIFO
        int16_t buffer[96];
        readFIFO(buffer, numDataSamples);
        
        //Convert data to degrees per second and load into user provided array
        uint8_t bufferIndex;
        uint8_t rateDataIndex = 0;
        uint8_t axisIndex = 0;
        for (bufferIndex = 0; bufferIndex <= numDataSamples * 3; bufferIndex++) {
            rateData[rateDataIndex][axisIndex] = (float)buffer[bufferIndex] * angularRateScaleFactor[mRange];
            if (axisIndex == 2) {
                axisIndex = 0;
                rateDataIndex++;
            } else {
                axisIndex++;
            }
        }
        
    }
    
    //=========================================================================================
    //readTempChange()
    //Returns the temperature change in degrees C since the L3GD20H was powered on.
    //An increase of temperature is reported as a positive number
    //Parameters: None
    //Return: The temperature change since power on in degrees C
    //=========================================================================================
    int8_t L3GD20HDriver::readTempChange()
    {
        return ((mPowerOnTemp - readRegister(OUT_TEMP_REG)) * TEMP_SCALE_FACTOR);
    }
    
    //=========================================================================================
    //Methods for managing the FIFO
    //=========================================================================================

    //=========================================================================================
    //enableFIFO()
    //Sets the FIFO_EN bit and optionally the StopOnFTH bit in control register 5
    //Parameters:
    //  stopOnThreshold:   If true then the StopOnFTH bit is set, cleared otherwise
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::enableFIFO(bool stopOnThreshold)
    {
        uint8_t value = readRegister(CTRL_REG5);
        value |= FIFO_ENABLE;
        if (stopOnThreshold) {
            value |= STOP_ON_THRESHOLD;
        } else {
            value &= ~STOP_ON_THRESHOLD;
        }
        writeRegister(CTRL_REG5, value);
    }
    
    //=========================================================================================
    //disableFIFO()
    //Clears the FIFO_EN bit in control register 5 and clears the FIFO contents
    //Parameters:
    //  stopOnThreshold:   If true then the StopOnFTH bit is set, cleared otherwise
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::disableFIFO()
    {
        resetFIFO();
        uint8_t value = readRegister(CTRL_REG5) & ~FIFO_ENABLE;
        writeRegister(CTRL_REG5, value);
    }
    
    //=========================================================================================
    //setFIFO()
    //Sets the FIFO to the specified mode
    //Parameters:
    //  mode:   The desired FIFO mode (BYPASS, STREAM, etc.)
    //  threshold:  The FIFO threshold setting
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setFIFO(FIFO_Mode_t  mode, uint8_t threshold)
    {
        //First reset the FIFO
        resetFIFO();
        //Set the new FIFO mode
        writeRegister(FIFO_CTRL, (mode | threshold));
    }
    
    //=========================================================================================
    //resetFIFO()
    //Resets and clears the FIFO content by placing the FIFO in BYPASS mode
    //Parameters: None
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::resetFIFO()
    {
        uint8_t value = (readRegister(FIFO_CTRL) & ~FIFO_MODE_MASK);
        writeRegister(FIFO_CTRL, (value | BYPASS));
    }
    
    //=========================================================================================
    //isFIFO_Empty()
    //Returns true if the FIFO empty flag (EMPTY) is set. Returns false otherwise.
    //Parameters: None
    //Return: True if the EMPTY bit of the FIFO_SRC register is set
    //=========================================================================================
    bool L3GD20HDriver::isFIFO_Empty()
    {
        return (bool)(readRegister(FIFO_SRC) & FIFO_EMPTY);
    }
    
    //=========================================================================================
    //isFIFO_AtThreshold()
    //Returns true if the FIFO threshold flag (FTH) is set. Returns false otherwise.
    //Parameters: None
    //Return: True if the FTH bit of the FIFO_SRC register is set
    //=========================================================================================
    bool L3GD20HDriver::isFIFO_AtThreshold()
    {
        return (bool)(readRegister(FIFO_SRC) & FIFO_AT_THRESHOLD);
    }
    
    //=========================================================================================
    //isFIFO_Overrun()
    //Returns true if the FIFO overrun flag (OVRN) is set. Returns false otherwise.
    //Parameters: None
    //Return: True if the OVRN bit of the FIFO_SRC register is set
    //=========================================================================================
    bool L3GD20HDriver::isFIFO_Overrun()
    {
        return (bool)(readRegister(FIFO_SRC) & FIFO_OVERRUN);
    }
    
    //=========================================================================================
    //getFIFODataLevel()
    //Returns the number of data samples currently stored in the FIFO
    //Parameters: None
    //Return: The number of FIFO data samples currently available for reading
    //=========================================================================================
    uint8_t L3GD20HDriver::getFIFODataLevel()
    {
        return (readRegister(FIFO_SRC) & FIFO_DATA_LEVEL_MASK);
    }
    
    //=========================================================================================
    //Methods for managing Interrupts
    //=========================================================================================
    
    //=========================================================================================
    //configureInterrupts()
    //Sets the drive type (push-pull or open drain) for interrupt pins 1 and 2, and the drive
    //level (active high or active low) for interrupt pin 1 depending on the state of the
    //activeHigh and pushPull parameters.
    //Parameters:
    //  activeHigh: If true the Pin 1 interrupt signal will be active high
    //  pushPull:   If true the Pin 1 and 2 interrupt signal interface will be configured as
    //              push-pull. Otherwise they are configured as open drain.
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::configureInterrupts(bool activeHigh, bool pushPull)
    {
        uint8_t ctrlReg3 = readRegister(CTRL_REG3);
        
        if (activeHigh) {
            //Clear the H_Lactive bit in control register 3
            ctrlReg3 &= ~PIN1_ACTIVE_LEVEL;
        } else {
            //set the H_Lactive bit in control register 3
            ctrlReg3 |= PIN1_ACTIVE_LEVEL;
        }
        
        if (pushPull) {
            //Clear the PP_OD bit in control register 3
            ctrlReg3 &= ~PIN1_DRIVE_TYPE;
        } else {
            //Set the PP_OD bit in control register 3
            ctrlReg3 |= PIN1_DRIVE_TYPE;
        }
        
        writeRegister(CTRL_REG3, ctrlReg3);
    }
    
    
    //=========================================================================================
    //enableDataReadyInterrupt()
    //Enables or disables the data ready interrupt depending on the state of the enable parameter.
    //If enabled, the INT2_DRDY bit in control register 3 is set. It is cleared otherwise.
    //Interrupt pin 2 is shared with the FIFO interrupt signals therefore the user must read
    //the STATUS and FIFO_SRC registers to determine if the interrupt is FIFO related or a data
    //ready interrupt.
    //Parameters:
    //  enable: If true the data ready interrupt is enabled. If false the interrupt is disabled
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::enableDataReadyInterrupt(bool enable)
    {
        uint8_t ctrlReg3 = readRegister(CTRL_REG3);
        if (enable) {
            //Set the INT2_DRDY bit
            writeRegister(CTRL_REG3, ctrlReg3 | DATA_READY_ON_PIN2);
        } else {
            //Clear the INT2_DRDY bit
            writeRegister(CTRL_REG3, ctrlReg3 & ~DATA_READY_ON_PIN2);
        }
    }
    
    //=========================================================================================
    //enableFIFOInterrupts()
    //Enables or disables the FIFO threshold, overrun, and empty interrupts depending upon the
    //state of the threshold, overrun, and empty parameters. If enabled, the INT2_FTH,
    //INT2_ORun, and/or INT2_Empty bits in control register 3 are set. They are cleared
    //otherwise. Interrupt pin 2 is shared with the data ready interrupt signal therefore the
    //user must read the STATUS and FIFO_SRC registers to determine if the interrupt is FIFO
    //related or a data ready interrupt.
    //Parameters:
    //  threshold:  If true the FIFO threshold interrupt is enabled. It is disabled otherwise
    //  overrun:    If true the FIFO overrun interrupt is enabled. It is disabled otherwise
    //  empty:      If true the FIFO empty interrupt is enabled. It is disabled otherwise
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::enableFIFOInterrupts(bool threshold, bool overrun, bool empty)
    {
        uint8_t ctrlReg3 = readRegister(CTRL_REG3);
        if (threshold) {
            //Set the INT2_FTH bit
            ctrlReg3 |= FIFO_THRESHOLD_INTR;
        } else {
            //Clear the INT2_FTH bit
            ctrlReg3 &= ~FIFO_THRESHOLD_INTR;
        }
        
        if (overrun) {
            //Set the INT2_ORun bit
            ctrlReg3 |= FIFO_OVERRUN_INTR;
        } else {
            //Clear the INT2_ORun bit
            ctrlReg3 &= ~FIFO_OVERRUN_INTR;
        }
        
        if (empty) {
            //Set the INT2_Empty bit
            ctrlReg3 |= FIFO_EMPTY_INTR;
        } else {
            //Clear the INT2_Empty bit
            ctrlReg3 &= ~FIFO_EMPTY_INTR;
        }
        
        writeRegister(CTRL_REG3, ctrlReg3);

    }
    
    //=========================================================================================
    //enableIGInterrupts()
    //Enables or disables the IG interrupts specified in the enableAxis parameter. If a
    //corresponding bit is set then the interrupt is enabled. It is disabled otherwise.
    //The IG interrupt will be either a logic AND or logic OR of the enabled interrupts depending
    //on the logicAND parameter. The IG interrupt will be latched until cleared by reading the
    //status register if the latch parameter is set to true. Otherwise the IG interrupt is
    //reset as soon as the interrupt condition is no longer in effect (rate signal below
    //specified threshold).
    //Parameters:
    //  latch:      If true the IG interrupt is latched. It is cleared othewise
    //  logicAND:   If true the IG interrupt is a logic AND of the enabled interrupts.
    //              It is a logic OR of the enabled interrupts if the parameter is false
    //  enableAxis: A 8-bit value with the appropriate bits set for the desired interrupt axis.
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::enableIGInterrupts(bool latch, bool logicAND, uint8_t enableAxis)
    {
        uint8_t registerValue = enableAxis;
        if (latch) {
            //Set the LIR bit
            registerValue |= LATCH_INTERRUPT_REQUEST;
        }
        
        if (logicAND) {
            //Set the AND/OR bit
            registerValue |= AND_OR_SEL;
        }
        
        writeRegister(IG_CFG, registerValue);
        
        //Set the INT1_IG bit in control register 3 (IG interrupt enable)
        registerValue = readRegister(CTRL_REG3);
        registerValue |= INTERNAL_INTERUPT_ON_PIN1;
        writeRegister(CTRL_REG3, registerValue);
    
    }
    
    //=========================================================================================
    //setInterruptIGThreshold
    //Sets the IG interrupt magnitude threshold above which interrupts will be signaled if they
    //are enabled. The maximum allowable value for the threshold setting is limited to the
    //current full scale range setting. For example, if the full scale range is set to 500 dps
    //then the maximum allowable threshold setting is 500 dps. If values larger than this are
    //passed to this method then the value will be set to the maximum allowable.
    //Parameters:
    //  xAxis:  The x-axis threshold setting in degrees per second
    //  yAxis:  The y-axis threshold setting in degrees per second
    //  zAxis:  The z-axis threshold setting in degrees per second
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setInterruptIGThreshold(uint16_t xAxis, uint16_t yAxis, uint16_t zAxis)
    {
        //Check for out of bounds parameters
        if ((float)(xAxis / 32.767) > intrThresholdScaleFactor[mRange]) {
            xAxis = (uint16_t)(intrThresholdScaleFactor[mRange] * 32767 / 1000);
        }
        
        if ((float)(yAxis / 32.767) > intrThresholdScaleFactor[mRange]) {
            yAxis = (uint16_t)(intrThresholdScaleFactor[mRange] * 32767 / 1000);
        }
        
        if ((float)(zAxis / 32.767) > intrThresholdScaleFactor[mRange]) {
            zAxis = (uint16_t)(intrThresholdScaleFactor[mRange] * 32767 / 1000);
        }
        
        uint16_t value;
        
        //Set x-Axis threshold
        value = (xAxis * 1000) / intrThresholdScaleFactor[mRange];
        writeRegister(IG_THS_XL, (uint8_t)value);
        value >>= 8;
        value &= 0x7F;
        writeRegister(IG_THS_XH, (uint8_t)value);
        
        //Set y-Axis threshold
        value = (yAxis * 1000) / intrThresholdScaleFactor[mRange];
        writeRegister(IG_THS_YL, (uint8_t)value);
        value >>= 8;
        value &= 0x7F;
        writeRegister(IG_THS_YH, (uint8_t)value);
        
        //Set z-Axis threshold
        value = (zAxis * 1000) / intrThresholdScaleFactor[mRange];
        writeRegister(IG_THS_ZL, (uint8_t)value);
        value >>= 8;
        value &= 0x7F;
        writeRegister(IG_THS_ZH, (uint8_t)value);
    }
    
    //=========================================================================================
    //setInterruptIGDuration()
    //Sets the IG interrupt duration magnitude register (IG_DURATION). The maximum value of the
    //specified duration is dependent upon the current output data rate setting as follows:
    //  ODR (Hz)            Max
    //  12.5                10.16 seconds
    //  25                  5.08 seconds
    //  50                  2.54 seconds
    //  100                 1.27 seconds
    //  200                 0.635 seconds
    //  400                 0.3175 seconds
    //  800                 0.15875 seconds
    //If values greater than these are passed to this method then they will be capped at the
    //maximum value.
    //Parameters:
    //  duration: The desired duration value in seconds
    //  wait: If true then the wait condition will be enabled (see data sheet)
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setInterruptIGDuration(float duration, bool wait)
    {
        //Check for out of bounds duration
        if (((duration * 1000) / 127.0) > intrDurationScaleFactor[mODR]) {
            duration = (127.0 * intrDurationScaleFactor[mODR]) / 1000.0;
        }
        
        uint8_t value = (uint8_t)((duration * 1000) / intrDurationScaleFactor[mODR]);
        
        value &= 0x7F;
        if (wait) {
            value |= WAIT;
        }
        
        writeRegister(IG_DURATION, value);
    }

        
    //=========================================================================================
    //getInterruptSource()
    //Returns the value of the IG_SRC register. The user must use the IG_SRC register constants
    //to check which interrupts are active. For example use Z_AXIS_HIGH_INTR to check if the
    //angular rate exceeded the high threshold level on the Z axis. If a value of 0 is returned
    //then no interrupt is active.
    //Parameters: None
    //Return: The IG_SRC register contents
    //=========================================================================================
    uint8_t L3GD20HDriver::getInterruptSource()
    {
        return (readRegister(IG_SRC));
    }
    
    //=========================================================================================
    //Methods for managing the filters
    //=========================================================================================

    //=========================================================================================
    //setHighPassFilterReference()
    //Computes the Reference register value based on the provided refValue and full scale
    //range setting and then sets the Reference register to this value
    //Parameters:
    //  refValue:   The desired reference register setting in milli dps. This value will be
    //              capped at +/- 250, +/-500, or +/- 2000 milli dps depending on the full
    //              scale range setting.
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setHighPassFilterReference(int16_t refValue)
    {
        int8_t registerValue;
        
        //Check that the reference value is in range for the given full scale range given.
        //If it exceeds the allowable range than set it to the maximum allowable
        switch (mRange) {
            case 0:
                if (refValue < -250) {
                    refValue = -250;
                } else if (refValue > 250) {
                    refValue = 250;
                }
                registerValue = (int8_t)(refValue / (int16_t)referenceScaleFactor[0]);
                break;
                
            case 1:
                if (refValue < -500) {
                    refValue = -500;
                } else if (refValue > 500) {
                    refValue = 500;
                }
                registerValue = (int8_t)(refValue / (int16_t)referenceScaleFactor[1]);
                break;
                
            case 2:
                if (refValue < -2000) {
                    refValue = -2000;
                } else if (refValue > 2000) {
                    refValue = 2000;
                }
                registerValue = (int8_t)(refValue / (int16_t)referenceScaleFactor[2]);
                break;
                
        }
        writeRegister(REFERENCE_REG, registerValue);
        
    }
    
    //=========================================================================================
    //resetHPFilter()
    //Resets the High Pass filter by instantly deleting the DC component of the angular rate.
    //Parameters: None
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::resetHPFilter()
    {
        readRegister(REFERENCE_REG);
    }
    
    //=========================================================================================
    //Misc. Methods
    //=========================================================================================
    
    //=========================================================================================
    //setBDU()
    //Sets or clears the BDU bit in Control Register 4 depending on the value of the parameter
    //bdu.
    //Parameters:
    //  bdu:    If true then the BDU bit is set. It is cleared otherwise.
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setBDU(bool bdu)
    {
        uint8_t value = readRegister(CTRL_REG4);
        if (bdu) {
            if (value & BLOCK_DATA_UPDATE_ENABLE) {
                //Already set to return
                return;
            } else {
                //Not set so set it
                value |= BLOCK_DATA_UPDATE_ENABLE;
                writeRegister(CTRL_REG4, value);
            }
            
        } else {
            if (value & BLOCK_DATA_UPDATE_ENABLE) {
                //It's set so clear it
                value &= ~BLOCK_DATA_UPDATE_ENABLE;
            }
        }
        return;
    }
    
    //=========================================================================================
    //setBigEndian()
    //Sets or clears the BLE bit in Control Register 4 depending on the value of the parameter
    //bigEndian.
    //Parameters:
    //  bigEndian:    If true then the BLE bit is set. It is cleared otherwise.
    //Return: None
    //=========================================================================================
    void L3GD20HDriver::setBigEndian(bool bigEndian)
    {
        uint8_t value = readRegister(CTRL_REG4);
        if (bigEndian) {
            if (value & BIG_LITTLE_ENDIAN_SELECT) {
                //Already set to return
                return;
            } else {
                //Not set so set it
                value |= BIG_LITTLE_ENDIAN_SELECT;
                writeRegister(CTRL_REG4, value);
            }
            
        } else {
            if (value & BIG_LITTLE_ENDIAN_SELECT) {
                //It's set so clear it
                value &= ~BIG_LITTLE_ENDIAN_SELECT;
            }
        }
        return;
    }
    
    //=========================================================================================
    //isL3GD20H()
    //Checks if the device connected to is an L3GD20H.
    //Parameters: None
    //Return: Returns true if the device is an L3GD20H. Returns false otherwise
    //=========================================================================================
    bool L3GD20HDriver::isL3GD20H()
    {
        return (readRegister(WHO_AM_I_REG) == L3GD20H_ID);
    }
    
    
    //=========================================================================================
    //  L3GD20H_I2C_Driver Class Method Definitions
    //=========================================================================================
    
    //TODO: Write I2C driver class methods
    
    
    //=========================================================================================
    //  L3GD20H_SPI_Driver Class Method Definitions
    //=========================================================================================
    
    //Constructor
    L3GD20H_SPI_Driver::L3GD20H_SPI_Driver(uint8_t csPin)
    {
        mCS_Pin = csPin;
    }
    
    void L3GD20H_SPI_Driver::begin(SPIClass * spiDriver)
    {
        mSPI_Driver = spiDriver;
    }
    
    //=======================================================================
    //readRegister()
    //Reads the specified L3GD20H 8-bit internal register and returns the result
    //Parameters:
    //reg: The register address that is to be read.
    //Return: The register value
    //=======================================================================
    uint8_t L3GD20H_SPI_Driver::readRegister(uint8_t reg) const
    {
        //Compose the address byte, set the read bit
        uint8_t addressByte = reg | SPI_READ;
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(mCS_Pin, LOW);
        //Send the adddress byte. Ignore the returned value
        mSPI_Driver->transfer(addressByte);
        //Retrieve the register data. The data sent is ignored by the L3GD20H
        uint8_t regValue = mSPI_Driver->transfer(0x00);
        //Deassert the chip select
        digitalWrite(mCS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
        
        return regValue;
        
    }
    
    //=======================================================================
    //readDataRegisterPair()
    //Reads the specified L3GD20H 8-bit internal register pair and returns
    //the result as a 16-bit value
    //Parameters:
    //reg: The register address that is to be read.
    //Return: The register value
    //=======================================================================
    int16_t L3GD20H_SPI_Driver::readDataRegisterPair(uint8_t reg) const
    {
        //Compose the address byte, set the read bit and set the address increment bit
        uint8_t addressByte = reg | SPI_READ | SPI_MULT_REG_ACCESS;
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(mCS_Pin, LOW);
        //Send the address byte, ignore the returned value
        mSPI_Driver->transfer(addressByte);
        
        int16_t rateValue = 0;
        int16_t temp = 0;
        
        //Retrieve the register data. The data sent is ignored by the L3GD20H
        temp = mSPI_Driver->transfer(0x00);
        rateValue = mSPI_Driver->transfer(0x00);
        //Deassert the chip select
        digitalWrite(mCS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
        
        //Combine the two register values into a single 16-bit value and return it
        rateValue <<= 8;
        rateValue |= temp;
        return rateValue;
        
    }
    
    //=======================================================================
    //read3AxisDataRegisters()
    //Reads the  six L3GD20H internal registers containing the angular rate
    //data and returns the result in the caller provided array.
    //Parameters:
    //dataArray: The array where the angular rate data will be stored
    //Return: None
    //=======================================================================
    void L3GD20H_SPI_Driver::read3AxisDataRegisters(int16_t dataArray[]) const
    {
        //Compose the address byte, set the read bit
        uint8_t addressByte = OUT_X_L | SPI_READ | SPI_MULT_REG_ACCESS;
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(mCS_Pin, LOW);
        //Send the address byte, ignore the returned value
        mSPI_Driver->transfer(addressByte);
        
        int16_t dataValue = 0;
        int16_t temp = 0;
        
        //Retrieve the 6 bytes of acceleration data
        for (int index = 0; index < 3; index++) {
            temp = mSPI_Driver->transfer(0x00);
            dataValue = mSPI_Driver->transfer(0x00);
            dataValue <<= 8;
            dataValue |= temp;
            dataArray[index] = dataValue;
        }
        
        //Deassert the chip select
        digitalWrite(mCS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
    }
    
    void L3GD20H_SPI_Driver::readFIFO(int16_t dataArray[], uint8_t numDataSamples)
    {
        //Compose the address byte, set the read bit
        uint8_t addressByte = OUT_X_L | SPI_READ | SPI_MULT_REG_ACCESS;
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(mCS_Pin, LOW);
        //Send the address byte, ignore the returned value
        mSPI_Driver->transfer(addressByte);
        
        //Retrieve the FIFO data sets
        uint8_t counter = 0;
        int16_t dataValue = 0;
        int16_t temp = 0;
        
        for (int set = 0; set < numDataSamples; set++) {
            for (int index = 0; index < 3; index++) {
                temp = mSPI_Driver->transfer(0x00);
                dataValue = mSPI_Driver->transfer(0x00);
                dataValue <<= 8;
                dataValue |= temp;
                dataArray[(set * 3) + index] = dataValue;
            }
        }
        
        //Deassert the chip select
        digitalWrite(mCS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
    }
    
    //=======================================================================
    //writeRegister()
    //Writes a new value to the specified L3GD20H internal register
    //Parameters:
    //reg: The internal register that is to be written
    //value: The value to be written
    //Return: None
    //=======================================================================
    void L3GD20H_SPI_Driver::writeRegister(uint8_t reg, uint8_t value) const
    {
        //Initialize the SPI bus
        mSPI_Driver->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        //Assert the chip select
        digitalWrite(mCS_Pin, LOW);
        //Send the address byte
        mSPI_Driver->transfer(reg);
        //Send the data byte
        mSPI_Driver->transfer(value);
        //Deassert the chip select
        digitalWrite(mCS_Pin, HIGH);
        //Release the SPI bus
        mSPI_Driver->endTransaction();
        
    }


} //End namespace L3GD20H
