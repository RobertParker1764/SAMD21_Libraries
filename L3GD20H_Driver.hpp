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

#ifndef L3GD20H_Driver_hpp
#define L3GD20H_Driver_hpp

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

namespace L3GD20H
{
    /****************************** Data Type and Constant Defintions ******************************/
    
    //===============================   Who AM I Register (R only)   ================================//
    //===============================================================================================//
    const uint8_t WHO_AM_I_REG = 0x0F;      //Who Am I register address
    const uint8_t L3GD20H_ID = 0xD7;
    
    //===============================   Control Register 1 (R/W)   ==================================//
    //===============================================================================================//
    const uint8_t CTRL_REG1 = 0x20;         //Control Register 1 address
    
    //CTRL1 Register Structure Definition
    #define L3GD20H_CTRL1_REG_DR_Pos 6
    #define L3GD20H_CTRL1_REG_DR_Msk (0x3 << L3GD20H_CTRL1_REG_DR_Pos)
    #define L3GD20H_CTRL1_REG_DR(value) (L3GD20H_CTRL1_REG_DR_Msk & ((value) << L3GD20H_CTRL1_REG_DR_Pos))
    #define L3GD20H_CTRL1_REG_DR_100HZ  0x0
    #define L3GD20H_CTRL1_REG_DR_200HZ  0x1
    #define L3GD20H_CTRL1_REG_DR_400HZ  0x2
    #define L3GD20H_CTRL1_REG_DR_800HZ  0x3
    #define L3GD20H_CTRL1_REG_BW_Pos 4
    #define L3GD20H_CTRL1_REG_BW_Msk (0x3 << L3GD20H_CTRL1_REG_BW_Pos)
    #define L3GD20H_CTRL1_REG_BW(value) (L3GD20H_CTRL1_REG_BW_Msk & ((value) << L3GD20H_CTRL1_REG_BW_Pos))
    #define L3GD20H_CTRL1_REG_BW_LOW 0x0
    #define L3GD20H_CTRL1_REG_BW_MEDLOW 0x1
    #define L3GD20H_CTRL1_REG_BW_MEDHIGH 0x2
    #define L3GD20H_CTRL1_REG_BW_HIGH 0x3
    #define L3GD20H_CTRL1_REG_PD_Pos 3
    #define L3GD20H_CTRL1_REG_PD (0x1 << L3GD20H_CTRL1_REG_PD_Pos)
    #define L3GD20H_CTRL1_REG_ZEN_Pos 2
    #define L3GD20H_CTRL1_REG_ZEN (0x1 << L3GD20H_CTRL1_REG_ZEN_Pos)
    #define L3GD20H_CTRL1_REG_YEN_Pos 1
    #define L3GD20H_CTRL1_REG_YEN (0x1 << L3GD20H_CTRL1_REG_YEN_Pos)
    #define L3GD20H_CTRL1_REG_XEN_Pos 0
    #define L3GD20H_CTRL1_REG_XEN (0x1 << L3GD20H_CTRL1_REG_XEN_Pos)
    
    const uint8_t DATA_RATE_MASK = L3GD20H_CTRL1_REG_DR_Msk;    //Mask bits for the data rate (DR) bits
    //See table 21 in the L3GD20H data sheet to see how DR and BW settings affect the data rate
    //and cuttoff frequency
    enum normal_dataRate_t {
        ODR_100HZ = L3GD20H_CTRL1_REG_DR(L3GD20H_CTRL1_REG_DR_100HZ),
        ODR_200HZ = L3GD20H_CTRL1_REG_DR(L3GD20H_CTRL1_REG_DR_200HZ),
        ODR_400HZ = L3GD20H_CTRL1_REG_DR(L3GD20H_CTRL1_REG_DR_400HZ),
        ODR_800HZ = L3GD20H_CTRL1_REG_DR(L3GD20H_CTRL1_REG_DR_800HZ)
    };
    
    const uint8_t BANDWIDTH_MASK = L3GD20H_CTRL1_REG_BW_Msk;
    //See table 21 in the L3GD20H data sheet to see how DR and BW settings affect the data rate
    //and cuttoff frequency
    enum LPF2_bandwidth_t {
        LOW_BW = L3GD20H_CTRL1_REG_BW(L3GD20H_CTRL1_REG_BW_LOW),
        MEDIUM_LOW_BW = L3GD20H_CTRL1_REG_BW(L3GD20H_CTRL1_REG_BW_MEDLOW),
        MEDIUM_HIGH_BW = L3GD20H_CTRL1_REG_BW(L3GD20H_CTRL1_REG_BW_MEDHIGH),
        HIGH_BW = L3GD20H_CTRL1_REG_BW(L3GD20H_CTRL1_REG_BW_HIGH)
    };
    
    //Power mode bit
    const uint8_t POWER_MODE_BIT = L3GD20H_CTRL1_REG_PD;    //Power mode bit. Normal mode when set.
                                                            //Power down when cleared
    //Measurement axis enable bits
    const uint8_t AXIS_ENABLE_MASK = 0x07;  //Mask for the axis enable bits
    const uint8_t ZEN_BIT = L3GD20H_CTRL1_REG_ZEN;  //Z axis enable bit. When set measurement is enabled
    const uint8_t YEN_BIT = L3GD20H_CTRL1_REG_YEN;  //Y axis enable bit. When set measurement is enabled
    const uint8_t XEN_BIT = L3GD20H_CTRL1_REG_XEN;  //X axis enable bit. When set measurement is enabled
    enum axis_t {
        NO_AXIS = 0x00,             //All axis disabled
        X = 0x01,                   //Enable X-Axis only
        Y = 0x02,                   //Enable Y-Axis only
        XY = 0x03,                  //Enable X and Y-Axis
        Z = 0x04,                   //Enable Z-Axis only
        XZ = 0x05,                  //Enable X and Z-Axis
        YZ = 0x06,                  //Enable Y and Z-Axis
        XYZ = 0x07                  //Enable X, Y, and Z-Axis
    };
    
    //===============================   Control Register 2 (R/W)   ==================================//
    //===============================================================================================//
    const uint8_t CTRL_REG2 = 0x21;         //Control Register 2 address
    
    //CTRL2 Register Structure Definition
    #define L3GD20H_CTRL2_REG_EXTREN_Pos 7
    #define L3GD20H_CTRL2_REG_EXTREN (0x1 << L3GD20H_CTRL2_REG_EXTREN_Pos)
    #define L3GD20H_CTRL2_REG_LVLEN_Pos 6
    #define L3GD20H_CTRL2_REG_LVLEN (0x1 << L3GD20H_CTRL2_REG_LVLEN_Pos)
    #define L3GD20H_CTRL2_REG_HPM_Pos 4
    #define L3GD20H_CTRL2_REG_HPM_Msk (0x3 << L3GD20H_CTRL2_REG_HPM_Pos)
    #define L3GD20H_CTRL2_REG_HPM(value) (L3GD20H_CTRL2_REG_HPM_Msk & ((value) << L3GD20H_CTRL2_REG_HPM_Pos))
    #define L3GD20H_CTRL2_REG_HPM_NORMAL_RESET 0x00
    #define L3GD20H_CTRL2_REG_HPM_REF_SIGNAL 0x01
    #define L3GD20H_CTRL2_REG_HPM_NORMAL 0x02
    #define L3GD20H_CTRL2_REG_HPM_AUTO_RESET 0x03
    #define L3GD20H_CTRL2_REG_HPCF_Pos 0
    #define L3GD20H_CTRL2_REG_HPCF_Msk (0xF << L3GD20H_CTRL2_REG_HPCF_Pos)
    #define L3GD20H_CTRL2_REG_HPCF(value) (L3GD20H_CTRL2_REG_HPCF_Msk & ((value) << L3GD20H_CTRL2_REG_HPCF_Pos))
    #define L3GD20H_CTRL2_REG_HPCF_0 0x00
    #define L3GD20H_CTRL2_REG_HPCF_1 0x01
    #define L3GD20H_CTRL2_REG_HPCF_2 0x02
    #define L3GD20H_CTRL2_REG_HPCF_3 0x03
    #define L3GD20H_CTRL2_REG_HPCF_4 0x04
    #define L3GD20H_CTRL2_REG_HPCF_5 0x05
    #define L3GD20H_CTRL2_REG_HPCF_6 0x06
    #define L3GD20H_CTRL2_REG_HPCF_7 0x07
    #define L3GD20H_CTRL2_REG_HPCF_8 0x08
    #define L3GD20H_CTRL2_REG_HPCF_9 0x09
    
    const uint8_t EDGE_TRIGGER_ENABLE_BIT = L3GD20H_CTRL2_REG_EXTREN;   //When set external trigger is enabled
                                                                        //When cleared external trigger is disabled
    const uint8_t LEVEL_TRIGGER_ENABLE_BIT = L3GD20H_CTRL2_REG_LVLEN;   //When set level sensitive trigger is enabled
                                                                        //When cleared level sensitive trigger is disabled
    //High Pass Filter mode control. See table 25 in the data sheet
    const uint8_t HPM_FILTER_MODE_BIT_MASK = L3GD20H_CTRL2_REG_HPM_Msk;
    enum HP_FiterMode_t{
        NORMAL_MODE_RESET = L3GD20H_CTRL2_REG_HPM(L3GD20H_CTRL2_REG_HPM_NORMAL_RESET),  //Reset on read to
                                                                                        //REFERENCE reg
        REFERENCE_SIGNAL = L3GD20H_CTRL2_REG_HPM(L3GD20H_CTRL2_REG_HPM_REF_SIGNAL), //Reference signal for
                                                                                    //filtering
        NORMAL_MODE = L3GD20H_CTRL2_REG_HPM(L3GD20H_CTRL2_REG_HPM_NORMAL), //Normal mode
        AUTO_RESET = L3GD20H_CTRL2_REG_HPM(L3GD20H_CTRL2_REG_HPM_AUTO_RESET) //Autoreset on interrupt event
    };
    
    //High Pass Filter Cut-Off Frequency control. See table 26 in the data sheet for
    //cut off frequency vs HPCF setting and ODR setting
    const uint8_t HPCF_BIT_MASK = L3GD20H_CTRL2_REG_HPCF_Msk;
    enum highPassCutOff_t{
        HPCF_LVL_0 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_0),
        HPCF_LVL_1 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_1),
        HPCF_LVL_2 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_2),
        HPCF_LVL_3 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_3),
        HPCF_LVL_4 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_4),
        HPCF_LVL_5 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_5),
        HPCF_LVL_6 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_6),
        HPCF_LVL_7 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_7),
        HPCF_LVL_8 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_8),
        HPCF_LVL_9 = L3GD20H_CTRL2_REG_HPCF(L3GD20H_CTRL2_REG_HPCF_9)
    };
    
    //===============================   Control Register 3 (R/W)   ==================================//
    //===============================================================================================//
    const uint8_t CTRL_REG3 = 0x22; //Control register 3 address
    
    //CTRL3 Register Structure Definition
    #define L3GD20H_CTRL3_REG_INT1_IG_Pos 7
    #define L3GD20H_CTRL3_REG_INT1_IG (0x1 << L3GD20H_CTRL3_REG_INT1_IG_Pos)
    #define L3GD20H_CTRL3_REG_INT1_BOOT_Pos 6
    #define L3GD20H_CTRL3_REG_INT1_BOOT (0x1 << L3GD20H_CTRL3_REG_INT1_BOOT_Pos)
    #define L3GD20H_CTRL3_REG_INT1_LVL_Pos 5
    #define L3GD20H_CTRL3_REG_INT1_LVL (0x1 << L3GD20H_CTRL3_REG_INT1_LVL_Pos)
    #define L3GD20H_CTRL3_REG_INT_DRIVE_Pos 4
    #define L3GD20H_CTRL3_REG_INT_DRIVE (0x1 << L3GD20H_CTRL3_REG_INT_DRIVE_Pos)
    #define L3GD20H_CTRL3_REG_INT2_DRDY_Pos 3
    #define L3GD20H_CTRL3_REG_INT2_DRDY (0x1 << L3GD20H_CTRL3_REG_INT2_DRDY_Pos)
    #define L3GD20H_CTRL3_REG_INT2_FIFO_TH_Pos 2
    #define L3GD20H_CTRL3_REG_INT2_FIFO_TH (0x1 << L3GD20H_CTRL3_REG_INT2_FIFO_TH_Pos)
    #define L3GD20H_CTRL3_REG_INT2_FIFO_ORUN_Pos 1
    #define L3GD20H_CTRL3_REG_INT2_FIFO_ORUN (0x1 << L3GD20H_CTRL3_REG_INT2_FIFO_ORUN_Pos)
    #define L3GD20H_CTRL3_REG_INT2_FIFO_EMPTY_Pos 0
    #define L3GD20H_CTRL3_REG_INT2_FIFO_EMPTY (0x1 << L3GD20H_CTRL3_REG_INT2_FIFO_EMPTY_Pos)
    
    const uint8_t INTERNAL_INTERUPT_ON_PIN1 = L3GD20H_CTRL3_REG_INT1_IG;    //When set internally generated interrupts
                                                                            //will drive pin 1
    const uint8_t BOOT_STATUS_ON_PIN1 = L3GD20H_CTRL3_REG_INT1_BOOT;    //When set the boot status will drive pin 1
    const uint8_t PIN1_ACTIVE_LEVEL = L3GD20H_CTRL3_REG_INT1_LVL;   //When set the pin 1 output will be active low
                                                                    //When cleared the pin 1 output will be active high
    const uint8_t PIN1_DRIVE_TYPE = L3GD20H_CTRL3_REG_INT_DRIVE;    //When set pin 1 output is configured as open drain
                                                                    //When cleared pin 1 output is configured as push-pull
    const uint8_t DATA_READY_ON_PIN2 = L3GD20H_CTRL3_REG_INT2_DRDY; //When set the data ready signal is assigned
                                                                    //to the DRDY/INT2 pin
                                                                    //When clear the FIFO interrupts are
                                                                    //assigned to the DRDY/INT2 pin
    const uint8_t FIFO_THRESHOLD_INTR = L3GD20H_CTRL3_REG_INT2_FIFO_TH; //When set the FIFO threshold interrupt
                                                                        //is assigned to the DRDY/INT2 pin
    const uint8_t FIFO_OVERRUN_INTR = L3GD20H_CTRL3_REG_INT2_FIFO_ORUN; //When set the FIFO overrun interrupt
                                                                        //is assigned to the DRDY/INT2 pin
    const uint8_t FIFO_EMPTY_INTR = L3GD20H_CTRL3_REG_INT2_FIFO_EMPTY;  //When set the FIFO empty interrupt is
                                                                        //assigned to the DRDY/INT2 pin
    
    
    
    //===============================   Control Register 4 (R/W)   ==================================//
    //===============================================================================================//
    const uint8_t CTRL_REG4 = 0x23; //Control register 4 address
    
    //CTRL4 Register Structure Definition
    #define L3GD20H_CTRL4_REG_BDU_Pos 7
    #define L3GD20H_CTRL4_REG_BDU (0x1 << L3GD20H_CTRL4_REG_BDU_Pos)
    #define L3GD20H_CTRL4_REG_BLE_Pos 6
    #define L3GD20H_CTRL4_REG_BLE (0x1 << L3GD20H_CTRL4_REG_BLE_Pos)
    #define L3GD20H_CTRL4_REG_FS_Pos 4
    #define L3GD20H_CTRL4_REG_FS_Msk (0x3 << L3GD20H_CTRL4_REG_FS_Pos)
    #define L3GD20H_CTRL4_REG_FS(value) (L3GD20H_CTRL4_REG_FS_Msk & ((value) << L3GD20H_CTRL4_REG_FS_Pos))
    #define L3GD20H_CTRL4_REG_FS_245DPS 0x0
    #define L3GD20H_CTRL4_REG_FS_500DPS 0x1
    #define L3GD20H_CTRL4_REG_FS_2000DPS 0x2
    #define L3GD20H_CTRL4_REG_IMPEN_Pos 3
    #define L3GD20H_CTRL4_REG_IMPEN (0x1 << L3GD20H_CTRL4_REG_IMPEN_Pos)
    #define L3GD20H_CTRL4_REG_ST_Pos 1
    #define L3GD20H_CTRL4_REG_ST_Msk (0x3 << L3GD20H_CTRL4_REG_ST_Pos)
    #define L3GD20H_CTRL4_REG_ST(value) (L3GD20H_CTRL4_REG_ST_Msk &((value) << L3GD20H_CTRL4_REG_ST_Pos))
    #define L3GD20H_CTRL4_REG_ST_NORMAL 0x0
    #define L3GD20H_CTRL4_REG_ST_SELFTEST0 0x1
    #define L3GD20H_CTRL4_REG_ST_SELFTEST1 0x3
    #define L3GD20H_CTRL4_REG_SIM_Pos 0
    #define L3GD20H_CTRL4_REG_SIM (0x1 << L3GD20H_CTRL4_REG_SIM_Pos)
    
    const uint8_t BLOCK_DATA_UPDATE_ENABLE = L3GD20H_CTRL4_REG_BDU; //When set block data update is enabled
                                                                    //Default is cleared (BDU disabled)
    const uint8_t BIG_LITTLE_ENDIAN_SELECT = L3GD20H_CTRL4_REG_BLE; //When set the data MSB is at the lower
                                                                    //address
                                                                    //When cleared the data LSB is at the
                                                                    //lower address (default)
    //Full scale gyroscope setting
    const uint8_t FULL_SCALE_MASK = L3GD20H_CTRL4_REG_FS_Msk;
    enum fullScale_t {
        _245DPS = L3GD20H_CTRL4_REG_FS(L3GD20H_CTRL4_REG_FS_245DPS),    //245 degrees/sec full scale (default)
        _500DPS = L3GD20H_CTRL4_REG_FS(L3GD20H_CTRL4_REG_FS_500DPS),    //500 degrees/sec full scale
        _2000DPS = L3GD20H_CTRL4_REG_FS(L3GD20H_CTRL4_REG_FS_2000DPS)  //2,000 degrees/sec full scale
    };
    
    const uint8_t LEVEL_SENSITIVE_LATCH_ENABLE = L3GD20H_CTRL4_REG_IMPEN;   //When set the level sensitive
                                                                            //latch is enabled.
                                                                            //Default is cleared (disabled)
    //Self test settings
    const uint8_t SELF_TEST_MASK = L3GD20H_CTRL4_REG_ST_Msk;
    enum selfTest_t {
        NO_SELF_TEST = L3GD20H_CTRL4_REG_ST(L3GD20H_CTRL4_REG_ST_NORMAL),            //No self test (default)
        SELF_TEST_MODE_0 = L3GD20H_CTRL4_REG_ST(L3GD20H_CTRL4_REG_ST_SELFTEST0),    //Mode 0 self test
        SELF_TEST_MODE_1 = L3GD20H_CTRL4_REG_ST(L3GD20H_CTRL4_REG_ST_SELFTEST1)     //Mode 1 self test
    };
    
    const uint8_t SPI_MODE = L3GD20H_CTRL4_REG_SIM; //When set SPI interface uses 3 wires
                                                    //When cleared SPI interface uses 4 wires (default)
    
    const uint16_t LOW_RANGE = 245;
    const uint16_t MID_RANGE = 500;
    const uint16_t HIGH_RANGE = 2000;
    
    //===============================   Control Register 5 (R/W)   ==================================//
    //===============================================================================================//
    const uint8_t CTRL_REG5 = 0x24; //Control register 5 address
    
    //CTRL5 Register Structure Definition
    #define L3GD20H_CTRL5_REG_BOOT_Pos 7
    #define L3GD20H_CTRL5_REG_BOOT (0x1 << L3GD20H_CTRL5_REG_BOOT_Pos)
    #define L3GD20H_CTRL5_REG_FIFOEN_Pos 6
    #define L3GD20H_CTRL5_REG_FIFOEN (0x1 << L3GD20H_CTRL5_REG_FIFOEN_Pos)
    #define L3GD20H_CTRL5_REG_STOPONFTH_Pos 5
    #define L3GD20H_CTRL5_REG_STOPONFTH (0x1 << L3GD20H_CTRL5_REG_STOPONFTH_Pos)
    #define L3GD20H_CTRL5_REG_HPEN_Pos 4
    #define L3GD20H_CTRL5_REG_HPEN (0x1 << L3GD20H_CTRL5_REG_HPEN_Pos)
    #define L3GD20H_CTRL5_REG_IGSEL_Pos 2
    #define L3GD20H_CTRL5_REG_IGSEL_Msk (0x3 << L3GD20H_CTRL5_REG_IGSEL_Pos)
    #define L3GD20H_CTRL5_REG_IGSEL(value) (L3GD20H_CTRL5_REG_IGSEL_Msk & ((value) << L3GD20H_CTRL5_REG_IGSEL_Pos))
    #define L3GD20H_CTRL5_REG_IGSEL_MODE0 0x0
    #define L3GD20H_CTRL5_REG_IGSEL_MODE1 0x1
    #define L3GD20H_CTRL5_REG_IGSEL_MODE2 0x2
    #define L3GD20H_CTRL5_REG_OUTSEL_Pos 0
    #define L3GD20H_CTRL5_REG_OUTSEL_Msk (0x3 << L3GD20H_CTRL5_REG_OUTSEL_Pos)
    #define L3GD20H_CTRL5_REG_OUTSEL(value) (L3GD20H_CTRL5_REG_OUTSEL_Msk & ((value) << L3GD20H_CTRL5_REG_OUTSEL_Pos))
    #define L3GD20H_CTRL5_REG_OUTSEL_MODE0 0x0
    #define L3GD20H_CTRL5_REG_OUTSEL_MODE1 0x1
    #define L3GD20H_CTRL5_REG_OUTSEL_MODE2 0x2
    
    const uint8_t REBOOT = L3GD20H_CTRL5_REG_BOOT;  //When set the stored trimming parameters are reloaded
    const uint8_t FIFO_ENABLE = L3GD20H_CTRL5_REG_FIFOEN;   //When set the FIFO is enabled
                                                            //When cleared the FIfO is disabled (default)
    const uint8_t STOP_ON_THRESHOLD = L3GD20H_CTRL5_REG_STOPONFTH;  //When set the FIFO depth is limited to threshold
                                                                    //When cleared (default) the FIFO is not limited
    const uint8_t HIGHPASS_FILTER_ENABLE = L3GD20H_CTRL5_REG_HPEN;  //When set the high pass filter is enabled
                                                                    //When cleared (default) the high pass filter is
                                                                    //disabled.
    //Internal Interrupt generator input configuration
    const uint8_t INTERRUPT_GEN_INPUT_SELECT_MASK = L3GD20H_CTRL5_REG_IGSEL_Msk;
    enum interruptGenInput_t {
        INPUT_LOWPASS_FILTER_1 = L3GD20H_CTRL5_REG_IGSEL(L3GD20H_CTRL5_REG_IGSEL_MODE0),
        INPUT_HIGHPASS_FILTER = L3GD20H_CTRL5_REG_IGSEL(L3GD20H_CTRL5_REG_IGSEL_MODE1),
        INPUT_LOWPASS_FILTER_2 = L3GD20H_CTRL5_REG_IGSEL(L3GD20H_CTRL5_REG_IGSEL_MODE2)
    };
    
    //Output source selection
    const uint8_t OUTPUT_SOURCE_SEL_MASK = L3GD20H_CTRL5_REG_OUTSEL_Msk;
    enum outputSource_t {
        SRC_LOWPASS_FILTER_1 = L3GD20H_CTRL5_REG_OUTSEL(L3GD20H_CTRL5_REG_OUTSEL_MODE0),    //LPF1 only
        SRC_HIGHPASS_FILTER = L3GD20H_CTRL5_REG_OUTSEL(L3GD20H_CTRL5_REG_OUTSEL_MODE1),
        SRC_LOWPASS_FILTER_2 = L3GD20H_CTRL5_REG_OUTSEL(L3GD20H_CTRL5_REG_OUTSEL_MODE2)
    };
    
    //===============================   Reference Register (R/W)   ==================================//
    //===============================================================================================//
    const uint8_t REFERENCE_REG = 0x25; //Reference register address
    
    //===============================   Temperature Output Register (R only)   ======================//
    //===============================================================================================//
    const uint8_t OUT_TEMP_REG = 0x26;  //Temperature output register address
    const int8_t TEMP_SCALE_FACTOR = -1;    //-1 LSB/degree C
    
    //===============================   Status Register (R only)   ==================================//
    //===============================================================================================//
    const uint8_t STATUS_REG = 0x27;    //Status register address
    
    //Status Register Structure Definition
    #define L3GD20H_STATUS_REG_ZYXOR_Pos 7
    #define L3GD20H_STATUS_REG_ZYXOR (0x1 << L3GD20H_STATUS_REG_ZYXOR_Pos)
    #define L3GD20H_STATUS_REG_ZOR_Pos 6
    #define L3GD20H_STATUS_REG_ZOR (0x1 << L3GD20H_STATUS_REG_ZOR_Pos)
    #define L3GD20H_STATUS_REG_YOR_Pos 5
    #define L3GD20H_STATUS_REG_YOR (0x1 << L3GD20H_STATUS_REG_YOR_Pos)
    #define L3GD20H_STATUS_REG_XOR_Pos 4
    #define L3GD20H_STATUS_REG_XOR (0x1 << L3GD20H_STATUS_REG_XOR_Pos)
    #define L3GD20H_STATUS_REG_ZYXDA_Pos 3
    #define L3GD20H_STATUS_REG_ZYXDA (0x1 << L3GD20H_STATUS_REG_ZYXDA_Pos)
    #define L3GD20H_STATUS_REG_ZDA_Pos 2
    #define L3GD20H_STATUS_REG_ZDA (0x1 << L3GD20H_STATUS_REG_ZDA_Pos)
    #define L3GD20H_STATUS_REG_YDA_Pos 1
    #define L3GD20H_STATUS_REG_YDA (0x1 << L3GD20H_STATUS_REG_YDA_Pos)
    #define L3GD20H_STATUS_REG_XDA_Pos 0
    #define L3GD20H_STATUS_REG_XDA (0x1 << L3GD20H_STATUS_REG_XDA_Pos)

    //When set a data overrun has occurred in the indicated channel
    const uint8_t XYZ_DATA_OVERRUN = L3GD20H_STATUS_REG_ZYXOR;
    const uint8_t Z_DATA_OVERRUN = L3GD20H_STATUS_REG_ZOR;
    const uint8_t Y_DATA_OVERRUN = L3GD20H_STATUS_REG_YOR;
    const uint8_t X_DATA_OVERRUN = L3GD20H_STATUS_REG_XOR;
    //When set there is new data avialable from the indicated channel
    const uint8_t XYZ_DATA_READY = L3GD20H_STATUS_REG_ZYXDA;
    const uint8_t Z_DATA_READY = L3GD20H_STATUS_REG_ZDA;
    const uint8_t Y_DATA_READY = L3GD20H_STATUS_REG_YDA;
    const uint8_t X_DATA_READY = L3GD20H_STATUS_REG_XDA;
    
    //===============================   Data Output Registers (R only)   ============================//
    //===============================================================================================//
    //Data output register addresses
    const uint8_t OUT_X_L = 0x28;
    const uint8_t OUT_X_H = 0x29;
    const uint8_t OUT_Y_L = 0x2A;
    const uint8_t OUT_Y_H = 0x2B;
    const uint8_t OUT_Z_L = 0x2C;
    const uint8_t OUT_Z_H = 0x2D;
    
    //===============================   FIFO Control Register (R/W)   ===============================//
    //===============================================================================================//
    const uint8_t FIFO_CTRL = 0x2E; //FIFO Control register address
    
    //FIFO Control Register Structure Definition
    #define L3GD20H_FIFO_CTRL_REG_FM_Pos 5
    #define L3GD20H_FIFO_CTRL_REG_FM_Msk (0x7 << L3GD20H_FIFO_CTRL_REG_FM_Pos)
    #define L3GD20H_FIFO_CTRL_REG_FM(value) (L3GD20H_FIFO_CTRL_REG_FM_Msk &((value) << L3GD20H_FIFO_CTRL_REG_FM_Pos))
    #define L3GD20H_FIFO_CTRL_REG_FM_BYPASS 0x0
    #define L3GD20H_FIFO_CTRL_REG_FM_FIFO 0x1
    #define L3GD20H_FIFO_CTRL_REG_FM_STREAM 0x2
    #define L3GD20H_FIFO_CTRL_REG_FM_STREAMTOFIFO 0x3
    #define L3GD20H_FIFO_CTRL_REG_FM_BYPASSTOSTREAM 0x4
    #define L3GD20H_FIFO_CTRL_REG_FM_DYNAMICSTREAM 0x6
    #define L3GD20H_FIFO_CTRL_REG_FM_BYPASSTOFIFO 0x7
    #define L3GD20H_FIFO_CTRL_REG_THRESHOLD_Pos 0
    #define L3GD20H_FIFO_CTRL_REG_THRESHOLD_Msk (0x1F << L3GD20H_FIFO_CTRL_REG_THRESHOLD_Pos)
    
    //FIFO Mode Setting
    const uint8_t FIFO_MODE_MASK = L3GD20H_FIFO_CTRL_REG_FM_Msk;
    
    enum FIFO_Mode_t {
        BYPASS = L3GD20H_FIFO_CTRL_REG_FM(L3GD20H_FIFO_CTRL_REG_FM_BYPASS),
        FIFO = L3GD20H_FIFO_CTRL_REG_FM(L3GD20H_FIFO_CTRL_REG_FM_FIFO),
        STREAM = L3GD20H_FIFO_CTRL_REG_FM(L3GD20H_FIFO_CTRL_REG_FM_STREAM),
        STREAM_TO_FIFO = L3GD20H_FIFO_CTRL_REG_FM(L3GD20H_FIFO_CTRL_REG_FM_STREAMTOFIFO),
        BYPASS_TO_STREAM = L3GD20H_FIFO_CTRL_REG_FM(L3GD20H_FIFO_CTRL_REG_FM_BYPASSTOSTREAM),
        DYNAMIC_STREAM = L3GD20H_FIFO_CTRL_REG_FM(L3GD20H_FIFO_CTRL_REG_FM_DYNAMICSTREAM),
        BYPASS_TO_FIFO = L3GD20H_FIFO_CTRL_REG_FM(L3GD20H_FIFO_CTRL_REG_FM_BYPASSTOFIFO)
    };
    
    const uint8_t MAX_FIFO_THRESHOLD = 31;
    const uint8_t FIFO_THRESHOLD_MASK = L3GD20H_FIFO_CTRL_REG_THRESHOLD_Msk;
    
    //===============================   FIFO SRC Register (R only)   ================================//
    //===============================================================================================//
    const uint8_t FIFO_SRC = 0x2F;  //FIFO SRC register address
    
    //FIFO SRC Register Structure Definition
    #define L3GD20H_FIFO_SRC_REG_FTH_Pos 7
    #define L3DG20H_FIFO_SRC_REG_FTH (0x1 << L3GD20H_FIFO_SRC_REG_FTH_Pos)
    #define L3GD20H_FIFO_SRC_REG_OVRN_Pos 6
    #define L3DG20H_FIFO_SRC_REG_OVRN (0x1 << L3GD20H_FIFO_SRC_REG_OVRN_Pos)
    #define L3GD20H_FIFO_SRC_REG_EMPTY_Pos 5
    #define L3DG20H_FIFO_SRC_REG_EMPTY (0x1 << L3GD20H_FIFO_SRC_REG_EMPTY_Pos)
    #define L3GD20H_FIFO_SRC_REG_DATALEVEL_Pos 0
    #define L3GD20H_FIFO_SRC_REG_DATALEVEL_Msk (0x1F << L3GD20H_FIFO_SRC_REG_DATALEVEL_Pos)
    
    
    const uint8_t FIFO_AT_THRESHOLD = L3DG20H_FIFO_SRC_REG_FTH; //When set the FIFO is filled to the threshold level
    const uint8_t FIFO_OVERRUN = L3DG20H_FIFO_SRC_REG_OVRN; //When set the FIFO is completely filled
    const uint8_t FIFO_EMPTY = L3DG20H_FIFO_SRC_REG_EMPTY;  //When set the FIFO is empty
    const uint8_t FIFO_DATA_LEVEL_MASK = L3GD20H_FIFO_SRC_REG_DATALEVEL_Msk;
    
    //===============================   IG_CFG Register (R/W)   =====================================//
    //===============================================================================================//
    const uint8_t IG_CFG = 0x30;    //IG_CFG register address
    
    //IG_CFG Register Structure Definition
    #define L3GD20H_IG_CFG_REG_ANDOR_Pos 7
    #define L3GD20H_IG_CFG_REG_ANDOR (0x1 << L3GD20H_IG_CFG_REG_ANDOR_Pos)
    #define L3GD20H_IG_CFG_REG_LIR_Pos 6
    #define L3GD20H_IG_CFG_REG_LIR (0x1 << L3GD20H_IG_CFG_REG_LIR_Pos)
    #define L3GD20H_IG_CFG_REG_ZHIE_Pos 5
    #define L3GD20H_IG_CFG_REG_ZHIE (0x1 << L3GD20H_IG_CFG_REG_ZHIE_Pos)
    #define L3GD20H_IG_CFG_REG_ZLIE_Pos 4
    #define L3GD20H_IG_CFG_REG_ZLIE (0x1 << L3GD20H_IG_CFG_REG_ZLIE_Pos)
    #define L3GD20H_IG_CFG_REG_YHIE_Pos 3
    #define L3GD20H_IG_CFG_REG_YHIE (0x1 << L3GD20H_IG_CFG_REG_YHIE_Pos)
    #define L3GD20H_IG_CFG_REG_YLIE_Pos 2
    #define L3GD20H_IG_CFG_REG_YLIE (0x1 << L3GD20H_IG_CFG_REG_YLIE_Pos)
    #define L3GD20H_IG_CFG_REG_XHIE_Pos 1
    #define L3GD20H_IG_CFG_REG_XHIE (0x1 << L3GD20H_IG_CFG_REG_XHIE_Pos)
    #define L3GD20H_IG_CFG_REG_XLIE_Pos 0
    #define L3GD20H_IG_CFG_REG_XLIE (0x1 << L3GD20H_IG_CFG_REG_XLIE_Pos)
    
    const uint8_t AND_OR_SEL = L3GD20H_IG_CFG_REG_ANDOR;    //When set the IG uses an AND combination of
                                                            //interrupt events. When cleared the IG uses
                                                            //an OR combination of interrupt events
    const uint8_t LATCH_INTERRUPT_REQUEST = L3GD20H_IG_CFG_REG_LIR;  //When set interrupt requests are latched
                                                                     //When cleared they are not latched
    const uint8_t Z_AXIS_HIGH_ENABLE = L3GD20H_IG_CFG_REG_ZHIE; //When set the interrupt is enabled
    const uint8_t Z_AXIS_LOW_ENABLE = L3GD20H_IG_CFG_REG_ZLIE; //When set the interrupt is enabled
    const uint8_t Y_AXIS_HIGH_ENABLE = L3GD20H_IG_CFG_REG_YHIE; //When set the interrupt is enabled
    const uint8_t Y_AXIS_LOW_ENABLE = L3GD20H_IG_CFG_REG_YLIE; //When set the interrupt is enabled
    const uint8_t X_AXIS_HIGH_ENABLE = L3GD20H_IG_CFG_REG_XHIE; //When set the interrupt is enabled
    const uint8_t X_AXIS_LOW_ENABLE = L3GD20H_IG_CFG_REG_XLIE; //When set the interrupt is enabled
    const uint8_t ALL_AXIS_HIGH_ENABLE = Z_AXIS_HIGH_ENABLE | Y_AXIS_HIGH_ENABLE | X_AXIS_HIGH_ENABLE;
    const uint8_t ALL_AXIS_LOW_ENABLE = Z_AXIS_LOW_ENABLE | Y_AXIS_LOW_ENABLE | X_AXIS_LOW_ENABLE;
    const uint8_t NO_AXIS_ENABLE = 0x0;
    
    //===============================   IG_SRC Register (R only)   ==================================//
    //===============================================================================================//
    const uint8_t IG_SRC = 0x31;    //IG_SRC register address
    
    //IG_SRC Register Structure Definition
    #define L3GD20H_IG_SRC_REG_IA_Pos 6
    #define L3GD20H_IG_SRC_REG_IA (0x1 << L3GD20H_IG_SRC_REG_IA_Pos)
    #define L3GD20H_IG_SRC_REG_ZH_Pos 5
    #define L3GD20H_IG_SRC_REG_ZH (0x1 << L3GD20H_IG_SRC_REG_ZH_Pos)
    #define L3GD20H_IG_SRC_REG_ZL_Pos 4
    #define L3GD20H_IG_SRC_REG_ZL (0x1 << L3GD20H_IG_SRC_REG_ZL_Pos)
    #define L3GD20H_IG_SRC_REG_YH_Pos 3
    #define L3GD20H_IG_SRC_REG_YH (0x1 << L3GD20H_IG_SRC_REG_YH_Pos)
    #define L3GD20H_IG_SRC_REG_YL_Pos 2
    #define L3GD20H_IG_SRC_REG_YL (0x1 << L3GD20H_IG_SRC_REG_YL_Pos)
    #define L3GD20H_IG_SRC_REG_XH_Pos 1
    #define L3GD20H_IG_SRC_REG_XH (0x1 << L3GD20H_IG_SRC_REG_XH_Pos)
    #define L3GD20H_IG_SRC_REG_XL_Pos 0
    #define L3GD20H_IG_SRC_REG_XL (0x1 << L3GD20H_IG_SRC_REG_XL_Pos)
    
    const uint8_t INTERRUPT_ACTIVE = L3GD20H_IG_SRC_REG_IA; //When set one or more interrupts are active
    const uint8_t Z_AXIS_HIGH_INTR = L3GD20H_IG_SRC_REG_ZH; //When set the Z axis high interrupt is active
    const uint8_t Z_AXIS_LOW_INTR = L3GD20H_IG_SRC_REG_ZL; //When set the Z axis low interrupt is active
    const uint8_t Y_AXIS_HIGH_INTR = L3GD20H_IG_SRC_REG_YH; //When set the Y axis high interrupt is active
    const uint8_t Y_AXIS_LOW_INTR = L3GD20H_IG_SRC_REG_YL; //When set the Y axis low interrupt is active
    const uint8_t X_AXIS_HIGH_INTR = L3GD20H_IG_SRC_REG_XH; //When set the X axis high interrupt is active
    const uint8_t X_AXIS_LOW_INTR = L3GD20H_IG_SRC_REG_XL; //When set the X axis low interrupt is active
    
    //===============================   IG_THS Registers (R/W)   ====================================//
    //===============================================================================================//
    const uint8_t IG_THS_XH = 0x32; //IG_THS_XH register address
    const uint8_t IG_THS_XL = 0x33; //IG_THS_XL register address
    const uint8_t IG_THS_YH = 0x34; //IG_THS_YH register address
    const uint8_t IG_THS_YL = 0x35; //IG_THS_YL register address
    const uint8_t IG_THS_ZH = 0x36; //IG_THS_ZH register address
    const uint8_t IG_THS_ZL = 0x37; //IG_THS_ZL register address
    
    #define L3GD20H_IG_THS_Pos 0
    #define L3GD20H_IG_THS_H_Msk (0x7F << L3GD20H_IG_THS_Pos)
    
    const uint8_t THRESHOLD_HIGHBYTE_MASK = L3GD20H_IG_THS_H_Msk;
    
    //Interrupt threshold register scale factor (milli dps/LSB)
    const float intrThresholdScaleFactor[] = {7.48, 15.26, 61.04};
    
    
    //===============================   IG_DURATION Register (R/W)   ================================//
    //===============================================================================================//
    const uint8_t IG_DURATION = 0x38; //IG_DURATION register address
    
    //IG_DURATION Register Structure Definition
    #define L3GD20H_IG_DURATION_REG_WAIT_Pos 7
    #define L3GD20H_IG_DURATION_REG_WAIT (0x1 << L3GD20H_IG_DURATION_REG_WAIT_Pos)
    #define L3GD20H_IG_DURATION_REG_DURATION_Pos 0
    #define L3GD20H_IG_DURATION_REG_DURATION_Msk (0x7F << L3GD20H_IG_DURATION_REG_DURATION_Pos)
    
    const uint8_t WAIT = L3GD20H_IG_DURATION_REG_WAIT;
    const uint8_t INTR_DURATION_MASK = L3GD20H_IG_DURATION_REG_DURATION_Msk;
    
    //Interrupt duration register scale factor (milli sec/LSB)
    const float intrDurationScaleFactor[] = {80.0, 40.0, 20.0, 10.0, 5.0, 2.5, 1.25};
    
    //===============================   LOW_ODR Register (R/W)   ====================================//
    //===============================================================================================//
    const uint8_t LOW_ODR = 0x39; //LOW_ODR register address
    
    //LOW_ODR Register Structure Definition
    #define L3GD20H_LOW_ODR_REG_DRDY_HL_Pos 5
    #define L3GD20H_LOW_ODR_REG_DRDY_HL (0x1 << L3GD20H_LOW_ODR_REG_DRDY_HL_Pos)
    #define L3GD20H_LOW_ODR_REG_I2C_DISABLE_Pos 3
    #define L3GD20H_LOW_ODR_REG_I2C_DISABLE (0x1 << L3GD20H_LOW_ODR_REG_I2C_DISABLE_Pos)
    #define L3GD20H_LOW_ODR_REG_SW_RESET_Pos 2
    #define L3GD20H_LOW_ODR_REG_SW_RESET (0x1 << L3GD20H_LOW_ODR_REG_SW_RESET_Pos)
    #define L3GD20H_LOW_ODR_REG_LOW_ODR_Pos 0
    #define L3GD20H_LOW_ODR_REG_LOW_ODR (0x1 << L3GD20H_LOW_ODR_REG_LOW_ODR_Pos)
    
    const uint8_t DATA_READY_INTR_HIGH = L3GD20H_LOW_ODR_REG_DRDY_HL; //When set the data ready
    //interrupt is active high
    const uint8_t I2C_DISABLE = L3GD20H_LOW_ODR_REG_I2C_DISABLE; //When set the I2C interface is
    //disabled (only SPI is active)
    const uint8_t SW_RESET = L3GD20H_LOW_ODR_REG_SW_RESET;  //When set a software reset is initiated
    const uint8_t LOW_ODR_MODE = L3GD20H_LOW_ODR_REG_LOW_ODR; //When set low speed ODR is enabled
    
    //Other constant definitions
    enum filterMode_t {
        LPF1_ONLY,
        LPF1_HPF,
        LPF1_LPF2,
        LPF1_HPF_LPF2
    };
    
    enum outputDataRate_t {
        _12_5HZ = 0,
        _25HZ,
        _50HZ,
        _100HZ,
        _200HZ,
        _400HZ,
        _800HZ
    };
    
    //Reference register scale factor (milli dps/LSB)
    const uint8_t referenceScaleFactor[] = {2, 4, 16};
    
    //Angular rate data scale factor (dps/LSB)
    const float angularRateScaleFactor[] = {0.00875, 0.0175, 0.070};
    
    
    
    
    
    //SPI Read/Write Bit
    const uint8_t SPI_READ = 0x80;  //Bit set for SPI read operation
    
    //SPI Multi-Register Access Bit
    const uint8_t SPI_MULT_REG_ACCESS = 0x40;  //Bit set in first word written to allow register address increment
    
    //SPI Clock Speed
    const uint32_t SPI_CLOCK = 10000000;

    
    /********************************************************************************************/
    /*               L3GD20HDriver Abstract Base Class Declartion                               */
    /********************************************************************************************/
    /*Base class for the L3GD20H driver object                                                  */
    /*The begin() method must be implemented by derived classes                                 */
    /********************************************************************************************/
    class L3GD20HDriver
    {
    public:
        //Constructor (Does nothing)
        L3GD20HDriver();
        
        //Destructor (Does nothing)
        virtual ~L3GD20HDriver() {}
        
        //Methods for basic initialization/operation
        void run(fullScale_t range,
                 outputDataRate_t odr,
                 filterMode_t filterMode = LPF1_ONLY,
                 LPF2_bandwidth_t bw = LOW_BW,
                 HP_FiterMode_t hpMode = NORMAL_MODE,
                 highPassCutOff_t cutOff =  HPCF_LVL_0,
                 int16_t referenceSetting = 0,
                 axis_t axis = XYZ);
        void run();
        void sleep();
        void setRange(fullScale_t range);
        void setDataRate(outputDataRate_t odr);
        void setMeasurementAxis(axis_t axis);
        
        //Methods to retrieve measured data
        bool isDataReady(axis_t axis = XYZ);
        bool isDataOverrun(axis_t axis = XYZ);
        void calibrate(uint8_t samples = 10, axis_t axis = XYZ);
        float readXAxisRate();
        float readYAxisRate();
        float readZAxisRate();
        void readAngularRates(float angularRates[]);
        void readFIFOData(float rateData[][3], uint8_t numDataSamples = 32);
        int8_t readTempChange();
        
        //Methods for managing the FIFO
        void enableFIFO(bool stopOnThreshold);
        void disableFIFO();
        void setFIFO(FIFO_Mode_t  mode, uint8_t threshold = MAX_FIFO_THRESHOLD);
        void resetFIFO();
        bool isFIFO_Empty();
        bool isFIFO_AtThreshold();
        bool isFIFO_Overrun();
        uint8_t getFIFODataLevel();
        
        //Methods for managing interrupts
        void configureInterrupts(bool activeHigh, bool pushPull);
        void enableDataReadyInterrupt(bool enable);
        void enableFIFOInterrupts(bool threshold, bool overrun, bool empty);
        void enableIGInterrupts(bool latch, bool logicAND, uint8_t enableAxis);
        void setInterruptIGThreshold(uint16_t xAxis, uint16_t yAxis, uint16_t zAxis);
        void setInterruptIGDuration(float duration, bool wait);
        uint8_t getInterruptSource();
        
        //Methods for managing the filters
        void resetHPFilter();
        void setHighPassFilterReference(int16_t refValue);
        
        //Misc. methods
        bool isL3GD20H();
        void setBDU(bool bdu);
        void setBigEndian(bool bigEndian);
        
        //Low level functions. All pure virtual, must be implemented in derived classes
        virtual uint8_t readRegister(uint8_t reg) const = 0;
        virtual int16_t readDataRegisterPair(uint8_t reg) const = 0;
        virtual void read3AxisDataRegisters(int16_t dataArray[]) const = 0;
        virtual void readFIFO(int16_t dataArray[], uint8_t numDataSamples) = 0;
        virtual void writeRegister(uint8_t reg, uint8_t value) const = 0;
    private:
        int8_t mPowerOnTemp;    //The reference temperature measurement made at turnon
        uint8_t mRange; //The current full scale measurement setting
        axis_t mAxis;       //The measurement axis that were last enabled for measurement
        outputDataRate_t mODR;  //The current output data rate setting
        float mCalibration[3];  //Calibration values used to correct for zero offset errors
        
    };
    
    /********************************************************************************************/
    /*               L3GD20H_I2C_Driver Class Declartion                                        */
    /********************************************************************************************/
    /*Derived class for L3GD20H driver object that uses I2C for communications                  */
    /********************************************************************************************/
    class L3GD20H_I2C_Driver : public L3GD20HDriver
    {
    public:
        //Constructor
        L3GD20H_I2C_Driver(uint8_t i3cAddress);
        
        void begin(TwoWire * I2cDriver = nullptr);
        
        //Low level functions
        virtual uint8_t readRegister(uint8_t reg) const;
        virtual int16_t readDataRegisterPair(uint8_t reg) const;
        virtual void read3AxisDataRegisters(int16_t dataArray[]);
        virtual void readFIFO(int16_t dataArray[], uint8_t numDataSamples);
        virtual void writeRegister(uint8_t reg, uint8_t value) const;
    private:
        uint8_t mI2C_Address;
        TwoWire * mI2C_Driver;
    };
    
    /********************************************************************************************/
    /*               L3GD20H_SPI_Driver Class Declartion                                        */
    /********************************************************************************************/
    /*Derived class for L3GD20H driver object that uses SPI for communications                  */
    /********************************************************************************************/
    class L3GD20H_SPI_Driver : public L3GD20HDriver
    {
    public:
        //Constructor
        L3GD20H_SPI_Driver(uint8_t csPin);
        
        void begin(SPIClass * spiDriver);
        
        //Low level functions
        virtual uint8_t readRegister(uint8_t reg) const;
        virtual int16_t readDataRegisterPair(uint8_t reg) const;
        virtual void read3AxisDataRegisters(int16_t dataArray[]) const;
        virtual void readFIFO(int16_t dataArray[], uint8_t numDataSamples);
        virtual void writeRegister(uint8_t reg, uint8_t value) const;
        
    private:
        uint8_t mCS_Pin;    //Chip select pin for SPI
        SPIClass * mSPI_Driver; //Pointer to the DPI driver object
    };
    
    

} //End namespace L3GD20H

#endif /* L3GD20H_Driver_hpp */
