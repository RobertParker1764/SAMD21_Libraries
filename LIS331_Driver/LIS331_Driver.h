//LIS331 MEMS Accelerometer Driver
//Verision 1.0
//March 26, 2018
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


#ifndef LIS331_Driver_H_
#define LIS331_Driver_H_


#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

namespace LIS331
{
    /****************************** Data Type and Constant Defintions ******************************/
    
    //===============================   Control Register 1 (R/W)   ================================//
    //=============================================================================================//
    const uint8_t CTRL_REG1 = 0x20;      //Control Register 1 address
    
    //CTRL_REG1 Register Structure Definition
    #define LIS331_CTRL_REG1_PM_Pos  5
    #define LIS_331_CTRL_REG1_PM_Msk  (0x7 << LIS331_CTRL_REG1_PM_Pos)
    #define LIS_331_CTRL_REG1_PM(value)  (LIS_331_CTRL_REG1_PM_Msk & ((value) << LIS331_CTRL_REG1_PM_Pos))
    #define LIS_331_CTRL_REG1_PM_POWER_DOWN  0x0
    #define LIS_331_CTRL_REG1_PM_NORMAL  0x1
    #define LIS_331_CTRL_REG1_PM_LOWP_05ODR  0x2
    #define LIS_331_CTRL_REG1_PM_LOWP_1ODR  0x3
    #define LIS_331_CTRL_REG1_PM_LOWP_2ODR  0x4
    #define LIS_331_CTRL_REG1_PM_LOWP_5ODR  0x5
    #define LIS_331_CTRL_REG1_PM_LOWP_10ODR  0x6
    #define LIS_331_CTRL_REG1_DR_Pos  3
    #define LIS_331_CTRL_REG1_DR_Msk  (0x3 << LIS_331_CTRL_REG1_DR_Pos)
    #define LIS_331_CTRL_REG1_DR(value) (LIS_331_CTRL_REG1_DR_Msk & ((value) <<LIS_331_CTRL_REG1_DR_Pos))
    #define LIS_331_CTRL_REG1_DR_50HZ  0x0
    #define LIS_331_CTRL_REG1_DR_100HZ  0x1
    #define LIS_331_CTRL_REG1_DR_400HZ  0x2
    #define LIS_331_CTRL_REG1_DR_1000HZ  0x3
    #define LIS_331_CTRL_REG1_ZEN_Pos  2
    #define LIS_331_CTRL_REG1_ZEN  (0x1 << LIS_331_CTRL_REG1_ZEN_Pos)
    #define LIS_331_CTRL_REG1_YEN_Pos  1
    #define LIS_331_CTRL_REG1_YEN  (0x1 << LIS_331_CTRL_REG1_YEN_Pos)
    #define LIS_331_CTRL_REG1_XEN_Pos  0
    #define LIS_331_CTRL_REG1_XEN  (0x1 << LIS_331_CTRL_REG1_XEN_Pos)
    
    //Power mode settings
    const uint8_t POWER_MODE_MASK = LIS_331_CTRL_REG1_PM_Msk;    //Mask for the PM bits
    enum powerMode_t {
        POWER_DOWN = LIS_331_CTRL_REG1_PM_POWER_DOWN << LIS331_CTRL_REG1_PM_Pos,      //Power down mode
        NORMAL_MODE = LIS_331_CTRL_REG1_PM_NORMAL << LIS331_CTRL_REG1_PM_Pos,         //Normal mode
        LOW_POWER_0_5HZ = LIS_331_CTRL_REG1_PM_LOWP_05ODR << LIS331_CTRL_REG1_PM_Pos, //Low power mode with 0.5Hz ODR
        LOW_POWER_1HZ = LIS_331_CTRL_REG1_PM_LOWP_1ODR << LIS331_CTRL_REG1_PM_Pos,    //Low power mode with 1Hz ODR
        LOW_POWER_2HZ = LIS_331_CTRL_REG1_PM_LOWP_2ODR << LIS331_CTRL_REG1_PM_Pos,    //Low power mode with 2Hz ODR
        LOW_POWER_5HZ = LIS_331_CTRL_REG1_PM_LOWP_5ODR << LIS331_CTRL_REG1_PM_Pos,    //Low power mode with 5Hz ODR
        LOW_POWER_10HZ = LIS_331_CTRL_REG1_PM_LOWP_10ODR << LIS331_CTRL_REG1_PM_Pos   //Low power mode with 10Hz ODR
    };
    
    //Data rate settings
    const uint8_t DATA_RATE_MASK = LIS_331_CTRL_REG1_DR_Msk;     //Mask for the DR bits
    enum outputDataRate_t {
        ODR_50HZ = LIS_331_CTRL_REG1_DR_50HZ << LIS_331_CTRL_REG1_DR_Pos,        //50Hz ODR with 37Hz low pass filter cutoff
        ODR_100HZ = LIS_331_CTRL_REG1_DR_100HZ << LIS_331_CTRL_REG1_DR_Pos,      //100Hz ODR with 74Hz low pass filter cutoff
        ODR_400HZ = LIS_331_CTRL_REG1_DR_400HZ << LIS_331_CTRL_REG1_DR_Pos,      //400Hz ODR with 292Hz low pass filter cutoff
        ODR_1000HZ = LIS_331_CTRL_REG1_DR_1000HZ << LIS_331_CTRL_REG1_DR_Pos     //1000Hz ODR with 780Hz low pass filter cutoff
    };
    
    //Measurement axis enable bits
    const uint8_t AXIS_ENABLE_MASK = 0x07;     //Mask for the axis enable bits
    const uint8_t ZEN_BIT = LIS_331_CTRL_REG1_ZEN;     //Z axis enable bit. When set Z axis measurements are enabled.
    const uint8_t YEN_BIT = LIS_331_CTRL_REG1_YEN;     //Y axis enable bit. When set Y axis measurements are enabled.
    const uint8_t XEN_BIT = LIS_331_CTRL_REG1_XEN;     //X axis enable bit. When set X axis measurements are enabled.
    enum enableAxisSettings_t {
        NO_AXIS = 0x00,             //All axis disabled
        X = 0x01,                   //Enable X-Axis only
        Y = 0x02,                   //Enable Y-Axis only
        XY = 0x03,                  //Enable X and Y-Axis
        Z = 0x04,                   //Enable Z-Axis only
        XZ = 0x05,                  //Enable X and Z-Axis
        YZ = 0x06,                  //Enable Y and Z-Axis
        XYZ = 0x07                  //Enable X, Y, and Z-Axis
    };
    
    //======================================  Control Register 2 (R/W)  ===============================================//
    //=================================================================================================================//
    const uint8_t CTRL_REG2 = 0x21;      //Control Register 2 address
    
    //CTRL_REG2 Register Structure Definition
    #define LIS_331_CTRL_REG2_BOOT_Pos  7
    #define LIS_331_CTRL_REG2_BOOT  (0x1 << LIS_331_CTRL_REG2_BOOT_Pos)
    #define LIS_331_CTRL_REG2_HPM_Pos 5
    #define LIS_331_CTRL_REG2_HPM_Msk (0x3 << LIS_331_CTRL_REG2_HPM_Pos)
    #define LIS_331_CTRL_REG2_HPM(value) = LIS_331_CTRL_REG2_HPM_Msk & ((value) << LIS_331_CTRL_REG2_HPM_Pos)
    #define LIS_331_CTRL_REG2_HPM_NORMAL 0x0
    #define LIS_331_CTRL_REG2_HPM_REFSIG 0x1
    #define LIS_331_CTRL_REG2_FDS_Pos 4
    #define LIS_331_CTRL_REG2_FDS (0x1 << LIS_331_CTRL_REG2_FDS_Pos)
    #define LIS_331_CTRL_REG2_HPEN2_Pos 3
    #define LIS_331_CTRL_REG2_HPEN2 (0x1 << LIS_331_CTRL_REG2_HPEN2_Pos)
    #define LIS_331_CTRL_REG2_HPEN1_Pos 2
    #define LIS_331_CTRL_REG2_HPEN1 (0x1 << LIS_331_CTRL_REG2_HPEN1_Pos)
    #define LIS_331_CTRL_REG2_HPC_Pos 0
    #define LIS_331_CTRL_REG2_HPC_Msk (0x3 << LIS_331_CTRL_REG2_HPC_Pos)
    #define LIS_331_CTRL_REG2_HPC(value) LIS_331_CTRL_REG2_HPC_Msk & ((value) << LIS_331_CTRL_REG2_HPC_Pos)
    #define LIS_331_CTRL_REG_HPC_1 0x0
    #define LIS_331_CTRL_REG_HPC_2 0x1
    #define LIS_331_CTRL_REG_HPC_3 0x2
    #define LIS_331_CTRL_REG_HPC_4 0x3

    const uint8_t BOOT_BIT = LIS_331_CTRL_REG2_BOOT;    //Boot bit. When set causes the internal register to be restored.
    
    //High pass filter mode settings
    const uint8_t HPM0_BIT = 1 << LIS_331_CTRL_REG2_HPM_Pos;    //High pass filter bit. When set high pass filter is in
                                                                //reference mode.
    const uint8_t FDS_BIT = LIS_331_CTRL_REG2_FDS;  //Filter data selection bit. When clear the high pass filter data is
                                                    //bypassed. When set high pass filter data is sent to the output
                                                    //register.
    const uint8_t HPEN2_BIT = LIS_331_CTRL_REG2_HPEN2;  //High pass filter enable bit for interrupt 2 source. When set the
                                                        //filter is enabled.
    const uint8_t HPEN1_BIT = LIS_331_CTRL_REG2_HPEN1;   //High pass filter enable bit for interrupt 1 source. When set the
                                        //filter is enabled.
    const uint8_t HPCF_MASK = LIS_331_CTRL_REG2_HPC_Msk;     //Mask for the HPCF bits.
    enum hp_Filter_Coeficient_t {
        HPC_8 = LIS_331_CTRL_REG2_HPC(LIS_331_CTRL_REG_HPC_1),   //See Table 23 of the data sheet for cutoff frequencies
        HPC_16 = LIS_331_CTRL_REG2_HPC(LIS_331_CTRL_REG_HPC_2),
        HPC_32 = LIS_331_CTRL_REG2_HPC(LIS_331_CTRL_REG_HPC_3),
        HPC_64 = LIS_331_CTRL_REG2_HPC(LIS_331_CTRL_REG_HPC_4)
    };
    
    //==================================  Control Register 3 (R/W)  ====================================================//
    //==================================================================================================================//
    const uint8_t CTRL_REG3 = 0x22;      //Control Register 3 address
    
    //CTRL_REG3 Register Structure Definition
    #define LIS_313_CTRL_REG3_IHL_Pos 7
    #define LIS_313_CTRL_REG3_IHL (0x1 << LIS_313_CTRL_REG3_IHL_Pos)
    #define LIS_313_CTRL_REG3_PPOD_Pos 6
    #define LIS_313_CTRL_REG3_PPOD (0x1 << LIS_313_CTRL_REG3_PPOD_Pos)
    #define LIS_313_CTRL_REG3_LIR2_Pos 5
    #define LIS_313_CTRL_REG3_LIR2 (0x1 << LIS_313_CTRL_REG3_LIR2_Pos)
    #define LIS_313_CTRL_REG3_I2CFG_Pos 3
    #define LIS_313_CTRL_REG3_I2CFG_Msk (0x3 << LIS_313_CTRL_REG3_I2CFG_Pos)
    #define LIS_313_CTRL_REG3_I2CFG(value) LIS_313_CTRL_REG3_I2CFG_Msk & ((value) << LIS_313_CTRL_REG3_I2CFG_Pos)
    #define LIS_313_CTRL_REG3_I2CFG_I2 0x0
    #define LIS_313_CTRL_REG3_I2CFG_I2ORI1 0x1
    #define LIS_313_CTRL_REG3_I2CFG_DATARDY 0x2
    #define LIS_313_CTRL_REG3_I2CFG_BOOT 0x03
    #define LIS_313_CTRL_REG3_LIR1_Pos 2
    #define LIS_313_CTRL_REG3_LIR1 (0x1 << LIS_313_CTRL_REG3_LIR1_Pos)
    #define LIS_313_CTRL_REG3_I1CFG_Pos 0
    #define LIS_313_CTRL_REG3_I1CFG_Msk (0x3 << LIS_313_CTRL_REG3_I1CFG_Pos)
    #define LIS_313_CTRL_REG3_I1CFG(value) LIS_313_CTRL_REG3_I1CFG_Msk & ((value) << LIS_313_CTRL_REG3_I1CFG_Pos)
    #define LIS_313_CTRL_REG3_I1CFG_I1 0x0
    #define LIS_313_CTRL_REG3_I1CFG_I1ORI2 0x1
    #define LIS_313_CTRL_REG3_I1CFG_DATARDY 0x2
    #define LIS_313_CTRL_REG3_I1CFG_BOOT 0x03
    
    
    //Interrupt configuration settings
    const uint8_t IHL_BIT = LIS_313_CTRL_REG3_IHL;     //Interrupt active high/low bit. When set interrupt is active low.
    const uint8_t PP_OD_BIT = LIS_313_CTRL_REG3_PPOD;  //Interrupt output configuration bit. When set the output is open drain.
                                                       //When cleared the output is push-pull.
    const uint8_t LIR2_BIT = LIS_313_CTRL_REG3_LIR2;   //Latch interrupt request bit for pin 2. When set interrupt request is
                                                       //latched.
    const uint8_t LIR1_BIT = LIS_313_CTRL_REG3_LIR1;   //Latch interrupt request bit for pin 1. When set interrupt request is
                                                       //latched.
    const uint8_t I2_CFG_MASK = LIS_313_CTRL_REG3_I2CFG_Msk;   //Mask for interrupt pin 2 configuration bits
    const uint8_t I1_CFG_MASK = LIS_313_CTRL_REG3_I1CFG_Msk;   //Mask for interrupt pin 1 configuration bits
    
    enum intrPin1Config_t {
        INT_1_SRC = LIS_313_CTRL_REG3_I1CFG(LIS_313_CTRL_REG3_I1CFG_I1), //Interrupt pin 1 is sourced from internal interrupt 1.
        INT_1_OR_2_SRC = LIS_313_CTRL_REG3_I1CFG(LIS_313_CTRL_REG3_I1CFG_I1ORI2), //Interrupt pin 1 is sourced from the logical
                                                                                  //OR of internal interrupts 1 and 2
        DATA_RDY = LIS_313_CTRL_REG3_I1CFG(LIS_313_CTRL_REG3_I1CFG_DATARDY),    //Interrupt pin 1 is sourced from the data ready
                                                                                //bit of the status register
        BOOT_RUN = LIS_313_CTRL_REG3_I1CFG(LIS_313_CTRL_REG3_I1CFG_BOOT)  //Interrupt pin 1 is sourced from the boot running
                                                                          //condition
    };
    
    enum intrPin2Config_t {
        INT_2_SRC = LIS_313_CTRL_REG3_I2CFG(LIS_313_CTRL_REG3_I2CFG_I2), //Interrupt pin 2 is sourced from internal interrupt 2.
        xINT_1_OR_2_SRC = LIS_313_CTRL_REG3_I2CFG(LIS_313_CTRL_REG3_I2CFG_I2ORI1), //Interrupt pin 2 is sourced from the logical
                                                                                  //OR of internal interrupts 1 and 2
        xDATA_RDY = LIS_313_CTRL_REG3_I2CFG(LIS_313_CTRL_REG3_I2CFG_DATARDY),    //Interrupt pin 2 is sourced from the data ready
                                                                                //bit of the status register
        xBOOT_RUN = LIS_313_CTRL_REG3_I2CFG(LIS_313_CTRL_REG3_I2CFG_BOOT)  //Interrupt pin 2 is sourced from the boot running
                                                                          //condition
    };
    
    //========================================  Control Register 4 (R/W)  ====================================================//
    //========================================================================================================================//
    const uint8_t CTRL_REG4 = 0x23;         //Control register 4 address
    
    //CTRL_REG4 Register Structure Definition
    #define LIS_313_CTRL_REG4_BDU_Pos 7
    #define LIS_313_CTRL_REG4_BDU (0x1 << LIS_313_CTRL_REG4_BDU_Pos)
    #define LIS_313_CTRL_REG4_BLE_Pos 6
    #define LIS_313_CTRL_REG4_BLE (0x1 << LIS_313_CTRL_REG4_BLE_Pos)
    #define LIS_313_CTRL_REG4_FS_Pos 4
    #define LIS_313_CTRL_REG4_FS_Msk (0x3 << LIS_313_CTRL_REG4_FS_Pos)
    #define LIS_313_CTRL_REG4_FS(value) LIS_313_CTRL_REG4_FS_Msk & ((value) << LIS_313_CTRL_REG4_FS_Pos)
    #define LIS_313_CTRL_REG4_FS_6G 0x0
    #define LIS_313_CTRL_REG4_FS_12G 0x1
    #define LIS_313_CTRL_REG4_FS_24G 0x3
    #define LIS_313_CTRL_REG4_STS_Pos 3
    #define LIS_313_CTRL_REG4_STS (0x1 << LIS_313_CTRL_REG4_STS_Pos)
    #define LIS_313_CTRL_REG4_ST_Pos 1
    #define LIS_313_CTRL_REG4_ST (0x1 << LIS_313_CTRL_REG4_ST_Pos)
    #define LIS_313_CTRL_REG4_SIM_Pos 0
    #define LIS_313_CTRL_REG4_SIM (0x1 << LIS_313_CTRL_REG4_SIM_Pos)
    
    
    const uint8_t BDU_BIT = LIS_313_CTRL_REG4_BDU;  //Block data update bit. When set block data update is enabled.
    const uint8_t BLE_BIT = LIS_313_CTRL_REG4_BLE;  //Big/little endian bit. When set data MSB at lower address.
    const uint8_t FULL_SCALE_MASK = LIS_313_CTRL_REG4_FS_Msk;   //Mask for the full scale selection bits
    
    enum fullScaleSetting_t {
        _6G = LIS_313_CTRL_REG4_FS(LIS_313_CTRL_REG4_FS_6G),    //+/- 6 G full scale
        _12G = LIS_313_CTRL_REG4_FS(LIS_313_CTRL_REG4_FS_12G),  //+/- 12 G full scale
        _24G = LIS_313_CTRL_REG4_FS(LIS_313_CTRL_REG4_FS_24G),  //+/- 24 G full scale
    };
    
    const uint8_t ST_SIGN_BIT = LIS_313_CTRL_REG4_STS;  //Self test sign bit. When set self test provides a negative input
    const uint8_t ST_BIT = LIS_313_CTRL_REG4_ST;        //Self test bit. When set self test is enabled.
    const uint8_t SIM_BIT = LIS_313_CTRL_REG4_SIM;      //SPI mode selection bit. When set SPI is configured for 3-wire mode.
                                                        //When cleard SPI is configured for 4-wire mode.
    
    //========================================  Control Register 5 (R/W)  ===================================================//
    //=======================================================================================================================//
    const uint8_t CTRL_REG5 = 0x24;         //Control register 5 address
    
    //CTRL_REG5 Register Structure Definition
    #define LIS_313_CTRL_REG5_TURNON_Pos 0
    #define LIS_313_CTRL_REG5_TURNON_Msk (0x3 << LIS_313_CTRL_REG5_TURNON_Pos)
    #define LIS_313_CTRL_REG5_TURNON(value) LIS_313_CTRL_REG5_TURNON_Msk & ((value) << LIS_313_CTRL_REG5_TURNON_Pos)
    #define LIS_313_CTRL_REG5_TURNON_DISABLE 0x0
    #define LIS_313_CTRL_REG5_TURNON_ENABLE 0x3
    
    //Sleep / Wake configuration
    const uint8_t TURN_ON_BITS_MASK = LIS_313_CTRL_REG5_TURNON_Msk; //Mask for the turn on sleep to wake mode configuration bits
    
    enum sleepToWakeMode_t {
        SLEEP_TO_WAKE_DISABLED = LIS_313_CTRL_REG5_TURNON(LIS_313_CTRL_REG5_TURNON_DISABLE),
        SLEEP_TO_WAKE_ENABLED = LIS_313_CTRL_REG5_TURNON(LIS_313_CTRL_REG5_TURNON_ENABLE)
    };
    
    //===============================  High Pass Filter Reset Register (Read Only)  =========================================//
    //=======================================================================================================================//
    const uint8_t HP_FILTER_RESET_REG = 0x25;       //High pass filter reset register address
    
    //=======================================  Reference Register (R/W)  ====================================================//
    //=======================================================================================================================//
    const uint8_t REFERENCE_REG = 0x26;             //Reference register address
    
    //===================================  Status Register (Read Only)  =====================================================//
    //=======================================================================================================================//
    const uint8_t STATUS_REG = 0x27;                //Status register address
    
    //Status Register Structure Definition
    #define LIS_313_STATUS_REG_ZYXOR_Pos 7
    #define LIS_313_STATUS_REG_ZYXOR (0x1 << LIS_313_STATUS_REG_ZYXOR_Pos)
    #define LIS_313_STATUS_REG_ZOR_Pos 6
    #define LIS_313_STATUS_REG_ZOR (0x1 << LIS_313_STATUS_REG_ZOR_Pos)
    #define LIS_313_STATUS_REG_YOR_Pos 5
    #define LIS_313_STATUS_REG_YOR (0x1 << LIS_313_STATUS_REG_YOR_Pos)
    #define LIS_313_STATUS_REG_XOR_Pos 4
    #define LIS_313_STATUS_REG_XOR (0x1 << LIS_313_STATUS_REG_XOR_Pos)
    #define LIS_313_STATUS_REG_ZYXDA_Pos 3
    #define LIS_313_STATUS_REG_ZYXDA (0x1 << LIS_313_STATUS_REG_ZYXDA_Pos)
    #define LIS_313_STATUS_REG_ZDA_Pos 2
    #define LIS_313_STATUS_REG_ZDA (0x1 << LIS_313_STATUS_REG_ZDA_Pos)
    #define LIS_313_STATUS_REG_YDA_Pos 1
    #define LIS_313_STATUS_REG_YDA (0x1 << LIS_313_STATUS_REG_YDA_Pos)
    #define LIS_313_STATUS_REG_XDA_Pos 0
    #define LIS_313_STATUS_REG_XDA (0x1 << LIS_313_STATUS_REG_XDA_Pos)
    
    
    const uint8_t ZYXOR_BIT = LIS_313_STATUS_REG_ZYXOR; //When set data in one of the channels has been over-written
    const uint8_t ZOR_BIT = LIS_313_STATUS_REG_ZOR;     //When set data in the Z channel has been over-written
    const uint8_t YOR_BIT = LIS_313_STATUS_REG_YOR;     //When set data in the Y channel has been over-written
    const uint8_t XOR_BIT = LIS_313_STATUS_REG_XOR;     //When set data in the X channel has been over-written
    const uint8_t ZYXDA_BIT = LIS_313_STATUS_REG_ZYXDA; //When set a new set of data is ready to be read
    const uint8_t ZDA_BIT = LIS_313_STATUS_REG_ZDA;     //When set new data is ready in the Z channel
    const uint8_t YDA_BIT = LIS_313_STATUS_REG_YDA;     //When set new data is ready in the Y channel
    const uint8_t XDA_BIT = LIS_313_STATUS_REG_XDA;     //When set new data is ready in the X channel
    
    //=======================  Accelerometer Data Output Registers (Read Only)  =============================================//
    //=======================================================================================================================//
    const uint8_t OUT_X_L_REG = 0x28;       //X-axis low data byte register address
    const uint8_t OUT_X_H_REG = 0x29;       //X-axis high data byte register address
    const uint8_t OUT_Y_L_REG = 0x2A;       //Y-axis low data byte register address
    const uint8_t OUT_Y_H_REG = 0x2B;       //Y-axis high data byte register address
    const uint8_t OUT_Z_L_REG = 0x2C;       //Z-axis low data byte register address
    const uint8_t OUT_Z_H_REG = 0x2D;       //Z-axis high data byte register address
    
    //Accelerometer sensitivity (from data sheet Table 3)
    const float _2G_SENSITIVITY = 0.003;     //3 mG/LSB
    const float _6G_SENSITIVITY = 0.006;     //6 mG/LSB
    const float _12_SENSITIVITY = 0.012;     //12 mG/LSB
    
    //=============================  Interrupt Configuration Registers (R/W)  ==============================================//
    //======================================================================================================================//
    const uint8_t INT1_CFG_REG = 0x30;      //Interrupt 1 configuration register address
    const uint8_t INT2_CFG_REG = 0x34;      //Interrupt 2 configuration register address
    
    //Interrupt Configuration Register Structure Definition
    #define LIS_313_INTR_CONFIG_REG_AOI6D_Pos 6
    #define LIS_313_INTR_CONFIG_REG_AOI6D_Msk (0x3 << LIS_313_INTR_CONFIG_REG_AOI6D_Pos)
    #define LIS_313_INTR_CONFIG_REG_AOI6D(value) LIS_313_INTR_CONFIG_REG_AOI6D_Msk & ((value) << LIS_313_INTR_CONFIG_REG_AOI6D_Pos)
    #define LIS_313_INTR_CONFIG_REG_AOI6D_OREVENTS 0x00
    #define LIS_313_INTR_CONFIG_REG_AOI6D_6DMOVEMENT 0x01
    #define LIS_313_INTR_CONFIG_REG_AOI6D_ANDEVENTS 0x2
    #define LIS_313_INTR_CONFIG_REG_AOI6D_6DPOSITION 0x3
    #define LIS_313_INTR_CONFIG_REG_ZHIE_Pos 5
    #define LIS_313_INTR_CONFIG_REG_ZHIE (0x1 << LIS_313_INTR_CONFIG_REG_ZHIE_Pos)
    #define LIS_313_INTR_CONFIG_REG_ZLIE_Pos 4
    #define LIS_313_INTR_CONFIG_REG_ZLIE (0x1 << LIS_313_INTR_CONFIG_REG_ZLIE_Pos)
    #define LIS_313_INTR_CONFIG_REG_YHIE_Pos 3
    #define LIS_313_INTR_CONFIG_REG_YHIE (0x1 << LIS_313_INTR_CONFIG_REG_YHIE_Pos)
    #define LIS_313_INTR_CONFIG_REG_YLIE_Pos 2
    #define LIS_313_INTR_CONFIG_REG_YLIE (0x1 << LIS_313_INTR_CONFIG_REG_YLIE_Pos)
    #define LIS_313_INTR_CONFIG_REG_XHIE_Pos 1
    #define LIS_313_INTR_CONFIG_REG_XHIE (0x1 << LIS_313_INTR_CONFIG_REG_XHIE_Pos)
    #define LIS_313_INTR_CONFIG_REG_XLIE_Pos 0
    #define LIS_313_INTR_CONFIG_REG_XLIE (0x1 << LIS_313_INTR_CONFIG_REG_XLIE_Pos)
    
    
    const uint8_t INT_MODE_MASK = LIS_313_INTR_CONFIG_REG_AOI6D_Msk;    //Mask for the interrupt configuration bits AOI and 6D
    
    enum interruptModeSetting_t {
        OR_INT_EVENTS = LIS_313_INTR_CONFIG_REG_AOI6D(LIS_313_INTR_CONFIG_REG_AOI6D_OREVENTS), //Logical OR of interrupt events
        SIX_DIR_MOVEMENT = LIS_313_INTR_CONFIG_REG_AOI6D(LIS_313_INTR_CONFIG_REG_AOI6D_6DMOVEMENT), //6 direction movement
                                                                                                    //recognition
        AND_INT_EVENTS = LIS_313_INTR_CONFIG_REG_AOI6D(LIS_313_INTR_CONFIG_REG_AOI6D_ANDEVENTS), //Logical AND of interrupt events
        SIX_DIR_POSITION = LIS_313_INTR_CONFIG_REG_AOI6D(LIS_313_INTR_CONFIG_REG_AOI6D_6DPOSITION)  //6 direction position
                                                                                                    //recognition
    };
    
    const uint8_t ZHIE_BIT = LIS_313_INTR_CONFIG_REG_ZHIE;  //When set enable interrupt request on Z-axis acceleration greater than
                                                            //threshold
    const uint8_t ZLIE_BIT = LIS_313_INTR_CONFIG_REG_ZLIE;  //When set enable interrupt request on Z-axis acceleration less than
                                                            //threshold
    const uint8_t YHIE_BIT = LIS_313_INTR_CONFIG_REG_YHIE;  //When set enable interrupt request on Y-axis acceleration greater than
                                                            //threshold
    const uint8_t YLIE_BIT = LIS_313_INTR_CONFIG_REG_YLIE;  //When set enable interrupt request on Y-axis acceleration less than
                                                            //threshold
    const uint8_t XHIE_BIT = LIS_313_INTR_CONFIG_REG_XHIE;  //When set enable interrupt request on X-axis acceleration greater than
                                                            //threshold
    const uint8_t XLIE_BIT = LIS_313_INTR_CONFIG_REG_XLIE;  //When set enable interrupt request on X-axis acceleration less than
                                                            //threshold
    
    //==============================  Interrupt Source Registers (Read Only)  ===================================================//
    //===========================================================================================================================//
    const uint8_t INT1_SRC_REG = 0x31;      //Interrupt 1 source register address
    const uint8_t INT2_SRC_REG = 0x35;      //Interrupt 2 source register address
    
    //Interrupt Source Register Structure Definition
    #define LIS_313_INTR_SRC_REG_IA_Pos 6
    #define LIS_313_INTR_SRC_REG_IA (0x1 << LIS_313_INTR_SRC_REG_IA_Pos)
    #define LIS_313_INTR_SRC_REG_ZH_Pos 5
    #define LIS_313_INTR_SRC_REG_ZH (0x1 << LIS_313_INTR_SRC_REG_ZH_Pos)
    #define LIS_313_INTR_SRC_REG_ZL_Pos 4
    #define LIS_313_INTR_SRC_REG_ZL (0x1 << LIS_313_INTR_SRC_REG_ZL_Pos)
    #define LIS_313_INTR_SRC_REG_YH_Pos 3
    #define LIS_313_INTR_SRC_REG_YH (0x1 << LIS_313_INTR_SRC_REG_YH_Pos)
    #define LIS_313_INTR_SRC_REG_YL_Pos 2
    #define LIS_313_INTR_SRC_REG_YL (0x1 << LIS_313_INTR_SRC_REG_YL_Pos)
    #define LIS_313_INTR_SRC_REG_XH_Pos 1
    #define LIS_313_INTR_SRC_REG_XH (0x1 << LIS_313_INTR_SRC_REG_XH_Pos)
    #define LIS_313_INTR_SRC_REG_XL_Pos 0
    #define LIS_313_INTR_SRC_REG_XL (0x1 << LIS_313_INTR_SRC_REG_XL_Pos)
    
    
    const uint8_t AXIS_BITS_MASK = 0x3F;    //Mask for the axis interrupt source bits
    const uint8_t IA_BIT = LIS_313_INTR_SRC_REG_IA; //Master interrupt bit. When set one or more interrupts have been generated.
    const uint8_t ZH_BIT = LIS_313_INTR_SRC_REG_ZH; //When set a Z-axis high event has occurred.
    const uint8_t ZL_BIT = LIS_313_INTR_SRC_REG_ZL; //When set a Z-axis low event has occurred.
    const uint8_t YH_BIT = LIS_313_INTR_SRC_REG_YH; //When set a Y-axis high event has occurred.
    const uint8_t YL_BIT = LIS_313_INTR_SRC_REG_YL; //When set a Y-axis low event has occurred.
    const uint8_t XH_BIT = LIS_313_INTR_SRC_REG_XH; //When set a X-axis high event has occurred.
    const uint8_t XL_BIT = LIS_313_INTR_SRC_REG_XL; //When set a X-axis low event has occurred.
    
    //======================  Interrupt Threshold and Duration Configuration Registers (R/W)  ===========================//
    //===================================================================================================================//
    const uint8_t INT1_THS_REG = 0x32;      //Interrupt 1 threshold register address
    const uint8_t INT1_DURATION_REG = 0x33; //Interrupt 1 duration register address
    const uint8_t INT2_THS_REG = 0x36;      //Interrupt 1 threshold register address
    const uint8_t INT2_DURATION_REG = 0x37; //Interrupt 1 duration register address
    
    //I2C Address
    const uint8_t LIS331_I2C_ADDRESS  = 0x18;      //Address if SAO pin is connected to ground
    //const uint8_t LIS331_I2C_ADDRESS  = 0x19;    //Address if SAO pin is connected to supply voltage
    
    //I2C Multi-Register Read Bit
    const uint8_t I2C_MULT_REG_READ = 0x80;        //Bit set when multiple registers are being read
    
    //SPI Multi-Register Access Bit
    const uint8_t SPI_MULT_REG_ACCESS = 0x40;  //Bit set in first word written to allow register address increment
    
    //SPI Read/Write Bit
    const uint8_t SPI_READ = 0x80;  //Bit set for SPI read operation
    
    //SPI Clock Speed
    const uint32_t SPI_CLOCK = 10000000;

    /*********************************************************************************/
    /*     LIS331Driver Abstract Base Class Declaration                              */
    /*********************************************************************************/
    //Base class for the LIS331 driver object
    //The begin() method must be implemented by derived classes
    /*********************************************************************************/
    class LIS331Driver
    {
    public:
        //Constructor
        LIS331Driver();
        
        //Destructor
        virtual ~LIS331Driver(){}
        
        //Methods for initializing and configuring the LIS331 chip and the driver object
        void initialize(powerMode_t mode, outputDataRate_t rate, enableAxisSettings_t axis, fullScaleSetting_t range);
        void initIntrInterface(intrPin1Config_t intr1Pin = INT_1_SRC,
                               intrPin2Config_t intr2Pin = INT_2_SRC,
                               bool activeLow = false, bool openDrain = false,
                               bool inter1Latched = false, bool inter2Latched = false);
        void setMode(powerMode_t mode);
        void setODR(outputDataRate_t rate);
        void enableAxis(enableAxisSettings_t axis);
        void refreshInternalRegisters();
        void setBlockDataUpdate(bool set);
        void setBigEndian(bool bigEndian);
        void setMeasurementRange(fullScaleSetting_t range);
        
        //Methods for reading the acceleration data
        float readXAxisAcceleration();
        float readYAxisAcceleration();
        float readZAxisAcceleration();
        void readAcceleration(float accelArray[]);
        
        //Methods for checking status
        bool isDataReady();
        bool isDataOverrun();
        uint8_t readStatusRegister();
        
        //Methods for configuring and checking interrupts
        void enableInterrupt1(interruptModeSetting_t mode, uint8_t enableAxis, float threshold, float duration);
        void enableInterrupt2(interruptModeSetting_t mode, uint8_t enableAxis, float threshold, float duration);
        uint8_t getInterrupt1Source();
        uint8_t getInterrupt2Source();
        void disableInterrupt1();
        void disableInterrupt2();
        
        //Methods for configuring and controlling the high pass filter
        void resetHPF();
        void configureHPF(bool bypass, hp_Filter_Coeficient_t coef, bool refMode = false, float reference = 0);
        void setHPFMode(bool referenceMode);
        void setFilterDataSelect(bool byPassed);
        void enableHPFInt1(bool enable);
        void enableHPFInt2(bool enable);
        void setHPFCoeficients(hp_Filter_Coeficient_t coef);
        
        
        //Methods for controlling self test
        void setSelfTestMinus(bool stMinus);
        void enableSelfTest(bool selfTest);
        
        //Methods supporting calibration
        void setOffsetCalibration(float xOffset, float yOffset, float zOffset);
        void setGainCalibration(float xCal, float yCal, float zCal);
        
        //Low level functions. All pure virtual, must be implemented in derived classes
        virtual uint8_t readRegister(uint8_t reg) const = 0;
        virtual int16_t readAccelRegisterPair(uint8_t reg) const = 0;
        virtual void readAccelRegisters(int16_t accelArray[]) const = 0;
        virtual void writeRegister(uint8_t reg, uint8_t value) const = 0;
        
    private:
        
        uint8_t _mgPerLSB;      //The current value of an LSB in the measured acceleration
                                //This is 3, 6, or 12 mg/LSB
        float _ODR;             //The current output data rate in Hz
        float _offset[3];       //Offset calibration values
        float _gainCal[3];      //Gain calibration values
    };
    
    
    /*********************************************************************************/
    /*     LIS331_I2C_Driver Class Declaration                                       */
    /*********************************************************************************/
    //Derived class for an LIS331 driver object that uses I2C for communications
    /*********************************************************************************/
    class LIS331_I2C_Driver : public LIS331Driver
    {
    public:
        //Constructor
        LIS331_I2C_Driver(uint8_t i2cAddress);
        
        void begin(TwoWire * i2cDriver = nullptr);
        
        //Low level functions
        virtual uint8_t readRegister(uint8_t reg) const;
        virtual int16_t readAccelRegisterPair(uint8_t reg) const;
        virtual void readAccelRegisters(int16_t accelArray[]) const;
        virtual void writeRegister(uint8_t reg, uint8_t value) const;
    private:
        //Private data members
        uint8_t _I2C_Address;
        TwoWire * _I2C_Driver;
        
    };
    
    
    /*********************************************************************************/
    /*     LIS331_SPI_Driver Class Declaration                                       */
    /*********************************************************************************/
    //Derived class for an LIS331 driver object that uses SPI for communications.
    //LIS331 SPI parameters
    //Max SPI clock rate: 10 MHz
    //Data Transmission: MSB First
    //Clock Idle Polarity: High
    //Clock Phase: Sample on rising (trailing) edge, clock data out on falling (leading) edge
    //SPI Mode: 3
    /*********************************************************************************/
    class LIS331_SPI_Driver : public LIS331Driver
    {
    public:
        //Constructor
        LIS331_SPI_Driver(uint8_t csPin);
        void begin(SPIClass * spiDriver);
        
        //Low level functions
        virtual uint8_t readRegister(uint8_t reg) const;
        virtual int16_t readAccelRegisterPair(uint8_t reg) const;
        virtual void readAccelRegisters(int16_t accelArray[]) const;
        virtual void writeRegister(uint8_t reg, uint8_t value) const;
        
    private:
        //Private data members
        uint8_t _CS_Pin;
        SPIClass * mSPI_Driver;
      
        
    };
    
} //End Namespace LIS331
#endif // LIS331_Driver
