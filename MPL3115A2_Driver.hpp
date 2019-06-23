//MPL3115A2 Pressure Sensor Driver
//Verision 1.0
//May 13, 2019
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


#ifndef MPL3115A2_Driver_hpp
#define MPL3115A2_Driver_hpp

#include "Arduino.h"
#include <Wire.h>

namespace MPL3115A2
{
    /****************************** Data Type and Constant Defintions ******************************/
    
    //===============================   Status Register (Read Only)   =============================//
    //=============================================================================================//
    const uint8_t STATUS_REG = 0x00;      //Status Register address
    const uint8_t DR_STATUS_REG = 0x06;   //Data Ready Status Register address
    
    //STATUS_REG / DR_STATUS_REG Register Structure Definition
    #define MPL3115A2_STATUS_PTOW_Pos 7
    #define MPL3115A2_STATUS_PTOW (0x1 << MPL3115A2_STATUS_PTOW_Pos)
    #define MPL3115A2_STATUS_POW_Pos 6
    #define MPL3115A2_STATUS_POW (0x1 << MPL3115A2_STATUS_POW_Pos)
    #define MPL3115A2_STATUS_TOW_Pos 5
    #define MPL3115A2_STATUS_TOW (0x1 << MPL3115A2_STATUS_TOW_Pos)
    #define MPL3115A2_STATUS_PTDR_Pos 3
    #define MPL3115A2_STATUS_PTDR (0x1 << MPL3115A2_STATUS_PTDR_Pos)
    #define MPL3115A2_STATUS_PDR_Pos 2
    #define MPL3115A2_STATUS_PDR (0x1 << MPL3115A2_STATUS_PDR_Pos)
    #define MPL3115A2_STATUS_TDR_Pos 1
    #define MPL3115A2_STATUS_TDR (0x1 << MPL3115A2_STATUS_TDR_Pos)
    
    const uint8_t PTOW_BIT = MPL3115A2_STATUS_PTOW; //Pressure/temperature data overwrite bit
    const uint8_t POW_BIT = MPL3115A2_STATUS_POW;   //Pressure data overwrite bit
    const uint8_t TOW_BIT = MPL3115A2_STATUS_TOW;   //Temperature data overwirte bit
    const uint8_t PTDR_BIT = MPL3115A2_STATUS_PTDR; //Pressure/temperature data ready bit
    const uint8_t PDR_BIT = MPL3115A2_STATUS_PDR;   //Pressure data ready bit
    const uint8_t TDR_BIT = MPL3115A2_STATUS_TDR;   //Temperature data ready bit
    
    
    //=========================   Pressure Data Output Registers (Read Only)   ====================//
    //=============================================================================================//
    const uint8_t OUT_P_MSB_REG = 0x01;      //Pressure Data MSB Register address
    const uint8_t OUT_P_CSB_REG = 0x02;      //Pressure Data CSB Register address
    const uint8_t OUT_P_LSB_REG = 0x03;      //Pressure Data LSB Register address
    
    //OUT_P_LSB_REG Register Structure Defintion
    #define MPL3115A2_OUT_P_LSB_DATA_Pos 4
    #define MPL3115A2_OUT_P_LSB_DATA_Msk (0xF << MPL3115A2_OUT_P_LSB_DATA_Pos)
    
    const uint8_t PRES_DATA_LSB_MASK = MPL3115A2_OUT_P_LSB_DATA_Msk; //Mask for the data LSB
    
    //=======================   Temperature Data Output Registers (Read Only)   ===================//
    //=============================================================================================//
    const uint8_t OUT_T_MSB_REG = 0x04;      //Temperature Data MSB Register address
    const uint8_t OUT_T_LSB_REG = 0x05;      //Temperature Data LSB Register address
    
    //OUT_T_LSB_REG Register Structure Definition
    #define MPL3115A2_OUT_T_LSB_DATA_Pos 4
    #define MPL3115A2_OUT_T_LSB_DATA_Msk (0xF << MPL3115A2_OUT_T_LSB_DATA_Pos)
    
    const uint8_t TEMP_DATA_LSB_MASK = MPL3115A2_OUT_T_LSB_DATA_Msk;    //Mask for data LSB
    
    //=========================   Delta Pressure Data Registers (Read Only)   =====================//
    //=============================================================================================//
    const uint8_t OUT_P_DELTA_MSB_REG = 0x07;      //Pressure Data Delta MSB Register address
    const uint8_t OUT_P_DELTA_CSB_REG = 0x08;      //Pressure Data Delta CSB Register address
    const uint8_t OUT_P_DELTA_LSB_REG = 0x09;      //Pressure Data Delta LSB Register address
    
    //OUT_P_DELTA_LSB_REG Register Structure Definition
    #define MPL3115A2_OUT_P_DELTA_LSB_DATA_Pos 4
    #define MPL3115A2_OUT_P_DELTA_LSB_DATA_Msk (0xF << MPL3115A2_OUT_P_DELTA_LSB_DATA_Pos)
    
    const uint8_t DELTA_PRES_DATA_LSB_MASK = MPL3115A2_OUT_P_DELTA_LSB_DATA_Msk;
    
    //=======================   Delta Temperature Data Registers (Read Only)   ====================//
    //=============================================================================================//
    const uint8_t OUT_T_DELTA_MSB_REG = 0x0A;      //Temp Data Delta MSB Register address
    const uint8_t OUT_T_DELTA_LSB_REG = 0x0B;      //Temp Data Delta LSB Register address
    
    //OUT_T_DELTA_LSB_REG Register Structure Definition
    #define MPL3115A2_OUT_T_DELTA_LSB_DATA_Pos 4
    #define MPL3115A2_OUT_T_DELTA_LSB_DATA_Msk (0xF << MPL3115A2_OUT_T_DELTA_LSB_DATA_Pos)
    
    const uint8_t DELTA_TEMP_DATA_LSB_MASK = MPL3115A2_OUT_T_DELTA_LSB_DATA_Msk;
    
    //==============================   Who Am I Register (Read Only)   ============================//
    //=============================================================================================//
    const uint8_t WHO_AM_I_REG = 0x0C;      //Who Am I Register address
    
    const uint8_t WHO_AM_I = 0xC4;  //ID constant for MPL3115A2 chip
    
    //=============================   FIFO Status Register (Read Only)   ==========================//
    //=============================================================================================//
    const uint8_t F_STATUS_REG = 0x0D;      //FIFO Status Register address
    
    //F_STATUS_REG Register Structure Definition
    #define MPL3115A2_F_STATUS_F_OVF_Pos 7
    #define MPL3115A2_F_STATUS_F_OVF (0x01 << MPL3115A2_F_STATUS_F_OVF_Pos)
    #define MPL3115A2_F_STATUS_F_WMRK_FLAG_Pos 6
    #define MPL3115A2_F_STATUS_F_WMRK_FLAG (0x01 << MPL3115A2_F_STATUS_F_WMRK_FLAG_Pos)
    #define MPL3115A2_F_STATUS_F_CNT_Pos 0
    #define MPL3115A2_F_STATUS_F_CNT_Msk (0x1F << MPL3115A2_F_STATUS_F_CNT_Pos)
    
    const uint8_t FIFO_OVERFLOW_BIT = MPL3115A2_F_STATUS_F_OVF;
    const uint8_t FIFO_WATERMARK_FLAG = MPL3115A2_F_STATUS_F_WMRK_FLAG;
    const uint8_t FIFO_SAMPLE_COUNT_MASK = MPL3115A2_F_STATUS_F_CNT_Msk;
    
    //==============================   FIFO Data Register (Read Only)   ===========================//
    //=============================================================================================//
    const uint8_t F_DATA_REG = 0x00E      //FIFO 8-Bit Data Register address
    
    //===============================   FIFO Setup Register (R/W)   ===============================//
    //=============================================================================================//
    const uint8_t F_SETUP_REG = 0x0F;      //FIFO Setup Register address
    
    //F_SETUP_REG Register Structure Definition
    #define MPL3115A2_F_SETUP_F_MODE_Pos 6
    #define MPL3115A2_F_SETUP_F_MODE_Msk (0x3 << MPL3115A2_F_SETUP_F_MODE_Pos)
    #define MPL3115A2_F_SETUP_F_WMRK_Pos 0
    #define MPL3115A2_F_SETUP_F_WMRK_Msk (0x3F << MPL3115A2_F_SETUP_F_WMRK_Pos)
    
    //=============================   Time Delay Register (Read Only)   ===========================//
    //=============================================================================================//
    const uint8_t TIME_DLY_REG = 0x10;      //Time Delay Register address
    
    //============================   System Mode Register (Read Only)   ===========================//
    //=============================================================================================//
    const uint8_t SYSMOD_REG = 0x11;      //System Mode Register address
    
    //SYSMOD_REG Register Structure Definition
    #define MPL3115A2_SYSMOD_SYSMOD_Pos 0
    #define MPL3115A2_SYSMOD_SYSMOD (0x1 << MPL3115A2_SYSMOD_SYSMOD_Pos)
    
    const uint8_t SYSTEM_MODE_BIT = MPL3115A2_SYSMOD_SYSMOD;    //When set mode = Active
                                                                //When cleared mode = Standby
    
    //=========================   Interrupt Source Register (Read Only)   =========================//
    //=============================================================================================//
    const uint8_t INT_SOURCE_REG = 0x12;      //Interrupt Source Register address
    
    //INT_SOURCE_REG Regiseter Structure Definition
    #define MPL3115A2_INT_SOURCE_SRC_DRDY_Pos 7
    #define MPL3115A2_INT_SOURCE_SRC_DRDY (0x1 << MPL3115A2_INT_SOURCE_SRC_DRDY_Pos)
    #define MPL3115A2_INT_SOURCE_SRC_FIFO_Pos 6
    #define MPL3115A2_INT_SOURCE_SRC_FIFO (0x1 << MPL3115A2_INT_SOURCE_SRC_FIFO_Pos)
    #define MPL3115A2_INT_SOURCE_SRC_PW_Pos 5
    #define MPL3115A2_INT_SOURCE_SRC_PW (0x1 << MPL3115A2_INT_SOURCE_SRC_PW_Pos)
    #define MPL3115A2_INT_SOURCE_SRC_TW_Pos 4
    #define MPL3115A2_INT_SOURCE_SRC_TW (0x1 << MPL3115A2_INT_SOURCE_SRC_TW_Pos)
    #define MPL3115A2_INT_SOURCE_SRC_PTH_Pos 3
    #define MPL3115A2_INT_SOURCE_SRC_PTH (0x1 << MPL3115A2_INT_SOURCE_SRC_PTH_Pos)
    #define MPL3115A2_INT_SOURCE_SRC_TTH_Pos 2
    #define MPL3115A2_INT_SOURCE_SRC_TTH (0x1 << MPL3115A2_INT_SOURCE_SRC_YYH_Pos)
    #define MPL3115A2_INT_SOURCE_SRC_PCHG_Pos 1
    #define MPL3115A2_INT_SOURCE_SRC_PCHG (0x1 << MPL3115A2_INT_SOURCE_SRC_PCHG_Pos)
    #define MPL3115A2_INT_SOURCE_SRC_TCHG_Pos 0
    #define MPL3115A2_INT_SOURCE_SRC_TCHG (0x1 << MPL3115A2_INT_SOURCE_SRC_TCHG_Pos)
    
    //See Table 32 in MPL3115A2 Data Sheet
    const uint8_t DATA_READY_INTR = MPL3115A2_INT_SOURCE_SRC_DRDY;
    const uint8_t FIFO_INTR = MPL3115A2_INT_SOURCE_SRC_FIFO;
    const uint8_t PRESSURE_INTR = MPL3115A2_INT_SOURCE_SRC_PW;
    const uint8_t TEMP_INTR = MPL3115A2_INT_SOURCE_SRC_TW;
    const uint8_t PRESSURE_THRESHOLD_INTR = MPL3115A2_INT_SOURCE_SRC_PTH;
    const uint8_t TEMP_THRESHOLD_INTR = MPL3115A2_INT_SOURCE_SRC_TTH;
    const uint8_t PRESSURE_CHANGE_INTR = MPL3115A2_INT_SOURCE_SRC_PCHG;
    const uint8_t TEMP_CHANGE_INTR = MPL3115A2_INT_SOURCE_SRC_TCHG;
    
    //==============   Pressure/Temperature Data Configuration Register (R/W)   ===================//
    //=============================================================================================//
    const uint8_t PT_DATA_CFG_REG = 0x13;      //Press/Temp Data Configuration Register address
    
    //PT_DATA_CFG_REG Register Structure Definition
    #define MPL3115A2_PT_DATA_CFG_DREM_Pos 2
    #define MPL3115A2_PT_DATA_CFG_DREM (0x1 << MPL3115A2_PT_DATA_DFG_DREM_Pos)
    #define MPL3115A2_PT_DATA_CFG_PDEFE_Pos 1
    #define MPL3115A2_PT_DATA_CFG_PDEFE (0x1 << MPL3115A2_PT_DATA_DFG_PDEFE_Pos)
    #define MPL3115A2_PT_DATA_CFG_TDEFE_Pos 0
    #define MPL3115A2_PT_DATA_CFG_TDEFE (0x1 << MPL3115A2_PT_DATA_DFG_TDEFE_Pos)
    
    //See Table 34 in MPL3115A2 Data Sheet
    const uint8_t DATA_READY_EVENT_FLAG = MPL3115A2_PT_DATA_CFG_DREM;
    const uint8_t ENABLE_PRESSURE_EVENT_FLAG = MPL3115A2_PT_DATA_CFG_PDEFE;
    const uint8_t ENABLE_TEMP_EVENT_FLAG = MPL3115A2_PT_DATA_CFG_TDEFE;
    
    //==================   Barometric Pressure Input MSB & LSB Registers (R/W)   ==================//
    //=============================================================================================//
    //These registers hold the barometric pressure eqivalent to local sea level (default = 101,326 Pa).
    //Value is expressed as an unsigned 16-bit value with the LSB = 2 Pa.
    //See Paragraph 14.13 in MPL3115A2 Data Sheet.
    const uint8_t BAR_IN_MSB_REG = 0x14;    //Barometric Pressure Input MSB Register address
    const uint8_t BAR_IN_LSB_REG = 0x15;    //Barometric Pressure Input LSB Register address
    
    const SEA_LVL_BARO_PRESS_SF = 2;    //2 Pa/LSB
    
    //========================    Pressure Target MSB & LSB Registers (R/W)   =====================//
    //=============================================================================================//
    //These registers hold the target pressure value which is used in conjuction with the pressure
    //window setting. In altitude mode the value is a 16-bit signed integer in meters. In barometer
    //mode the pressure is expressed as an unsigned 16-bit value with a scale factor of 2 Pa per LSB.
    //See Paragraph 14.14 of the MPL3115A2 Data Sheet.
    const uint8_t P_TGT_MSB_REG = 0x16;     //Pressure Target MSB Register address
    const uint8_t P_TGT_LSB_REG = 0x17;     //Pressure Target LSB Register address
    
    const TARGET_PRESS_SF = 2;  //2 Pa/LSB
    
    
    //============================    Temperature Target Register (R/W)   =========================//
    //=============================================================================================//
    //This register holds the target temperature value which is used in conjuction with the
    //temperature window setting. The temperature is expressed as an 8-bit signed value in degree C.
    //See Paragraph 14.15 of the MPL3115A2 Data Sheet.
    const uint8_t T_TGT_REG = 0x18;      //Temperature Target Register address
    
    //==================    Pressure/Altitude Window MSB & LSB Registers (R/W)   ==================//
    //=============================================================================================//
    //These registers are used to provide the pressure/altitude window comparison value.
    //See Paragraph 14.16 of the MPL3115A2 Data Sheet.
    const uint8_t P_WND_MSB_REG = 0x19;      //Pressure/Altitude Window MSB Register address
    const uint8_t P_WND_LSB_REG = 0x1A;      //Pressure/Altitude Window LSB Register address
    
    //============================    Temperature Window Register (R/W)   =========================//
    //=============================================================================================//
    //This register is used to provide the temperature window comparision value.
    //See Paragraph 14.17 of the MPL3115A2 Data Sheet.
    const uint8_t T_WND_REG = 0x1B;      //Temperature Window Register address
    
    //==========================    Minimum Pressure Data Registers (R/W)   =======================//
    //=============================================================================================//
    //These registers store the minimum measured pressure/altitude value.
    //See Paragraph 14.18 of the MPL3115A2 Data Sheet
    const uint8_t P_MIN_MSB_REG = 0x1C;      //Minimum Pressure Data MSB Register address
    const uint8_t P_MIN_CSB_REG = 0x1D;      //Minimum Pressure Data CSB Register address
    const uint8_t P_MIN_LSB_REG = 0x1E;      //Minimum Pressure Data LSB Register address
    
    //P_MIN_LSB_REG Register Structure Defintion
    #define MPL3115A2_P_MIN_LSB_DATA_Pos 4
    #define MPL3115A2_P_MIN_LSB_DATA_Msk (0xF << MPL3115A2_P_MIN_LSB_DATA_Pos)
    
    const MIN_PRESSURE_LSB_MASK = MPL3115A2_P_MIN_LSB_DATA_Msk;
    
    //=========================    Minimum Temperature Data Registers (R/W)   =====================//
    //=============================================================================================//
    //These registers store the minimum measured temperature value.
    //See Paragraph 14.20 of the MPL3115A2 Data Sheet
    const uint8_t T_MIN_MSB_REG = 0x1F;      //Minimum Temperature Data MSB Register address
    const uint8_t T_MIN_LSB_REG = 0x20;      //Minimum Temperature Data LSB Register address
    
    //T_MIN_LSB_REG Register Structure Definition
    #define MPL3115A2_T_MIN_LSB_DATA_Pos 4
    #define MPL3115A2_T_MIN_LSB_DATA_Msk (0xF << MPL3115A2_T_MIN_LSB_DATA_Pos)
    
    const MIN_TEMP_LSB_MASK = MPL3115A2_T_MIN_LSB_DATA_Msk;
    
    //==========================    Maximum Pressure Data Registers (R/W)   =======================//
    //=============================================================================================//
    //These registers store the maximum measured pressure value.
    //See Paragraph 14.19 of the MPL3115A2 Data Sheet.
    const uint8_t P_MAX_MSB_REG = 0x21;      //Maximum Pressure Data MSB Register address
    const uint8_t P_MAX_CSB_REG = 0x22;      //Maximum Pressure Data CSB Register address
    const uint8_t P_MAX_LSB_REG = 0x23;      //Maximum Pressure Data LSB Register address
    
    //P_MAX_LSB_REG Register Structure Definition
    #define MPL3115A2_P_MAX_LSB_DATA_Pos 4
    #define MPL3115A2_P_MAX_LSB_DATA_Msk (0xF << MPL3115A2_P_MAX_LSB_DATA_Pos)
    
    const MAX_PRESSURE_LSB_MASK = MPL3115A2_P_MAX_LSB_DATA_Msk;
    
    //=========================    Maximum Temperature Data Registers (R/W)   =====================//
    //=============================================================================================//
    //These registers store the maximum measured temperature value.
    //See Paragraph 14.21 of the MPL3115A2 Data Sheet
    const uint8_t T_MAX_MSB_REG = 0x24;      //Maximum Temperature Data MSB Register address
    const uint8_t T_MAX_LSB_REG = 0x25;      //Maximum Temperature Data LSB Register address
    
    //T_MAX_LSB_REG Register Structure Definition
    #define MPL3115A2_T_MAX_LSB_DATA_Pos 4
    #define MPL3115A2_T_MAX_LSB_DATA_Msk (0xF << MPL3115A2_T_MAX_LSB_DATA_Pos)
    
    const uint8_t MAX_TEMP_LSB_MASK = MPL3115A2_T_MAX_LSB_DATA_Msk;
    
    //==============================    Control Register 1 (R/W)   ================================//
    //=============================================================================================//
    const uint8_t CTRL_REG1 = 0x26;      //Control Register 1 address
    
    //CTRL_REG1 Register Structure Definition
    #define MPL3115A2_CTRL_REG1_ALT_Pos 7
    #define MPL3115A2_CTRL_REG1_ALT (0x1 << MPL3115A2_CTRL_REG1_ALT_Pos)
    #define MPL3115A2_CTRL_REG1_OS_Pos 3
    #define MPL3115A2_CTRL_REG1_OS_Msk (0x7 << MPL3115A2_CTRL_REG1_OS_Pos)
    #define MPL3115A2_CTRL_REG1_RST_Pos 2
    #define MPL3115A2_CTRL_REG1_RST (0x1 << MPL3115A2_CTRL_REG1_RST_Pos)
    #define MPL3115A2_CTRL_REG1_OST_Pos 1
    #define MPL3115A2_CTRL_REG1_OST (0x1 << MPL3115A2_CTRL_REG1_OST_Pos)
    #define MPL3115A2_CTRL_REG1_SBYB_Pos 0
    #define MPL3115A2_CTRL_REG1_SBYB (0x1 << MPL3115A2_CTRL_REG1_SBYB_Pos)
    
    //See Table 45 in MPL3115A2 Data Sheet
    const uint8_t ALTIMETER_MODE_BIT = MPL3115A2_CTRL_REG1_ALT;
    const uint8_t SOFTWARE_RESET_BIT = MPL3115A2_CTRL_REG1_RST;
    const uint8_t ONESHOT_MODE_BIT = MPL3115A2_CTRL_REG1_OST;
    const uint8_t STANDBY_MODE_BIT = MPL3115A2_CTRL_REG1_SBYB;
    
    //See Table 46 of MPL3115A2 Data Sheet
    enum overSample_t {
        OS_RATIO_1,
        OS_RATIO_2,
        OS_RATIO_4,
        OS_RATIO_8,
        OS_RATIO_16,
        OS_RATIO_32,
        OS_RATIO_64,
        OS_RATIO_128,
    };
    
    
    //==============================    Control Register 2 (R/W)   ================================//
    //=============================================================================================//
    const uint8_t CTRL_REG2 = 0x27;      //Control Register 2 address
    
    //CTRL_REG2 Register Structure Definition
    #define MPL3115A2_CTRL_REG2_LOAD_OUTPUT_Pos 5
    #define MPL3115A2_CTRL_REG2_LOAD_OUTPUT (0x1 << MPL3115A2_CTRL_REG2_LOAD_OUTPUT_Pos)
    #define MPL3115A2_CTRL_REG2_ALARM_SEL_Pos 5
    #define MPL3115A2_CTRL_REG2_ALARM_SEL (0x1 << MPL3115A2_CTRL_REG2_ALARM_SEL_Pos)
    #define MPL3115A2_CTRL_REG2_ST_Pos 0
    #define MPL3115A2_CTRL_REG2_ST_Msk (0xF << MPL3115A2_CTRL_REG2_ST_Pos)
    
    //See Table 48 of MPL3115A2 Data Sheet
    const uint8_t LOAD_OUTPUT_BIT = MPL3115A2_CTRL_REG2_LOAD_OUTPUT;
    const uint8_t ALARM_SELECT_BIT = MPL3115A2_CTRL_REG2_ALARM_SEL;
    
    
    //==============================    Control Register 3 (R/W)   ================================//
    //=============================================================================================//
    const uint8_t CTRL_REG3 = 0x28;      //Control Register 3 address
    
    //CTRL_REG3 Register Structure Definition
    #define MPL3115A2_CTRL_REG3_IPOL1_Pos 5
    #define MPL3115A2_CTRL_REG3_IPOL1 (0x1 << MPL3115A2_CTRL_REG3_IPOL1_Pos)
    #define MPL3115A2_CTRL_REG3_PP_OD1_Pos 4
    #define MPL3115A2_CTRL_REG3_PP_OD1 (0x1 << MPL3115A2_CTRL_REG3_PP_OD1_Pos)
    #define MPL3115A2_CTRL_REG3_IPOL2_Pos 1
    #define MPL3115A2_CTRL_REG3_IPOL2 (0x1 << MPL3115A2_CTRL_REG3_IPOL2_Pos)
    #define MPL3115A2_CTRL_REG3_PP_OD2_Pos 0
    #define MPL3115A2_CTRL_REG3_PP_OD2 (0x1 << MPL3115A2_CTRL_REG3_PP_OD2_Pos)
    
    //See Table 50 of MPL3115A2 Data Sheet. These bits control the configuration of the interrupt pins
    const uint8_t INT1_POLARITY_BIT = MPL3115A2_CTRL_REG3_IPOL1; //Set: Active High Clear: Active Low
    const uint8_t INT1_DRIVE_BIT = MPL3115A2_CTRL_REG3_PP_OD1;   //Set: Active High Clear: Active Low
    const uint8_t INT2_POLARITY_BIT = MPL3115A2_CTRL_REG3_IPOL2; //Set: Open Drain Clear: Push Pull
    const uint8_t INT2_DRIVE_BIT = MPL3115A2_CTRL_REG3_PP_OD2;   //Set: Open Drain Clear: Push Pull
    
    //==============================    Control Register 4 (R/W)   ================================//
    //=============================================================================================//
    const uint8_t CTRL_REG4 = 0x29;      //Control Register 4 address
    
    //CTRL_REG4 Register Structure Definition
    #define MPL3115A2_CTRL_REG4_INT_EN_DRDY_Pos 7
    #define MPL3115A2_CTRL_REG4_INT_EN_DRDY (0x1 << MPL3115A2_CTRL_REG4_INT_EN_DRDY_Pos)
    #define MPL3115A2_CTRL_REG4_INT_EN_FIFO_Pos 6
    #define MPL3115A2_CTRL_REG4_INT_EN_FIFO (0x1 << MPL3115A2_CTRL_REG4_INT_EN_FIFO_Pos)
    #define MPL3115A2_CTRL_REG4_INT_EN_PW_Pos 5
    #define MPL3115A2_CTRL_REG4_INT_EN_PW (0x1 << MPL3115A2_CTRL_REG4_INT_EN_PW_Pos)
    #define MPL3115A2_CTRL_REG4_INT_EN_TW_Pos 4
    #define MPL3115A2_CTRL_REG4_INT_EN_TW (0x1 << MPL3115A2_CTRL_REG4_INT_EN_TW_Pos)
    #define MPL3115A2_CTRL_REG4_INT_EN_PTH_Pos 3
    #define MPL3115A2_CTRL_REG4_INT_EN_PTH (0x1 << MPL3115A2_CTRL_REG4_INT_EN_PTH_Pos)
    #define MPL3115A2_CTRL_REG4_INT_EN_TTH_Pos 2
    #define MPL3115A2_CTRL_REG4_INT_EN_TTH (0x1 << MPL3115A2_CTRL_REG4_INT_EN_TTH_Pos)
    #define MPL3115A2_CTRL_REG4_INT_EN_PCHG_Pos 1
    #define MPL3115A2_CTRL_REG4_INT_EN_PCHG (0x1 << MPL3115A2_CTRL_REG4_INT_EN_PCHG_Pos)
    #define MPL3115A2_CTRL_REG4_INT_EN_TCHG_Pos 0
    #define MPL3115A2_CTRL_REG4_INT_EN_TCHG (0x1 << MPL3115A2_CTRL_REG4_INT_EN_TCHG_Pos)
    
    //See Table 52 in MPL3115A2 Data Sheet
    //Set: Interrupt enabled. Clear: Interrupt disabled (default)
    const uint8_t ENABLE_DATA_READY_INTR = MPL3115A2_CTRL_REG4_INT_EN_DRDY;
    const uint8_t ENABLE_FIFO_INTR = MPL3115A2_CTRL_REG4_INT_EN_FIFO;
    const uint8_t ENABLE_PRESS_WINDOW_INTR = MPL3115A2_CTRL_REG4_INT_EN_PW;
    const uint8_t ENABLE_TEMP_WINDOW_INTR = MPL3115A2_CTRL_REG4_INT_EN_TW;
    const uint8_t ENABLE_PRESS_THRESHOLD_INTR = MPL3115A2_CTRL_REG4_INT_EN_PTH;
    const uint8_t ENABLE_TEMP_THRESHOLD_INTR = MPL3115A2_CTRL_REG4_INT_EN_TTH;
    const uint8_t ENABLE_PRESS_CHANGE_INTR = MPL3115A2_CTRL_REG4_INT_EN_PCHG;
    const uint8_t ENABLE_TEMP_CHANGE_INTR = MPL3115A2_CTRL_REG4_INT_EN_TCHG;
    
    
    //==============================    Control Register 5 (R/W)   ================================//
    //=============================================================================================//
    const uint8_t CTRL_REG5 = 0x2A;      //Control Register 5 address
    
    //CTRL_REG5 Regiseter Structure Definition
    #define MPL3115A2_CTRL_REG5_INT_CFG_DRDY_Pos 7
    #define MPL3115A2_CTRL_REG5_INT_CFG_DRDY (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_DRDY_Pos)
    #define MPL3115A2_CTRL_REG5_INT_CFG_FIFO_Pos 6
    #define MPL3115A2_CTRL_REG5_INT_CFG_FIFO (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_FIFO_Pos)
    #define MPL3115A2_CTRL_REG5_INT_CFG_PW_Pos 5
    #define MPL3115A2_CTRL_REG5_INT_CFG_PW (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_PW_Pos)
    #define MPL3115A2_CTRL_REG5_INT_CFG_TW_Pos 4
    #define MPL3115A2_CTRL_REG5_INT_CFG_TW (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_TW_Pos)
    #define MPL3115A2_CTRL_REG5_INT_CFG_PTH_Pos 3
    #define MPL3115A2_CTRL_REG5_INT_CFG_PTH (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_PTH_Pos)
    #define MPL3115A2_CTRL_REG5_INT_CFG_TTH_Pos 2
    #define MPL3115A2_CTRL_REG5_INT_CFG_TTH (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_TTH_Pos)
    #define MPL3115A2_CTRL_REG5_INT_CFG_PCHG_Pos 1
    #define MPL3115A2_CTRL_REG5_INT_CFG_PCHG (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_PCHG_Pos)
    #define MPL3115A2_CTRL_REG5_INT_CFG_TCHG_Pos 0
    #define MPL3115A2_CTRL_REG5_INT_CFG_TCHG (0x1 << MPL3115A2_CTRL_REG5_INT_CFG_TCHG_Pos)
    
    //See Table 54 in MPL3115A2 Data Sheet
    //Set: Interrupt on INT1 pin. Cleared: Interrupt on INT2 pin (default)
    const uint_t DATA_READY_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_DRDY;
    const uint_t FIFO_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_FIFO;
    const uint_t PRESS_WINDOW_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_PW;
    const uint_t TEMP_WINDOW_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_TW;
    const uint_t PRESS_THRESHOLD_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_PTH;
    const uint_t TEMP_THRESHOLD_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_TTH;
    const uint_t PRESS_CHANGE_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_PCHG;
    const uint_t TEMP_CHANGE_INTR_ON_INT1 = MPL3115A2_CTRL_REG5_INT_CFG_TCHG;
    
    
    //==================    Pressure Data Offset Correction Register (R/W)   ======================//
    //=============================================================================================//
    //This registers holds the pressure offset correction value (0 default). Correction value is
    //expressed as a signed 2's complement value with the LSB = 4 Pa. Correction range is from
    //-512 to +508 Pa. See Paragraph 14.23.1 in MPL3115A2 data sheet.
    const uint8_t OFF_P_REG = 0x2B;      //Pressure data offset Register address
    
    const uint8_t PRESS_OFFSET_CORRECTION_SF = 4;   //Pa/LSB
    
    //=================    Temperature Data Offset Correction Register (R/W)   ====================//
    //=============================================================================================//
    //This register holds the temperature offset correction value (0 default). Correction value is
    //expressed as a signed 2's complement value with the LSB = 0.0625 degree C. Correction range is
    //from -8 to + 7.9375 degree C. See Paragraph 14.23.2 in MPL3115A2 data sheet.
    const uint8_t OFF_T_REG = 0x2C;      //Temperature data offset Register address
    
    const float TEMP_OFFSET_CORRECTION_SF = 0.0625  //deg/LSB
    
    //==================    Altitude Data Offset Correction Register (R/W)   ======================//
    //=============================================================================================//
    //This register holds the altitude offset correction value (0 default). Correction value is
    //expressed as a signed 2's complement value with LSB = 1 meter.
    const uint8_t OFF_H_REG = 0x2D;      //Altitude data offset Register address
    
    //Other Symbolic Constants
    const uint8_t I2C_ADDRESS = 0x60;
    const uint8_t I2C_READ_BIT = 0x1;
    
    enum mode_t {
        STANDBY,
        ACTIVE_ALTIMETER,
        ACTIVE_BAROMETER
    };
    
    /********************************************************************************************/
    /*                 MPL3115A2_Driver Class Declartion                                        */
    /********************************************************************************************/
    /*Class for MPL3115A2 driver object                                                         */
    /********************************************************************************************/
    class MPL3115A2_Driver
    {
    public:
        //Constructor
        MPL3115A2();
        
        void begin(TwoWire * I2CDriver = nullptr);
        
        //Low Level General Purpose Functions
        uint8_t readRegister(uint8_t reg) const;
        writeRegister(uint8_t reg, uint8_t value);
        bool isMPL3115A2() const;
        mode_t getMode() const;
        
        
        //Calibration Methods
        void setSeaLevelPressurePa(uint16_t pressure);
        uint16_t getSeaLevelPressurePa() const;
        void setSeaLevelPressureMB(float pressure);
        float getSeaLevelPressureMB() const;
        void setPressureOffsetCorrection(int8_t correction);
        int8_t getPressureOffsetCorrection() const;
        void setTemperatureOffsetCorrection(int8_t correction);
        int8_t getTemperatureOffsetCorrection() const;
        void setAltitudeOffsetCorrection(int8_t correction);
        int8_t getAltitudeOffsetCorrection() const;
        
        //Device Operation/Configuration
        void run(mode_t mode, overSample_t os);
        void standby();
        void reset();
        void sampleNow();
        void enableEventFlags(bool dataReady, bool pressureReady, bool temperatureReady);
        
        //Interrupt/Event Configuration
        void setPressureAltitudeTarget(double target);
        void setTemperatureTarget(int8_t target);
        void setPressureWindow(uint16_t window);
        void setTemperatureWindow(uint8_t window);
        
        //Reading Sensor Data
        //Test sensor data status
        bool isDataReady() const;
        bool isPressureDataReady() const;
        bool IsTemperatureDataReady() const;
        bool wasDataOverwritten() const;
        bool wasPressureDataOverwritten() const;
        bool wasTemperatureDataOverwritten() const;
        //Retrieve sensor data
        double getPressureAltitude() const;
        double getTemperature() const;
        double getPressureAltitudeChange() const;
        double getTemperatureChange() const;
        void clearMinimumPressureAltitude();
        double getMinimumPressureAltitude() const;
        void clearMaximumPressureAltitude();
        double getMaximumPressureAltitude() const;
        void clearMinimumTemperature();
        double getMinimumTemperature() const;
        void clearMaximumTemperature();
        double getMaximumTemperature() const;
        
        //FIFO Management
        
    private:
        TwoWire * mI2C_Driver;
    };
    
} //End MPL3115A2 namespace declaration

#endif /* MPL3115A2_Driver_hpp */
