/**
 ******************************************************************************
 * File Name          : LDC16xx_lib.h
 * Description        : LDC16xx library
 ******************************************************************************
 *
 * Copyright (c) 2018 CNR-STIIMA DASM Group
 * Copyright (c) 2023 Colton Baldridge
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * Credits go to Jeremi Wójcicki (and the current maintainers) of this software.
 *
 **/


#ifndef INC_STM_LDC16XX_H_
#define INC_STM_LDC16XX_H_

#define   LDC16xx_DATA_MSB_CH0          0x00  //  Channel 0 MSB Conversion Result and Error Status
#define   LDC16xx_DATA_LSB_CH0          0x01  //  Channel 0 LSB Conversion Result. Must be read after Register address 0x00.
#define   LDC16xx_DATA_MSB_CH1          0x02  //  Channel 1 MSB Conversion Result and Error Status.
#define   LDC16xx_DATA_LSB_CH1          0x03  //  Channel 1 LSB Conversion Result. Must be read after Register address 0x02.
#define   LDC16xx_DATA_MSB_CH2          0x04  //  Channel 2 MSB Conversion Result and Error Status. (LDC1614 only)
#define   LDC16xx_DATA_LSB_CH2          0x05  //  Channel 2 LSB Conversion Result. Must be read after Register address 0x04. (LDC1614 only)
#define   LDC16xx_DATA_MSB_CH3          0x06  //  Channel 3 MSB Conversion Result and Error Status. (LDC1614 only)
#define   LDC16xx_DATA_LSB_CH3          0x07  //  Channel 3 LSB Conversion Result. Must be read after Register address 0x06. (LDC1614 only)
#define   LDC16xx_RCOUNT_CH0            0x08  //  Reference Count setting for Channel 0
#define   LDC16xx_RCOUNT_CH1            0x09  //  Reference Count setting for Channel 1
#define   LDC16xx_RCOUNT_CH2            0x0A  //  Reference Count setting for Channel 2.�(LDC16xx only)
#define   LDC16xx_RCOUNT_CH3            0x0B  //  Reference Count setting for Channel 3.(LDC16xx only)
#define   LDC16xx_OFFSET_CH0            0x0C  //  Offset value for Channel 0
#define   LDC16xx_OFFSET_CH1            0x0D  //  Offset value for Channel 1
#define   LDC16xx_OFFSET_CH2            0x0E  //  Offset value for Channel 2 (LDC1614 only)
#define   LDC16xx_OFFSET_CH3            0x0F  //  Offset value for Channel 3 (LDC1614 only)
#define   LDC16xx_SETTLECOUNT_CH0       0x10  //  Channel 0 Settling Reference Count
#define   LDC16xx_SETTLECOUNT_CH1       0x11  //  Channel 1 Settling Reference Count
#define   LDC16xx_SETTLECOUNT_CH2       0x12  //  Channel 2 Settling Reference Count�(LDC16xx only)
#define   LDC16xx_SETTLECOUNT_CH3       0x13  //  Channel 3 Settling Reference Count�(LDC16xx only)
#define   LDC16xx_CLOCK_DIVIDERS_CH0    0x14  //  Reference and Sensor Divider settings for Channel 0
#define   LDC16xx_CLOCK_DIVIDERS_CH1    0x15  //  Reference and Sensor Divider settings for Channel 1
#define   LDC16xx_CLOCK_DIVIDERS_CH2    0x16  //  Reference and Sensor Divider settings for Channel 2 (LDC1614 only)
#define   LDC16xx_CLOCK_DIVIDERS_CH3    0x17  //  Reference and Sensor Divider settings for Channel 3 (LDC1614 only)
#define   LDC16xx_STATUS                0x18  //  Device Status Report
#define   LDC16xx_ERROR_CONFIG          0x19  //  Error Reporting Configuration
#define   LDC16xx_CONFIG                0x1A  //  Conversion Configuration
#define   LDC16xx_MUX_CONFIG            0x1B  //  Channel Multiplexing Configuration
#define   LDC16xx_RESET_DEV             0x1C  //  Reset Device
#define   LDC16xx_DRIVE_CURRENT_CH0     0x1E  //  Channel 0 sensor current drive configuration
#define   LDC16xx_DRIVE_CURRENT_CH1     0x1F  //  Channel 1 sensor current drive configuration
#define   LDC16xx_DRIVE_CURRENT_CH2     0x20  //  Channel 2 sensor current drive configuration (LDC1614 only)
#define   LDC16xx_DRIVE_CURRENT_CH3     0x21  //  Channel 3 sensor current drive configuration (LDC1614 only)
#define   LDC16xx_MANUFACTURER_ID       0x7E  //  Manufacturer ID
#define   LDC16xx_DEVICE_ID             0x7F  //  Device ID

#define LDC16xx_BITS_RESET_DEV       (1 << 15)

// CONFIG bits
#define LDC16xx_BITS_ACTIVE_CHAN_CH0      (0 << 14)
#define LDC16xx_BITS_ACTIVE_CHAN_CH1      (1 << 14)
#define LDC16xx_BITS_ACTIVE_CHAN_CH2      (2 << 14)
#define LDC16xx_BITS_ACTIVE_CHAN_CH3      (3 << 14)

#define LDC16xx_BITS_SLEEP_MODE_EN        (1 << 13)

#define LDC16xx_BITS_RP_OVERRIDE_EN       (1 << 12)

#define LDC16xx_BITS_SENSOR_ACTIVATE_SEL  (1 << 11)

#define LDC16xx_BITS_AUTO_AMP_DIS         (1 << 10)

#define LDC16xx_BITS_REF_CLK_SRC          (1 << 9)

#define LDC16xx_BITS_INTB_DIS             (1 << 7)

#define LDC16xx_BITS_HIGH_CURRENT_DRV     (1 << 6)

#define LDC16xx_BITS_DRDY                 (1 << 6)

// ERROR_CONFIG
#define LDC16xx_BITS_DRDY_2INT            (1 << 0)

// MUX_CONFIG bits
#define LDC16xx_BITS_AUTOSCAN_EN     (1 << 15)

#define LDC16xx_BITS_DEGLITCH_1_0Mhz    1
#define LDC16xx_BITS_DEGLITCH_3_3Mhz    4
#define LDC16xx_BITS_DEGLITCH_10_0Mhz   5
#define LDC16xx_BITS_DEGLITCH_33_0Mhz   7

#define LDC16xx_BITS_RR_SEQUENCE_CH0_CH1          (0 << 13)
#define LDC16xx_BITS_RR_SEQUENCE_CH0_CH1_CH2      (1 << 13)
#define LDC16xx_BITS_RR_SEQUENCE_CH0_CH1_CH2_CH3  (2 << 13)

typedef struct __LDC_regNameAddr {
  uint8_t addr;
  const char* name;
} LDC_regNameAddr;

#define LCD16xx_DEFAULT_DEVICE_ID       0x3055
#define LDC16xx_DEFAULT_MANUFACTURER_ID 0x5449

typedef struct __LDC_configReg {
  uint8_t reg;
  uint16_t value;
} LDC_configReg;



void writeRegister(uint8_t LDC_addr, uint8_t reg, uint16_t data);
void writeLDCConfig(uint8_t LDC_addr, LDC_configReg cfg);
void readRegister(uint8_t LDC_addr, uint8_t reg, uint16_t * data);
int8_t readChannel(uint8_t LDC_addr, uint8_t channel, uint32_t *data);
void resetDevice(uint8_t LDC_addr);
void loadConfig(uint8_t LDC_addr, LDC_configReg cfg[], uint8_t size);
void clearAndSetRegisterBits(uint8_t LDC_addr, uint8_t reg, uint16_t clear_mask, uint16_t set_mask);
void waitForCoilDataReady(uint8_t LDC_addr, uint8_t coil);


#endif /* INC_STM_LDC16XX_H_ */
