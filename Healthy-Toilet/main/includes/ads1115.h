#pragma once

#include "stdint.h"
#include "esp_log.h"

#define PORT_A_SDA_PIN GPIO_NUM_32
#define PORT_A_SCL_PIN GPIO_NUM_33
#define PORT_A_I2C_STANDARD_BAUD 100000

#define ADS1115_ADDRESS                           0x48 ///< 1001 000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_POINTER_MASK                  0x03      ///< Point mask
#define ADS1115_REG_POINTER_CONVERT               0x00   ///< Conversion
#define ADS1115_REG_POINTER_CONFIG                0x01    ///< Configuration
#define ADS1115_REG_POINTER_LOWTHRESH             0x02 ///< Low threshold
#define ADS1115_REG_POINTER_HITHRESH              0x03  ///< High threshold
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_CONFIG_OS_MASK                0x8000 ///< OS Mask
#define ADS1115_REG_CONFIG_OS_SINGLE              0x8000 ///< Write: Set to start a single-conversion
#define ADS1115_REG_CONFIG_OS_BUSY                0x0000 ///< Read: Bit = 0 when conversion is in progress
#define ADS1115_REG_CONFIG_OS_NOTBUSY             0x8000 ///< Read: Bit = 1 when device is not performing a conversion

#define ADS1115_REG_CONFIG_MUX_MASK               0x7000 ///< Mux Mask
#define ADS1115_REG_CONFIG_MUX_DIFF_0_1           0x0000 ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_3           0x1000 ///< Differential P = AIN0, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_1_3           0x2000 ///< Differential P = AIN1, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_2_3           0x3000 ///< Differential P = AIN2, N = AIN3
#define ADS1115_REG_CONFIG_MUX_SINGLE_0           0x4000 ///< Single-ended AIN0
#define ADS1115_REG_CONFIG_MUX_SINGLE_1           0x5000 ///< Single-ended AIN1
#define ADS1115_REG_CONFIG_MUX_SINGLE_2           0x6000 ///< Single-ended AIN2
#define ADS1115_REG_CONFIG_MUX_SINGLE_3           0x7000 ///< Single-ended AIN3

#define ADS1115_REG_CONFIG_PGA_MASK               0x0E00   ///< PGA Mask
#define ADS1115_REG_CONFIG_PGA_6_144V             0x0000 ///< +/-6.144V range = Gain 2/3
#define ADS1115_REG_CONFIG_PGA_4_096V             0x0200 ///< +/-4.096V range = Gain 1
#define ADS1115_REG_CONFIG_PGA_2_048V             0x0400 ///< +/-2.048V range = Gain 2 (default)
#define ADS1115_REG_CONFIG_PGA_1_024V             0x0600 ///< +/-1.024V range = Gain 4
#define ADS1115_REG_CONFIG_PGA_0_512V             0x0800 ///< +/-0.512V range = Gain 8
#define ADS1115_REG_CONFIG_PGA_0_256V             0x0A00 ///< +/-0.256V range = Gain 16

#define ADS1115_REG_CONFIG_MODE_MASK              0x0100   ///< Mode Mask
#define ADS1115_REG_CONFIG_MODE_CONTIN            0x0000 ///< Continuous conversion mode
#define ADS1115_REG_CONFIG_MODE_SINGLE            0x0100 ///< Power-down single-shot mode (default)

#define ADS1115_REG_CONFIG_RATE_MASK              0x00E0 ///< Data Rate Mask

#define ADS1115_REG_CONFIG_CMODE_MASK             0x0010 ///< CMode Mask
#define ADS1115_REG_CONFIG_CMODE_TRAD             0x0000 ///< Traditional comparator with hysteresis (default)
#define ADS1115_REG_CONFIG_CMODE_WINDOW           0x0010  ///< Window comparator

#define ADS1115_REG_CONFIG_CPOL_MASK              0x0008 ///< CPol Mask
#define ADS1115_REG_CONFIG_CPOL_ACTVLOW           0x0000 ///< ALERT/RDY pin is low when active (default)
#define ADS1115_REG_CONFIG_CPOL_ACTVHI            0x0008 ///< ALERT/RDY pin is high when active

#define ADS1115_REG_CONFIG_CLAT_MASK              0x0004 ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1115_REG_CONFIG_CLAT_NONLAT            0x0000  ///< Non-latching comparator (default)
#define ADS1115_REG_CONFIG_CLAT_LATCH             0x0004 ///< Latching comparator

#define ADS1115_REG_CONFIG_CQUE_MASK              0x0003 ///< CQue Mask
#define ADS1115_REG_CONFIG_CQUE_1CONV             0x0000 ///< Assert ALERT/RDY after one conversions
#define ADS1115_REG_CONFIG_CQUE_2CONV             0x0001 ///< Assert ALERT/RDY after two conversions
#define ADS1115_REG_CONFIG_CQUE_4CONV             0x0002 ///< Assert ALERT/RDY after four conversions
#define ADS1115_REG_CONFIG_CQUE_NONE              0x0003 ///< Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

/** Data rates */
#define RATE_ADS1115_8SPS                         0x0000  ///Slowest speed with up to 8 conversions each second, this is also the range with least noise.
#define RATE_ADS1115_16SPS                        0x0020  
#define RATE_ADS1115_32SPS                        0x0040  
#define RATE_ADS1115_64SPS                        0x0060  
#define RATE_ADS1115_128SPS                       0x0080  ///< 128 samples per second (default)
#define RATE_ADS1115_250SPS                       0x00A0 
#define RATE_ADS1115_475SPS                       0x00C0 
#define RATE_ADS1115_860SPS                       0x00E0  ///Fastest speed with up to 860 conversions each second.

/** Gain settings */
typedef enum {
  GAIN_TWOTHIRDS = ADS1115_REG_CONFIG_PGA_6_144V,//Maximum scale is 6.144V, but input will be limited by VCC  
  GAIN_ONE = ADS1115_REG_CONFIG_PGA_4_096V,      //Maximum scale is 4.096V, but will be lower in a 3.3V system
  GAIN_TWO = ADS1115_REG_CONFIG_PGA_2_048V,      //Maximum scale is 2.048V
  GAIN_FOUR = ADS1115_REG_CONFIG_PGA_1_024V,     //Maximum scale is 1.024V
  GAIN_EIGHT = ADS1115_REG_CONFIG_PGA_0_512V,    //Maximum scale is 0.512V
  GAIN_SIXTEEN = ADS1115_REG_CONFIG_PGA_0_256V   //Maximum scale is 0.256V
} adsGain_t;

//uint16_t m_dataRate; // = RATE_ADS1115_860SPS;       // 

void ADS1115_I2CInit();
esp_err_t ADS1115_I2C_Read(uint8_t register_address, uint8_t *data, uint16_t length);
esp_err_t ADS1115_I2C_Write(uint8_t register_address, uint8_t *data, uint16_t length);
//void ADS1115_I2C_Close(ads1115_device);
bool ADS1115_I2C_ReadU16(uint8_t reg_addr, uint16_t* value);
bool ADS1115_I2C_WriteU16(uint8_t reg_addr, uint16_t value);
void ADS1115_setGain(adsGain_t gain);
adsGain_t ADS1115_getGain();
void ADS1115_setDataRate(uint16_t rate);
uint16_t ADS1115_getDataRate();
int16_t ADS1115_readADC_SingleEnded(uint8_t channel);
void ADS1115_startComparator_SingleEnded(uint8_t channel, int16_t threshold);
int16_t ADS1115_getLastConversionResults();
float ADS1115_computeVolts(int16_t counts);
bool ADS1115_conversionComplete();
