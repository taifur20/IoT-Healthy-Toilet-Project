#include "i2c_device.h"
#include "esp_err.h"
#include "ads1115.h"


static I2CDevice_t ads1115_device;

void ADS1115_I2CInit() {
    ads1115_device = i2c_malloc_device(I2C_NUM_0, PORT_A_SDA_PIN, PORT_A_SCL_PIN, PORT_A_I2C_STANDARD_BAUD, ADS1115_ADDRESS);
}

esp_err_t ADS1115_I2C_Read(uint8_t register_address, uint8_t *data, uint16_t length){
    return i2c_read_bytes(ads1115_device, register_address, data, length);
}

esp_err_t ADS1115_I2C_Write(uint8_t register_address, uint8_t *data, uint16_t length){
    return i2c_write_bytes(ads1115_device, register_address, data, length);
}
/*
void ADS1115_I2C_Close(ads1115_device){
    i2c_free_device(ads1115_device);
}
*/
bool ADS1115_I2C_ReadU16(uint8_t reg_addr, uint16_t* value) {
  uint8_t read_buf[2] = {0x00, 0x00};
  bool result = ADS1115_I2C_Read(reg_addr, read_buf, 2);
  *value = (read_buf[0] << 8) | read_buf[1];
  return result;
}

bool ADS1115_I2C_WriteU16(uint8_t reg_addr, uint16_t value) {
  uint8_t write_buf[2];
  write_buf[0] = value >> 8;
  write_buf[1] = value & 0xff;
  return ADS1115_I2C_Write(reg_addr, write_buf, 2);
}

static adsGain_t m_gain = GAIN_TWOTHIRDS;               //+/- 6.144V range (limited to VDD +0.3V max!) 
uint16_t m_dataRate = RATE_ADS1115_860SPS;       //

void ADS1115_setGain(adsGain_t gain) { m_gain = gain; }
adsGain_t ADS1115_getGain() { return m_gain; }
void ADS1115_setDataRate(uint16_t rate) { m_dataRate = rate; }
uint16_t ADS1115_getDataRate() { return m_dataRate; }

int16_t ADS1115_readADC_SingleEnded(uint8_t channel) {
  if (channel > 3) {
    return 0;
  }

  // Start with default values
  uint16_t config =
      ADS1115_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1115_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1115_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set data rate
  config |= m_dataRate;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  ADS1115_I2C_WriteU16(ADS1115_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  while (!ADS1115_conversionComplete())
    ;

  // Read the conversion results
  return ADS1115_getLastConversionResults();
}


void ADS1115_startComparator_SingleEnded(uint8_t channel, int16_t threshold) {
  // Start with default values
  uint16_t config =
      ADS1115_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1115_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1115_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1115_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1115_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set data rate
  config |= m_dataRate;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set the high threshold register
  ADS1115_I2C_WriteU16(ADS1115_REG_POINTER_HITHRESH, threshold);

  // Write config register to the ADC
  ADS1115_I2C_WriteU16(ADS1115_REG_POINTER_CONFIG, config);
}

int16_t ADS1115_getLastConversionResults() {
  // Read the conversion results
  uint16_t value = 0x00;
  ADS1115_I2C_ReadU16(ADS1115_REG_POINTER_CONVERT, &value);
  return value;
}

float ADS1115_computeVolts(int16_t counts) {
  // see data sheet Table 3
  float fsRange;
  switch (m_gain) {
  case GAIN_TWOTHIRDS:
    fsRange = 6.144f;
    break;
  case GAIN_ONE:
    fsRange = 4.096f;
    break;
  case GAIN_TWO:
    fsRange = 2.048f;
    break;
  case GAIN_FOUR:
    fsRange = 1.024f;
    break;
  case GAIN_EIGHT:
    fsRange = 0.512f;
    break;
  case GAIN_SIXTEEN:
    fsRange = 0.256f;
    break;
  default:
    fsRange = 0.0f;
  }
  return counts * (fsRange / 32768);
}

bool ADS1115_conversionComplete() {
  uint16_t value = 0x00;
  ADS1115_I2C_ReadU16(ADS1115_REG_POINTER_CONFIG, &value);
  return (value & 0x8000) != 0;
}