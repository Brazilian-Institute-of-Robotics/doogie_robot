#ifndef DOOGIE_DRIVERS_ADS1115_DRIVER_HPP_
#define DOOGIE_DRIVERS_ADS1115_DRIVER_HPP_

#include <cstdint>
#include <string>

/*=========================================================================
CONFIG REGISTER
-----------------------------------------------------------------------*/
#define CONFIG_REG_OS_SINGLE    	(0x8000)
#define CONFIG_REG_OS_BUSY      	(0x0000)
#define CONFIG_REG_OS_NOTBUSY   	(0x8000)

#define CONFIG_REG_MUX_MASK     	(0x7000)
#define CONFIG_REG_MUX_DIFF_0_1		(0x0000) // default
#define CONFIG_REG_MUX_DIFF_0_3		(0x1000)
#define CONFIG_REG_MUX_DIFF_1_3		(0x2000)
#define CONFIG_REG_MUX_DIFF_2_3		(0x3000)
#define CONFIG_REG_MUX_CHAN_0 		(0x4000)
#define CONFIG_REG_MUX_CHAN_1 		(0x5000)
#define CONFIG_REG_MUX_CHAN_2 		(0x6000)
#define CONFIG_REG_MUX_CHAN_3 		(0x7000)

#define CONFIG_REG_PGA_6_144V   	(0x0000) // +/-6.144V range
#define CONFIG_REG_PGA_4_096V   	(0x0200) // +/-4.096V range
#define CONFIG_REG_PGA_2_048V   	(0x0400) // +/-2.048V range // default
#define CONFIG_REG_PGA_1_024V   	(0x0600) // +/-1.024V range
#define CONFIG_REG_PGA_0_512V   	(0x0800) // +/-0.512V range
#define CONFIG_REG_PGA_0_256V   	(0x0A00) // +/-0.256V range

#define CONFIG_REG_MODE_CONTIN		(0x0000)
#define CONFIG_REG_MODE_SINGLE		(0x0100) // default

#define CONFIG_REG_DR_8SPS			  (0x0000)
#define CONFIG_REG_DR_16SPS			  (0x0020)
#define CONFIG_REG_DR_32SPS			  (0x0040)
#define CONFIG_REG_DR_64SPS			  (0x0060)
#define CONFIG_REG_DR_128SPS		  (0x0080) // default
#define CONFIG_REG_DR_250SPS		  (0x00A0)
#define CONFIG_REG_DR_475SPS		  (0x00C0)
#define CONFIG_REG_DR_860SPS		  (0x00E0)

#define CONFIG_REG_CMODE_TRAD		    (0x0000) // default
#define CONFIG_REG_CMODE_WINDOW		  (0x0010)

#define CONFIG_REG_CPOL_ACTIV_LOW	  (0x0000) // default
#define CONFIG_REG_CPOL_ACTIV_HIGH	(0x0080)

#define CONFIG_REG_CLATCH_NONLATCH	(0x0000) // default
#define CONFIG_REG_CLATCH_LATCH		  (0x0040)

#define CONFIG_REG_CQUE_1CONV		    (0x0000)
#define CONFIG_REG_CQUE_2CONV		    (0x0001)
#define CONFIG_REG_CQUE_4CONV		    (0x0002)
#define CONFIG_REG_CQUE_NONE		    (0x0003) // default

#define REG_POINTER_MASK       (0x03) ///< Point mask
#define REG_POINTER_CONVERT    (0x00) ///< Conversion
#define REG_POINTER_CONFIG     (0x01) ///< Configuration
#define REG_POINTER_LOWTHRESH  (0x02) ///< Low threshold
#define REG_POINTER_HITHRESH   (0x03) ///< High threshold

namespace doogie_drivers {

class ADS115Driver {
 public:
  ADS115Driver(uint8_t addr);
  ~ADS115Driver();
  void init(std::string i2c_bus_uri);
  uint16_t readRawValue(uint channel);
  float readVoltage(uint channel);
  void writeInRegister(uint8_t reg_ptr, uint16_t value);
  uint16_t readRegister(uint8_t reg_ptr);

 private:
  void openI2CBus(std::string bus_uri);
  void setI2CSlave(uint8_t slave_addr);
  void configDevice();

  uint16_t dev_settings_;
  uint8_t addr_;
  int i2c_bus_fd_;
};

}  // namespace doogie_drivers

#endif  // DOOGIE_DRIVERS_ADS1115_DRIVER_HPP_
