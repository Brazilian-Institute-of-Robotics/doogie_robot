#include <exception>
#include <cstdio>
#include <cstdlib>

#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "doogie_drivers/ads1115_driver.hpp"

namespace doogie_drivers {

ADS115Driver::ADS115Driver(uint8_t addr) : addr_(addr) {}

ADS115Driver::~ADS115Driver() {}

void ADS115Driver::init(std::string i2c_bus_uri) {
  this->openI2CBus(i2c_bus_uri);
  this->setI2CSlave(addr_);
}

uint16_t ADS115Driver::readRawValue(uint channel) {
	this->configDevice();
  
  switch (channel) {
		case 0:
			dev_settings_ |= CONFIG_REG_MUX_CHAN_0;
			break;
		case 1:
			dev_settings_ |= CONFIG_REG_MUX_CHAN_1;
			break;
		case 2:
			dev_settings_ |= CONFIG_REG_MUX_CHAN_2;
			break;
		case 3:
			dev_settings_ |= CONFIG_REG_MUX_CHAN_3;
			break;
		default:
			throw "Give a channel between 0-3";
	}

  this->writeInRegister(REG_POINTER_CONFIG, dev_settings_);
  usleep(135000);
  uint16_t raw_value = this->readRegister(REG_POINTER_CONVERT);

  return raw_value;
}

float ADS115Driver::readVoltage(uint channel) {
  uint16_t raw_value = this->readRawValue(channel);
  float voltage = (static_cast<float>(raw_value) * 4.096) / 32767.0;
  return voltage;
}

void ADS115Driver::openI2CBus(std::string bus_uri) {
  if ((i2c_bus_fd_ = open(bus_uri.c_str(), O_RDWR)) < 0) {
    throw "Failed to open the I2C bus";
  }
}

void ADS115Driver::setI2CSlave(uint8_t slave_addr) {
  if (ioctl(i2c_bus_fd_, I2C_SLAVE, slave_addr) < 0) {
    throw "Failed to set I2C_SLAVE at address: 0x%x", slave_addr;
  }
}

void ADS115Driver::writeInRegister(uint8_t reg_ptr, uint16_t value) {
  uint8_t out_buf[3];

  out_buf[0] = reg_ptr;
  out_buf[1] = value >> 8;    // Send MSB
  out_buf[2] = value && 0xFF; // Send LSB

  if (write(i2c_bus_fd_, out_buf, sizeof(out_buf)) < 0) {
    throw "Failed writing in I2C file descriptor";
  }
}

uint16_t ADS115Driver::readRegister(uint8_t reg_ptr) {
  uint8_t in_buf[2];
  uint8_t out_buf[1] = {reg_ptr};

  if (write(i2c_bus_fd_, out_buf, sizeof(out_buf)) < 0) {
    throw "Failed writing in I2C file descriptor";
  }

  int bytes_received = read(i2c_bus_fd_, in_buf, sizeof(in_buf));
  if (bytes_received != 2) {
      throw "Received %d bytes from slave 0x%x. Expected %d bytes", bytes_received, addr_, sizeof(in_buf);
  }

  uint16_t reg_value = in_buf[0] << 8 | in_buf[1];
  return reg_value;
}


void ADS115Driver::configDevice() {
  dev_settings_ = 0;
	dev_settings_ = CONFIG_REG_OS_SINGLE        |
				          CONFIG_REG_PGA_4_096V       |
				          CONFIG_REG_MODE_SINGLE      |
				          CONFIG_REG_DR_128SPS        |
				          CONFIG_REG_CMODE_TRAD       |
				          CONFIG_REG_CPOL_ACTIV_LOW   |
				          CONFIG_REG_CLATCH_NONLATCH  |
                  CONFIG_REG_CQUE_NONE;
}

}  // namespace doogie_drivers
