/*
 * MIT License

Copyright (c) 2019 Sam Kingdon

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef COMPONENTS_SMK_I2C_INCLUDE_I2C_MASTER_H_
#define COMPONENTS_SMK_I2C_INCLUDE_I2C_MASTER_H_

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

namespace SMK {

class i2c_master {
public:
	i2c_master();
	virtual ~i2c_master();

	/**
	 * @brief Initialise I2C driver as master
	 *
	 * @param i2c_num 		I2C port number
	 * @param sda_io_num 	GPIO number for I2C sda signal
	 * @param scl_io_num	GPIO number for I2C scl signal
	 * @param clk_speed		I2C clock frequency in master mode in Hz (max 1MHz)
	 * @param pullup		Enable internal pullup resistors (default true)
	 *
	 * @return
	 * 		- ESP_OK				Success
	 * 		- ESP_ERR_INVALID_ARG	Parameter error
	 * 		- ESP_FAIL				Driver install error
	 */
	esp_err_t init(i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed, bool pullup = true);

	/**
	 * @brief Write bites to a slave over established I2C bus
	 *
	 * @param slave_addr	Address of device we want to write to
	 * @param data			Buffer of data to sent to slave
	 * @param len			Length of data array
	 *
	 * @return
	 * 		- ESP_OK 				Success
	 * 		- ESP_ERR_INVALID_ARG 	Parameter error
	 *		- ESP_FAIL 				Sending command error, slave doesn't ACK the transfer.
	 *		- ESP_ERR_INVALID_STATE	I2C driver not installed or not in master mode.
	 *		- ESP_ERR_TIMEOUT 		Operation timeout because the bus is busy.
	 */
	esp_err_t write(uint16_t slave_addr, void* data, size_t len);

	/**
	 * @brief Wrapper to simplify writing to a single 8 bit register
	 *
	 * @param slave_addr	Address of device we want to write to
	 * @param reg_addr		Address of register to write on slave device
	 * @param data			Byte to write to register
	 *
	 * @return
	 * 		- ESP_OK 				Success
	 * 		- ESP_ERR_INVALID_ARG 	Parameter error
	 *		- ESP_FAIL 				Sending command error, slave doesn't ACK the transfer.
	 *		- ESP_ERR_INVALID_STATE	I2C driver not installed or not in master mode.
	 *		- ESP_ERR_TIMEOUT 		Operation timeout because the bus is busy.
	 */
	esp_err_t write_register(uint16_t slave_addr, uint8_t reg_addr, uint8_t data);


	/**
	 * @brief Read data from slave device on I2C bus
	 *
	 * @param slave_addr	Address of slave device to read from
	 * @param write_data	Initial data to write to slave such as the address we want to read
	 * @param write_len		Length of initial write data
	 * @param read_data		Pointer to memory location to fill with data from slave
	 * @param read_len		Number of bytes to read from slave device
	 *
	 * @return
	 * 		- ESP_OK 				Success
	 * 		- ESP_ERR_INVALID_ARG 	Parameter error
	 *		- ESP_FAIL 				Sending command error, slave doesn't ACK the transfer.
	 *		- ESP_ERR_INVALID_STATE	I2C driver not installed or not in master mode.
	 *		- ESP_ERR_TIMEOUT 		Operation timeout because the bus is busy.
	 */
	esp_err_t read(uint16_t slave_addr, void* write_data, size_t write_len, void* read_data, size_t read_len);


	/**
		 * @brief Wrapper to simplify reading from a single 8 bit register
		 *
		 * @param slave_addr	Address of device we want to read from
		 * @param reg_addr		Address of register to write on slave device
		 * @param data			Pointer to location to store the data from slave
		 *
		 * @return
		 * 		- ESP_OK 				Success
		 * 		- ESP_ERR_INVALID_ARG 	Parameter error
		 *		- ESP_FAIL 				Sending command error, slave doesn't ACK the transfer.
		 *		- ESP_ERR_INVALID_STATE	I2C driver not installed or not in master mode.
		 *		- ESP_ERR_TIMEOUT 		Operation timeout because the bus is busy.
		 */
	esp_err_t read_register(uint16_t slave_addr, uint8_t reg_addr, uint8_t* data);
private:
	i2c_config_t config;
	i2c_port_t i2c_num = I2C_NUM_0;
	bool isInstalled = false;
};


}  // namespace SMK

#endif /* COMPONENTS_SMK_I2C_INCLUDE_I2C_MASTER_H_ */
