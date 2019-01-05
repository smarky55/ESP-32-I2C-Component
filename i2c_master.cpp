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

#include "i2c_master.h"

namespace SMK {

static const char* TAG = "SMK_I2C_MASTER";

i2c_master::i2c_master() {
}

i2c_master::~i2c_master() {
	i2c_driver_delete(i2c_num);
}

esp_err_t i2c_master::init(i2c_port_t i2c_num, gpio_num_t sda_io_num,
		gpio_num_t scl_io_num, uint32_t clk_speed, bool pullup) {

	config.mode = I2C_MODE_MASTER;
	config.sda_io_num = sda_io_num;
	config.scl_io_num = scl_io_num;
	config.master.clk_speed = clk_speed;
	if(pullup){
		config.sda_pullup_en = config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	} else {
		config.sda_pullup_en = config.scl_pullup_en = GPIO_PULLUP_DISABLE;
	}

	i2c_param_config(i2c_num, &config);

	isInstalled = true;
	this->i2c_num = i2c_num;
	return i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t i2c_master::write(uint16_t slave_addr, uint8_t* data,
		size_t len) {

	if(!isInstalled){
		ESP_LOGE(TAG, "Attempt to use I2C bus without initialising driver!");
		return ESP_ERR_INVALID_STATE;
	}

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
	if(len > 0 && data != nullptr) i2c_master_write(cmd, data, len, true);
	i2c_master_stop(cmd);

	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_RATE_MS);

	i2c_cmd_link_delete(cmd);

	return ret;
}

esp_err_t i2c_master::write_register(uint16_t slave_addr, uint8_t reg_addr,
		uint8_t data) {

	uint8_t buff [] = {reg_addr, data};
	return write(slave_addr, buff, 2);
}

esp_err_t i2c_master::read(uint16_t slave_addr, uint8_t* write_data,
		size_t write_len, uint8_t* read_data, size_t read_len) {

	if(!isInstalled){
		ESP_LOGE(TAG, "Attempt to use I2C bus without initialising driver!");
		return ESP_ERR_INVALID_STATE;
	}

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
	if(write_len > 0 && write_data != nullptr) i2c_master_write(cmd, write_data, write_len, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true);
	if(read_len > 1) i2c_master_read(cmd, read_data, read_len-1, I2C_MASTER_ACK);
	if(read_len > 0) i2c_master_read_byte(cmd, read_data + (read_len - 1), I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_RATE_MS);

	i2c_cmd_link_delete(cmd);

	return ret;
}

esp_err_t i2c_master::read_register(uint16_t slave_addr, uint8_t reg_addr,
		uint8_t* data) {

	return read(slave_addr, &reg_addr, 1, data, 1);

}  // namespace SMK

}
