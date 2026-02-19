#include "I2C_Driver.h"
#include <esp_log.h>

// Initialize I2C using legacy ESP-IDF driver (compatible with ESP-IDF 5.1)
void I2C_Init(void) {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_SDA_PIN;
  conf.scl_io_num = (gpio_num_t)I2C_SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000;
  
  esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
  if (err != ESP_OK) {
    ESP_LOGE("I2C", "Failed to configure I2C: %s", esp_err_to_name(err));
    return;
  }
  
  err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK) {
    ESP_LOGE("I2C", "Failed to install I2C driver: %s", esp_err_to_name(err));
  }
}

bool I2C_Read(uint8_t Driver_addr, uint8_t Reg_addr, uint8_t *Reg_data, uint32_t Length)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (Driver_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, Reg_addr, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (Driver_addr << 1) | I2C_MASTER_READ, true);
  if (Length > 1) {
    i2c_master_read(cmd, Reg_data, Length - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, Reg_data + Length - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  
  esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  
  if (err != ESP_OK) {
    printf("I2C read failed: %s\r\n", esp_err_to_name(err));
    return -1;
  }
  
  return 0;
}

bool I2C_Write(uint8_t Driver_addr, uint8_t Reg_addr, const uint8_t *Reg_data, uint32_t Length)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (Driver_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, Reg_addr, true);
  i2c_master_write(cmd, (uint8_t*)Reg_data, Length, true);
  i2c_master_stop(cmd);
  
  esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  
  if (err != ESP_OK) {
    printf("I2C write failed: %s\r\n", esp_err_to_name(err));
    return -1;
  }
  
  return 0;
}
