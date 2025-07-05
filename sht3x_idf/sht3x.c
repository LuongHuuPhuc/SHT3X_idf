/**
 * @author Luong Huu Phuc
 * @date 2025/06/29
 * @file sht3x.c - Library/driver for Humidity and Temperature sensor SHT3X
 * @related Datasheet SHT3x-DIS (SENSIRION)
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <driver/i2c.h>
#include <driver/i2c_master.h>
#include "esp_log.h"
#include "sht3x.h"
#include "I2C_dev.h"

/**
 * @note - Trong truong hop Clock Stretching, sensor se phan hoi ACK ngay ca khi chua co du lieu, 
 * nhung chan SCL o muc thap, khine master phai cho den khi do xong (Khong can delay phan mem)
 * @note - Trong truong hop khong bat Clock Stretching, sensor se khong giu chan SCL. Neu ban gui lenh doc
 * qua som (tuc cam bien chua kip do xong), no se khong phai hoi voi ACK, ma thay vao do gui NACK -> Tu choi viec doc
 * => The nen can phai delay du lau truoc khi doc du lieu
 */
static uint16_t get_single_shot_cmd(const sht3x_handle_t *handle){
  if(handle->clock_stretch){
    switch(handle->repeatability){
      case sht3x_repeatability_high: return SHT3X_CLOCK_STRETCH_HIGH;
      case sht3x_repeatability_medium: return SHT3X_CLOCK_STRETCH_MEDIUM;
      case sht3x_repeatability_low: return SHT3X_CLOCK_STRETCH_LOW;
      default: return SHT3X_CLOCK_STRETCH_HIGH; //fallback an toan
    }
  } else {
    switch(handle->repeatability){
      case sht3x_repeatability_high: return SHT3X_CLOCK_NO_STRETCH_HIGH;
      case sht3x_repeatability_medium: return SHT3X_CLOCK_NO_STRETCH_MEDIUM;
      case sht3x_repeatability_low: return SHT3X_CLOCK_NO_STRETCH_LOW;
      default: return SHT3X_CLOCK_NO_STRETCH_HIGH;
    }
  }
  return 0;
}

static uint16_t get_period_cmd(const sht3x_handle_t *handle){
  switch(handle->freq){
    case sht3x_frequency_05_hz:
      switch(handle->repeatability){
        case sht3x_repeatability_high: return SHT3X_05_MPS_HIGH;
        case sht3x_repeatability_medium: return SHT3X_05_MPS_MEDIUM;
        case sht3x_repeatability_low: return SHT3X_05_MPS_LOW;
      }
      break;

    case sht3x_frequency_1_hz:
      switch(handle->repeatability){
        case sht3x_repeatability_high: return SHT3X_1_MPS_HIGH;
        case sht3x_repeatability_medium: return SHT3X_1_MPS_MEDIUM;
        case sht3x_repeatability_low: return SHT3X_1_MPS_LOW;
      }
      break;

    case sht3x_frequency_2_hz: 
      switch(handle->repeatability){
        case sht3x_repeatability_high: return SHT3X_2_MPS_HIGH;
        case sht3x_repeatability_medium: return SHT3X_2_MPS_MEDIUM;
        case sht3x_repeatability_low: return SHT3X_2_MPS_LOW;
      }
      break;

    case sht3x_frequency_4_hz: 
      switch(handle->repeatability){
        case sht3x_repeatability_high: return SHT3X_4_MPS_HIGH;
        case sht3x_repeatability_medium: return SHT3X_4_MPS_MEDIUM;
        case sht3x_repeatability_low: return SHT3X_4_MPS_LOW;
      }
      break;

    case sht3x_frequency_10_hz:
      switch(handle->repeatability){
        case sht3x_repeatability_high: return SHT3X_10_MPS_HIGH;
        case sht3x_repeatability_medium: return SHT3X_10_MPS_MEDIUM;
        case sht3x_repeatability_low: return SHT3X_10_MPS_LOW;
      }
      break;
    default: return 0; //Truong hop loi
  }
  return 0; //Trong truong hop loi
}

sht3x_status_t sht3x_check_crc(const uint8_t *data, uint8_t len, uint8_t checksum){
  uint8_t crc = SHT3X_CRC_INIT;
  const uint8_t poly = SHT3X_CRC_POLYNOMIAL;

  for(uint8_t i = 0; i < len; i++){
    crc ^= data[i];
    for(uint8_t bit = 0; bit < 8; bit++){
      if(crc & 0x80){
        crc = (crc << 1) ^ poly;
      }else {
        crc = (crc << 1);
      }
    }
  }
  return (crc == checksum) ? SHT3X_OK : SHT3X_ERR_CRC;
}

sht3x_status_t sht3x_enable_heater(sht3x_handle_t *handle, bool enable){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }

  uint16_t cmd = (enable == true) ? SHT3X_CMD_HEATER_ENABLE : SHT3X_CMD_HEATER_DISABLE;
  if(i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, cmd, NULL)){
    handle->heater = enable; //Cap nhat trang thai khi thay doi thanh cong
    return SHT3X_OK;
  }
  return SHT3X_ERR_I2C;
}

sht3x_status_t sht3x_soft_reset(sht3x_handle_t *handle){
  if(handle == NULL)
    return SHT3X_ERR_NULL_PTR;

  bool ok = i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, SHT3X_CMD_SOFT_RESET, NULL);

  if(ok){
    //Reset lai trang thai trong handle
    handle->heater = false;
    handle->state = sht3x_status_idle;
    handle->clock_stretch = false;
    vTaskDelay(pdMS_TO_TICKS(2)); //Toi thieu la 0.5ms theo datasheet, them 2ms cho an toan
    return SHT3X_OK;
  } else {
    return SHT3X_ERR_I2C;
  }
}

sht3x_status_t sht3x_set_repeatability(sht3x_handle_t *handle, sht3x_repeatability_t repeat){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }
  handle->repeatability = repeat;
  return SHT3X_OK;
}

sht3x_status_t sht3x_set_frequency(sht3x_handle_t *handle, sht3x_frequency_t freq){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }
  handle->freq = freq;
  return SHT3X_OK;
}

sht3x_status_t sht3x_set_mode(sht3x_handle_t *handle, sht3x_mode_t mode){
  if(handle == NULL)
    return SHT3X_ERR_NULL_PTR;
  
  if(handle->mode == sht3x_mode_periodic){
    sht3x_status_t ret = sht3x_stop_periodart(handle);
    if(ret != SHT3X_OK)
      return ret;
  }

  //Cap nhat mode moi vao handle
  handle->mode = mode;
  return SHT3X_OK;
}

sht3x_status_t sht3x_set_clock_stretch(sht3x_handle_t *handle, bool clock_stretch){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }
  handle->clock_stretch = clock_stretch;
  return SHT3X_OK;
}

sht3x_status_t sht3x_stop_periodart(sht3x_handle_t *handle){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }

  //Neu che do khong phai period thi ko stop
  if(handle->mode != sht3x_mode_periodic){
    return SHT3X_ERR_WRONG_MODE;
  }

  //Gui lenh BREAK de dung che do period
  if(!i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, SHT3X_CMD_STOP_PERIOD, NULL)){
    return SHT3X_ERR_I2C;
  }

  //Sau khi stop che do period, cam bien se ve trang thai `idle`
  handle->state = sht3x_status_idle;

  return SHT3X_OK;
}

sht3x_status_t sht3x_initDevice(sht3x_handle_t *handle, I2C_dev_init_t *dev, i2c_port_t port, gpio_num_t scl_pin, gpio_num_t sda_pin, bool addr_pin){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }
  
  //Cau hinh I2C cho cam bien voi thanh vien i2c_dev
  handle->i2c_dev.port = port;
  handle->i2c_dev.cfg.mode = I2C_MODE_MASTER;
  handle->i2c_dev.cfg.scl_io_num = scl_pin;
  handle->i2c_dev.cfg.sda_io_num = sda_pin;
  handle->i2c_dev.cfg.master.clk_speed = 400000; // Tuy chinh (Mac dinh la 10000)

  //Dia chi I2C tuy vao chan AD (true -> noi 3.3v: 0x45, false -> noi gnd: default 0x44)
  handle->i2c_addr = addr_pin ? SHT3X_ADDRESS_VDD : SHT3X_ADDRESS_DEFAULT;
  handle->i2c_dev.address = handle->i2c_addr;

  // Copy cau hinh I2C sang 
  *dev = handle->i2c_dev;

  //Cau hinh chuc nang cua cam bien
  handle->clock_stretch = false;
  handle->heater = false;

  I2C_dev_create_mutex(&handle->i2c_dev);

  return SHT3X_OK;
}

sht3x_status_t sht3x_start_single_shot(sht3x_handle_t *handle, sht3x_data_t *data){
  if(handle == NULL || data == NULL){
    return SHT3X_ERR_NULL_PTR;
  }
  
  //Cap nhat trang thai do 
  handle->state = sht3x_status_measuring;

  uint16_t cmd = get_single_shot_cmd(handle);

  //Gui lenh do single_shot
  if(i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, cmd, NULL) != ESP_OK){
    handle->state = sht3x_status_idle;
    ESP_LOGE("SHT3X", "Loi ghi command !");
    return SHT3X_ERR_I2C;
  }

  // Neu khong bat clock stretching -> delay theo do chinh xac
  //Can phai delay trong khoang thoi gian theo datasheet de du lieu tu cam bien san sang
  if(handle->clock_stretch == false){
    switch(handle->repeatability){
      case sht3x_repeatability_high:
        vTaskDelay(pdMS_TO_TICKS(15));
        break;

      case sht3x_repeatability_medium:
        vTaskDelay(pdMS_TO_TICKS(6));
        break;

      case sht3x_repeatability_low:
        vTaskDelay(pdMS_TO_TICKS(4));
        break;  

      default:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    }
  }

  //Doc du lieu: 6 bytes (Temp MSB, Temp LSB, CRC) + (Humi MSB, Humi LSB, CRC)
  uint8_t raw_data[6] = {0};
  if(i2cdev_read_bytes_data(&handle->i2c_dev, handle->i2c_addr, raw_data, 6, NULL) != ESP_OK){
    handle->state = sht3x_status_idle;
    ESP_LOGE("SHT3X", "Loi doc du lieu !");
    return SHT3X_ERR_I2C;
  }

  //Kiem tra CRC
  if(sht3x_check_crc(&raw_data[0], 2, raw_data[2]) != SHT3X_OK || 
    sht3x_check_crc(&raw_data[3], 2, raw_data[5]) != SHT3X_OK) {
    handle->state = sht3x_status_idle;
    return SHT3X_ERR_CRC;
  }
  
  // Convert du lieu raw thanh du lieu thuc
  uint16_t temp_raw = (raw_data[0] << 8) | raw_data[1];
  uint16_t humi_raw = (raw_data[3] << 8) | raw_data[4];

  data->temperature_C = -45 + 175 * ((float)temp_raw / 65536.0f); //(°C)
  data->temperature_F = -45 + 315 * ((float)temp_raw / 65536.0f); //(°F)
  data->humidity = 100 * ((float)humi_raw / 65536.0f);

  //Sau moi lan do single-shot, hardware cam bien lai ve trang thai idle 
  handle->state = sht3x_status_idle;

  return SHT3X_OK;
}

sht3x_status_t sht3x_start_period(sht3x_handle_t *handle){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }

  uint16_t cmd = get_period_cmd(handle);
  
  if(!i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, cmd, NULL)){
    ESP_LOGE("SHT3X", "Failed to send periodic measurement command: 0x%04X", cmd);
    handle->state = sht3x_status_idle;
    return SHT3X_ERR_I2C;
  }

  handle->state = sht3x_status_measuring;
  return SHT3X_OK;
}

sht3x_status_t sht3x_fetch_data(sht3x_handle_t *handle, sht3x_data_t *data){
  if(handle == NULL || data == NULL){
    return SHT3X_ERR_NULL_PTR;
  }

  if(handle->state != sht3x_status_measuring){
    return SHT3X_ERR_INVALID_STATE;
  }

  //Gui lenh fetch 
  if(!i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, SHT3X_CMD_FETCH_DATA, NULL)){
    handle->state = sht3x_status_idle;
    return SHT3X_ERR_I2C;
  }

  //Doc du lieu
  uint8_t raw_data[6] = {0};
  if(!i2cdev_read_bytes_data(&handle->i2c_dev, handle->i2c_addr, raw_data, 6, NULL)){
    handle->state = sht3x_status_idle;
    return SHT3X_ERR_I2C;
  }

  //Kiem tra CRC 
  if(sht3x_check_crc(&raw_data[0], 2, raw_data[2]) != SHT3X_OK ||
    sht3x_check_crc(&raw_data[3], 2, raw_data[5]) != SHT3X_OK){
    handle->state = sht3x_status_idle;
    return SHT3X_ERR_CRC;
  }

  //Convert sang nhiet do va do am
  uint16_t temp_raw = (raw_data[0] << 8 | raw_data[1]);
  uint16_t humi_raw = (raw_data[3] << 8 | raw_data[4]);

  data->temperature_C = -45 + 175 * ((float)temp_raw / 65536.0f); //(°C)
  data->temperature_F = -45 + 315 * ((float)temp_raw / 65536.0f); //(°F)
  data->humidity = 100 * ((float)humi_raw / 65536.0f);

  return SHT3X_OK;
}

sht3x_status_t sht3x_read_status(sht3x_handle_t *handle, uint16_t *status){
  if(handle == NULL || status == NULL){
    return SHT3X_ERR_NULL_PTR;
  }

  //Gui lenh doc status register 
  if(!i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, SHT3X_CMD_READ_STATUS, NULL)){
    return SHT3X_ERR_I2C;
  }

  //Doc 2 byte status + 1 byte CRC
  uint8_t raw[3] = {0};
  if(!i2cdev_read_bytes_data(&handle->i2c_dev, handle->i2c_addr, raw, 3, NULL)){
    return SHT3X_ERR_I2C;
  }

  //Kiem tra CRC 
  if(sht3x_check_crc(raw, 2, raw[2]) != SHT3X_OK){
    return SHT3X_ERR_CRC;
  }

  //Ghep thanh 16-bit status
  *status = (raw[0] << 8) | raw[1];

  return SHT3X_OK;
}

sht3x_status_t sht3x_clear_status(sht3x_handle_t *handle){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }

  //Gui lenh clear status register
  if(!i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, SHT3X_CMD_CLEAR_STATUS, NULL)){
    return SHT3X_ERR_I2C;
  }
  return SHT3X_OK;
}

sht3x_status_t sht3x_start_art(sht3x_handle_t *handle){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }
 
  //ART chi hoat dong trong che do period
  handle->mode = sht3x_mode_periodic;
  handle->repeatability = sht3x_repeatability_high; //ART yeu cau su dung high repeatability
  handle->freq = sht3x_frequency_4_hz; //ART su dung tan so 4Hz

  //Gui lenh ART  
  if(!i2cdev_write_command(&handle->i2c_dev, handle->i2c_addr, SHT3X_CMD_ART_MODE, NULL)){
    handle->state = sht3x_status_idle;
    return SHT3X_ERR_I2C;
  }
  
  handle->state = sht3x_status_measuring;
  return SHT3X_OK;
}

sht3x_status_t sht3x_process_by_mode(sht3x_handle_t *handle, sht3x_data_t *data){
  if(handle == NULL){
    return SHT3X_ERR_NULL_PTR;
  }

  switch(handle->mode){
    case sht3x_mode_single_shot:
      return sht3x_start_single_shot(handle, data); //Tra ve trang thai doc duoc hay khong

    case sht3x_mode_periodic: {
      sht3x_status_t ret = sht3x_start_period(handle);
      if(ret != SHT3X_OK) return ret;
      return sht3x_fetch_data(handle, data);
    }

    case sht3x_mode_art: {
      sht3x_status_t ret = sht3x_start_art(handle);
      if(ret != SHT3X_OK) return ret;
      return sht3x_fetch_data(handle, data);
    }

    default: 
      return SHT3X_ERR_INVALID_PARAM;
  }
}

#ifdef __cplusplus
}
#endif