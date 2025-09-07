#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "I2C_dev.h"
#include "sht3x.h"

#define SCL_PIN     GPIO_NUM_22
#define SDA_PIN     GPIO_NUM_21
#define ADDR_PIN    false //GND
#define I2C_PORT    I2C_NUM_0
#define TAG         "SHT3X"

TaskHandle_t sht3x_task_handle = NULL;
sht3x_handle_t handle; 
I2C_dev_init_t dev;

sht3x_status_t sht3x_install(void){
  //Khoi tao giao tiep I2C
  SHT3X_ERR_CHECK(sht3x_initDevice(&handle, &dev, I2C_PORT, SCL_PIN, SDA_PIN, ADDR_PIN));
  ESP_ERROR_CHECK(I2Cdev_install_device(&dev));
  ESP_LOGI(TAG, "I2C config OK ! Address: 0x%02X", handle.i2c_addr);

  //Khoi tao thong so 
  SHT3X_ERR_CHECK(sht3x_set_clock_stretch(&handle, false)); // Khong bat clock stretch
  SHT3X_ERR_CHECK(sht3x_set_repeatability(&handle, sht3x_repeatability_high)); //Do chinh xac cao
  SHT3X_ERR_CHECK(sht3x_set_mode(&handle, sht3x_mode_single_shot)); //Che do single-shot

  return SHT3X_OK;
}

void sht3x_task(void *pvParameter){
  sht3x_data_t data;

  // while(1){
  //   sht3x_status_t ret = sht3x_start_single_shot(&handle, &data);
  //   if(ret == SHT3X_OK){
  //     printf("%.2f,%.2lf,%.2f\n", data.temperature_C, data.temperature_F, data.humidity);
  //   }else {
  //     ESP_LOGI(TAG, "Doc du lieu SHT3X that bai, ma loi: %d", ret);
  //   }
  //   vTaskDelay(pdMS_TO_TICKS(1000)); //Delay 2s giua cac lan do
  // }
  
  while(1){
    if(sht3x_process_by_mode(&handle, &data) == SHT3X_OK){
      printf("%.2f,%.2lf,%.2f\n", data.temperature_C, data.temperature_F, data.humidity);
    }else {
      ESP_LOGI(TAG, "Doc du lieu SHT3X that bai !");  
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void){
  ESP_ERROR_CHECK(i2c_dev_initialize());
  if(sht3x_install() != SHT3X_OK){
    ESP_LOGE(TAG, "Loi khoi tao SHT3X !");
    for(;;){
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  ESP_LOGI(TAG, "Khoi tao SHT3X thanh cong !");
  xTaskCreatePinnedToCore(sht3x_task, "sht3x task", 1024 * 2, NULL, 5, &sht3x_task_handle, 0);
}