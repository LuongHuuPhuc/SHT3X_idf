/**
 * @author Luong Huu Phuc
 * @date 2025/06/29
 * @file sht3x.h - Library/driver for Humidity and Temperature sensor SHT3X
 * @related Datasheet SHT3x-DIS (SENSIRION)
 */

#ifndef __SHT3X_H__
#define __SHT3X_H__

#pragma once 

#ifdef __cplusplus
#define EXPORT extern "C" 
#else
#define EXPORT
#endif

#include <stdint.h>
#include <stdbool.h>
#include "I2C_dev.h"

//-> START BIT | I2C Address | R/W | A | Command MSB | A | Command LSB | A |  CRC (Status register)  | STOP BIT ->
//-------------<---7 bits---><1 bit>---<----------16 bits command------>---<---------8 bits---------->
/** 
 * @note === CAC BUOC TRUYEN DU LIEU I2C DE DOC STATUS REGISTER ===
 * Buoc 1: Gui lenh READ STATUS 
 * 1. Start condition (S): Tin hieu tren bus I2C 
 * 2. I2C address + Write (W): Dia chi cam bien (7-bits) + bit Write (0)
 * 3. Command MSB + LSB: Lenh `0xF32D` duoc gui thanh 2 byte MSB - `0xF3`, LSB - `0x2D`
 * 4. ACK sau moi byte
 * START -> (I2C_ADDR + W) -> 0xF3 -> 0x2D -> ACK -> ...

 * Buoc 2: Doc du lieu
 * 1. Start condition (S): Bat dau phien doc 
 * 2. I2C address + Read (R): Dia chi cam bien (7-bits) + bit read (1)
 * 3. Register MSB + LSB: 2 byte du lieu cua thanh ghi trang thai MSB - Byte cao, LSB - Byte thap
 * 4. CRC: 1 byte kiem tra CRC cho 2 byte du lieu MSB + LSB
 * 5. ACK giua cac byte va NACK + Stop (P) sau CRC
 * START -> (I2C_ADDR + R) -> MSB -> LSB -> CRC -> P
 * 
 * @return Byte 1 - MSB cua status, Byte 2 - LSB cua status, Byte 3 - CRC checksum
*/ 

#define READ_ADDR(addr)               ((addr << 1) | 0x01)
#define WRITE_ADDR(addr)              (addr << 1)

// Dia chi cam bien (7-bits + bit cuoi la R/W)
// Noi chan AD - VCC -> 0x44
// Noi chan AD - GND -> 0x45
#define SHT3X_ADDRESS_DEFAULT          __UINT16_C(0x44) 
#define SHT3X_ADDRESS_VDD              __UINT16_C(0x45)

// == Single shot measurement - Clock stretching enabled (Keo dai thoi gian clock cua SCL) ==
// La tin hieu wait cua Slave keo SCL xuong LOW khi chua san sang (can them thoi gian do/du lieu chua san sang)
// Luc nay Master co the ngay lap tuc doc du lieu, neu Slave chua san sang, Master co the doi, khong can delay hay doi dung thoi gian
// Phan cung I2C cua MCU phai ho tro thi moi dung duoc 
#define SHT3X_CLOCK_STRETCH_HIGH      (0x2C06)  //Do chinh xac cao - cham
#define SHT3X_CLOCK_STRETCH_MEDIUM    (0x2C0D)  //Do chinh xac trung binh
#define SHT3X_CLOCK_STRETCH_LOW       (0x2C10)  //Do chinh xac thap - nhanh

// == Single shot measurement - Clock stretching disabled (Khong keo dai SCL) ==
// Sau khi gui lenh do, Master phai doi mot khoang thoi gian (vai ms) truoc khi gui lenh doc du lieu
// Neu doc qua som, se nha du lieu sai hoac loi CR, can phai delay vai ms
// Mot so MCU khong ho tro Clock Stretching nen phai dung cai nay
#define SHT3X_CLOCK_NO_STRETCH_HIGH   (0x2400) // ~ 15ms
#define SHT3X_CLOCK_NO_STRETCH_MEDIUM (0x240B) // ~ 6ms
#define SHT3X_CLOCK_NO_STRETCH_LOW    (0x2416) // ~ 4ms

// == Period measurement - Do lien tuc theo tan so co dinh ==
// Do lien tuc, tu dong lay mau nhiet do & do am voi toc do dinh san (so lan do/giay, tinh bang Hz)
// Luu du lieu vao data register, MCU co the doc du lieu bat ky luc nao ma khong can khoi dong lai qua trinh do
// Do chinh xac cao nhung tieu thu dien cao, neu khong goi Break Command, cam bien se lien tuc do
// 0.5 Hz
#define SHT3X_05_MPS_HIGH             (0x2032)
#define SHT3X_05_MPS_MEDIUM           (0x2024)
#define SHT3X_05_MPS_LOW              (0x202F)

// 1 Hz
#define SHT3X_1_MPS_HIGH              (0x2130)
#define SHT3X_1_MPS_MEDIUM            (0x2126)
#define SHT3X_1_MPS_LOW               (0x212D)

// 2 Hz
#define SHT3X_2_MPS_HIGH              (0x2236)
#define SHT3X_2_MPS_MEDIUM            (0x2220)
#define SHT3X_2_MPS_LOW               (0x222B)

// 4 Hz
#define SHT3X_4_MPS_HIGH              (0x2334)
#define SHT3X_4_MPS_MEDIUM            (0x2322)
#define SHT3X_4_MPS_LOW               (0x2329)

// 10 Hz
#define SHT3X_10_MPS_HIGH             (0x2737)
#define SHT3X_10_MPS_MEDIUM           (0x2721)
#define SHT3X_10_MPS_LOW              (0x272A)

// Commands (16-bits)
#define SHT3X_CMD_SOFT_RESET          (0x30A2)
#define SHT3X_CMD_HEATER_ENABLE       (0x306D)
#define SHT3X_CMD_HEATER_DISABLE      (0x3066)
#define SHT3X_CMD_READ_STATUS         (0xF32D)
#define SHT3X_CMD_CLEAR_STATUS        (0x3041)
#define SHT3X_CMD_STOP_PERIOD        (0x3093) 
#define SHT3X_CMD_FETCH_DATA          (0xE000) //Lay du lieu moi nhat tu bo nho
#define SHT3X_CMD_ART_MODE            (0x2B32) //Che do do voi tan so 4Hz

// CRC (Checksum Calculation - 8bits)
#define SHT3X_CRC_POLYNOMIAL          (0x31)
#define SHT3X_CRC_INIT                (0xFF)

typedef enum {
  SHT3X_OK = 0,
  SHT3X_ERR_I2C,
  SHT3X_ERR_CRC,
  SHT3X_ERR_TIMEOUT,
  SHT3X_ERR_INVALID_PARAM,
  SHT3X_ERR_NULL_PTR,
  SHT3X_ERR_WRONG_MODE,
  SHT3X_ERR_INVALID_STATE
} sht3x_status_t;

typedef enum {
  sht3x_repeatability_high,
  sht3x_repeatability_medium,
  sht3x_repeatability_low
} sht3x_repeatability_t;

typedef enum {
  sht3x_mode_single_shot,
  sht3x_mode_periodic,
  sht3x_mode_art,
} sht3x_mode_t;

//Quan ly trang thai cua cam bien
typedef enum {
  sht3x_status_idle, // Dang trang thai tiet kiem nang luong
  sht3x_status_measuring, //Dang do
  sht3x_status_data_ready //Do xong, du lieu da san sang (dung cho `period`)
} sht3x_state_t;

typedef enum {
  sht3x_frequency_05_hz,
  sht3x_frequency_1_hz,
  sht3x_frequency_2_hz,
  sht3x_frequency_4_hz,
  sht3x_frequency_10_hz
} sht3x_frequency_t;

typedef enum __attribute__((unused)) {
  SHT3X_MODE_CHANGE_ONLY, //Chi thay doi mode va khong thuc thi
  SHT3X_MODE_CHANGE_AND_EXECUTE // Thay doi mode va thuc thi luon
} sht3x_mode_change_t;

typedef struct __sht3x_data {
  float temperature_C;
  float temperature_F;
  float humidity;
} sht3x_data_t;

typedef struct {
  uint8_t i2c_addr;
  sht3x_state_t state;
  sht3x_mode_t mode; //Che do do
  sht3x_repeatability_t repeatability; //Do lap (high, medium, low)
  sht3x_frequency_t freq; //Tan so che do period
  bool heater; //Trang thai heater (bat/tat)
  bool clock_stretch; //Trang thai clock stretch (bat/tat)
  I2C_dev_init_t i2c_dev; //Struct I2C (Khong can khai bao trong file main.c nua) 
} sht3x_handle_t;

// == Function API ==

/**
 * @brief Khoi tao cau hinh cam bien 
 * @note Sau khi truyen het tham so vao, goi ham `I2Cdev_install_device()` cua `I2C_dev.h` ngay sau do de khoi tao I2C cho cam bien 
 */
EXPORT sht3x_status_t sht3x_initDevice(sht3x_handle_t *handle, I2C_dev_init_t *dev, i2c_port_t port, gpio_num_t scl_pin, gpio_num_t sda_pin, bool addr_pin);

/**
 * @brief Thuc hien reset mem cam bien ve trang thai mac dinh
 */
EXPORT sht3x_status_t sht3x_soft_reset(sht3x_handle_t *handle);

/**
 * @brief Bat dau doc du lieu che do single shot (1 lan)
 * @note Thiet lap het thong so (`mode`, `repeat`, `freq`, `clock`) roi moi goi ham nay
 * \note - Phai set che do do bang ham `sht3x_set_mode()` truoc khi goi ham
 * @return - SHT3X_OK: Ghi command thanh cong
 */
EXPORT sht3x_status_t sht3x_start_single_shot(sht3x_handle_t *handle, sht3x_data_t *data);

/**
 * @brief Khoi dong che do period (lien tuc)
 * @note Thiet lap het thong so (`mode`, `repeat`, `freq`, `clock`) roi moi goi ham nay
 * \note - Phai set che do do bang ham `sht3x_set_mode()` truoc khi goi ham 
 * @return - SHT3X_OK: Ghi command thanh cong
 */
EXPORT sht3x_status_t sht3x_start_period(sht3x_handle_t *handle);

/**
 * @brief Khoi dong che do ART (Accelerated Response Time) voi tan so 4Hz va thoi gian phan hoi nhanh
 * @note - Che do nay duoc nha san xuat toi uu hoa noi bo thuat toan loc & chuyen doi ADC
 * @note - Che do nay chi co o repeatability `high` - do chinh xac cao, voi thoi gian phan hoi nhanh hon
 * so voi che do do 4Hz cua `period`
 * @return - SHT3X_OK: Ghi command thanh cong
 */
EXPORT sht3x_status_t sht3x_start_art(sht3x_handle_t *handle);

/**
 * @brief Ham lay du lieu tu data register cho che do `period` va che do `ART`
 * @note Neu khong lay data kip ben trong `data register` thi data cu se bi ghi de
 * \note - Phai delay 1 khoang `T = 1 / freq + T_conv`, voi `T_conv` la thoi gian chuyen doi tin hieu ADC -> so.
 * \note - Low ~4ms, Medium ~6ms, High ~15ms
 */
EXPORT sht3x_status_t sht3x_fetch_data(sht3x_handle_t *handle, sht3x_data_t *data);

/**
 * @brief Ham stop che do `period` va dua cam bien ve che do `sht3x_mode_idle`
 */
EXPORT sht3x_status_t sht3x_stop_periodart(sht3x_handle_t *handle);

/**
 * @brief Ham bat heater cam bien de loai bo hoi nuoc ngung dong tu moi truong
 * @brief - Nhiet do cam bien cung se tang len, gay sai so. Nen bat mot luc roi tat di
 */
EXPORT sht3x_status_t sht3x_enable_heater(sht3x_handle_t *handle, bool enable);

/**
 * @brief Ham de doc trang thai cua cam bien tu status register (2 bytes status + 1 byte CRC)
 * @note Phuc vu debug/check loi
 */
EXPORT sht3x_status_t sht3x_read_status(sht3x_handle_t *handle, uint16_t *status);

/**
 * @brief Ham de xoa trang thai cua cam bien tu status register (2 bytes status + 1 byte CRC)
 * @note Xoa loi trong thanh ghi trang thai
 */
EXPORT sht3x_status_t sht3x_clear_status(sht3x_handle_t *handle);

/**
 * @brief Ham thay doi repeatability trong luc chay (neu can)
 * @note Dung cho ca che do `period` va `single shot`
 */
EXPORT sht3x_status_t sht3x_set_repeatability(sht3x_handle_t *handle, sht3x_repeatability_t repeat);

/**
 * @brief Ham thay doi frequency trong luc chay (neu can)
 * @note Dung cho che do do `period`
 */
EXPORT sht3x_status_t sht3x_set_frequency(sht3x_handle_t *handle, sht3x_frequency_t freq);

/**
 * @brief Ham set clock stretch
 * @note Dung cho che do do `single shot`
 */
EXPORT sht3x_status_t sht3x_set_clock_stretch(sht3x_handle_t *handle, bool clock_stretch);

/**
 * @brief Ham set che do do & kiem tra & thuc thi che do do (neu can)
 */
EXPORT sht3x_status_t sht3x_set_mode(sht3x_handle_t *handle, sht3x_mode_t mode);

/**
 * @brief Ham tu dong xu ly che do theo ham `sht3x_set_mode()`
 * @note Phai goi ham `sht3x_set_mode()` thi moi su dung duoc ham nay
 */
EXPORT sht3x_status_t sht3x_process_by_mode(sht3x_handle_t *handle, sht3x_data_t *data);

/**
 * @brief Ham kiem tra CRC (Checksum) de debug
 * @note Bat buoc phai checksum de dam bao du lieu doc dung (dac biet trong moi truong nhieu dien tu cao)
 * \note - Trong datasheet (trang 13) co noi: "Voi moi du lieu do, CRC 8-bits se duoc truyen theo 2 bytes du lieu (1 cho temp, 1 cho humi)"
 */
EXPORT sht3x_status_t sht3x_check_crc(const uint8_t *data, uint8_t len, uint8_t checksum);

/**
 * @brief Ham doc du lieu raw nhiet do va do am cua cam bien
 * @note Dung de phuc vu cho xu ly convert rieng
 * @param handle Doi tuong cua kieu du lieu sht3x_handle_t 
 * @param raw_temp Con tro de nhan gia tri nhiet do raw (16-bit)
 * @param raw_hum Con tro nhan gia tri do am raw (16-bit)
 */
EXPORT sht3x_status_t sht3x_read_single_shot_raw(sht3x_handle_t *handle, uint16_t *raw_temp, uint16_t *raw_hum);

#define SHT3X_ERR_CHECK(func) \
    do { \
      sht3x_status_t __err_rc = (func); \
      if(__err_rc != SHT3X_OK){ \
        ESP_LOGE("SHT3X", "%s failed: 0x%02X", #func, __err_rc); \
        return __err_rc; \
      } \
    } while(0)
    
#endif // __SHT3X_H__