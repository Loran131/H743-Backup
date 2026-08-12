/**
  ******************************************************************************
  * @file    eeprom.h
  * @brief   M24C02 2Kbit EEPROM driver API declarations
  *          I2C2: PH4=SCL, PH5=SDA (AF4, Open-Drain, external pull-up 2k2 to 3.3V)
  ******************************************************************************
  */

#ifndef __EEPROM_H__
#define __EEPROM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"

/* Defines -------------------------------------------------------------------*/
#define EEPROM_ADDR       0xA0U   /* 8-bit I2C address (7-bit: 0x50 << 1) */
#define EEPROM_SIZE        256U   /* Total bytes (2 Kbit)                   */
#define EEPROM_PAGE_SIZE    16U   /* Page size for page writes              */
#define EEPROM_WRITE_TIME    5U   /* Max internal write cycle time (ms)     */

/* API -----------------------------------------------------------------------*/

/**
  * @brief  Initialise the EEPROM driver and verify the chip is present.
  * @retval HAL_OK if the chip responds at the expected address, error otherwise.
  */
HAL_StatusTypeDef EEPROM_Init(void);

/**
  * @brief  Read a block of bytes from the EEPROM.
  * @param  addr  Start address within the EEPROM (0 .. EEPROM_SIZE-1).
  * @param  buf   Pointer to the destination buffer.
  * @param  len   Number of bytes to read.
  * @retval HAL_OK on success, HAL_ERROR if out-of-range, or HAL I2C error.
  */
HAL_StatusTypeDef EEPROM_Read(uint16_t addr, uint8_t *buf, uint16_t len);

/**
  * @brief  Write a block of bytes to the EEPROM (handles page boundaries
  *         internally).
  * @param  addr  Start address within the EEPROM (0 .. EEPROM_SIZE-1).
  * @param  buf   Pointer to the source buffer.
  * @param  len   Number of bytes to write.
  * @retval HAL_OK on success, HAL_ERROR if out-of-range, or HAL I2C error.
  */
HAL_StatusTypeDef EEPROM_Write(uint16_t addr, const uint8_t *buf, uint16_t len);

/**
  * @brief  Write a single byte to the EEPROM.
  * @param  addr  Address within the EEPROM (0 .. EEPROM_SIZE-1).
  * @param  data  Byte to write.
  * @retval HAL_OK on success, HAL_ERROR if out-of-range, or HAL I2C error.
  */
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data);

/**
  * @brief  Read a single byte from the EEPROM.
  * @param  addr    Address within the EEPROM (0 .. EEPROM_SIZE-1).
  * @param  status  Pointer to store the operation status (may be NULL).
  * @retval The byte read, or 0xFF on error (status will indicate the error).
  */
uint8_t EEPROM_ReadByte(uint16_t addr, HAL_StatusTypeDef *status);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H__ */
