/**
  ******************************************************************************
  * @file    eeprom.c
  * @brief   M24C02 2Kbit EEPROM driver implementation
  *          Uses HAL I2C MEMORY functions over I2C2.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef EEPROM_CheckAddrRange(uint16_t addr, uint16_t len);

/* ---------------------------------------------------------------------------*/
/**
  * @brief  Verify that the address range fits within the EEPROM.
  * @param  addr  Start address.
  * @param  len   Number of bytes.
  * @retval HAL_OK if valid, HAL_ERROR if out of bounds.
  */
static HAL_StatusTypeDef EEPROM_CheckAddrRange(uint16_t addr, uint16_t len)
{
    if ((addr >= EEPROM_SIZE) || (len > EEPROM_SIZE) ||
        ((uint32_t)addr + (uint32_t)len > (uint32_t)EEPROM_SIZE))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief  Initialise the EEPROM driver.
  *         Performs a dummy read of address 0 to verify the chip is present.
  * @retval HAL_OK if the chip responds, error otherwise.
  */
HAL_StatusTypeDef EEPROM_Init(void)
{
    uint8_t dummy;
    /* A single-byte read from address 0 probes the device */
    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDR, 0x00,
                            I2C_MEMADD_SIZE_8BIT, &dummy, 1,
                            HAL_MAX_DELAY);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief  Read a block of bytes from the EEPROM.
  * @param  addr  Start address.
  * @param  buf   Destination buffer.
  * @param  len   Number of bytes to read.
  * @retval HAL status.
  */
HAL_StatusTypeDef EEPROM_Read(uint16_t addr, uint8_t *buf, uint16_t len)
{
    if (EEPROM_CheckAddrRange(addr, len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (buf == NULL)
    {
        return HAL_ERROR;
    }

    if (len == 0U)
    {
        return HAL_OK;
    }

    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDR, (uint16_t)addr,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            HAL_MAX_DELAY);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief  Write a block of bytes to the EEPROM.
  *         Breaks long writes into page-aligned chunks and waits for the
  *         internal write cycle after each chunk.
  * @param  addr  Start address.
  * @param  buf   Source buffer.
  * @param  len   Number of bytes to write.
  * @retval HAL status.
  */
HAL_StatusTypeDef EEPROM_Write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    uint16_t remaining = len;
    uint16_t offset    = 0U;
    uint16_t chunk;

    if (EEPROM_CheckAddrRange(addr, len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (buf == NULL)
    {
        return HAL_ERROR;
    }

    if (len == 0U)
    {
        return HAL_OK;
    }

    while (remaining > 0U)
    {
        /* Bytes that fit in the current page before a page boundary */
        uint16_t page_remain = EEPROM_PAGE_SIZE - ((addr + offset) & (EEPROM_PAGE_SIZE - 1U));

        chunk = (remaining < page_remain) ? remaining : page_remain;

        if (HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDR, (uint16_t)(addr + offset),
                              I2C_MEMADD_SIZE_8BIT, (uint8_t *)(buf + offset),
                              chunk, HAL_MAX_DELAY) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Wait for internal write cycle to complete (5 ms max) */
        HAL_Delay(EEPROM_WRITE_TIME);

        offset    += chunk;
        remaining -= chunk;
    }

    return HAL_OK;
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief  Write a single byte to the EEPROM.
  * @param  addr  Address.
  * @param  data  Byte to write.
  * @retval HAL status.
  */
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data)
{
    if (EEPROM_CheckAddrRange(addr, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDR, (uint16_t)addr,
                          I2C_MEMADD_SIZE_8BIT, &data, 1,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Wait for internal write cycle */
    HAL_Delay(EEPROM_WRITE_TIME);

    return HAL_OK;
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief  Read a single byte from the EEPROM.
  * @param  addr    Address.
  * @param  status  Pointer to store HAL status (may be NULL).
  * @retval Byte read, or 0xFF on error.
  */
uint8_t EEPROM_ReadByte(uint16_t addr, HAL_StatusTypeDef *status)
{
    uint8_t          data = 0xFFU;
    HAL_StatusTypeDef ret;

    if (EEPROM_CheckAddrRange(addr, 1U) != HAL_OK)
    {
        if (status != NULL)
        {
            *status = HAL_ERROR;
        }
        return 0xFFU;
    }

    ret = HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDR, (uint16_t)addr,
                           I2C_MEMADD_SIZE_8BIT, &data, 1,
                           HAL_MAX_DELAY);

    if (status != NULL)
    {
        *status = ret;
    }

    return data;
}
