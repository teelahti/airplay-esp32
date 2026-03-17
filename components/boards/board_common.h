#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>

typedef void *board_res_handle_t;

typedef enum {
  NULL_RESOURCE = 0,
  BOARD_I2C0_ID,
  BOARD_SPI2_ID,
  BOARD_DAC_ID,
} board_res_id_t;

/**
 * @brief Initialize board-specific hardware
 *
 * This function is called early during startup to initialize any
 * board-specific peripherals such as DACs, GPIOs, power management, etc.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t iot_board_init(void);

/**
 * @brief Deinitialize board-specific hardware
 *
 * This function is called during shutdown to clean up board-specific
 * resources.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t iot_board_deinit(void);

/**
 * @brief Check if board is initialized
 *
 * @return true if board is initialized, false otherwise
 */
bool iot_board_is_init(void);

/**
 * @brief Get a handle to a board resource
 *
 * @param id Resource identifier (from board_res_id_t)
 * @return Handle to the resource, or NULL if not available
 */
board_res_handle_t iot_board_get_handle(int id);

/**
 * @brief Get board information string
 *
 * @return Board name string (never NULL)
 */
const char *iot_board_get_info(void);

/* ========== Common I2C bus helpers ========== */

/** Default I2C timeout in ms used by the helpers below. */
#define BOARD_I2C_TIMEOUT_MS 100

/**
 * @brief Add an I2C device to a master bus
 *
 * @param bus        Master bus handle
 * @param addr       7-bit device address
 * @param speed_hz   SCL clock speed in Hz
 * @param[out] dev   Resulting device handle
 * @return ESP_OK on success
 */
esp_err_t board_i2c_add_device(i2c_master_bus_handle_t bus, uint8_t addr,
                               uint32_t speed_hz, i2c_master_dev_handle_t *dev);

/**
 * @brief Remove an I2C device from the bus
 */
esp_err_t board_i2c_remove_device(i2c_master_dev_handle_t dev);

/**
 * @brief Write data to an I2C device register
 *
 * Prepends the register byte to @p data and transmits in a single
 * I2C transaction.
 *
 * @param dev   Device handle
 * @param reg   Register address
 * @param data  Payload bytes
 * @param len   Number of payload bytes
 * @return ESP_OK on success
 */
esp_err_t board_i2c_write(i2c_master_dev_handle_t dev, uint8_t reg,
                          const uint8_t *data, size_t len);

/**
 * @brief Write raw bytes to an I2C device (no register prefix)
 *
 * @param dev   Device handle
 * @param data  Payload bytes
 * @param len   Number of bytes
 * @return ESP_OK on success
 */
esp_err_t board_i2c_write_raw(i2c_master_dev_handle_t dev, const uint8_t *data,
                              size_t len);

/**
 * @brief Read data from an I2C device register
 *
 * Sends the single-byte register address, then reads @p len bytes.
 *
 * @param dev   Device handle
 * @param reg   Register address
 * @param data  Buffer to receive data
 * @param len   Number of bytes to read
 * @return ESP_OK on success
 */
esp_err_t board_i2c_read(i2c_master_dev_handle_t dev, uint8_t reg,
                         uint8_t *data, size_t len);
