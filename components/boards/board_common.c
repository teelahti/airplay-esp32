#include "board_common.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

static const char TAG[] = "board_common";

/**
 * Weak default implementations of iot_board_*() functions.
 * Board-specific board.c files override these as needed.
 */

__attribute__((weak)) esp_err_t iot_board_init(void) {
  return ESP_OK;
}

__attribute__((weak)) esp_err_t iot_board_deinit(void) {
  return ESP_OK;
}

__attribute__((weak)) bool iot_board_is_init(void) {
  return false;
}

__attribute__((weak)) board_res_handle_t iot_board_get_handle(int id) {
  (void)id;
  return NULL;
}

__attribute__((weak)) const char *iot_board_get_info(void) {
  return "Unknown Board";
}

/* ========== Common I2C bus helpers ========== */

esp_err_t board_i2c_add_device(i2c_master_bus_handle_t bus, uint8_t addr,
                               uint32_t speed_hz,
                               i2c_master_dev_handle_t *dev) {
  if (bus == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  i2c_device_config_t cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = addr,
      .scl_speed_hz = speed_hz,
  };
  return i2c_master_bus_add_device(bus, &cfg, dev);
}

esp_err_t board_i2c_remove_device(i2c_master_dev_handle_t dev) {
  return i2c_master_bus_rm_device(dev);
}

esp_err_t board_i2c_write(i2c_master_dev_handle_t dev, uint8_t reg,
                          const uint8_t *data, size_t len) {
  if (dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t *buf = malloc(len + 1);
  if (buf == NULL) {
    return ESP_ERR_NO_MEM;
  }

  buf[0] = reg;
  memcpy(buf + 1, data, len);

  esp_err_t ret = i2c_master_transmit(dev, buf, len + 1, BOARD_I2C_TIMEOUT_MS);
  free(buf);

  if (ret != ESP_OK) {
    ESP_LOGD(TAG, "I2C write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t board_i2c_write_raw(i2c_master_dev_handle_t dev, const uint8_t *data,
                              size_t len) {
  if (dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return i2c_master_transmit(dev, data, len, BOARD_I2C_TIMEOUT_MS);
}

esp_err_t board_i2c_read(i2c_master_dev_handle_t dev, uint8_t reg,
                         uint8_t *data, size_t len) {
  if (dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return i2c_master_transmit_receive(dev, &reg, 1, data, len,
                                     BOARD_I2C_TIMEOUT_MS);
}
