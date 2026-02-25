/**
 * @file board.c
 * @brief Board implementation for the esparagus audio brick
 *
 */

#include "iot_board.h"
#include "esp_log.h"

#include "dac.h"
#include "dac_tas58xx.h"

static const char TAG[] = "EsparagusBrick";

static bool s_board_initialized = false;

const char *iot_board_get_info(void) {
  return BOARD_NAME;
}

bool iot_board_is_init(void) {
  return s_board_initialized;
}

board_res_handle_t iot_board_get_handle(int id) {
  (void)id;
  return NULL;
}

esp_err_t iot_board_init(void) {
  if (s_board_initialized) {
    ESP_LOGW(TAG, "Board already initialized");
    return ESP_OK;
  }

  // Register and initialize DAC
  dac_register(&dac_tas58xx_ops);
  esp_err_t err = dac_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize DAC: %s", esp_err_to_name(err));
    return err;
  }

  s_board_initialized = true;
  ESP_LOGI(TAG, "Esparagus Brick initialized");
  return ESP_OK;
}

esp_err_t iot_board_deinit(void) {
  s_board_initialized = false;
  return ESP_OK;
}
