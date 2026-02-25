/**
 * Implementation of control interface to TI TAS58xx (TAS5825M) DAC/Amp
 * TAS5825M datasheet:
 * https://www.ti.com/lit/ds/symlink/tas5825m.pdf
 */

#include "dac_tas58xx.h"
#include <math.h>
#include <string.h>
#include <sys/param.h>

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------- TAS5825M I2C addresses (7-bit) ---------- */
#define TAS5825M_ADDR_GND   0x4C  // ADR pin = 0 Ω to GND
#define TAS5825M_ADDR_1K    0x4D  // ADR pin = 1 kΩ to GND
#define TAS5825M_ADDR_4K7   0x4E  // ADR pin = 4.7 kΩ to GND
#define TAS5825M_ADDR_15K   0x4F  // ADR pin = 15 kΩ to GND

/* ---------- Register addresses (Book 0, Page 0) ---------- */
#define REG_PAGE_SEL         0x00
#define REG_BOOK_SEL         0x7F

#define REG_RESET_CTRL       0x01
#define REG_DEVICE_CTRL1     0x02
#define REG_DEVICE_CTRL2     0x03

#define REG_SIG_CH_CTRL      0x28
#define REG_CLOCK_DET_CTRL   0x29

#define REG_SAP_CTRL1        0x33  // I2S format + word length
#define REG_SAP_CTRL2        0x34  // Data offset
#define REG_SAP_CTRL3        0x35  // L/R channel routing

#define REG_DSP_PGM_MODE     0x40
#define REG_DSP_CTRL         0x46

#define REG_DIG_VOL          0x4C  // Digital volume (both channels)
#define REG_DIG_VOL_CTRL1    0x4E  // Volume ramp control
#define REG_AUTO_MUTE_CTRL   0x50
#define REG_AUTO_MUTE_TIME   0x51
#define REG_ANA_CTRL         0x53
#define REG_AGAIN            0x54  // Analog gain

#define REG_DIE_ID           0x67  // Expected: 0x95
#define REG_POWER_STATE      0x68

#define REG_CHAN_FAULT        0x70
#define REG_GLOBAL_FAULT1    0x71
#define REG_GLOBAL_FAULT2    0x72
#define REG_WARNING           0x73
#define REG_FAULT_CLEAR      0x78

/* ---------- DEVICE_CTRL2 (0x03) bit fields ---------- */
#define CTRL2_MUTE           (1 << 3)
#define CTRL2_DIS_DSP        (1 << 4)
#define CTRL2_STATE_MASK     0x03
#define CTRL2_DEEP_SLEEP     0x00
#define CTRL2_SLEEP          0x01
#define CTRL2_HIZ            0x02
#define CTRL2_PLAY           0x03

/* ---------- DIG_VOL (0x4C) ---------- */
// 0x00 = +24.0 dB, 0x30 = 0.0 dB, 0xFE = -103.0 dB, 0xFF = mute
// step = -0.5 dB per increment
#define DIG_VOL_0DB          0x30
#define DIG_VOL_MUTE         0xFF

/* ---------- AGAIN (0x54) ---------- */
// bits[4:0]: 0x00 = 0 dB, each step = -0.5 dB, max 0x1F = -15.5 dB

/* ---------- RESET_CTRL (0x01) ---------- */
#define RESET_DIG_CORE       (1 << 4)
#define RESET_REG            (1 << 0)

/* ---------- Constants ---------- */
#define I2C_TIMEOUT    100    // ms
#define I2C_LINE_SPEED 400000 // TAS5825M supports fast-mode 400 kHz

#define TAS5825M_DIE_ID 0x95

static const char TAG[] = "TAS58xx DAC";

/* ---------- Init sequence ---------- */
struct tas58xx_cmd_s {
  uint8_t reg;
  uint8_t value;
};

/*
 * Startup procedure from datasheet §9.5.3.1:
 *   1. Go to Book 0 / Page 0
 *   2. Reset device registers
 *   3. Configure device into HiZ with DSP enabled
 *   4. Wait ≥5 ms for clocks to settle
 *   5. Configure I2S format + word length
 *   6. Set DSP to ROM mode (simple passthrough, no custom coefficients)
 *   7. Set default analog gain
 *   8. Set volume ramp rates
 *   9. Configure auto-mute
 *  10. Clear faults
 *  11. Transition to Play state
 */
static const struct tas58xx_cmd_s tas58xx_init_seq[] = {
    {REG_PAGE_SEL, 0x00},          // Select Book 0 Page 0
    {REG_BOOK_SEL, 0x00},          // Select Book 0
    {REG_PAGE_SEL, 0x00},          // Confirm Page 0
    {REG_RESET_CTRL, RESET_REG},   // Reset control port registers

    // Go to HiZ with DSP enabled (DIS_DSP=0, CTRL_STATE=HiZ)
    {REG_DEVICE_CTRL2, CTRL2_HIZ},

    // I2S format: standard I2S, 16-bit word length
    {REG_SAP_CTRL1, 0x00},        // DATA_FORMAT=I2S(00), WORD_LENGTH=16bit(00)

    // Clock detection: ignore missing SCLK during init
    {REG_CLOCK_DET_CTRL, 0x0C},   // Ignore SCLK halt + missing

    // DSP: ROM mode 1 (default passthrough)
    {REG_DSP_PGM_MODE, 0x01},
    {REG_DSP_CTRL, 0x01},         // Use default coefficients

    // Volume ramp: smooth transitions
    {REG_DIG_VOL_CTRL1, 0x33},    // Default ramp rates

    // Auto-mute: enable for both channels
    {REG_AUTO_MUTE_CTRL, 0x07},
    {REG_AUTO_MUTE_TIME, 0x00},

    // Clear any pending faults
    {REG_FAULT_CLEAR, 0x80},

    // Set digital volume to 0 dB initially
    {REG_DIG_VOL, DIG_VOL_0DB},

    // Analog gain: 0 dB
    {REG_AGAIN, 0x00},

    // Unmute and go to Play
    {REG_DEVICE_CTRL2, CTRL2_PLAY},

    {0xFF, 0xFF}                   // End of table sentinel
};

/* ---------- State ---------- */
static uint8_t tas58xx_addr;
static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t tas58xx_device_handle;

/* ---------- Forward declarations ---------- */
static esp_err_t i2c_init(int i2c_port, int sda_io, int scl_io);
static esp_err_t i2c_deinit(int i2c_port, int sda_io, int scl_io);
static esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev, uint8_t addr,
                               uint8_t reg, const uint8_t *data, size_t len);
static esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev, uint8_t addr,
                              uint8_t reg, uint8_t *data, size_t len);
static esp_err_t i2c_bus_add_device(uint8_t addr,
                                    i2c_master_dev_handle_t *dev_handle);
static esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle);

static esp_err_t tas58xx_write_reg(uint8_t reg, uint8_t value);
static esp_err_t tas58xx_read_reg(uint8_t reg, uint8_t *value);

/* ---------- Detect ---------- */

/**
 * Probe all known TAS5825M I2C addresses and verify the die ID.
 * Returns the 7-bit address if found, 0 otherwise.
 */
static uint8_t tas58xx_detect(i2c_master_bus_handle_t bus) {
  static const uint8_t addrs[] = {TAS5825M_ADDR_GND, TAS5825M_ADDR_1K,
                                  TAS5825M_ADDR_4K7, TAS5825M_ADDR_15K};

  if (!bus) {
    ESP_LOGE(TAG, "Invalid I2C handle");
    return 0;
  }

  for (int i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
    if (ESP_OK == i2c_master_probe(bus, addrs[i], I2C_TIMEOUT)) {
      ESP_LOGI(TAG, "Detected TAS5825M at @0x%02X", addrs[i]);
      return addrs[i];
    }
  }
  return 0;
}

/* ---------- DAC ops implementation ---------- */

static esp_err_t tas58xx_init(void) {
  esp_err_t err;

  // Set up I2C bus
  err = i2c_init(0, CONFIG_DAC_I2C_SDA, CONFIG_DAC_I2C_SCL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not configure I2C bus: %s", esp_err_to_name(err));
    return err;
  }

  // Detect device
  tas58xx_addr = tas58xx_detect(s_bus_handle);
  if (!tas58xx_addr) {
    ESP_LOGW(TAG, "No TAS5825M detected");
    return ESP_ERR_NOT_FOUND;
  }

  err = i2c_bus_add_device(tas58xx_addr, &tas58xx_device_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not add device to I2C bus: %s", esp_err_to_name(err));
    return err;
  }

  // Verify die ID
  uint8_t die_id = 0;
  err = tas58xx_read_reg(REG_DIE_ID, &die_id);
  if (err == ESP_OK && die_id != TAS5825M_DIE_ID) {
    ESP_LOGW(TAG, "Unexpected die ID: 0x%02X (expected 0x%02X)", die_id,
             TAS5825M_DIE_ID);
  }

  // Run init sequence
  for (int i = 0; tas58xx_init_seq[i].reg != 0xFF; i++) {
    err = tas58xx_write_reg(tas58xx_init_seq[i].reg,
                            tas58xx_init_seq[i].value);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Init failed at reg 0x%02X: %s", tas58xx_init_seq[i].reg,
               esp_err_to_name(err));
      return err;
    }

    // Pause after HiZ transition to let clocks settle
    if (tas58xx_init_seq[i].reg == REG_DEVICE_CTRL2 &&
        (tas58xx_init_seq[i].value & CTRL2_STATE_MASK) == CTRL2_HIZ) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  // Verify we're in play state
  uint8_t power_state = 0;
  err = tas58xx_read_reg(REG_POWER_STATE, &power_state);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "TAS5825M power state: %d (3=Play)", power_state);
  }

  // Check for faults
  uint8_t chan_fault = 0, global1 = 0, global2 = 0;
  tas58xx_read_reg(REG_CHAN_FAULT, &chan_fault);
  tas58xx_read_reg(REG_GLOBAL_FAULT1, &global1);
  tas58xx_read_reg(REG_GLOBAL_FAULT2, &global2);
  if (chan_fault || global1 || global2) {
    ESP_LOGW(TAG, "Faults after init: chan=0x%02X global1=0x%02X global2=0x%02X",
             chan_fault, global1, global2);
    // Clear them
    tas58xx_write_reg(REG_FAULT_CLEAR, 0x80);
  }

  ESP_LOGI(TAG, "TAS5825M initialized at I2C addr 0x%02X", tas58xx_addr);
  return ESP_OK;
}

static esp_err_t tas58xx_deinit(void) {
  esp_err_t err = ESP_OK;

  // Put device into deep sleep
  tas58xx_write_reg(REG_DEVICE_CTRL2, CTRL2_DEEP_SLEEP);

  if (tas58xx_device_handle) {
    err = i2c_bus_remove_device(tas58xx_device_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to remove from I2C bus: %s", esp_err_to_name(err));
    }
    tas58xx_device_handle = NULL;
  }

  err = i2c_deinit(0, CONFIG_DAC_I2C_SDA, CONFIG_DAC_I2C_SCL);
  return err;
}

static void tas58xx_set_power_mode(dac_power_mode_t mode) {
  uint8_t ctrl2_val;

  switch (mode) {
  case DAC_POWER_ON:
    ctrl2_val = CTRL2_PLAY;
    break;
  case DAC_POWER_STANDBY:
    ctrl2_val = CTRL2_HIZ;
    break;
  case DAC_POWER_OFF:
    ctrl2_val = CTRL2_DEEP_SLEEP;
    break;
  default:
    ESP_LOGW(TAG, "Unhandled power mode %d", mode);
    return;
  }

  tas58xx_write_reg(REG_DEVICE_CTRL2, ctrl2_val);
}

static void tas58xx_enable_speaker(bool enable) {
  // Use mute bit in DEVICE_CTRL2 to enable/disable output.
  // Read current register, modify mute bit, write back.
  uint8_t val;
  esp_err_t err = tas58xx_read_reg(REG_DEVICE_CTRL2, &val);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read DEVICE_CTRL2");
    return;
  }

  if (enable) {
    val &= ~CTRL2_MUTE;  // Clear mute bit
  } else {
    val |= CTRL2_MUTE;   // Set mute bit
  }

  tas58xx_write_reg(REG_DEVICE_CTRL2, val);
}

static void tas58xx_enable_line_out(bool enable) {
  (void)enable;
  ESP_LOGW(TAG, "Line out not supported on TAS5825M");
}

static void tas58xx_set_volume(float volume_airplay_db) {
  // Clamp AirPlay input range (-30 to 0)
  if (volume_airplay_db > 0.0f) {
    volume_airplay_db = 0.0f;
  }
  if (volume_airplay_db < -30.0f) {
    volume_airplay_db = -30.0f;
  }

  // TAS5825M DIG_VOL register:
  //   0x00 = +24.0 dB
  //   0x30 = 0.0 dB
  //   0xFE = -103.0 dB
  //   0xFF = mute
  //   Step = -0.5 dB per count
  //
  // Volume mapping (2:1 scaling):
  //   AirPlay 0 dB    -> DAC CONFIG_TAS58XX_MAX_VOLUME
  //   AirPlay -25 dB  -> DAC (MAX - 50)
  //   AirPlay -30..-25 dB -> steep roll-off to mute
  float max_db = (float)CONFIG_TAS58XX_MAX_VOLUME;
  float db_level;

  if (volume_airplay_db >= -25.0f) {
    // 2:1 linear scaling: 25 dB AirPlay range -> 50 dB DAC range
    db_level = max_db + (volume_airplay_db * 2.0f);
  } else {
    // Roll-off: map -30..-25 to -103..(MAX-50)
    float normalized = (volume_airplay_db + 30.0f) / 5.0f;
    float rolloff_top = max_db - 50.0f;
    db_level = -103.0f + normalized * (103.0f + rolloff_top);
  }

  // Clamp to TAS5825M valid range: +24 dB to -103 dB
  if (db_level > 24.0f) {
    db_level = 24.0f;
  }
  if (db_level < -103.0f) {
    db_level = -103.0f;
  }

  // Convert dB to register value:
  // reg = 0x30 - (db_level * 2)  (since 0x30 = 0 dB and step = -0.5 dB)
  uint8_t reg_val;
  if (db_level <= -103.0f) {
    reg_val = DIG_VOL_MUTE;
  } else {
    int raw = DIG_VOL_0DB - (int)(db_level * 2.0f);
    if (raw < 0x00) raw = 0x00;
    if (raw > 0xFE) raw = 0xFE;
    reg_val = (uint8_t)raw;
  }

  ESP_LOGD(TAG, "Volume: AirPlay %.1f dB -> DAC %.1f dB -> reg 0x%02X",
           volume_airplay_db, db_level, reg_val);

  tas58xx_write_reg(REG_DIG_VOL, reg_val);
}

/* ---------- Public ops struct ---------- */

const dac_ops_t dac_tas58xx_ops = {
    .init = tas58xx_init,
    .deinit = tas58xx_deinit,
    .set_volume = tas58xx_set_volume,
    .set_power_mode = tas58xx_set_power_mode,
    .enable_speaker = tas58xx_enable_speaker,
    .enable_line_out = tas58xx_enable_line_out,
};

/* ---------- Register read/write helpers ---------- */

static esp_err_t tas58xx_write_reg(uint8_t reg, uint8_t value) {
  return i2c_bus_write(tas58xx_device_handle, tas58xx_addr, reg, &value,
                       sizeof(uint8_t));
}

static esp_err_t tas58xx_read_reg(uint8_t reg, uint8_t *value) {
  return i2c_bus_read(tas58xx_device_handle, tas58xx_addr, reg, value,
                      sizeof(uint8_t));
}

/* =====================  I2C Bus  ===================== */

static esp_err_t i2c_init(int i2c_port, int sda_io, int scl_io) {
  esp_err_t err;

  if (s_bus_handle != NULL) {
    ESP_LOGW(TAG, "I2C already initialized");
    return ESP_OK;
  }
  if (sda_io < 0 || scl_io < 0) {
    ESP_LOGW(TAG, "Invalid I2C pins: sda=%d, scl=%d", sda_io, scl_io);
    return ESP_ERR_INVALID_ARG;
  }

  i2c_master_bus_config_t i2c_config = {
      .i2c_port = i2c_port,
      .sda_io_num = sda_io,
      .scl_io_num = scl_io,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  err = i2c_new_master_bus(&i2c_config, &s_bus_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s",
             esp_err_to_name(err));
    s_bus_handle = NULL;
    return err;
  }
  ESP_LOGI(TAG, "I2C bus %d initialized: sda=%d, scl=%d", i2c_port, sda_io,
           scl_io);

  return ESP_OK;
}

static esp_err_t i2c_deinit(int i2c_port, int sda_io, int scl_io) {
  (void)i2c_port;
  (void)sda_io;
  (void)scl_io;

  esp_err_t err = ESP_OK;
  if (s_bus_handle != NULL) {
    err = i2c_del_master_bus(s_bus_handle);
    s_bus_handle = NULL;
  }
  return err;
}

static esp_err_t i2c_bus_add_device(uint8_t addr,
                                    i2c_master_dev_handle_t *dev_handle) {
  if (s_bus_handle == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = addr,
      .scl_speed_hz = I2C_LINE_SPEED,
  };

  return i2c_master_bus_add_device(s_bus_handle, &dev_cfg, dev_handle);
}

static esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle) {
  return i2c_master_bus_rm_device(dev_handle);
}

static esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev, uint8_t addr,
                               uint8_t reg, const uint8_t *data, size_t len) {
  (void)addr;
  if (dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t *buf = malloc(len + 1);
  if (buf == NULL) {
    return ESP_ERR_NO_MEM;
  }

  buf[0] = reg;
  memcpy(buf + 1, data, len);

  esp_err_t ret = i2c_master_transmit(dev, buf, len + 1, I2C_TIMEOUT);
  free(buf);

  if (ret != ESP_OK) {
    ESP_LOGD(TAG, "I2C write reg 0x%02X failed: %s", reg,
             esp_err_to_name(ret));
  }
  return ret;
}

static esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev, uint8_t addr,
                              uint8_t reg, uint8_t *data, size_t len) {
  (void)addr;
  if (dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT);
}
