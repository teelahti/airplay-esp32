/**
 * Implementation of control interface to TI TAS58xx (TAS5825M) DAC/Amp
 * TAS5825M datasheet:
 * https://www.ti.com/lit/ds/symlink/tas5825m.pdf
 */

#include "dac_tas58xx.h"
#include "dac_tas58xx_eq.h"
#include <math.h>
#include <string.h>
#include <sys/param.h>

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------- TAS5825M I2C addresses (7-bit) ---------- */
#define TAS5825M_ADDR_GND 0x4C // ADR pin = 0 Ω to GND
#define TAS5825M_ADDR_1K  0x4D // ADR pin = 1 kΩ to GND
#define TAS5825M_ADDR_4K7 0x4E // ADR pin = 4.7 kΩ to GND
#define TAS5825M_ADDR_15K 0x4F // ADR pin = 15 kΩ to GND

/* ---------- Register addresses (Book 0, Page 0) ---------- */
#define REG_PAGE_SEL 0x00
#define REG_BOOK_SEL 0x7F

#define REG_RESET_CTRL   0x01
#define REG_DEVICE_CTRL1 0x02
#define REG_DEVICE_CTRL2 0x03

#define REG_SIG_CH_CTRL    0x28
#define REG_CLOCK_DET_CTRL 0x29

#define REG_SAP_CTRL1 0x33 // I2S format + word length
#define REG_SAP_CTRL2 0x34 // Data offset
#define REG_SAP_CTRL3 0x35 // L/R channel routing

#define REG_DSP_PGM_MODE 0x40
#define REG_DSP_CTRL     0x46

#define REG_DIG_VOL        0x4C // Digital volume (both channels)
#define REG_DIG_VOL_CTRL1  0x4E // Volume ramp control
#define REG_AUTO_MUTE_CTRL 0x50
#define REG_AUTO_MUTE_TIME 0x51
#define REG_ANA_CTRL       0x53
#define REG_AGAIN          0x54 // Analog gain

#define REG_DIE_ID      0x67 // Expected: 0x95
#define REG_POWER_STATE 0x68

#define REG_CHAN_FAULT    0x70
#define REG_GLOBAL_FAULT1 0x71
#define REG_GLOBAL_FAULT2 0x72
#define REG_WARNING       0x73
#define REG_FAULT_CLEAR   0x78

/* ---------- DEVICE_CTRL2 (0x03) bit fields ---------- */
#define CTRL2_MUTE       (1 << 3)
#define CTRL2_DIS_DSP    (1 << 4)
#define CTRL2_STATE_MASK 0x03
#define CTRL2_DEEP_SLEEP 0x00
#define CTRL2_SLEEP      0x01
#define CTRL2_HIZ        0x02
#define CTRL2_PLAY       0x03

/* ---------- DIG_VOL (0x4C) ---------- */
// 0x00 = +24.0 dB, 0x30 = 0.0 dB, 0xFE = -103.0 dB, 0xFF = mute
// step = -0.5 dB per increment
#define DIG_VOL_0DB  0x30
#define DIG_VOL_MUTE 0xFF

/* ---------- AGAIN (0x54) ---------- */
// bits[4:0]: 0x00 = 0 dB, each step = -0.5 dB, max 0x1F = -15.5 dB

/* ---------- RESET_CTRL (0x01) ---------- */
#define RESET_DIG_CORE (1 << 4)
#define RESET_REG      (1 << 0)

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
 *
 * NOTE: We do NOT transition to Play here — I2S clocks are not yet
 * running when dac_init() is called, so the PLL cannot lock and the
 * device will stay stuck in HiZ.  The transition to Play happens
 * later via dac_set_power_mode(DAC_POWER_ON) once I2S is active.
 */
static const struct tas58xx_cmd_s tas58xx_init_seq[] = {
    {REG_PAGE_SEL, 0x00},        // Select Book 0 Page 0
    {REG_BOOK_SEL, 0x00},        // Select Book 0
    {REG_PAGE_SEL, 0x00},        // Confirm Page 0
    {REG_RESET_CTRL, RESET_REG}, // Reset control port registers

    // Go to HiZ with DSP enabled (DIS_DSP=0, CTRL_STATE=HiZ)
    {REG_DEVICE_CTRL2, CTRL2_HIZ},

    // I2S format: standard I2S, 16-bit word length
    {REG_SAP_CTRL1, 0x00}, // DATA_FORMAT=I2S(00), WORD_LENGTH=16bit(00)

    // Clock detection: re-enable detection (0x00 = detect all errors)
    // The PLL needs valid SCLK to lock; masking errors just hides the
    // problem.  We stay in HiZ here anyway, so faults are expected.
    {REG_CLOCK_DET_CTRL, 0x00},

    // DSP: ROM mode 1 (default passthrough)
    {REG_DSP_PGM_MODE, 0x01},
    {REG_DSP_CTRL, 0x01}, // Use default coefficients

    // Volume ramp: smooth transitions
    {REG_DIG_VOL_CTRL1, 0x33}, // Default ramp rates

    // Auto-mute: enable for both channels
    {REG_AUTO_MUTE_CTRL, 0x07},
    {REG_AUTO_MUTE_TIME, 0x00},

    // Clear any pending faults
    {REG_FAULT_CLEAR, 0x80},

    // Set digital volume to 0 dB initially
    {REG_DIG_VOL, DIG_VOL_0DB},

    // Analog gain: 0 dB
    {REG_AGAIN, 0x00},

    // Stay in HiZ — transition to Play deferred to set_power_mode()

    {0xFF, 0xFF} // End of table sentinel
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

/**
 * Read and log all diagnostic registers.
 * Call after init or any power state change to verify DAC status.
 */
static void tas58xx_dump_status(const char *context) {
  uint8_t val = 0;

  ESP_LOGI(TAG, "--- %s: TAS5825M status dump ---", context);

  if (tas58xx_read_reg(REG_DEVICE_CTRL2, &val) == ESP_OK) {
    const char *state_str;
    switch (val & CTRL2_STATE_MASK) {
    case CTRL2_DEEP_SLEEP:
      state_str = "DEEP_SLEEP";
      break;
    case CTRL2_SLEEP:
      state_str = "SLEEP";
      break;
    case CTRL2_HIZ:
      state_str = "HIZ";
      break;
    case CTRL2_PLAY:
      state_str = "PLAY";
      break;
    default:
      state_str = "UNKNOWN";
      break;
    }
    ESP_LOGI(TAG, "  DEVICE_CTRL2=0x%02X  state=%s  mute=%s  dsp=%s", val,
             state_str, (val & CTRL2_MUTE) ? "YES" : "no",
             (val & CTRL2_DIS_DSP) ? "DISABLED" : "enabled");
  }

  if (tas58xx_read_reg(REG_POWER_STATE, &val) == ESP_OK) {
    const char *ps_str;
    switch (val) {
    case 0x00:
      ps_str = "DEEP_SLEEP";
      break;
    case 0x01:
      ps_str = "SLEEP";
      break;
    case 0x02:
      ps_str = "HIZ";
      break;
    case 0x03:
      ps_str = "PLAY";
      break;
    default:
      ps_str = "UNKNOWN";
      break;
    }
    ESP_LOGI(TAG, "  POWER_STATE=0x%02X (%s)", val, ps_str);
  }

  if (tas58xx_read_reg(REG_SAP_CTRL1, &val) == ESP_OK) {
    const char *fmt_str;
    switch ((val >> 4) & 0x03) {
    case 0:
      fmt_str = "I2S";
      break;
    case 1:
      fmt_str = "TDM/DSP";
      break;
    case 2:
      fmt_str = "RJ";
      break;
    case 3:
      fmt_str = "LJ";
      break;
    default:
      fmt_str = "?";
      break;
    }
    int wlen = 16 + ((val >> 0) & 0x03) * 8; // 00=16, 01=20, 10=24, 11=32
    ESP_LOGI(TAG, "  SAP_CTRL1=0x%02X  format=%s  word_len=%d-bit", val,
             fmt_str, wlen);
  }

  if (tas58xx_read_reg(REG_DIG_VOL, &val) == ESP_OK) {
    float db = (0x30 - (int)val) * 0.5f;
    ESP_LOGI(TAG, "  DIG_VOL=0x%02X (%.1f dB%s)", val, db,
             val == DIG_VOL_MUTE ? " MUTED" : "");
  }

  if (tas58xx_read_reg(REG_AGAIN, &val) == ESP_OK) {
    float again_db = -(val & 0x1F) * 0.5f;
    ESP_LOGI(TAG, "  AGAIN=0x%02X (%.1f dB)", val, again_db);
  }

  if (tas58xx_read_reg(REG_AUTO_MUTE_CTRL, &val) == ESP_OK) {
    ESP_LOGI(TAG, "  AUTO_MUTE_CTRL=0x%02X", val);
  }

  uint8_t chan_fault = 0, global1 = 0, global2 = 0, warning = 0;
  tas58xx_read_reg(REG_CHAN_FAULT, &chan_fault);
  tas58xx_read_reg(REG_GLOBAL_FAULT1, &global1);
  tas58xx_read_reg(REG_GLOBAL_FAULT2, &global2);
  tas58xx_read_reg(REG_WARNING, &warning);
  if (chan_fault || global1 || global2 || warning) {
    ESP_LOGW(TAG,
             "  FAULTS: chan=0x%02X global1=0x%02X global2=0x%02X warn=0x%02X",
             chan_fault, global1, global2, warning);
  } else {
    ESP_LOGI(TAG, "  FAULTS: none");
  }

  if (tas58xx_read_reg(REG_DSP_PGM_MODE, &val) == ESP_OK) {
    ESP_LOGI(TAG, "  DSP_PGM_MODE=0x%02X", val);
  }
  if (tas58xx_read_reg(REG_DSP_CTRL, &val) == ESP_OK) {
    ESP_LOGI(TAG, "  DSP_CTRL=0x%02X", val);
  }

  ESP_LOGI(TAG, "--- end status dump ---");
}

static esp_err_t tas58xx_init(void) {
  esp_err_t err;

  ESP_LOGI(TAG, "Initializing TAS5825M (I2C SDA=%d SCL=%d)", CONFIG_DAC_I2C_SDA,
           CONFIG_DAC_I2C_SCL);

  // Set up I2C bus
  err = i2c_init(0, CONFIG_DAC_I2C_SDA, CONFIG_DAC_I2C_SCL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not configure I2C bus: %s", esp_err_to_name(err));
    return err;
  }

  // Detect device
  tas58xx_addr = tas58xx_detect(s_bus_handle);
  if (!tas58xx_addr) {
    ESP_LOGE(TAG, "No TAS5825M detected on I2C bus!");
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
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Die ID: 0x%02X %s", die_id,
             (die_id == TAS5825M_DIE_ID) ? "(OK)" : "(UNEXPECTED!)");
  } else {
    ESP_LOGE(TAG, "Failed to read die ID: %s", esp_err_to_name(err));
  }

  // Run init sequence
  ESP_LOGI(TAG, "Running init sequence...");
  for (int i = 0; tas58xx_init_seq[i].reg != 0xFF; i++) {
    err = tas58xx_write_reg(tas58xx_init_seq[i].reg, tas58xx_init_seq[i].value);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Init failed at step %d: reg 0x%02X val 0x%02X: %s", i,
               tas58xx_init_seq[i].reg, tas58xx_init_seq[i].value,
               esp_err_to_name(err));
      return err;
    }
    ESP_LOGD(TAG, "  [%02d] reg 0x%02X <- 0x%02X", i, tas58xx_init_seq[i].reg,
             tas58xx_init_seq[i].value);

    // Pause after HiZ transition to let clocks settle
    if (tas58xx_init_seq[i].reg == REG_DEVICE_CTRL2 &&
        (tas58xx_init_seq[i].value & CTRL2_STATE_MASK) == CTRL2_HIZ) {
      ESP_LOGD(TAG, "  Waiting 10 ms for HiZ clock settle");
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Pause after DSP configuration before going to PLAY
    if (tas58xx_init_seq[i].reg == REG_DSP_CTRL) {
      ESP_LOGD(TAG, "  Waiting 5 ms for DSP settle");
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
  ESP_LOGI(TAG, "Init sequence complete");

  // Give the device time to reach PLAY state
  vTaskDelay(pdMS_TO_TICKS(10));

  // Dump full status after init
  tas58xx_dump_status("post-init");

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
  const char *mode_str;
  switch (mode) {
  case DAC_POWER_ON:
    mode_str = "ON(PLAY)";
    break;
  case DAC_POWER_STANDBY:
    mode_str = "STANDBY(HIZ)";
    break;
  case DAC_POWER_OFF:
    mode_str = "OFF(DEEP_SLEEP)";
    break;
  default:
    ESP_LOGW(TAG, "Unhandled power mode %d", mode);
    return;
  }

  // Read current state for logging
  uint8_t cur_ctrl2 = 0;
  tas58xx_read_reg(REG_DEVICE_CTRL2, &cur_ctrl2);
  ESP_LOGI(TAG, "Power mode -> %s (DEVICE_CTRL2 was 0x%02X)", mode_str,
           cur_ctrl2);

  uint8_t cur_state = cur_ctrl2 & CTRL2_STATE_MASK;

  if (mode == DAC_POWER_ON) {
    // Always go through HIZ first (per datasheet §9.5.3.1)
    // The PLL needs valid I2S clocks to lock — they must be present
    // by the time this function is called.
    if (cur_state != CTRL2_HIZ) {
      ESP_LOGI(TAG, "Transitioning to HIZ first (from state %d)", cur_state);
      tas58xx_write_reg(REG_DEVICE_CTRL2, CTRL2_HIZ);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Clear any faults accumulated while clocks were absent
    tas58xx_write_reg(REG_FAULT_CLEAR, 0x80);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Request transition to PLAY (unmuted)
    ESP_LOGI(TAG, "Requesting PLAY...");
    tas58xx_write_reg(REG_DEVICE_CTRL2, CTRL2_PLAY);

    // Poll POWER_STATE until the device actually reaches PLAY.
    // The TAS5825M won't transition until its PLL locks on SCLK.
    uint8_t ps = 0;
    bool reached_play = false;
    for (int attempt = 0; attempt < 50; attempt++) { // up to ~500 ms
      vTaskDelay(pdMS_TO_TICKS(10));
      if (tas58xx_read_reg(REG_POWER_STATE, &ps) == ESP_OK && ps == 0x03) {
        ESP_LOGI(TAG, "Reached PLAY state after %d ms", (attempt + 1) * 10);
        reached_play = true;
        break;
      }
    }
    if (!reached_play) {
      ESP_LOGE(TAG,
               "FAILED to reach PLAY — POWER_STATE=0x%02X "
               "(is I2S providing BCLK/WS on GPIO %d/%d?)",
               ps, CONFIG_I2S_BCK_IO, CONFIG_I2S_WS_IO);
    }

    // Clear any faults from PLAY transition
    tas58xx_write_reg(REG_FAULT_CLEAR, 0x80);

    tas58xx_dump_status("power-on");
  } else if (mode == DAC_POWER_STANDBY) {
    tas58xx_write_reg(REG_DEVICE_CTRL2, CTRL2_HIZ);
  } else {
    tas58xx_write_reg(REG_DEVICE_CTRL2, CTRL2_DEEP_SLEEP);
  }
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

  ESP_LOGI(TAG, "Speaker %s (DEVICE_CTRL2 was 0x%02X)",
           enable ? "ENABLE" : "DISABLE", val);

  if (enable) {
    val &= ~CTRL2_MUTE; // Clear mute bit
  } else {
    val |= CTRL2_MUTE; // Set mute bit
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
    if (raw < 0x00)
      raw = 0x00;
    if (raw > 0xFE)
      raw = 0xFE;
    reg_val = (uint8_t)raw;
  }

  ESP_LOGI(TAG, "Volume: AirPlay %.1f dB -> DAC %.1f dB -> reg 0x%02X",
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

/* ==================  15-Band Parametric EQ  ================== */

/*
 * Biquad coefficient addresses for TAS5825M ROM Mode 1 (Book 0x8C).
 *
 * Each biquad occupies 20 bytes (5 × 32-bit coefficients: b0 b1 b2 a1 a2).
 * Six biquads fit per page (registers 0x08–0x7B), then wrap to the next page.
 *
 * ** IMPORTANT ** — Verify these against TAS5825M datasheet (SLASEH6),
 * Table "ROM Coefficient Tuning Addresses" for Mode 1.  Use
 * tas58xx_eq_verify_addresses() at startup to confirm on real hardware.
 *
 * If channel pages are wrong, update CH1_BQ_START_PAGE / CH2_BQ_START_PAGE
 * to match your device's ROM coefficient map.
 */

#define BQ_COEFF_BOOK  0x8C
#define BQ_COEFF_SIZE  20   /* bytes per biquad (5 × 4) */
#define BQ_FIRST_REG   0x08 /* first usable register on coeff pages */
#define BYTES_PER_PAGE 120  /* 0x08..0x7F = 120 usable bytes per coeff page */
#define BQ_BASE_PAGE   0x2C /* first page of coefficient area (Book 0x8C) */

/*
 * Linear byte offsets into the coefficient RAM for each channel's biquad
 * block.  The TAS5805M reference (sonocotta) shows left and right biquads
 * are laid out contiguously with no gap.  CH2 starts immediately after
 * CH1's 15 biquads.
 */
#define CH1_BQ_BYTE_OFFSET 0
#define CH2_BQ_BYTE_OFFSET (TAS58XX_EQ_BANDS * BQ_COEFF_SIZE) /* = 300 */

/* 15 ISO 1/3-octave center frequencies at ~2/3-octave spacing */
static const float eq_center_freq[TAS58XX_EQ_BANDS] = {
    25.0f,   40.0f,   63.0f,   100.0f,  160.0f,  250.0f,   400.0f,   630.0f,
    1000.0f, 1600.0f, 2500.0f, 4000.0f, 6300.0f, 10000.0f, 16000.0f,
};

/* Q for ~2/3-octave bandwidth: Q = √(2^BW) / (2^BW − 1), BW = 2/3 */
#define EQ_DEFAULT_Q   2.145f
#define EQ_SAMPLE_RATE 44100.0f

/* 1.0 in 5.27 fixed-point (1 sign + 4 int + 27 frac = 32-bit) */
#define FP_ONE 0x08000000

/* ---------- helpers ---------- */

/** Select a book/page for coefficient access. */
static inline esp_err_t select_book_page(uint8_t book, uint8_t page) {
  esp_err_t err;
  err = tas58xx_write_reg(REG_PAGE_SEL, 0x00);
  if (err != ESP_OK)
    return err;
  err = tas58xx_write_reg(REG_BOOK_SEL, book);
  if (err != ESP_OK)
    return err;
  return tas58xx_write_reg(REG_PAGE_SEL, page);
}

/** Return to Book 0, Page 0. */
static inline esp_err_t select_default_page(void) {
  esp_err_t err;
  err = tas58xx_write_reg(REG_PAGE_SEL, 0x00);
  if (err != ESP_OK)
    return err;
  err = tas58xx_write_reg(REG_BOOK_SEL, 0x00);
  if (err != ESP_OK)
    return err;
  return tas58xx_write_reg(REG_PAGE_SEL, 0x00);
}

/**
 * Compute page and start-register for biquad @p bq (0-14) on a channel.
 * @p ch_byte_offset is the linear byte offset of the channel's first biquad
 * within the coefficient area (0 for CH1, 300 for CH2 in contiguous layout).
 */
static inline void bq_address(int ch_byte_offset, int bq, uint8_t *page,
                              uint8_t *reg) {
  int byte_off = ch_byte_offset + bq * BQ_COEFF_SIZE;
  *page = BQ_BASE_PAGE + (uint8_t)(byte_off / BYTES_PER_PAGE);
  *reg = BQ_FIRST_REG + (uint8_t)(byte_off % BYTES_PER_PAGE);
}

/**
 * Compute peaking-EQ biquad coefficients (Audio EQ Cookbook).
 *
 * TI convention:  H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 − a1·z⁻¹ − a2·z⁻²)
 * so stored a1/a2 are the *negated* textbook values.
 *
 * Coefficients are returned as 32-bit signed 5.27 fixed-point.
 */
static void calc_peaking_biquad(float fc, float gain_db, float q, float fs,
                                int32_t coeff[5]) {
  /* Flat / bypass shortcut */
  if (fabsf(gain_db) < 0.05f) {
    coeff[0] = FP_ONE; /* b0 = 1.0 */
    coeff[1] = 0;
    coeff[2] = 0;
    coeff[3] = 0;
    coeff[4] = 0;
    return;
  }

  float w0 = 2.0f * (float)M_PI * fc / fs;
  float A = powf(10.0f, gain_db / 40.0f);
  float sinw = sinf(w0);
  float cosw = cosf(w0);
  float alpha = sinw / (2.0f * q);

  float b0 = 1.0f + alpha * A;
  float b1 = -2.0f * cosw;
  float b2 = 1.0f - alpha * A;
  float a0 = 1.0f + alpha / A;
  float a1_txt = -2.0f * cosw;     /* textbook a1 */
  float a2_txt = 1.0f - alpha / A; /* textbook a2 */

  /* Normalise by a0 */
  float inv_a0 = 1.0f / a0;
  b0 *= inv_a0;
  b1 *= inv_a0;
  b2 *= inv_a0;

  /* TI format: negate a1/a2 */
  float a1_ti = -a1_txt * inv_a0;
  float a2_ti = -a2_txt * inv_a0;

  /* Convert to 5.27 fixed-point */
  const float scale = (float)(1 << 27);
  coeff[0] = (int32_t)roundf(b0 * scale);
  coeff[1] = (int32_t)roundf(b1 * scale);
  coeff[2] = (int32_t)roundf(b2 * scale);
  coeff[3] = (int32_t)roundf(a1_ti * scale);
  coeff[4] = (int32_t)roundf(a2_ti * scale);
}

/**
 * Write a single biquad's 5 coefficients (20 bytes, big-endian) to the
 * TAS5825M coefficient RAM.  Caller must already have selected Book 0x8C.
 */
static esp_err_t write_biquad_coeff(uint8_t page, uint8_t reg_start,
                                    const int32_t coeff[5]) {
  esp_err_t err;
  err = tas58xx_write_reg(REG_PAGE_SEL, page);
  if (err != ESP_OK)
    return err;

  uint8_t buf[BQ_COEFF_SIZE];
  for (int i = 0; i < 5; i++) {
    buf[i * 4 + 0] = (uint8_t)((coeff[i] >> 24) & 0xFF);
    buf[i * 4 + 1] = (uint8_t)((coeff[i] >> 16) & 0xFF);
    buf[i * 4 + 2] = (uint8_t)((coeff[i] >> 8) & 0xFF);
    buf[i * 4 + 3] = (uint8_t)((coeff[i]) & 0xFF);
  }

  return i2c_bus_write(tas58xx_device_handle, tas58xx_addr, reg_start, buf,
                       BQ_COEFF_SIZE);
}

/**
 * Read 20 bytes back from a biquad location and decode into int32[5].
 */
static esp_err_t read_biquad_coeff(uint8_t page, uint8_t reg_start,
                                   int32_t coeff[5]) {
  esp_err_t err;
  err = tas58xx_write_reg(REG_PAGE_SEL, page);
  if (err != ESP_OK)
    return err;

  uint8_t buf[BQ_COEFF_SIZE];
  err = i2c_bus_read(tas58xx_device_handle, tas58xx_addr, reg_start, buf,
                     BQ_COEFF_SIZE);
  if (err != ESP_OK)
    return err;

  for (int i = 0; i < 5; i++) {
    coeff[i] = ((int32_t)buf[i * 4 + 0] << 24) |
               ((int32_t)buf[i * 4 + 1] << 16) |
               ((int32_t)buf[i * 4 + 2] << 8) | ((int32_t)buf[i * 4 + 3]);
  }
  return ESP_OK;
}

/**
 * Program one biquad on both channels.
 * Enters Book 0x8C, writes CH1 then CH2, and returns to Book 0 / Page 0.
 */
static esp_err_t program_biquad(int bq, const int32_t coeff[5]) {
  uint8_t page, reg;
  esp_err_t err;

  /* Enter coefficient book */
  err = select_book_page(BQ_COEFF_BOOK, 0x00);
  if (err != ESP_OK)
    goto out;

  /* Channel 1 (Left) */
  bq_address(CH1_BQ_BYTE_OFFSET, bq, &page, &reg);
  err = write_biquad_coeff(page, reg, coeff);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "EQ: CH1 BQ%d write failed: %s", bq, esp_err_to_name(err));
    goto out;
  }

  /* Channel 2 (Right) */
  bq_address(CH2_BQ_BYTE_OFFSET, bq, &page, &reg);
  err = write_biquad_coeff(page, reg, coeff);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "EQ: CH2 BQ%d write failed: %s", bq, esp_err_to_name(err));
    goto out;
  }

out:
  select_default_page();
  return err;
}

/* ---------- Public API ---------- */

esp_err_t tas58xx_eq_set_band(int band, float gain_db) {
  if (band < 0 || band >= TAS58XX_EQ_BANDS) {
    ESP_LOGE(TAG, "EQ: invalid band %d", band);
    return ESP_ERR_INVALID_ARG;
  }

  /* Clamp gain */
  if (gain_db > TAS58XX_EQ_MAX_GAIN_DB)
    gain_db = TAS58XX_EQ_MAX_GAIN_DB;
  if (gain_db < TAS58XX_EQ_MIN_GAIN_DB)
    gain_db = TAS58XX_EQ_MIN_GAIN_DB;

  ESP_LOGI(TAG, "EQ: band %d (%.0f Hz) -> %+.1f dB", band, eq_center_freq[band],
           gain_db);

  int32_t coeff[5];
  calc_peaking_biquad(eq_center_freq[band], gain_db, EQ_DEFAULT_Q,
                      EQ_SAMPLE_RATE, coeff);

  return program_biquad(band, coeff);
}

esp_err_t tas58xx_eq_set_all(const float gains_db[TAS58XX_EQ_BANDS]) {
  if (!gains_db)
    return ESP_ERR_INVALID_ARG;

  esp_err_t first_err = ESP_OK;
  for (int i = 0; i < TAS58XX_EQ_BANDS; i++) {
    esp_err_t err = tas58xx_eq_set_band(i, gains_db[i]);
    if (err != ESP_OK && first_err == ESP_OK) {
      first_err = err;
    }
  }
  return first_err;
}

esp_err_t tas58xx_eq_flat(void) {
  ESP_LOGI(TAG, "EQ: resetting all bands to flat");
  int32_t flat[5] = {FP_ONE, 0, 0, 0, 0};

  esp_err_t first_err = ESP_OK;
  for (int i = 0; i < TAS58XX_EQ_BANDS; i++) {
    esp_err_t err = program_biquad(i, flat);
    if (err != ESP_OK && first_err == ESP_OK) {
      first_err = err;
    }
  }
  return first_err;
}

float tas58xx_eq_get_center_freq(int band) {
  if (band < 0 || band >= TAS58XX_EQ_BANDS)
    return 0.0f;
  return eq_center_freq[band];
}

esp_err_t tas58xx_eq_verify_addresses(void) {
  ESP_LOGI(TAG, "EQ: verifying biquad addresses (Book 0x%02X, 5.27 format)...",
           BQ_COEFF_BOOK);

  esp_err_t err;
  err = select_book_page(BQ_COEFF_BOOK, 0x00);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "EQ: cannot select coefficient book 0x%02X", BQ_COEFF_BOOK);
    select_default_page();
    return err;
  }

  /*
   * Verify strategy: write-and-readback test.
   *
   * Rather than relying on ROM Mode 1 initializing all biquads to flat
   * (it doesn't — some biquads are used for internal DSP processing),
   * we test each address by:
   *   1. Reading the original value
   *   2. Writing a known test pattern
   *   3. Reading it back
   *   4. Restoring the original value
   *
   * If the readback matches the test pattern, that address is writable
   * coefficient RAM and our address mapping is correct.
   *
   * Coefficient format: 5.27 fixed-point.
   * Flat (unity passthrough): {0x08000000, 0, 0, 0, 0}.
   */
  const int32_t test_pattern[5] = {0x07654321, 0x01234567, 0x0ABCDEF0,
                                   0x05A5A5A5, 0x0F0F0F0F};
  int ch1_ok = 0, ch2_ok = 0;
  int ch1_fail = 0, ch2_fail = 0;
  int ch1_flat = 0, ch2_flat = 0;

  for (int ch = 0; ch < 2; ch++) {
    int ch_offset = (ch == 0) ? CH1_BQ_BYTE_OFFSET : CH2_BQ_BYTE_OFFSET;
    const char *ch_str = (ch == 0) ? "CH1" : "CH2";
    int *ok = (ch == 0) ? &ch1_ok : &ch2_ok;
    int *fail = (ch == 0) ? &ch1_fail : &ch2_fail;
    int *flat = (ch == 0) ? &ch1_flat : &ch2_flat;

    for (int bq = 0; bq < TAS58XX_EQ_BANDS; bq++) {
      uint8_t page, reg;
      bq_address(ch_offset, bq, &page, &reg);

      /* 1. Read original coefficients */
      int32_t orig[5];
      err = read_biquad_coeff(page, reg, orig);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "  %s BQ%02d READ FAIL page=0x%02X reg=0x%02X: %s",
                 ch_str, bq, page, reg, esp_err_to_name(err));
        (*fail)++;
        continue;
      }

      /* Check if this biquad has flat (unity) coefficients */
      bool is_flat_bq = (orig[0] == FP_ONE && orig[1] == 0 && orig[2] == 0 &&
                         orig[3] == 0 && orig[4] == 0);
      if (is_flat_bq)
        (*flat)++;

      /* 2. Write test pattern */
      err = write_biquad_coeff(page, reg, test_pattern);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "  %s BQ%02d WRITE FAIL page=0x%02X reg=0x%02X: %s",
                 ch_str, bq, page, reg, esp_err_to_name(err));
        (*fail)++;
        continue;
      }

      /* 3. Read back and compare */
      int32_t readback[5];
      err = read_biquad_coeff(page, reg, readback);
      bool match = (err == ESP_OK);
      if (match) {
        for (int i = 0; i < 5; i++) {
          if (readback[i] != test_pattern[i]) {
            match = false;
            break;
          }
        }
      }

      /* 4. Restore original value */
      write_biquad_coeff(page, reg, orig);

      if (match) {
        (*ok)++;
      } else {
        ESP_LOGW(TAG,
                 "  %s BQ%02d MISMATCH page=0x%02X reg=0x%02X: "
                 "wrote=0x%08lX readback=0x%08lX",
                 ch_str, bq, page, reg, (long)test_pattern[0],
                 (long)readback[0]);
        (*fail)++;
      }
    }
  }

  select_default_page();

  ESP_LOGI(TAG,
           "EQ verify: CH1 %d/%d writable (%d flat), "
           "CH2 %d/%d writable (%d flat)",
           ch1_ok, TAS58XX_EQ_BANDS, ch1_flat, ch2_ok, TAS58XX_EQ_BANDS,
           ch2_flat);

  /* Log computed addresses for reference */
  for (int ch = 0; ch < 2; ch++) {
    int ch_offset = (ch == 0) ? CH1_BQ_BYTE_OFFSET : CH2_BQ_BYTE_OFFSET;
    uint8_t p0, r0, pn, rn;
    bq_address(ch_offset, 0, &p0, &r0);
    bq_address(ch_offset, TAS58XX_EQ_BANDS - 1, &pn, &rn);
    ESP_LOGI(TAG, "  %s: BQ00=page 0x%02X:0x%02X .. BQ14=page 0x%02X:0x%02X",
             (ch == 0) ? "CH1" : "CH2", p0, r0, pn, rn);
  }

  int total_ok = ch1_ok + ch2_ok;
  int total_expected = TAS58XX_EQ_BANDS * 2;
  if (total_ok < total_expected) {
    ESP_LOGW(TAG, "EQ: %d/%d addresses failed write-readback test",
             total_expected - total_ok, total_expected);
  }

  return (total_ok == total_expected) ? ESP_OK : ESP_ERR_NOT_FOUND;
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
    ESP_LOGD(TAG, "I2C write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
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
