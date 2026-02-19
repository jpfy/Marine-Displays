#include "Display_ST7701.h"  
#include "driver/ledc.h"
#include "SD_Card.h"
#include "st7701_init.h"
#include <atomic>

// LCD command constants (from MIPI DCS standard)
#define LCD_CMD_SLPOUT  0x11  // Sleep Out
#define LCD_CMD_DISPON  0x29  // Display On
#define LCD_CMD_COLMOD  0x3A  // Interface Pixel Format
#define LCD_CMD_MADCTL  0x36  // Memory Data Access Control

// Guard to avoid multiple concurrent RX timing sweeps
static std::atomic<bool> g_rx_sweep_running(false);
static void rx_sweep_task(void *arg);

// VSync tracking globals
static volatile uint32_t g_vsync_count = 0;
static volatile uint32_t g_vsync_max_gap_us = 0;
static volatile uint32_t g_vsync_prev_us = 0;

// Panel handle definition (instantiate the external declared in header)
esp_lcd_panel_handle_t panel_handle = NULL;

// Initialize RGB panel and allocate framebuffers (if PSRAM available)
void ST7701_Init()
{
  //  RGB
  esp_lcd_rgb_panel_config_t rgb_config = {};
  rgb_config.clk_src = LCD_CLK_SRC_PLL160M;  // Use 160M PLL for more stable timing
  rgb_config.timings.pclk_hz = ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ;
  rgb_config.timings.h_res = ESP_PANEL_LCD_WIDTH;
  rgb_config.timings.v_res = ESP_PANEL_LCD_HEIGHT;
  rgb_config.timings.hsync_pulse_width = ESP_PANEL_LCD_RGB_TIMING_HPW;
  rgb_config.timings.hsync_back_porch = ESP_PANEL_LCD_RGB_TIMING_HBP;
  rgb_config.timings.hsync_front_porch = ESP_PANEL_LCD_RGB_TIMING_HFP;
  rgb_config.timings.vsync_pulse_width = ESP_PANEL_LCD_RGB_TIMING_VPW;
  rgb_config.timings.vsync_back_porch = ESP_PANEL_LCD_RGB_TIMING_VBP;
  rgb_config.timings.vsync_front_porch = ESP_PANEL_LCD_RGB_TIMING_VFP;
  rgb_config.timings.flags.pclk_active_neg = ESP_PANEL_LCD_RGB_PCLK_ACTIVE_NEG;
  rgb_config.data_width = ESP_PANEL_LCD_RGB_DATA_WIDTH;
  rgb_config.psram_trans_align = 64;
  rgb_config.hsync_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_HSYNC;
  rgb_config.vsync_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_VSYNC;
  rgb_config.de_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_DE;
  rgb_config.pclk_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_PCLK;
  rgb_config.disp_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_DISP;
  // Initialize all data pins to -1
  for (int i = 0; i < 16; i++) {
    rgb_config.data_gpio_nums[i] = -1;
  }

  // Arduino demo mapping (diagram labels are misleading - these are actually B0-B4, G0-G5, R0-R4)
  rgb_config.data_gpio_nums[0]  = 5;   // D0 / B0
  rgb_config.data_gpio_nums[1]  = 45;  // D1 / B1
  rgb_config.data_gpio_nums[2]  = 48;  // D2 / B2
  rgb_config.data_gpio_nums[3]  = 47;  // D3 / B3
  rgb_config.data_gpio_nums[4]  = 21;  // D4 / B4
  rgb_config.data_gpio_nums[5]  = 14;  // D5 / G0
  rgb_config.data_gpio_nums[6]  = 13;  // D6 / G1
  rgb_config.data_gpio_nums[7]  = 12;  // D7 / G2
  rgb_config.data_gpio_nums[8]  = 11;  // D8 / G3
  rgb_config.data_gpio_nums[9]  = 10;  // D9 / G4
  rgb_config.data_gpio_nums[10] = 9;   // D10 / G5
  rgb_config.data_gpio_nums[11] = 46;  // D11 / R0
  rgb_config.data_gpio_nums[12] = 3;   // D12 / R1
  rgb_config.data_gpio_nums[13] = 8;   // D13 / R2
  rgb_config.data_gpio_nums[14] = 18;  // D14 / R3
  rgb_config.data_gpio_nums[15] = 17;  // D15 / R4

  rgb_config.flags.fb_in_psram = true;

  size_t free_internal_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t free_spiram_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

  size_t bytes_per_pixel = ESP_PANEL_LCD_RGB_PIXEL_BITS / 8;  // Use macro directly (ESP-IDF 5.1)
  size_t est_fb_bytes = (size_t)ESP_PANEL_LCD_WIDTH * (size_t)ESP_PANEL_LCD_HEIGHT * bytes_per_pixel * ESP_PANEL_LCD_RGB_FRAME_BUF_NUM;

  // Check whether PSRAM appears available at runtime (free_spiram_before>0).
  bool spiram_ok = (free_spiram_before > 0);
  if (!spiram_ok) {
    Serial.println("[DISPLAY] PSRAM not initialized at runtime — falling back to internal RAM for framebuffers");
    rgb_config.flags.fb_in_psram = false;
    // Note: num_fbs and bounce_buffer_size_px not in ESP-IDF 5.1 struct
  } else {
    
    if (est_fb_bytes > free_spiram_before) {
      Serial.printf("[DISPLAY] Not enough PSRAM for framebuffer (%u bytes needed, %u available). Skipping RGB panel init.\n", (unsigned)est_fb_bytes, (unsigned)free_spiram_before);
      return;
    }
  }

  // CRITICAL: Send ST7701 vendor init BEFORE creating RGB panel
  // The RGB panel starts driving signals immediately, which interferes with 3-wire SPI
  Serial.println("[DISPLAY] Sending ST7701 vendor init via GPIO bit-bang (before RGB panel creation)");
  
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_CLK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_CLK_PIN, LOW);
  
  // Send init sequence from st7701_init.h
  extern const uint8_t st7701_type1_init_operations[];
  extern const size_t st7701_type1_init_operations_len;
  const uint8_t *ops = st7701_type1_init_operations;
  size_t idx = 0;
  
  // Helper function to bit-bang a 9-bit value (DC bit + 8 data bits)
  auto send_9bit = [](uint8_t dc, uint8_t value) {
    digitalWrite(LCD_CS_PIN, LOW);
    for (int bit = 0; bit < 9; bit++) {
      if (bit == 0) {
        digitalWrite(LCD_MOSI_PIN, dc);
      } else {
        digitalWrite(LCD_MOSI_PIN, (value >> (8 - bit)) & 1);
      }
      digitalWrite(LCD_CLK_PIN, HIGH);
      delayMicroseconds(1);
      digitalWrite(LCD_CLK_PIN, LOW);
      delayMicroseconds(1);
    }
    digitalWrite(LCD_CS_PIN, HIGH);
  };
  
  // Parse operation codes and send via GPIO
  while (idx < st7701_type1_init_operations_len) {
    uint8_t op = ops[idx++];
    
    switch (op) {
      case 0: // BEGIN_WRITE
        digitalWrite(LCD_CS_PIN, LOW);
        break;
        
      case 1: // WRITE_COMMAND_8
        if (idx < st7701_type1_init_operations_len) {
          send_9bit(0, ops[idx++]); // DC=0 for command
        }
        break;
        
      case 6: { // WRITE_BYTES
        if (idx < st7701_type1_init_operations_len) {
          uint8_t len = ops[idx++];
          for (uint8_t i = 0; i < len && idx < st7701_type1_init_operations_len; i++) {
            send_9bit(1, ops[idx++]); // DC=1 for data
          }
        }
        break;
      }
      
      case 7: { // WRITE_C8_D8
        if (idx + 1 < st7701_type1_init_operations_len) {
          send_9bit(0, ops[idx++]); // Command
          send_9bit(1, ops[idx++]); // Data
        }
        break;
      }
      
      case 8: { // WRITE_C8_D16
        if (idx + 2 < st7701_type1_init_operations_len) {
          send_9bit(0, ops[idx++]); // Command
          send_9bit(1, ops[idx++]); // Data byte 1
          send_9bit(1, ops[idx++]); // Data byte 2
        }
        break;
      }
      
      case 11: // END_WRITE
        digitalWrite(LCD_CS_PIN, HIGH);
        break;
        
      case 12: // DELAY
        if (idx < st7701_type1_init_operations_len) {
          uint8_t delay_ms = ops[idx++];
          vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
        break;
        
      default:
        // Unknown operation, skip
        break;
    }
  }
  
  Serial.println("[DISPLAY] ST7701 vendor init sequence sent via GPIO");
  
  // Release GPIO pins back to normal (RGB driver will reconfigure them)
  pinMode(LCD_CS_PIN, INPUT);
  pinMode(LCD_CLK_PIN, INPUT);
  pinMode(LCD_MOSI_PIN, INPUT);

  // If configured, perform a software-SPI (SWSPI) vendor init to match Arduino demo
#if defined(USE_SWSPI_VENDOR_INIT)
  // Initialize SWSPI pins and run sequence
  {
    // Setup pins (CS may be MCU GPIO if defined)
    if (LCD_CS_PIN >= 0) pinMode(LCD_CS_PIN, OUTPUT);
    pinMode(LCD_CLK_PIN, OUTPUT);
    pinMode(LCD_MOSI_PIN, OUTPUT);
    // Idle states
    if (LCD_CS_PIN >= 0) digitalWrite(LCD_CS_PIN, HIGH);
    digitalWrite(LCD_CLK_PIN, LOW);
    digitalWrite(LCD_MOSI_PIN, LOW);

    auto swspi_delay = [](){ delayMicroseconds(2); };
    auto swspi_begin = [&](){ if (LCD_CS_PIN >= 0) digitalWrite(LCD_CS_PIN, LOW); };
    auto swspi_end = [&](){ if (LCD_CS_PIN >= 0) digitalWrite(LCD_CS_PIN, HIGH); };
    auto swspi_write9 = [&](uint16_t val){
      // 9-bit: bit8 = DC (0=cmd,1=data), bits7..0 = value
      for (int i = 8; i >= 0; --i) {
        digitalWrite(LCD_MOSI_PIN, (val >> i) & 1);
        swspi_delay();
        digitalWrite(LCD_CLK_PIN, HIGH);
        swspi_delay();
        digitalWrite(LCD_CLK_PIN, LOW);
      }
    };

    // Helper lambdas
    auto writeCommand = [&](uint8_t c){
      swspi_write9(c & 0xFF); // DC=0 handled below by prefix
    };
    auto writeData = [&](uint8_t d){
      // set DC bit (1) as top bit in 9-bit sequence
      swspi_write9(((uint16_t)1 << 8) | (d & 0xFF));
    };

    // Parse the minimal init array from st7701_type1_init_operations
    extern const uint8_t st7701_type1_init_operations[];
    extern const size_t st7701_type1_init_operations_len;
    size_t i = 0;
    while (i < st7701_type1_init_operations_len) {
      uint8_t op = st7701_type1_init_operations[i++];
      switch (op) {
        case BEGIN_WRITE:
          swspi_begin();
          break;
        case END_WRITE:
          swspi_end();
          break;
        case WRITE_COMMAND_8: {
          if (i >= st7701_type1_init_operations_len) break;
          uint8_t cmd = st7701_type1_init_operations[i++];
          // Send 9-bit command with DC=0 -> we send DC=0 in top bit (bit8=0)
          swspi_write9(cmd);
          break;
        }
        case WRITE_BYTES: {
          if (i >= st7701_type1_init_operations_len) break;
          uint8_t n = st7701_type1_init_operations[i++];
          for (uint8_t k = 0; k < n && i < st7701_type1_init_operations_len; ++k) {
            uint8_t b = st7701_type1_init_operations[i++];
            // WRITE_BYTES assumes data stream after previously sent command; set DC=1
            swspi_write9(((uint16_t)1 << 8) | b);
          }
          break;
        }
        case WRITE_C8_D8: {
          if (i + 1 >= st7701_type1_init_operations_len) break;
          uint8_t c = st7701_type1_init_operations[i++];
          uint8_t d = st7701_type1_init_operations[i++];
          swspi_write9(c); // cmd (DC=0)
          swspi_write9(((uint16_t)1 << 8) | d); // data
          break;
        }
        case WRITE_C8_D16: {
          if (i + 2 >= st7701_type1_init_operations_len) break;
          uint8_t c = st7701_type1_init_operations[i++];
          uint8_t d1 = st7701_type1_init_operations[i++];
          uint8_t d2 = st7701_type1_init_operations[i++];
          swspi_write9(c); // cmd
          swspi_write9(((uint16_t)1 << 8) | d1);
          swspi_write9(((uint16_t)1 << 8) | d2);
          break;
        }
        case WRITE_DATA_8: {
          if (i >= st7701_type1_init_operations_len) break;
          uint8_t d = st7701_type1_init_operations[i++];
          swspi_write9(((uint16_t)1 << 8) | d);
          break;
        }
        case WRITE_DATA_16: {
          if (i + 1 >= st7701_type1_init_operations_len) break;
          uint8_t hi = st7701_type1_init_operations[i++];
          uint8_t lo = st7701_type1_init_operations[i++];
          swspi_write9(((uint16_t)1 << 8) | hi);
          swspi_write9(((uint16_t)1 << 8) | lo);
          break;
        }
        case DELAY: {
          if (i >= st7701_type1_init_operations_len) break;
          uint8_t ms = st7701_type1_init_operations[i++];
          delay(ms);
          break;
        }
        default:
          // skip unknown op
          break;
      }
    }
    // ensure CS released
    if (LCD_CS_PIN >= 0) digitalWrite(LCD_CS_PIN, HIGH);
  }
#endif


  // Simplified: Create RGB LCD panel directly (ST7701 already initialized via GPIO above)
  Serial.println("[DISPLAY] Creating RGB panel (ST7701 already configured)");
  
  esp_err_t rc = esp_lcd_new_rgb_panel(&rgb_config, &panel_handle);
  if (rc != ESP_OK || panel_handle == NULL) {
    Serial.printf("[DISPLAY] esp_lcd_new_rgb_panel failed: %s (panel_handle=%p)\n", esp_err_to_name(rc), panel_handle);
    panel_handle = NULL;
    size_t free_internal_after_fail = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t free_spiram_after_fail = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    Serial.printf("[DISPLAY] Free internal after failed panel: %u bytes, PSRAM: %u bytes\n", (unsigned)free_internal_after_fail, (unsigned)free_spiram_after_fail);
    return;
  }
  
  Serial.println("[DISPLAY] RGB panel created successfully");
  
  // Reset and initialize the panel
  Serial.println("[DISPLAY] Resetting and initializing RGB panel");
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  
  Serial.println("[DISPLAY] Display initialization complete");

  // Log free heap after successful panel creation

}
// Reset helper stub for ST7701 (no-op if hardware reset handled elsewhere)
void ST7701_Reset()
{
  // If board has a reset GPIO or expander control, toggle EXIO_PIN3 (expander reset).
  // Ensure expander reset pin is configured as output and perform a short reset pulse.
  Serial.println("[DISPLAY] ST7701_Reset: asserting expander reset (EXIO_PIN3)");
  // Configure EXIO3 as output and drive low to reset the panel
  Mode_EXIO(EXIO_PIN3, 0); // 0 = output
  Set_EXIO(EXIO_PIN3, Low);
  // Hold reset low briefly
  vTaskDelay(pdMS_TO_TICKS(20));
  // Release reset
  Set_EXIO(EXIO_PIN3, High);
  // Allow panel time to come out of reset
  vTaskDelay(pdMS_TO_TICKS(50));
}

bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
  uint32_t now = (uint32_t)esp_timer_get_time();
  if (g_vsync_prev_us != 0) {
    uint32_t delta = now - g_vsync_prev_us;
    if (delta > g_vsync_max_gap_us) g_vsync_max_gap_us = delta;
  }
  g_vsync_prev_us = now;
  g_vsync_count = g_vsync_count + 1;
  return false;
}

uint32_t get_vsync_count() {
  return g_vsync_count;
}

uint32_t get_vsync_max_gap_us() {
  return g_vsync_max_gap_us;
}

void reset_vsync_stats() {
  g_vsync_count = 0;
  g_vsync_max_gap_us = 0;
  g_vsync_prev_us = 0;
}

#if !REMOVE_PANEL_TESTS
// Background task used to run an automated RX timing sweep. It attempts a matrix of
// dummy clock counts and settle delays, forcing SLPOUT/DISPON for each variant,
// then logs MADCTL/COLMOD read results. Guarded by g_rx_sweep_running to avoid
// multiple simultaneous executions.
static void rx_sweep_task(void *arg)
{
  esp_lcd_panel_io_handle_t io = (esp_lcd_panel_io_handle_t)arg;
  if (!io) {
    Serial.println("[SWEEP] No control IO provided; aborting");
    vTaskDelete(NULL);
    return;
  }
  if (g_rx_sweep_running.exchange(true)) {
    Serial.println("[SWEEP] Another sweep already running; aborting");
    vTaskDelete(NULL);
    return;
  }

  Serial.println("[SWEEP] Starting 3-wire RX timing sweep");

  // Diagnostic precaution: disable SD (assert EXIO_PIN4 LOW) to avoid shared-SPI conflicts
  // while we exercise 3-wire reads. SD may be mounted; pulsing/holding CS low prevents
  // the SD card from driving MISO/SDA during our reads.
  Serial.println("[SWEEP] Asserting EXIO_PIN4 LOW to disable SD during sweep");
  SD_D3_Dis(); // drives EXIO_PIN4 LOW
  vTaskDelay(pdMS_TO_TICKS(20));

  int phases[] = {0, 1}; // 0 = normal, 1 = invert sampling phase
  int dummy_vals[] = {0, 1, 2, 4, 8, 16, 32, 64};
  uint32_t settle_vals[] = {0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};
  uint8_t readbuf[2] = {0, 0};

  for (size_t pi = 0; pi < sizeof(phases)/sizeof(phases[0]); ++pi) {
    int phase = phases[pi];
    esp_lcd_3wire_set_rx_phase(io, phase);
    Serial.printf("[SWEEP] phase=%d (0=normal,1=invert)\n", phase);

    for (size_t di = 0; di < sizeof(dummy_vals)/sizeof(dummy_vals[0]); ++di) {
      for (size_t si = 0; si < sizeof(settle_vals)/sizeof(settle_vals[0]); ++si) {
        uint8_t dummy = (uint8_t)dummy_vals[di];
        uint32_t settle = settle_vals[si];

        esp_lcd_3wire_set_rx_timing(io, dummy, settle);

        // Force controller out of sleep and onto the display on state
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
        esp_lcd_panel_io_tx_param(io, LCD_CMD_DISPON, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(50));


        esp_err_t rr1 = esp_lcd_panel_io_rx_param(io, LCD_CMD_MADCTL, readbuf, 1);
        esp_err_t rr2 = esp_lcd_panel_io_rx_param(io, LCD_CMD_COLMOD, readbuf + 1, 1);

        Serial.printf("[SWEEP] phase=%d dummy=%u settle=%uus MADCTL=0x%02x (%s) COLMOD=0x%02x (%s)\n",
                      phase, (unsigned)dummy, (unsigned)settle, (unsigned)readbuf[0], esp_err_to_name(rr1), (unsigned)readbuf[1], esp_err_to_name(rr2));

        // small delay between tests
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }
  }

  // Clear overrides (restore default behavior)
  esp_lcd_3wire_set_rx_timing(io, 0xFF, 0xFFFFFFFF);
  esp_lcd_3wire_set_rx_phase(io, -1);

  Serial.println("[SWEEP] Completed 3-wire RX timing sweep");

  // Re-enable SD (release EXIO_PIN4) now that sweep is finished
  Serial.println("[SWEEP] Releasing EXIO_PIN4 HIGH to re-enable SD (if present)");
  SD_D3_EN();
  vTaskDelay(pdMS_TO_TICKS(10));

  // Probe SDA directly to see whether the panel ever drives the line low
  esp_err_t pr = esp_lcd_3wire_sda_probe(io);
  Serial.printf("[SWEEP] SDA probe -> %s\n", esp_err_to_name(pr));

  esp_err_t hr = esp_lcd_3wire_sda_hold_test(io, 500);
  Serial.printf("[SWEEP] SDA hold test -> %s\n", esp_err_to_name(hr));

  g_rx_sweep_running = false;
  vTaskDelete(NULL);
}

#else
// Stub when panel tests are removed so any references still link
static void rx_sweep_task(void *arg) { (void)arg; Serial.println("[SWEEP] Disabled by REMOVE_PANEL_TESTS"); vTaskDelete(NULL); }
#endif

// SDA diagnostics disabled: return not supported
esp_err_t Display_SDA_Probe() {
  Serial.println("[DISPLAY] SDA probe disabled by REMOVE_PANEL_TESTS");
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t Display_SDA_Hold_Test(uint32_t hold_ms) {
  Serial.println("[DISPLAY] SDA hold test disabled by REMOVE_PANEL_TESTS");
  (void)hold_ms;
  return ESP_ERR_NOT_SUPPORTED;
}


// Fill the panel with a solid 16-bit color (RGB565) using a single line buffer.
static void LCD_FillSolid(uint16_t color565)
{
  if (!panel_handle) return;
  size_t line_pixels = ESP_PANEL_LCD_WIDTH;
  Serial.printf("[DISPLAY] LCD_FillSolid: color=0x%04x lines=%u\n", color565, (unsigned)ESP_PANEL_LCD_HEIGHT);
  size_t line_bytes = line_pixels * sizeof(uint16_t);
  uint16_t *line = (uint16_t*)heap_caps_malloc(line_bytes, MALLOC_CAP_INTERNAL);
  if (!line) {
    // fallback to PSRAM if internal allocation fails
    line = (uint16_t*)heap_caps_malloc(line_bytes, MALLOC_CAP_SPIRAM);
    if (!line) {
      ESP_LOGE("DISPLAY", "LCD_FillSolid: failed to allocate line buffer (%u bytes)", (unsigned)line_bytes);
      return;
    }
  }
  for (size_t i = 0; i < line_pixels; ++i) line[i] = color565;

  for (int y = 0; y < ESP_PANEL_LCD_HEIGHT; ++y) {
    esp_err_t r = esp_lcd_panel_draw_bitmap(panel_handle, 0, y, ESP_PANEL_LCD_WIDTH, y + 1, line);
    if (r != ESP_OK) {
      ESP_LOGE("DISPLAY", "LCD_FillSolid: draw_bitmap failed at row %d: %s", y, esp_err_to_name(r));
      break;
    }
    // Small yield to keep watchdog happy on long loops
    if ((y & 31) == 0) vTaskDelay(pdMS_TO_TICKS(1));
  }

  free(line);
}

void LCD_Init() {
  // Ensure expander-controlled backlight is off briefly to emulate a power-cycle
  Mode_EXIO(EXIO_PIN2, 0); // ensure BL pin is output
  Set_EXIO(EXIO_PIN2, Low);
  vTaskDelay(pdMS_TO_TICKS(50));

  ST7701_Reset();
  ST7701_Init();

  // Restore backlight state
  Backlight_Init();

  if (panel_handle) {
    esp_err_t rc = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (rc != ESP_OK) {
      Serial.printf("[DISPLAY] esp_lcd_panel_disp_on_off failed: %s\n", esp_err_to_name(rc));
    }
  }

}


void LCD_addWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,uint8_t* color) {
  Xend = Xend + 1;      // esp_lcd_panel_draw_bitmap: x_end End index on x-axis (x_end not included)
  Yend = Yend + 1;      // esp_lcd_panel_draw_bitmap: y_end End index on y-axis (y_end not included)
  if (Xend >= ESP_PANEL_LCD_WIDTH)
    Xend = ESP_PANEL_LCD_WIDTH;
  if (Yend >= ESP_PANEL_LCD_HEIGHT)
    Yend = ESP_PANEL_LCD_HEIGHT;
   
  if (!panel_handle) {
    ESP_LOGE("DISPLAY", "LCD_addWindow: panel_handle is NULL, skipping draw (%d,%d)-(%d,%d)", Xstart, Ystart, Xend, Yend);
    return;
  }
  esp_lcd_panel_draw_bitmap(panel_handle, Xstart, Ystart, Xend, Yend, color);                     // x_end End index on x-axis (x_end not included)
}


// backlight
uint8_t LCD_Backlight = 50;
void Backlight_Init()
{
  // Configure LEDC timer/channel only if an MCU pin is provided.
  if (LCD_Backlight_PIN >= 0) {
    ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = (ledc_timer_bit_t)Resolution,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = Frequency,
      .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
      .gpio_num = LCD_Backlight_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = (ledc_channel_t)PWM_Channel,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
  }

  // Ensure expander BL_EN (EXIO2) is configured as output and set initial state
  Mode_EXIO(EXIO_PIN2, 0);
  Set_Backlight(LCD_Backlight); // initialize to saved brightness (will toggle EXIO2)
}

void Set_Backlight(uint8_t Light)
{
  if (Light > Backlight_MAX) {
    printf("Set Backlight parameters in the range of 0 to 100 \r\n");
    return;
  }
  uint32_t max_duty = (1u << Resolution) - 1u;
  uint32_t duty = (uint32_t)Light * max_duty / Backlight_MAX;
  if (LCD_Backlight_PIN >= 0) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_Channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_Channel);
  }
  // Toggle expander BL_EN (EXIO2) so boards with expander-controlled
  // backlight enable behave correctly (power gate).
  if (Light > 0) Set_EXIO(EXIO_PIN2, High);
  else Set_EXIO(EXIO_PIN2, Low);
}

