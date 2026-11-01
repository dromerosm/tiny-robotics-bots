// tr_lcd: the CoreS3 screen (320x240, ILI9342C or ILI9342E on SPI) and text
// with a 5x7 font. See include/tr_board.h.
//
// The power-up order, the pins and the ILI9342E commands follow Espressif's
// BSP for the M5Stack CoreS3 (esp-bsp, bsp/m5stack_core_s3, Apache-2.0).
#include <string.h>
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "font5x7.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tr_board.h"

static const char *TAG = "tr_lcd";

#define LCD_HOST SPI3_HOST
#define LCD_SCLK 36
#define LCD_MOSI 37
#define LCD_DC   35
#define LCD_CS   3
#define LCD_HZ   (40 * 1000 * 1000)

// AW9523 pins we use (see tr_board_aw9523_on()).
#define AW_P0_TOUCH 0x01 // P0_0, touch chip reset (high = run)
#define AW_P1_LCD   0x02 // P1_1, screen reset (high = run)

#define AXP2101_ADDR 0x34
#define AXP_LDO_EN   0x90 // bit 7: DLDO1 on
#define AXP_DLDO1_V  0x99 // backlight voltage, (mV - 500) / 100
#define AXP_DLDO1_ON 0x80

// The touch chip's firmware ID tells which screen is fitted (as the BSP does).
#define TOUCH_ADDR   0x38
#define TOUCH_FIRMID 0xA6
#define FIRMID_9342E 0x12

#define BAND_H 16 // rows sent at once; the largest text (scale 2) is 16 rows high

static esp_lcd_panel_handle_t panel;
static esp_lcd_panel_io_handle_t io;
static volatile bool off; // tr_lcd_power(false): no drawing
// Two bands of pixels in DMA memory: while one goes out on SPI, the next one
// is filled. band is the one being filled.
static uint16_t *bands[2], *band;
static int next_band;
static SemaphoreHandle_t band_free; // counts the bands that are not being sent

// ILI9342E setup, copied from the BSP (ili9342e_init_cmds.h, Apache-2.0),
// without its 0x3A (0x55) and 0x36 (0x08): the driver sends the same values
// itself (16 bits, BGR) and warned that ours overwrote them.
static const ili9341_lcd_init_cmd_t ili9342e_init[] = {
    {0xCF, (uint8_t[]){0x00, 0xAA, 0xE0}, 3, 0},
    {0xED, (uint8_t[]){0x67, 0x03, 0x12, 0x81}, 4, 0},
    {0xE8, (uint8_t[]){0x8A, 0x01, 0x78}, 3, 0},
    {0xCB, (uint8_t[]){0x39, 0x2C, 0x00, 0x34, 0x02}, 5, 0},
    {0xF7, (uint8_t[]){0x20}, 1, 0},
    {0xF7, (uint8_t[]){0x20}, 1, 0},
    {0xEA, (uint8_t[]){0x00, 0x00}, 2, 0},
    {0xC0, (uint8_t[]){0x23}, 1, 0},
    {0xC1, (uint8_t[]){0x11}, 1, 0},
    {0xC5, (uint8_t[]){0x43, 0x4C}, 2, 0},
    {0xC7, (uint8_t[]){0xA0}, 1, 0},
    {0xB1, (uint8_t[]){0x00, 0x1B}, 2, 0},
    {0xF2, (uint8_t[]){0x00}, 1, 0},
    {0x26, (uint8_t[]){0x01}, 1, 0},
    {0xE0,
     (uint8_t[]){0x1F, 0x36, 0x36, 0x3A, 0x0C, 0x05, 0x4F, 0x87, 0x3C, 0x08, 0x11, 0x35, 0x19, 0x13,
                 0x00},
     15, 0},
    {0xE1,
     (uint8_t[]){0x00, 0x09, 0x09, 0x05, 0x13, 0x0A, 0x30, 0x78, 0x43, 0x07, 0x0E, 0x0A, 0x26, 0x2C,
                 0x1F},
     15, 0},
    {0xB7, (uint8_t[]){0x07}, 1, 0},
    {0xB6, (uint8_t[]){0x08, 0x82, 0x27}, 3, 0},
    {0xDD, (uint8_t[]){0x01}, 1, 0},
    {0x21, NULL, 0, 0},
    {0xD5, (uint8_t[]){0x00}, 1, 0},
    {0xB1, (uint8_t[]){0x22}, 1, 0},
    {0xC8, (uint8_t[]){0x38}, 1, 0},
    {0xCB, (uint8_t[]){0x1C}, 1, 0},
    {0xC9, (uint8_t[]){0x1A}, 1, 0},
    {0xCA, (uint8_t[]){0x1A}, 1, 0},
    {0xB7, (uint8_t[]){0x5A, 0x41, 0x11, 0x19}, 4, 0},
    {0xE4,
     (uint8_t[]){0x04, 0x08, 0x11, 0x06, 0x12, 0x07, 0x3A, 0x76, 0x47, 0x07, 0x0F, 0x0A, 0x11, 0x19,
                 0x05},
     15, 0},
    {0xE5,
     (uint8_t[]){0x02, 0x03, 0x07, 0x06, 0x12, 0x07, 0x36, 0x5F, 0x48, 0x06, 0x10, 0x0C, 0x16, 0x14,
                 0x09},
     15, 0},
    {0x11, NULL, 0, 120},
    {0x29, NULL, 0, 120},
};

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return i2c_master_transmit(dev, data, sizeof(data), 50);
}

static esp_err_t read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(dev, &reg, 1, value, 1, 50);
}

static esp_err_t add_device(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t hz,
                            i2c_master_dev_handle_t *dev)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = hz,
    };
    return i2c_master_bus_add_device(bus, &cfg, dev);
}

// True if the touch chip says the screen is an ILI9342E (units from August 2026 on).
static bool screen_is_9342e(i2c_master_bus_handle_t bus)
{
    i2c_master_dev_handle_t touch;
    if (add_device(bus, TOUCH_ADDR, 100000, &touch) != ESP_OK) {
        return false;
    }
    uint8_t id = 0;
    vTaskDelay(pdMS_TO_TICKS(300)); // the touch chip starts slowly
    for (int i = 0; i < 5; i++) {
        if (write_reg(touch, 0x00, 0x00) == ESP_OK &&
            read_reg(touch, TOUCH_FIRMID, &id) == ESP_OK && (id == 0x10 || id == FIRMID_9342E)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    i2c_master_bus_rm_device(touch);
    ESP_LOGI(TAG, "touch firmware ID 0x%02X: screen %s", id,
             id == FIRMID_9342E ? "ILI9342E" : "ILI9342C");
    return id == FIRMID_9342E;
}

static esp_err_t backlight_on(i2c_master_bus_handle_t bus)
{
    i2c_master_dev_handle_t axp;
    ESP_RETURN_ON_ERROR(add_device(bus, AXP2101_ADDR, 400000, &axp), TAG, "AXP2101");
    uint8_t en = 0;
    esp_err_t err = write_reg(axp, AXP_DLDO1_V, (3300 - 500) / 100); // 3.3 V, full brightness
    if (err == ESP_OK) {
        err = read_reg(axp, AXP_LDO_EN, &en);
    }
    if (err == ESP_OK) {
        err = write_reg(axp, AXP_LDO_EN, en | AXP_DLDO1_ON);
    }
    i2c_master_bus_rm_device(axp);
    return err;
}

// Like M5GFX on the CoreS3: brightness 0..255 sets DLDO1 to (b + 641) >> 5,
// 2.5 to 3.3 V in steps of 100 mV.
esp_err_t tr_lcd_brightness(int percent)
{
    if (!panel) {
        return ESP_ERR_INVALID_STATE;
    }
    percent = percent < 1 ? 1 : percent > 100 ? 100 : percent;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t axp;
    ESP_RETURN_ON_ERROR(tr_board_i2c(&bus), TAG, "I2C");
    ESP_RETURN_ON_ERROR(add_device(bus, AXP2101_ADDR, 400000, &axp), TAG, "AXP2101");
    const esp_err_t err = write_reg(axp, AXP_DLDO1_V, (percent * 255 / 100 + 641) >> 5);
    i2c_master_bus_rm_device(axp);
    return err;
}

bool tr_lcd_ready(void)
{
    return panel != NULL;
}

// DLDO1 on or off; its voltage (the brightness) stays.
static esp_err_t backlight(bool on)
{
    uint8_t en;
    const esp_err_t err = tr_board_reg_read(AXP2101_ADDR, AXP_LDO_EN, &en);
    if (err != ESP_OK) {
        return err; // no power chip, as in the emulator: nothing to say
    }
    const uint8_t now = on ? en | AXP_DLDO1_ON : en & ~AXP_DLDO1_ON;
    return now == en ? ESP_OK : tr_board_reg_write(AXP2101_ADDR, AXP_LDO_EN, now);
}

esp_err_t tr_lcd_power(bool on)
{
    if (!panel) {
        return on ? ESP_OK : backlight(false);
    }
    if (!on) {
        off = true;
        xSemaphoreTake(band_free, portMAX_DELAY); // both bands have gone out
        xSemaphoreTake(band_free, portMAX_DELAY);
        xSemaphoreGive(band_free);
        xSemaphoreGive(band_free);
        esp_lcd_panel_io_tx_param(io, 0x28, NULL, 0); // display off
        esp_lcd_panel_io_tx_param(io, 0x10, NULL, 0); // sleep in
        return backlight(false);
    }
    esp_lcd_panel_io_tx_param(io, 0x11, NULL, 0); // sleep out
    vTaskDelay(pdMS_TO_TICKS(120));               // the datasheet's wait after sleep out
    esp_lcd_panel_io_tx_param(io, 0x29, NULL, 0); // display on
    off = false;
    return backlight(true);
}

static bool on_band_sent(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(band_free, &woken);
    return woken == pdTRUE;
}

esp_err_t tr_lcd_init(void)
{
    if (panel) {
        return ESP_OK;
    }
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(tr_board_i2c(&bus), TAG, "I2C");

    // Take the touch chip (P0_0) and the screen (P1_1) out of reset.
    ESP_RETURN_ON_ERROR(tr_board_aw9523_on(AW_P0_TOUCH, AW_P1_LCD), TAG, "AW9523");
    const bool is_e = screen_is_9342e(bus);
    ESP_RETURN_ON_ERROR(backlight_on(bus), TAG, "backlight");

    const spi_bus_config_t spi = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = -1, // G35 is MISO for the SD card, but DC for the screen
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TR_LCD_W * BAND_H * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &spi, SPI_DMA_CH_AUTO), TAG, "SPI");

    band_free = xSemaphoreCreateCounting(2, 2);
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = LCD_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = on_band_sent,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io),
                        TAG, "panel IO");

    const ili9341_vendor_config_t vendor = {
        .init_cmds = ili9342e_init,
        .init_cmds_size = sizeof(ili9342e_init) / sizeof(ili9342e_init[0]),
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1, // reset is on the AW9523, set above
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
        .vendor_config = is_e ? (void *)&vendor : NULL,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9341(io, &panel_cfg, &panel), TAG, "panel");
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, true);

    bands[0] = heap_caps_malloc(TR_LCD_W * BAND_H * sizeof(uint16_t), MALLOC_CAP_DMA);
    bands[1] = heap_caps_malloc(TR_LCD_W * BAND_H * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!bands[0] || !bands[1]) {
        return ESP_ERR_NO_MEM;
    }
    tr_lcd_fill(TR_LCD_BLACK);
    return esp_lcd_panel_disp_on_off(panel, true);
}

// The screen takes the high byte first.
static inline uint16_t wire(uint16_t color)
{
    return (uint16_t)(color << 8 | color >> 8);
}

// Send w x h pixels from the band to (x, y). It returns at once: the band goes
// out by DMA while the caller fills the other one.
static void send_band(int x, int y, int w, int h)
{
    esp_lcd_panel_draw_bitmap(panel, x, y, x + w, y + h, band);
}

// Wait for a free band and make it the one to fill. The bands are sent in
// order, so the free one is always the older of the two.
static void take_band(void)
{
    xSemaphoreTake(band_free, portMAX_DELAY);
    band = bands[next_band];
    next_band ^= 1;
}

void tr_lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!panel || off || w <= 0 || h <= 0) {
        return;
    }
    for (int row = y; row < y + h; row += BAND_H) {
        const int rows = (y + h - row) < BAND_H ? (y + h - row) : BAND_H;
        take_band();
        for (int i = 0; i < w * rows; i++) {
            band[i] = wire(color);
        }
        send_band(x, row, w, rows);
    }
}

void tr_lcd_fill(uint16_t color)
{
    tr_lcd_fill_rect(0, 0, TR_LCD_W, TR_LCD_H, color);
}

void tr_lcd_text(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *text)
{
    if (!panel || off || scale < 1 || scale > 2) {
        return;
    }
    const int cell_w = 6 * scale, h = 8 * scale;
    int n = strlen(text);
    if (x + n * cell_w > TR_LCD_W) {
        n = (TR_LCD_W - x) / cell_w; // cut what does not fit
    }
    if (n <= 0 || y < 0 || y + h > TR_LCD_H) {
        return;
    }
    const int w = n * cell_w;
    take_band();
    for (int i = 0; i < w * h; i++) {
        band[i] = wire(bg);
    }
    for (int c = 0; c < n; c++) {
        unsigned char ch = (unsigned char)text[c];
        if (ch < FONT_FIRST || ch > FONT_LAST) {
            ch = '?';
        }
        const uint8_t *glyph = &font5x7[(ch - FONT_FIRST) * 5];
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 8; row++) {
                if (!(glyph[col] >> row & 1)) {
                    continue;
                }
                for (int dy = 0; dy < scale; dy++) {
                    for (int dx = 0; dx < scale; dx++) {
                        band[(row * scale + dy) * w + c * cell_w + col * scale + dx] = wire(fg);
                    }
                }
            }
        }
    }
    send_band(x, y, w, h);
}

void tr_lcd_blit(int x, int y, int w, int h, const uint8_t *pixels)
{
    if (!panel || off || w <= 0 || w > TR_LCD_W) {
        return;
    }
    // Copy through the DMA band: the frame may be in PSRAM.
    const int rows_per_band = BAND_H * TR_LCD_W / w;
    for (int top = 0; top < h; top += rows_per_band) {
        const int rows = h - top < rows_per_band ? h - top : rows_per_band;
        take_band();
        memcpy(band, pixels + (size_t)top * w * 2, (size_t)rows * w * 2);
        send_band(x, y + top, w, rows);
    }
}

void tr_lcd_render_rows(int y0, int y1, tr_lcd_pixel_fn pixel, void *ctx)
{
    if (!panel || off) {
        return;
    }
    y0 = y0 < 0 ? 0 : y0;
    y1 = y1 > TR_LCD_H ? TR_LCD_H : y1;
    for (int top = y0; top < y1; top += BAND_H) {
        const int rows = y1 - top < BAND_H ? y1 - top : BAND_H;
        take_band();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < TR_LCD_W; x++) {
                band[y * TR_LCD_W + x] = wire(pixel(x, top + y, ctx));
            }
        }
        send_band(0, top, TR_LCD_W, rows);
    }
}

void tr_lcd_render(tr_lcd_pixel_fn pixel, void *ctx)
{
    tr_lcd_render_rows(0, TR_LCD_H, pixel, ctx);
}
