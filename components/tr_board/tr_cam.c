// tr_cam: the CoreS3 camera (GC0308 on the parallel DVP port). See
// include/tr_board.h.
//
// Pins, power (AXP2101 ALDO3, BLDO1, BLDO2 and AW9523 P1_0) and the 20 MHz
// clock follow Espressif's BSP for the M5Stack CoreS3 (Apache-2.0). The
// driver is Espressif's esp32-camera.
#include "esp_camera.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_board_priv.h"

static const char *TAG = "tr_cam";

#define AXP2101_ADDR 0x34
#define AXP_LDO_EN   0x90
#define AXP_ALDO3_V  0x94 // (mV - 500) / 100
#define AXP_CAM_ON   0x34 // bit 2 ALDO3, bit 4 BLDO1, bit 5 BLDO2 (as the BSP sets them)
#define AW_P1_CAM    0x01 // AW9523 P1_0: camera enable

static bool started, powered;

esp_err_t tr_cam_power(void)
{
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(tr_board_i2c(&bus), TAG, "I2C");
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_ADDR,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t axp;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &axp), TAG, "AXP2101");
    const uint8_t volt[2] = {AXP_ALDO3_V, (3300 - 500) / 100};
    uint8_t reg = AXP_LDO_EN, en = 0;
    esp_err_t err = i2c_master_transmit(axp, volt, 2, 50);
    if (err == ESP_OK) {
        err = i2c_master_transmit_receive(axp, &reg, 1, &en, 1, 50);
    }
    if (err == ESP_OK) {
        const uint8_t on[2] = {AXP_LDO_EN, en | AXP_CAM_ON};
        err = i2c_master_transmit(axp, on, 2, 50);
    }
    i2c_master_bus_rm_device(axp);
    ESP_RETURN_ON_ERROR(err, TAG, "camera power");
    err = tr_board_aw9523_on(0, AW_P1_CAM);
    powered = err == ESP_OK;
    return err;
}

bool tr_cam_started(void)
{
    return started;
}

// The camera's power off again, for sleep: not while the camera runs.
esp_err_t tr_cam_power_off(void)
{
    ESP_RETURN_ON_FALSE(!started, ESP_ERR_INVALID_STATE, TAG, "camera running");
    if (!powered) {
        return ESP_OK; // never on, as in the emulator
    }
    powered = false;
    uint8_t en;
    ESP_RETURN_ON_ERROR(tr_board_reg_read(AXP2101_ADDR, AXP_LDO_EN, &en), TAG, "AXP2101");
    if (en & AXP_CAM_ON) {
        ESP_RETURN_ON_ERROR(tr_board_reg_write(AXP2101_ADDR, AXP_LDO_EN, en & ~AXP_CAM_ON), TAG,
                            "camera power");
    }
    return tr_board_aw9523_off(0, AW_P1_CAM);
}

esp_err_t tr_cam_start(void)
{
    if (started) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(tr_cam_power(), TAG, "power");
    vTaskDelay(pdMS_TO_TICKS(100));
    const bool psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0;
    const camera_config_t cfg = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = -1, // the camera has its own clock
        .pin_sccb_sda = -1,
        .pin_sccb_scl = -1,
        .sccb_i2c_port = TR_I2C_PORT, // share tr_board's I2C bus
        .pin_d0 = 39,
        .pin_d1 = 40,
        .pin_d2 = 41,
        .pin_d3 = 42,
        .pin_d4 = 15,
        .pin_d5 = 16,
        .pin_d6 = 48,
        .pin_d7 = 47,
        .pin_vsync = 46,
        .pin_href = 38,
        .pin_pclk = 45,
        .xclk_freq_hz = 20000000,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QVGA, // 320 x 240, the size of the screen
        .fb_count = psram ? 3 : 1,    // 3: a new frame is ready when a slow reader (10-track) asks
        .fb_location = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    ESP_RETURN_ON_ERROR(esp_camera_init(&cfg), TAG, "camera");
    started = true;
    return ESP_OK;
}
