// tr_mic: the CoreS3 microphones (ES7210 ADC on I2S). See include/tr_board.h.
//
// Pins, the AW9523 enable pin and the esp_codec_dev setup follow Espressif's
// BSP for the M5Stack CoreS3 (bsp_audio.c, Apache-2.0).
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "tr_board.h"

static const char *TAG = "tr_mic";

#define I2S_MCLK  0
#define I2S_BCLK  34
#define I2S_WS    33
#define I2S_DIN   14
#define AW_P0_MIC 0x04 // AW9523 P0_2: microphone (and speaker) enable

static esp_codec_dev_handle_t mic;

esp_err_t tr_mic_start(uint32_t sample_rate)
{
    if (mic) {
        return ESP_OK;
    }
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(tr_board_i2c(&bus), TAG, "I2C");
    ESP_RETURN_ON_ERROR(tr_board_aw9523_on(AW_P0_MIC, 0), TAG, "mic enable");

    i2s_chan_handle_t rx;
    const i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan, NULL, &rx), TAG, "I2S");
    const i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_MCLK,
                .bclk = I2S_BCLK,
                .ws = I2S_WS,
                .dout = I2S_GPIO_UNUSED,
                .din = I2S_DIN,
            },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &std), TAG, "I2S mode");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "I2S enable");

    audio_codec_i2s_cfg_t i2s_cfg = {.port = I2S_NUM_0, .rx_handle = rx};
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    audio_codec_i2c_cfg_t i2c_cfg = {
        .addr = ES7210_CODEC_DEFAULT_ADDR,
        .bus_handle = bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(data_if && ctrl_if, ESP_FAIL, TAG, "codec interfaces");
    es7210_codec_cfg_t es7210_cfg = {.ctrl_if = ctrl_if};
    const audio_codec_if_t *codec = es7210_codec_new(&es7210_cfg);
    ESP_RETURN_ON_FALSE(codec, ESP_FAIL, TAG, "ES7210");
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = codec,
        .data_if = data_if,
    };
    mic = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(mic, ESP_FAIL, TAG, "codec device");
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 1,
        .sample_rate = sample_rate,
    };
    ESP_RETURN_ON_FALSE(esp_codec_dev_open(mic, &fs) == ESP_CODEC_DEV_OK, ESP_FAIL, TAG, "open");
    esp_codec_dev_set_in_gain(mic, 30.0f);
    return ESP_OK;
}

esp_err_t tr_mic_read(int16_t *samples, int count)
{
    ESP_RETURN_ON_FALSE(mic, ESP_ERR_INVALID_STATE, TAG, "not started");
    return esp_codec_dev_read(mic, samples, count * sizeof(int16_t)) == ESP_CODEC_DEV_OK ? ESP_OK
                                                                                         : ESP_FAIL;
}
