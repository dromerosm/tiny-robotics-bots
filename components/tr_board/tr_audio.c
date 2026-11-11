// tr_audio: the CoreS3 speaker and microphones at the same time (full
// duplex). See include/tr_board.h.
//
// One I2S bus: the ESP32-S3 sends the speaker's samples to the AW88298
// amplifier and reads the ES7210's four inputs in TDM slots, with the same
// clock. Both directions use 64 bit clocks per sample: the speaker as two
// 32-bit slots, the inputs as four 16-bit slots. Pins and the AW88298 gain
// follow Espressif's CoreS3 BSP and xiaozhi-esp32's StackChan board (MIT).
#include <string.h>
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_heap_caps.h"
#include "tr_board.h"

static const char *TAG = "tr_audio";

#define I2S_MCLK 0
#define I2S_BCLK 34
#define I2S_WS   33
#define I2S_DOUT 13
#define I2S_DIN  14
#define AW_P0_EN 0x04 // AW9523 P0_2: microphones and speaker enable
#define CHUNK    256  // samples per write, for the stereo 32-bit copy

static esp_codec_dev_handle_t speaker, mics;
static int32_t *out; // CHUNK stereo 32-bit frames
static uint32_t rate;
static int volume = 70;
static float gain = 30.0f;
static bool paused;

// Open both directions and set the volume and the gain again.
static esp_err_t open_both(void)
{
    esp_codec_dev_sample_info_t fs = {.bits_per_sample = 32, .channel = 2, .sample_rate = rate};
    ESP_RETURN_ON_FALSE(esp_codec_dev_open(speaker, &fs) == ESP_CODEC_DEV_OK, ESP_FAIL, TAG,
                        "open speaker");
    fs = (esp_codec_dev_sample_info_t){
        .bits_per_sample = 16, .channel = TR_AUDIO_INPUTS, .sample_rate = rate};
    ESP_RETURN_ON_FALSE(esp_codec_dev_open(mics, &fs) == ESP_CODEC_DEV_OK, ESP_FAIL, TAG,
                        "open microphones");
    esp_codec_dev_set_out_vol(speaker, volume);
    esp_codec_dev_set_in_gain(mics, gain);
    return ESP_OK;
}

esp_err_t tr_audio_start(uint32_t sample_rate)
{
    if (speaker) {
        return ESP_OK;
    }
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(tr_board_i2c(&bus), TAG, "I2C");
    ESP_RETURN_ON_ERROR(tr_board_aw9523_on(AW_P0_EN, 0), TAG, "enable");
    out = heap_caps_malloc(CHUNK * 2 * sizeof(int32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(out, ESP_ERR_NO_MEM, TAG, "buffer");

    // TX and RX on I2S0. The library sets the slots when each side opens.
    i2s_chan_handle_t tx, rx;
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.dma_desc_num = 4;
    chan.dma_frame_num = 128; // 8 ms per buffer at 16 kHz: short, for the echo canceller
    chan.auto_clear_after_cb = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan, &tx, &rx), TAG, "I2S");
    const i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {.mclk = I2S_MCLK,
                     .bclk = I2S_BCLK,
                     .ws = I2S_WS,
                     .dout = I2S_DOUT,
                     .din = I2S_GPIO_UNUSED},
    };
    i2s_tdm_config_t tdm = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO,
            I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
        .gpio_cfg = {.mclk = I2S_MCLK,
                     .bclk = I2S_BCLK,
                     .ws = I2S_WS,
                     .dout = I2S_GPIO_UNUSED,
                     .din = I2S_DIN},
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std), TAG, "TX");
    ESP_RETURN_ON_ERROR(i2s_channel_init_tdm_mode(rx, &tdm), TAG, "RX");

    audio_codec_i2s_cfg_t i2s_cfg = {.port = I2S_NUM_0, .rx_handle = rx, .tx_handle = tx};
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_FAIL, TAG, "I2S data");

    // The speaker: AW88298, 16-bit samples sent as 32-bit stereo.
    audio_codec_i2c_cfg_t i2c_cfg = {.addr = AW88298_CODEC_DEFAULT_ADDR, .bus_handle = bus};
    const audio_codec_ctrl_if_t *spk_ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
    aw88298_codec_cfg_t aw = {
        .ctrl_if = spk_ctrl,
        .gpio_if = audio_codec_new_gpio(),
        .reset_pin = -1,
        .hw_gain = {.pa_voltage = 5.0, .codec_dac_voltage = 3.3, .pa_gain = 1},
    };
    const audio_codec_if_t *aw_if = aw88298_codec_new(&aw);
    ESP_RETURN_ON_FALSE(spk_ctrl && aw_if, ESP_FAIL, TAG, "AW88298");
    esp_codec_dev_cfg_t dev = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = aw_if, .data_if = data_if};
    speaker = esp_codec_dev_new(&dev);
    ESP_RETURN_ON_FALSE(speaker, ESP_FAIL, TAG, "speaker");

    // The inputs: ES7210, all four, in TDM.
    i2c_cfg.addr = ES7210_CODEC_DEFAULT_ADDR;
    const audio_codec_ctrl_if_t *mic_ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
    es7210_codec_cfg_t es = {
        .ctrl_if = mic_ctrl,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4,
    };
    const audio_codec_if_t *es_if = es7210_codec_new(&es);
    ESP_RETURN_ON_FALSE(mic_ctrl && es_if, ESP_FAIL, TAG, "ES7210");
    dev = (esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_IN, .codec_if = es_if, .data_if = data_if};
    mics = esp_codec_dev_new(&dev);
    ESP_RETURN_ON_FALSE(mics, ESP_FAIL, TAG, "microphones");

    rate = sample_rate;
    return open_both();
}

esp_err_t tr_audio_pause(bool pause)
{
    ESP_RETURN_ON_FALSE(speaker && mics, ESP_ERR_INVALID_STATE, TAG, "not started");
    if (pause == paused) {
        return ESP_OK;
    }
    paused = pause;
    if (!pause) {
        return open_both();
    }
    esp_codec_dev_close(speaker);
    esp_codec_dev_close(mics);
    return ESP_OK;
}

esp_err_t tr_audio_read(int16_t *frames, int count)
{
    ESP_RETURN_ON_FALSE(mics, ESP_ERR_INVALID_STATE, TAG, "not started");
    return esp_codec_dev_read(mics, frames, count * TR_AUDIO_INPUTS * sizeof(int16_t)) ==
                   ESP_CODEC_DEV_OK
               ? ESP_OK
               : ESP_FAIL;
}

esp_err_t tr_audio_write(const int16_t *samples, int count)
{
    ESP_RETURN_ON_FALSE(speaker, ESP_ERR_INVALID_STATE, TAG, "not started");
    while (count > 0) {
        const int n = count < CHUNK ? count : CHUNK;
        for (int i = 0; i < n; i++) {
            out[2 * i] = out[2 * i + 1] = (int32_t)samples[i] << 16;
        }
        if (esp_codec_dev_write(speaker, out, n * 2 * sizeof(int32_t)) != ESP_CODEC_DEV_OK) {
            return ESP_FAIL;
        }
        samples += n;
        count -= n;
    }
    return ESP_OK;
}

esp_err_t tr_audio_volume(int percent)
{
    ESP_RETURN_ON_FALSE(speaker, ESP_ERR_INVALID_STATE, TAG, "not started");
    volume = percent;
    return esp_codec_dev_set_out_vol(speaker, percent) == ESP_CODEC_DEV_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t tr_audio_gain(float db)
{
    ESP_RETURN_ON_FALSE(mics, ESP_ERR_INVALID_STATE, TAG, "not started");
    gain = db;
    return esp_codec_dev_set_in_gain(mics, db) == ESP_CODEC_DEV_OK ? ESP_OK : ESP_FAIL;
}
