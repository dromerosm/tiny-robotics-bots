// bridge: see bridge.h.
//
// HomeKit's protocol (HAP) comes from Espressif's esp-homekit-sdk. The robot
// is a bridge: one accessory for itself, and one bridged accessory with an
// Outlet service for each outlet of each plug. One accessory per outlet, and
// not one per power strip, so each outlet has its own name, tile and room in
// the Home app.
//
// Home writes "On": the order goes to the plugs' queue and the answer to Home
// is at once, so HomeKit's task never waits on the network. When the worker
// has read the plug again, bridge_fresh() tells Home what is true. That is
// also how Home learns of a button pressed on the plug.
//
// A plug that does not answer gives a read error, and Home shows "No
// Response" for its outlets.
#include "bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "hap.h"
#include "hap_apple_chars.h"
#include "hap_apple_servs.h"
#include "mdns.h"
#include "nvs.h"
#include "plugs.h"

static const char *TAG = "bridge";

static hap_char_t *on_char[PLUGS_MAX_DEV][PLUGS_MAX_SW];
static char code[11]; // "123-45-678"
static char setup_id[5];
static char qr_text[24]; // "X-HM://" + 9 characters + the setup id
static volatile bool started;
static int outlets;

// An outlet in a service's private pointer: device and switch, plus one so
// the first outlet is not NULL.
#define PRIV(dev, sw) ((void *)(intptr_t)((dev) * PLUGS_MAX_SW + (sw) + 1))
#define PRIV_DEV(p)   (((int)(intptr_t)(p) - 1) / PLUGS_MAX_SW)
#define PRIV_SW(p)    (((int)(intptr_t)(p) - 1) % PLUGS_MAX_SW)

// The setup code and the setup id, made once with the chip's random numbers
// and kept in NVS. HomeKit does not take codes that are easy to guess.
static void load_code(void)
{
    nvs_handle_t nvs;
    size_t code_size = sizeof(code), id_size = sizeof(setup_id);
    if (nvs_open("bridge", NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "no NVS: the setup code changes on every boot");
        nvs = 0;
    }
    if (nvs && nvs_get_str(nvs, "code", code, &code_size) == ESP_OK &&
        nvs_get_str(nvs, "setup_id", setup_id, &id_size) == ESP_OK) {
        nvs_close(nvs);
        return;
    }
    static const char *easy[] = {"12345678", "87654321"};
    char digits[9];
    bool ok;
    do {
        snprintf(digits, sizeof(digits), "%08u", (unsigned)(esp_random() % 100000000u));
        ok = strspn(digits, (const char[]){digits[0], 0}) != 8; // not one digit 8 times
        for (int i = 0; i < 2; i++) {
            ok = ok && strcmp(digits, easy[i]) != 0;
        }
    } while (!ok);
    snprintf(code, sizeof(code), "%.3s-%.2s-%.3s", digits, digits + 3, digits + 5);
    static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    for (int i = 0; i < 4; i++) {
        setup_id[i] = alphabet[esp_random() % 36];
    }
    setup_id[4] = 0;
    if (nvs) {
        nvs_set_str(nvs, "code", code);
        nvs_set_str(nvs, "setup_id", setup_id);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

// The text of the QR code: the SDK packs the accessory category, the setup
// code and the setup id. Home finds the accessory by a hash of that id, which
// the SDK puts in the mDNS record.
static void make_qr_text(void)
{
    char *text = esp_hap_get_setup_payload(code, setup_id, false, HAP_CID_BRIDGE);
    snprintf(qr_text, sizeof(qr_text), "%s", text ? text : "");
    free(text);
}

static int identify(hap_acc_t *acc)
{
    ESP_LOGI(TAG, "identify");
    return HAP_SUCCESS;
}

static int outlet_write(hap_write_data_t data[], int count, void *serv_priv, void *write_priv)
{
    const int dev = PRIV_DEV(serv_priv), sw = PRIV_SW(serv_priv);
    int ret = HAP_SUCCESS;
    for (int i = 0; i < count; i++) {
        hap_write_data_t *w = &data[i];
        if (strcmp(hap_char_get_type_uuid(w->hc), HAP_CHAR_UUID_ON) != 0) {
            *w->status = HAP_STATUS_RES_ABSENT;
        } else if (plugs_set(dev, sw, w->val.b)) {
            hap_char_update_val(w->hc, &w->val);
            *w->status = HAP_STATUS_SUCCESS;
        } else {
            *w->status = HAP_STATUS_RES_BUSY;
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static int outlet_read(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    if (plugs_state(PRIV_DEV(serv_priv), PRIV_SW(serv_priv)) < 0) {
        *status = HAP_STATUS_COMM_ERR;
        return HAP_FAIL;
    }
    *status = HAP_STATUS_SUCCESS;
    return HAP_SUCCESS;
}

void bridge_fresh(int dev)
{
    if (!started) {
        return;
    }
    for (int sw = 0; sw < plugs_switches(dev); sw++) {
        const int on = plugs_state(dev, sw);
        hap_char_t *hc = on_char[dev][sw];
        if (on >= 0 && hc && hap_char_get_val(hc)->b != (on == 1)) {
            hap_val_t val = {.b = on == 1};
            hap_char_update_val(hc, &val);
        }
    }
}

static void add_outlet(int dev, int sw)
{
    char name[48], serial[40];
    snprintf(name, sizeof(name), "%s %s", plugs_dev_name(dev), plugs_sw_name(dev, sw));
    snprintf(serial, sizeof(serial), "%s-%d", plugs_dev_id(dev), plugs_sw_dp(dev, sw));
    hap_acc_cfg_t cfg = {
        .name = name,
        .manufacturer = "Tuya",
        .model = "Smart plug",
        .serial_num = serial,
        .fw_rev = "1.0",
        .pv = "1.1.0",
        .identify_routine = identify,
        .cid = HAP_CID_BRIDGE,
    };
    hap_acc_t *acc = hap_acc_create(&cfg);
    hap_serv_t *serv = hap_serv_outlet_create(plugs_state(dev, sw) == 1, true);
    if (!acc || !serv) {
        ESP_LOGE(TAG, "no memory for %s", name);
        return;
    }
    hap_serv_add_char(serv, hap_char_name_create(name));
    hap_serv_set_priv(serv, PRIV(dev, sw));
    hap_serv_set_write_cb(serv, outlet_write);
    hap_serv_set_read_cb(serv, outlet_read);
    hap_acc_add_serv(acc, serv);
    on_char[dev][sw] = hap_serv_get_char_by_uuid(serv, HAP_CHAR_UUID_ON);
    outlets++;
    // The same number for this outlet on every boot, or Home would see a new
    // accessory each time. The SDK keeps it in NVS under a key of up to 15
    // characters: the end of the device id (the part that changes most) and
    // the data point.
    char key[16];
    const char *id = plugs_dev_id(dev);
    const size_t len = strlen(id);
    snprintf(key, sizeof(key), "%s-%d", id + (len > 11 ? len - 11 : 0), plugs_sw_dp(dev, sw));
    hap_add_bridged_accessory(acc, hap_get_unique_aid(key));
}

esp_err_t bridge_start(void)
{
    load_code();
    make_qr_text();
    if (hap_init(HAP_TRANSPORT_WIFI) != HAP_SUCCESS) {
        return ESP_FAIL;
    }
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char serial[13];
    snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3],
             mac[4], mac[5]);
    hap_acc_cfg_t cfg = {
        .name = "StackChan Bridge",
        .manufacturer = "tiny-robotics",
        .model = "17-homekit",
        .serial_num = serial,
        .fw_rev = "1.0",
        .pv = "1.1.0",
        .identify_routine = identify,
        .cid = HAP_CID_BRIDGE,
    };
    hap_acc_t *acc = hap_acc_create(&cfg);
    if (!acc) {
        return ESP_ERR_NO_MEM;
    }
    uint8_t product_data[] = {'T', 'R', 'B', 'R', 'I', 'D', 'G', 'E'};
    hap_acc_add_product_data(acc, product_data, sizeof(product_data));
    hap_add_accessory(acc);
    for (int dev = 0; dev < plugs_count(); dev++) {
        for (int sw = 0; sw < plugs_switches(dev); sw++) {
            add_outlet(dev, sw);
        }
    }
    hap_set_setup_code(code);
    hap_set_setup_id(setup_id);
    if (hap_start() != HAP_SUCCESS) {
        return ESP_FAIL;
    }
    // The SDK names the robot "MyHost" on the network. Give it back the name
    // tr_ota set, so http://stackchan.local/ still works.
    mdns_hostname_set("stackchan");
    started = true;
    ESP_LOGI(TAG, "HomeKit bridge with %d outlets, %d paired", outlets, bridge_paired());
    // With printf, so the code is on the USB port but never in GET /log,
    // which anyone on the network can read.
    printf("HomeKit setup code: %s\n", code);
    return ESP_OK;
}

const char *bridge_code(void)
{
    return code;
}

const char *bridge_qr_text(void)
{
    return qr_text;
}

int bridge_paired(void)
{
    return started ? hap_get_paired_controller_count() : 0;
}

void bridge_unpair(void)
{
    hap_reset_pairings();
}
