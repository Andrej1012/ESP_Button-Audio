

// main.c — ESP-IDF v5.4.2 + NimBLE + Button send + Chunked receive

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_SERVER";
#define DEVICE_NAME "ESP32_BLE_SERVER"

// Button pin config
#define BUTTON_PIN GPIO_NUM_4
#define DEBOUNCE_DELAY_MS 50

// UUIDs
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t CHR_UUID =
    BLE_UUID128_INIT(0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd,
                     0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab);

static const ble_uuid128_t svc_list[] = { SVC_UUID };

// BLE globals
static uint8_t ble_addr_type;
static uint16_t chr_val_handle = 0;
static uint16_t cur_conn_handle = 0;
static bool notify_enabled = false;

// Button globals
static int last_button_state = 1; // HIGH by default (pull-up)
static int button_state = 1;
static int64_t last_debounce_time = 0;
static bool start_state = true;

// Chunked receive buffers
static char received_buffer[1024] = {0};
static char last_chunk[256] = {0};

// Forward declarations
static void ble_app_advertise(void);
static void notify_text(const char *msg);

// Notify helper
static void notify_text(const char *msg) {
    if (!notify_enabled || cur_conn_handle == 0 || chr_val_handle == 0) {
        ESP_LOGW(TAG, "Notify skipped (enabled=%d conn=%u handle=0x%04x)",
                 notify_enabled, cur_conn_handle, chr_val_handle);
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) {
        ESP_LOGE(TAG, "Failed to alloc mbuf for notify");
        return;
    }
    int rc = ble_gatts_notify_custom(cur_conn_handle, chr_val_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_notify_custom rc=%d", rc);
    }
}

// GATT access callback with chunked receiving
static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        const char *reply = "Hello from ESP32";
        os_mbuf_append(ctxt->om, reply, strlen(reply));
        ESP_LOGI(TAG, "READ -> \"%s\"", reply);
        return 0;
    }

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        int len = OS_MBUF_PKTLEN(ctxt->om);
        char buf[256];
        if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
        os_mbuf_copydata(ctxt->om, 0, len, buf);
        buf[len] = '\0';

        // Skip duplicate chunk
        if (strcmp(buf, last_chunk) == 0) {
            ESP_LOGI(TAG, "Duplicate chunk ignored: \"%s\"", buf);
            return 0;
        }
        strcpy(last_chunk, buf);

        ESP_LOGI(TAG, "Chunk received: \"%s\"", buf);

        // Append chunk to buffer
        strncat(received_buffer, buf, sizeof(received_buffer) - strlen(received_buffer) - 1);

        // Look for <EOM>
        char *eom;
        while ((eom = strstr(received_buffer, "<EOM>")) != NULL) {
            *eom = '\0'; // terminate message
            ESP_LOGI(TAG, "Full message received: \"%s\"", received_buffer);

            // Clear processed part
            size_t remaining = strlen(eom + 5);
            memmove(received_buffer, eom + 5, remaining + 1);
        }

        return 0;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// GATT server definition
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &CHR_UUID.u,
                .access_cb = chr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &chr_val_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

// GAP event handler
static int gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            cur_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected (conn=%u)", cur_conn_handle);
        } else {
            ESP_LOGI(TAG, "Connect failed (status=%d); advertising again", event->connect.status);
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected (conn=%u reason=%d); advertising again",
                 event->disconnect.conn.conn_handle, event->disconnect.reason);
        cur_conn_handle = 0;
        notify_enabled = false;
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; restarting");
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == chr_val_handle) {
            notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "Subscribe: notify=%d indicate=%d",
                     event->subscribe.cur_notify, event->subscribe.cur_indicate);
        }
        return 0;

    default:
        return 0;
    }
}

// Advertising
static void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t*)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids128 = svc_list;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params advp;
    memset(&advp, 0, sizeof(advp));
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                               &advp, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising with service UUID");
    }
}

// Host task
static void host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE host task start");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Sync callback
static void ble_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

// Button monitor task
static void button_task(void *param) {
    while (1) {
        int reading = gpio_get_level(BUTTON_PIN);

        if (reading != last_button_state) {
            last_debounce_time = esp_timer_get_time() / 1000; // ms
        }

        if ((esp_timer_get_time() / 1000 - last_debounce_time) > DEBOUNCE_DELAY_MS) {
            if (reading != button_state) {
                button_state = reading;
                if (button_state == 0) { // pressed
                    if (notify_enabled) {
                        if (start_state) {
                            ESP_LOGI(TAG, "Sending: start");
                            notify_text("start");
                            start_state = false;
                        } else {
                            ESP_LOGI(TAG, "Sending: cancel");
                            notify_text("cancel");
                            start_state = true;
                        }
                    } else {
                        ESP_LOGI(TAG, "Button pressed but no device connected");
                    }
                }
            }
        }

        last_button_state = reading;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// app_main
void app_main(void) {
    // Init NVS
    ESP_ERROR_CHECK(nvs_flash_init());

    // Init GPIO for button
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Init NimBLE
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);

    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(host_task);

    // Start button monitoring task
    xTaskCreate(button_task, "button_task", 6144, NULL, 5, NULL);
}
