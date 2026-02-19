/* * -------------------------------------------------------------------------------
 * PROJECT: Open-Source Pro-Intercom MCU System
 * VERSION: 2.8.1 (Stable Release - AVRCP RN Struct Fix)
 * DESCRIPTION: Dual-profile (A2DP Sink & HFP Client) with Full-Duplex I2S Audio
 * HARDWARE: ESP32-WROOM Series (Arduino Core 2.0.17)
 * -------------------------------------------------------------------------------
 */

#include "esp_bt.h"
#include "esp_avrc_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_hf_client_api.h"
#include "driver/i2s.h"
#include "nvs_flash.h"

// --- I2S PIN CONFIGURATION ---
#define I2S_NUM           I2S_NUM_0
#define I2S_BCK_IO        26
#define I2S_WS_IO         25
#define I2S_DO_IO         22  // Output to Speaker/DAC (MAX98357A)
#define I2S_DI_IO         35  // Input from Microphone (INMP441)

// --- STATE & LATENCY TRACKING ---
bool isCallActive = false;
bool isMediaStreaming = false;
unsigned long transitionStartTime = 0;

// --- VOLUME CONTROL ---
volatile uint8_t current_volume = 64; // Default volume at ~50%

// Handles volume commands from the phone/PC
void avrc_tg_event_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    esp_avrc_rn_param_t rn_param;

    switch (event) {
        // 1. The device (Windows/iOS) requests a volume change
        case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
            current_volume = param->set_abs_vol.volume;
            Serial.print("[AVRCP] Volume changed to: ");
            Serial.println(current_volume);

            // IMPORTANT: Send a CHANGED notification response back to the sender
            rn_param.volume = current_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
            break;

        // 2. The device connects and registers for volume sync notifications
        case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
            if (param->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
                Serial.println("[AVRCP] Handshake: Volume sync registered!");
                
                // IMPORTANT: Send an INTERIM response with the current volume
                rn_param.volume = current_volume;
                esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
            }
            break;

        default:
            break;
    }
}

// Dummy function required by iOS to unlock AVRCP features
void avrc_ct_event_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {}

// --- AUDIO DATA CALLBACKS ---

void hf_incoming_data_cb(const uint8_t *data, uint32_t len) {
    size_t bytes_written;
    i2s_write(I2S_NUM, data, len, &bytes_written, portMAX_DELAY);
}

uint32_t hf_outgoing_data_cb(uint8_t *data, uint32_t len) {
    size_t bytes_read = 0;
    i2s_read(I2S_NUM, data, len, &bytes_read, portMAX_DELAY);
    return (uint32_t)bytes_read;
}

void a2dp_sink_data_cb(const uint8_t *data, uint32_t len) {
    static int16_t scaled_data[2048]; 
    int16_t *pcm_data = (int16_t *)data;
    uint32_t num_samples = len / 2;

    // Logarithmic volume curve for better human perception
    float linear_vol = (float)current_volume / 127.0f;
    float vol_factor = linear_vol * linear_vol * linear_vol; 

    for (uint32_t i = 0; i < num_samples; i++) {
        scaled_data[i] = (int16_t)(pcm_data[i] * vol_factor);
    }

    size_t bytes_written;
    i2s_write(I2S_NUM, scaled_data, len, &bytes_written, portMAX_DELAY);
}

// --- EVENT HANDLERS ---

void hf_client_event_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
    if (event == 0) {
        if (param->conn_stat.state == 1) Serial.println("[STATUS] HFP: Profile Connected");
    } else if (event == 2) { 
        if (param->audio_stat.state == 2) { 
            isCallActive = true;
            i2s_set_clk(I2S_NUM, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
            Serial.println("[EVENT] MODE: VOICE_CALL_ACTIVE");
        } else {
            isCallActive = false;
            i2s_set_clk(I2S_NUM, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
            Serial.println("[EVENT] MODE: VOICE_CALL_IDLE");
        }
    }
}

void a2dp_sink_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    if (event == 0) {
        if (param->conn_stat.state == 2) Serial.println("[STATUS] A2DP: Profile Connected");
    } else if (event == 1) {
        if (param->audio_stat.state == 2) { 
            isMediaStreaming = true;
            Serial.println("[EVENT] MODE: MEDIA_STREAMING_STARTED");
        } else {
            isMediaStreaming = false;
            Serial.println("[EVENT] MODE: MEDIA_STREAMING_PAUSED");
        }
    }
}

void setup() {
    Serial.begin(115200);
    while(!Serial && millis() < 3000);
    delay(500);

    Serial.println("\n###########################################");
    Serial.println("#     PRO-INTERCOM SYSTEM INITIALIZING    #");
    Serial.println("###########################################");

    // Initialize Non-Volatile Storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // 1. Setup I2S Audio Hardware
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO
    };
    i2s_set_pin(I2S_NUM, &pin_config);

    // 2. ARDUINO-SAFE BLUETOOTH INITIALIZATION
    if (!btStart()) {
        Serial.println("[FATAL] Failed to start BT Controller.");
        while (1);
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        if (esp_bluedroid_init() != ESP_OK) {
            Serial.println("[FATAL] Failed to init Bluedroid.");
            while (1);
        }
    }

    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        if (esp_bluedroid_enable() != ESP_OK) {
            Serial.println("[FATAL] Failed to enable Bluedroid.");
            while (1);
        }
    }

    // 3. Set Device Name and Identity Class
    esp_bt_dev_set_device_name("esp_intercom");
    esp_bt_cod_t cod;
    cod.major = 0x04; cod.minor = 0x08; 
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);

    // 4. Initialize Volume Control (AVRCP MUST START BEFORE A2DP!)
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(avrc_ct_event_cb);

    esp_avrc_tg_init();
    esp_avrc_tg_register_callback(avrc_tg_event_cb);
    
    // Announce Absolute Volume support to the OS
    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    esp_avrc_tg_set_rn_evt_cap(&evt_set);

    // 5. Initialize A2DP (Media Playback)
    esp_a2d_register_callback(a2dp_sink_event_cb);
    esp_a2d_sink_init();
    esp_a2d_sink_register_data_callback(a2dp_sink_data_cb);

    // 6. Initialize HFP (Voice Calls/Intercom)
    esp_hf_client_register_callback(hf_client_event_cb);
    esp_hf_client_init();
    esp_hf_client_register_data_callback(hf_incoming_data_cb, hf_outgoing_data_cb);

    // 7. Finalize Discoverability
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    Serial.println("[SUCCESS] System ready. Scan for 'esp_intercom' on iPhone or PC.");
}

void loop() {
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 5000) {
        lastLog = millis();
        Serial.print("[INFO] System State: ");
        if (isCallActive) Serial.println("IN_VOICE_CALL");
        else if (isMediaStreaming) Serial.println("STREAMING_MEDIA");
        else Serial.println("STANDBY_IDLE");
    }
}
