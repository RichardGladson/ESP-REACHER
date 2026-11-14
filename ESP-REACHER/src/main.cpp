/*
    ESP-REACHER by Richard Gladson
    https://github.com/RichardGladson/ESP-REACHER
    Copyright (c) 2026 Richard Gladson

    Licensed under the GNU General Public License v3.0
    https://www.gnu.org/licenses/gpl-3.0.html

    SPDX-License-Identifier: GPL-3.0-only
*/
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define SPI_SCK   4
#define SPI_MISO  5
#define SPI_MOSI  6
#define R1_CE     0
#define R1_CSN    1
#define R2_CE     7
#define R2_CSN    10
#define R3_CE     20
#define R3_CSN    21
#define LED_PIN   8

#define PROTOCOL_MAGIC            0xDB
#define REACHER_PROTOCOL_VERSION  0x04
#define DEVICE_ID_LEN             6
#define PROTOCOL_MAX_PAYLOAD      200
#define ESPNOW_CHANNEL            1

enum class PacketType : uint8_t {
    DISCOVERY_BEACON       = 0x01,
    PAIRING_REQUEST        = 0x02,
    PAIRING_RESPONSE       = 0x03,
    HEARTBEAT              = 0x04,
    DEVICE_IDENT           = 0x10,
    STATUS_REQUEST         = 0x11,
    STATUS_RESPONSE        = 0x12,
    ERROR_RESPONSE         = 0x13,
    MODE_COMMAND           = 0x20,
    RADIO_CONFIG_COMMAND   = 0x21,
    SCAN_TEST_COMMAND      = 0x22,
    STOP_COMMAND           = 0x23,
    POWER_PROFILE_COMMAND  = 0x24,
    DWELL_OVERRIDE_COMMAND = 0x25,
    CUSTOM_PAYLOAD         = 0xF0,
};

enum class DeviceRole : uint8_t {
    UNKNOWN = 0x00,
    REACHER = 0x01,
    CORE    = 0x02,
};

enum class OperatingMode : uint8_t {
    IDLE              = 0x00,
    BLUETOOTH_SURVEY  = 0x01,
    WIFI_SURVEY       = 0x02,
    BLE_SURVEY        = 0x03,
    ZIGBEE_SURVEY     = 0x04,
    ALL_SURVEY        = 0x05,
    STOPPED           = 0x06,
};

enum class PowerProfile : uint8_t {
    SUSTAINED     = 0x00,
    BURST         = 0x01,
    THERMAL_LIMIT = 0x02,
};

struct __attribute__((packed)) ProtocolHeader {
    uint8_t    magic;
    uint8_t    version;
    PacketType type;
    DeviceRole sender_role;
    uint8_t    sender_id[DEVICE_ID_LEN];
    uint16_t   sequence;
    uint8_t    payload_len;
    uint8_t    checksum;
};

struct __attribute__((packed)) ProtocolPacket {
    ProtocolHeader header;
    uint8_t        payload[PROTOCOL_MAX_PAYLOAD];
};

struct __attribute__((packed)) PairingRequestPayload {
    uint8_t reaper_id[DEVICE_ID_LEN];
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t radio_count;
};

struct __attribute__((packed)) PairingResponsePayload {
    uint8_t status;
    uint8_t core_id[DEVICE_ID_LEN];
    uint8_t assigned_slot;
};

struct __attribute__((packed)) HeartbeatPayload {
    uint8_t  reaper_id[DEVICE_ID_LEN];
    uint32_t uptime_ms;
    uint8_t  mode;
    uint8_t  radio_status;
    uint8_t  power_profile;
    uint16_t current_dwell_us;
    uint32_t hop_count;
};

struct __attribute__((packed)) StatusResponsePayload {
    uint8_t  mode;
    uint8_t  radio_status;
    uint8_t  core_connected;
    uint8_t  power_profile;
    uint16_t current_dwell_us;
    uint32_t hop_count;
    uint16_t free_heap;
    uint32_t uptime_ms;
};

struct __attribute__((packed)) ScanTestPayload {
    uint8_t survey_type;
    uint8_t start_channel;
    uint8_t end_channel;
    uint8_t hop_step;
};

struct __attribute__((packed)) PowerProfilePayload {
    uint8_t profile;
};

struct __attribute__((packed)) DwellOverridePayload {
    uint16_t dwell_us;
};

const uint8_t bt_channels[] = {0,1,2,4,6,8,22,24,26,28,30,32,34,46,48,50,52,74,76,78,80};
const uint8_t ble_adv_channels[] = {2, 26, 80};
const uint8_t ble_data_channels[] = {0,1,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,27,28,29,30,31,32,33,34,35,36,37,38,39,0};
const uint8_t wifi_channels[] = {12,17,22,27,32,37,42,47,52,57,62,67,72,12,17};
const uint8_t zigbee_channels[] = {11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,11,12};
const uint8_t all_channels[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125};

RF24 radio1(R1_CE, R1_CSN, 16000000);
RF24 radio2(R2_CE, R2_CSN, 16000000);
RF24 radio3(R3_CE, R3_CSN, 16000000);

uint8_t local_mac[6];
uint8_t doombox_mac[6];
bool paired = false;
uint16_t sequence = 0;
volatile OperatingMode currentMode = OperatingMode::IDLE;
volatile PowerProfile currentProfile = PowerProfile::SUSTAINED;
volatile uint16_t currentDwell_us = 0;
volatile uint32_t hopCount = 0;
volatile bool hopResetPending = false;
volatile bool profileChangePending = false;
uint32_t jamStartTime = 0;
uint32_t lastRadioCheck = 0;
uint8_t radio1_failCount = 0;
uint8_t radio2_failCount = 0;
uint8_t radio3_failCount = 0;
TaskHandle_t hopTaskHandle = nullptr;
SemaphoreHandle_t spiMutex = nullptr;

uint32_t prbs_state;
uint8_t payload[32];
volatile uint8_t payload_len = 32;

void initPRBS() {
    prbs_state = esp_random();
    if (prbs_state == 0) prbs_state = 0xDEADBEEF;
}

void generatePRBS() {
    for (int i = 0; i < 32; i += 4) {
        prbs_state ^= prbs_state << 13;
        prbs_state ^= prbs_state >> 17;
        prbs_state ^= prbs_state << 5;
        memcpy(&payload[i], &prbs_state, 4);
    }
    payload_len = (prbs_state % 32) + 1;
}

void configureRadio(RF24& radio, rf24_pa_dbm_e pa_level) {
    radio.setAutoAck(false);
    radio.stopListening();
    radio.setRetries(0, 0);
    radio.setPALevel(pa_level, true);
    radio.setDataRate(RF24_2MBPS);
    radio.setCRCLength(RF24_CRC_DISABLED);
}

void applyPowerProfile() {
    rf24_pa_dbm_e level;
    switch (currentProfile) {
        case PowerProfile::BURST:
            level = RF24_PA_MAX;
            break;
        case PowerProfile::THERMAL_LIMIT:
            if (millis() - jamStartTime > 30000) {
                level = RF24_PA_HIGH;
            } else {
                level = RF24_PA_MAX;
            }
            break;
        case PowerProfile::SUSTAINED:
        default:
            level = RF24_PA_HIGH;
            break;
    }
    radio1.setPALevel(level, true);
    radio2.setPALevel(level, true);
    radio3.setPALevel(level, true);
}

void hopTask(void* pvParameters) {
    uint32_t hopIndex1 = 0;
    uint32_t hopIndex3 = 0;
    OperatingMode localMode = OperatingMode::IDLE;

    while (true) {
        if (profileChangePending) {
            if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
                applyPowerProfile();
                xSemaphoreGive(spiMutex);
                profileChangePending = false;
            }
        }

        if (currentMode != localMode || hopResetPending) {
            if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
                radio1.stopListening();
                radio2.stopListening();
                radio3.stopListening();
                xSemaphoreGive(spiMutex);
            }
            localMode = currentMode;
            hopIndex1 = 0;
            hopIndex3 = 0;
            hopResetPending = false;
            hopCount = 0;
            jamStartTime = millis();
        }

        if (localMode == OperatingMode::IDLE || localMode == OperatingMode::STOPPED) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        generatePRBS();

        const uint8_t* channelArray1;
        const uint8_t* channelArray3;
        uint8_t arraySize1;
        uint8_t arraySize3;
        uint16_t dwell_us;

        switch (localMode) {
            case OperatingMode::BLUETOOTH_SURVEY:
                channelArray1 = bt_channels;
                channelArray3 = bt_channels;
                arraySize1 = sizeof(bt_channels);
                arraySize3 = sizeof(bt_channels);
                dwell_us = 80;
                break;
            case OperatingMode::BLE_SURVEY:
                channelArray1 = ble_data_channels;
                channelArray3 = ble_adv_channels;
                arraySize1 = sizeof(ble_data_channels);
                arraySize3 = sizeof(ble_adv_channels);
                dwell_us = 5000;
                break;
            case OperatingMode::WIFI_SURVEY:
                channelArray1 = wifi_channels;
                channelArray3 = wifi_channels;
                arraySize1 = sizeof(wifi_channels);
                arraySize3 = sizeof(wifi_channels);
                dwell_us = 50000;
                break;
            case OperatingMode::ZIGBEE_SURVEY:
                channelArray1 = zigbee_channels;
                channelArray3 = zigbee_channels;
                arraySize1 = sizeof(zigbee_channels);
                arraySize3 = sizeof(zigbee_channels);
                dwell_us = 100000;
                break;
            case OperatingMode::ALL_SURVEY:
            default:
                channelArray1 = all_channels;
                channelArray3 = all_channels;
                arraySize1 = sizeof(all_channels);
                arraySize3 = sizeof(all_channels);
                dwell_us = 200;
                break;
        }

        if (currentDwell_us > 0) {
            dwell_us = currentDwell_us;
        }

        uint8_t stagger1 = arraySize1 / 3;
        uint8_t stagger3 = arraySize3 / 3;

        uint8_t ch1 = channelArray1[hopIndex1 % arraySize1];
        uint8_t ch2 = channelArray1[(hopIndex1 + stagger1) % arraySize1];
        uint8_t ch3 = channelArray3[hopIndex3 % arraySize3];

        if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
            radio1.setChannel(ch1);
            radio2.setChannel(ch2);
            radio3.setChannel(ch3);
            radio1.startFastWrite(payload, payload_len, false, true);
            radio2.startFastWrite(payload, payload_len, false, true);
            radio3.startFastWrite(payload, payload_len, false, true);
            xSemaphoreGive(spiMutex);
        }

        hopIndex1++;
        hopIndex3++;
        hopCount++;

        if (dwell_us >= 1000) {
            vTaskDelay(pdMS_TO_TICKS(dwell_us / 1000));
        } else {
            delayMicroseconds(dwell_us);
        }
    }
}

uint8_t computeChecksum(const ProtocolHeader& hdr) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&hdr);
    uint8_t cs = 0;
    for (size_t i = 0; i < offsetof(ProtocolHeader, checksum); ++i) {
        cs ^= p[i];
    }
    return cs;
}

void fillHeader(ProtocolHeader& hdr, PacketType type, DeviceRole role, uint16_t seq, uint8_t payload_len) {
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = PROTOCOL_MAGIC;
    hdr.version = REACHER_PROTOCOL_VERSION;
    hdr.type = type;
    hdr.sender_role = role;
    memcpy(hdr.sender_id, local_mac, DEVICE_ID_LEN);
    hdr.sequence = seq;
    hdr.payload_len = payload_len;
    hdr.checksum = computeChecksum(hdr);
}

bool sendToDoombox(PacketType type, const void* payload, uint8_t payload_len) {
    if (!paired) return false;
    uint8_t buf[PROTOCOL_MAX_PAYLOAD + 64];
    ProtocolHeader hdr;
    fillHeader(hdr, type, DeviceRole::REACHER, sequence++, payload_len);
    memcpy(buf, &hdr, sizeof(hdr));
    if (payload && payload_len > 0) {
        memcpy(buf + sizeof(hdr), payload, payload_len);
    }
    return esp_now_send(doombox_mac, buf, sizeof(hdr) + payload_len) == ESP_OK;
}

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (len < sizeof(ProtocolHeader)) return;
    ProtocolPacket pkt;
    memcpy(&pkt, data, min(len, (int)sizeof(pkt)));
    if (pkt.header.magic != PROTOCOL_MAGIC) return;
    if (pkt.header.version == 0 || pkt.header.version > REACHER_PROTOCOL_VERSION + 2) return;
    if (pkt.header.checksum != computeChecksum(pkt.header)) return;

    switch (pkt.header.type) {
        case PacketType::DISCOVERY_BEACON:
            if (!paired) {
                memcpy(doombox_mac, mac, 6);
                PairingRequestPayload req = {};
                memcpy(req.reaper_id, local_mac, DEVICE_ID_LEN);
                req.fw_major = 4;
                req.fw_minor = 0;
                req.radio_count = 3;
                sendToDoombox(PacketType::PAIRING_REQUEST, &req, sizeof(req));
            }
            break;
        case PacketType::PAIRING_RESPONSE:
            if (!paired) {
                paired = true;
                memcpy(doombox_mac, mac, 6);
                digitalWrite(LED_PIN, HIGH);
            }
            break;
        case PacketType::HEARTBEAT:
            if (paired && memcmp(mac, doombox_mac, 6) == 0) {
                HeartbeatPayload hb = {};
                memcpy(hb.reaper_id, local_mac, DEVICE_ID_LEN);
                hb.uptime_ms = millis();
                hb.mode = static_cast<uint8_t>(currentMode);
                hb.radio_status = (radio1.isChipConnected() ? 0x01 : 0) |
                                  (radio2.isChipConnected() ? 0x02 : 0) |
                                  (radio3.isChipConnected() ? 0x04 : 0);
                hb.power_profile = static_cast<uint8_t>(currentProfile);
                hb.current_dwell_us = currentDwell_us;
                hb.hop_count = hopCount;
                sendToDoombox(PacketType::HEARTBEAT, &hb, sizeof(hb));
            }
            break;
        case PacketType::STATUS_REQUEST:
            if (paired && memcmp(mac, doombox_mac, 6) == 0) {
                StatusResponsePayload st = {};
                st.mode = static_cast<uint8_t>(currentMode);
                st.radio_status = (radio1.isChipConnected() ? 0x01 : 0) |
                                  (radio2.isChipConnected() ? 0x02 : 0) |
                                  (radio3.isChipConnected() ? 0x04 : 0);
                st.core_connected = paired ? 1 : 0;
                st.power_profile = static_cast<uint8_t>(currentProfile);
                st.current_dwell_us = currentDwell_us;
                st.hop_count = hopCount;
                st.free_heap = ESP.getFreeHeap();
                st.uptime_ms = millis();
                sendToDoombox(PacketType::STATUS_RESPONSE, &st, sizeof(st));
            }
            break;
        case PacketType::SCAN_TEST_COMMAND:
            if (paired && memcmp(mac, doombox_mac, 6) == 0) {
                ScanTestPayload scan;
                memcpy(&scan, pkt.payload, sizeof(scan));
                currentMode = static_cast<OperatingMode>(scan.survey_type);
                hopResetPending = true;
            }
            break;
        case PacketType::STOP_COMMAND:
            if (paired && memcmp(mac, doombox_mac, 6) == 0) {
                currentMode = OperatingMode::STOPPED;
                hopResetPending = true;
                digitalWrite(LED_PIN, LOW);
            }
            break;
        case PacketType::POWER_PROFILE_COMMAND:
            if (paired && memcmp(mac, doombox_mac, 6) == 0) {
                PowerProfilePayload pp;
                memcpy(&pp, pkt.payload, sizeof(pp));
                currentProfile = static_cast<PowerProfile>(pp.profile);
                profileChangePending = true;
            }
            break;
        case PacketType::DWELL_OVERRIDE_COMMAND:
            if (paired && memcmp(mac, doombox_mac, 6) == 0) {
                DwellOverridePayload dw;
                memcpy(&dw, pkt.payload, sizeof(dw));
                currentDwell_us = dw.dwell_us;
            }
            break;
    }
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    spiMutex = xSemaphoreCreateMutex();
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
        radio1.begin();
        radio2.begin();
        radio3.begin();
        configureRadio(radio1, RF24_PA_HIGH);
        configureRadio(radio2, RF24_PA_HIGH);
        configureRadio(radio3, RF24_PA_HIGH);
        xSemaphoreGive(spiMutex);
    }
    initPRBS();
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    delay(50);
    WiFi.macAddress(local_mac);
    Serial.printf("[REACHER] MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
        local_mac[0], local_mac[1], local_mac[2],
        local_mac[3], local_mac[4], local_mac[5]);
    if (esp_now_init() != ESP_OK) {
        Serial.println("[REACHER] ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
    uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, bcast, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    xTaskCreate(hopTask, "hopTask", 4096, nullptr, 1, &hopTaskHandle);
    Serial.println("[REACHER] Ready, waiting for doomBOX");
}

void loop() {
    if (millis() - lastRadioCheck > 100) {
        lastRadioCheck = millis();
        if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
            if (!radio1.isChipConnected()) {
                radio1_failCount++;
                if (radio1_failCount > 3) {
                    radio1.begin();
                    configureRadio(radio1, RF24_PA_HIGH);
                    radio1_failCount = 0;
                }
            } else {
                radio1_failCount = 0;
            }
            if (!radio2.isChipConnected()) {
                radio2_failCount++;
                if (radio2_failCount > 3) {
                    radio2.begin();
                    configureRadio(radio2, RF24_PA_HIGH);
                    radio2_failCount = 0;
                }
            } else {
                radio2_failCount = 0;
            }
            if (!radio3.isChipConnected()) {
                radio3_failCount++;
                if (radio3_failCount > 3) {
                    radio3.begin();
                    configureRadio(radio3, RF24_PA_HIGH);
                    radio3_failCount = 0;
                }
            } else {
                radio3_failCount = 0;
            }
            xSemaphoreGive(spiMutex);
        }
        if (currentProfile == PowerProfile::THERMAL_LIMIT) {
            if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
                applyPowerProfile();
                xSemaphoreGive(spiMutex);
            }
        }
    }
    delay(1);
}