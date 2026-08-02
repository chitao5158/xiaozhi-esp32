#include "ir_controller.h"

#include "mcp_server.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_common.h>
#include <driver/rmt_encoder.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <algorithm>
#include <cstring>
#include <vector>

#define TAG "IrCtrl"

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

IrController& IrController::GetInstance() {
    static IrController instance;
    return instance;
}

// ---------------------------------------------------------------------------
// Configuration — only stores GPIOs, does NOT touch RMT (lazy).
// ---------------------------------------------------------------------------

void IrController::Configure(gpio_num_t rx_gpio, gpio_num_t tx_gpio) {
    rx_gpio_ = rx_gpio;
    tx_gpio_ = tx_gpio;

    if (!tools_registered_) {
        RegisterMcpTools();
        tools_registered_ = true;
    }
    ESP_LOGI(TAG, "Configured: RX=GPIO%d TX=GPIO12 (lazy RMT)",
             (int)rx_gpio_, (int)tx_gpio_);
}

// ---------------------------------------------------------------------------
// Lazy RMT channel acquisition. Released as soon as the operation finishes
// so led_strip and other consumers can claim the channel between calls.
// ---------------------------------------------------------------------------

bool IrController::AcquireRx() {
    if (rx_handle_ != nullptr) return true;
    if (rx_gpio_ == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "RX GPIO not configured");
        return false;
    }

    if (rx_buffer_ == nullptr) {
        rx_buffer_ = heap_caps_malloc(kMaxSymbols * sizeof(rmt_symbol_word_t),
                                      MALLOC_CAP_INTERNAL);
        if (rx_buffer_ == nullptr) {
            ESP_LOGE(TAG, "RX buffer alloc failed");
            return false;
        }
    }

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num          = rx_gpio_,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = kRmtResolutionHz,
        .mem_block_symbols = kMaxSymbols,
        .intr_priority     = 0,
    };
    if (rmt_new_rx_channel(&rx_cfg, reinterpret_cast<rmt_channel_handle_t*>(&rx_handle_)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_rx_channel failed (channel pool exhausted?)");
        rx_handle_ = nullptr;
        return false;
    }
    ESP_LOGI(TAG, "RX channel acquired");
    return true;
}

bool IrController::AcquireTx() {
    if (tx_handle_ != nullptr && copy_encoder_ != nullptr) return true;
    if (tx_gpio_ == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "TX GPIO not configured");
        return false;
    }

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num          = tx_gpio_,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = kRmtResolutionHz,
        .mem_block_symbols = kMaxSymbols,
        .trans_queue_depth = 4,
        .intr_priority     = 0,
    };
    if (rmt_new_tx_channel(&tx_cfg, reinterpret_cast<rmt_channel_handle_t*>(&tx_handle_)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed");
        tx_handle_ = nullptr;
        return false;
    }
    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = kCarrierHz,
        .duty_cycle   = 0.33f,
        .flags = {
            .polarity_active_low = false,
        },
    };
    if (rmt_apply_carrier(reinterpret_cast<rmt_channel_handle_t>(tx_handle_),
                          &carrier_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_apply_carrier failed");
    }
    rmt_copy_encoder_config_t copy_cfg = {};
    if (rmt_new_copy_encoder(&copy_cfg,
                             reinterpret_cast<rmt_encoder_handle_t*>(&copy_encoder_)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder failed");
        ReleaseTx();
        return false;
    }
    ESP_LOGI(TAG, "TX channel acquired");
    return true;
}

void IrController::ReleaseRx() {
    if (rx_handle_ != nullptr) {
        rmt_del_channel(reinterpret_cast<rmt_channel_handle_t>(rx_handle_));
        rx_handle_ = nullptr;
        ESP_LOGI(TAG, "RX channel released");
    }
}

void IrController::ReleaseTx() {
    if (copy_encoder_ != nullptr) {
        rmt_del_encoder(reinterpret_cast<rmt_encoder_handle_t>(copy_encoder_));
        copy_encoder_ = nullptr;
    }
    if (tx_handle_ != nullptr) {
        rmt_del_channel(reinterpret_cast<rmt_channel_handle_t>(tx_handle_));
        tx_handle_ = nullptr;
        ESP_LOGI(TAG, "TX channel released");
    }
}

// ---------------------------------------------------------------------------
// Learn (RX) — acquires RX channel, blocks up to timeout_ms, releases.
// ---------------------------------------------------------------------------

bool IrController::Learn(const std::string& name, uint32_t timeout_ms) {
    if (name.empty() || name.size() > kMaxNameLen) {
        ESP_LOGE(TAG, "Name must be 1..%zu chars", kMaxNameLen);
        return false;
    }
    bool expected = false;
    if (!learning_.compare_exchange_strong(expected, true)) {
        ESP_LOGW(TAG, "Learn already in progress");
        return false;
    }
    auto restore = [&]() { learning_.store(false); };

    if (!AcquireRx()) {
        restore();
        return false;
    }

    rmt_receive_config_t recv_cfg = {
        .signal_range_min_ns = 1000000,
        .signal_range_max_ns = 20000000,
    };
    esp_err_t err = rmt_receive(reinterpret_cast<rmt_channel_handle_t>(rx_handle_),
                                rx_buffer_, kMaxSymbols * sizeof(rmt_symbol_word_t),
                                &recv_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_receive arm failed: %s", esp_err_to_name(err));
        ReleaseRx();
        restore();
        return false;
    }

    ESP_LOGI(TAG, "Learning '%s' — point your remote at the receiver now "
                  "(timeout %lu ms)", name.c_str(), (unsigned long)timeout_ms);

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    const TickType_t head_start = xTaskGetTickCount() + pdMS_TO_TICKS(200);
    bool got_signal = false;
    while (xTaskGetTickCount() < deadline) {
        if (xTaskGetTickCount() >= head_start) {
            auto* syms = reinterpret_cast<rmt_symbol_word_t*>(rx_buffer_);
            if (syms[0].val != 0) {
                got_signal = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ReleaseRx();
    restore();

    if (!got_signal) {
        ESP_LOGW(TAG, "No signal captured within %lu ms", (unsigned long)timeout_ms);
        return false;
    }

    auto* syms = reinterpret_cast<rmt_symbol_word_t*>(rx_buffer_);
    size_t count = 0;
    for (size_t i = 0; i < kMaxSymbols; i++) {
        if (syms[i].val == 0) break;
        count++;
    }
    if (count == 0) {
        ESP_LOGW(TAG, "Empty capture");
        return false;
    }
    ESP_LOGI(TAG, "Captured %zu symbols", count);

    std::vector<uint8_t> blob(sizeof(uint16_t) + count * sizeof(rmt_symbol_word_t));
    uint16_t count_le = (uint16_t)count;
    memcpy(blob.data(), &count_le, sizeof(count_le));
    memcpy(blob.data() + sizeof(count_le), syms, count * sizeof(rmt_symbol_word_t));

    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return false;
    }
    esp_err_t set_err = nvs_set_blob(nvs, name.c_str(), blob.data(), blob.size());
    esp_err_t commit_err = (set_err == ESP_OK) ? nvs_commit(nvs) : ESP_OK;
    nvs_close(nvs);
    if (set_err != ESP_OK || commit_err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob/commit failed");
        return false;
    }
    ESP_LOGI(TAG, "Stored '%s' (%zu bytes)", name.c_str(), blob.size());
    return true;
}

// ---------------------------------------------------------------------------
// Send (TX) — acquires TX channel, plays, releases.
// ---------------------------------------------------------------------------

bool IrController::Send(const std::string& name, uint32_t repeats) {
    if (repeats == 0) repeats = 1;

    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return false;
    }
    size_t blob_size = 0;
    if (nvs_get_blob(nvs, name.c_str(), nullptr, &blob_size) != ESP_OK || blob_size == 0) {
        ESP_LOGW(TAG, "Signal '%s' not found", name.c_str());
        nvs_close(nvs);
        return false;
    }
    std::vector<uint8_t> blob(blob_size);
    if (nvs_get_blob(nvs, name.c_str(), blob.data(), &blob_size) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob failed");
        nvs_close(nvs);
        return false;
    }
    nvs_close(nvs);

    if (blob.size() < sizeof(uint16_t)) {
        ESP_LOGE(TAG, "Blob too small");
        return false;
    }
    uint16_t count;
    memcpy(&count, blob.data(), sizeof(count));
    if (sizeof(uint16_t) + count * sizeof(rmt_symbol_word_t) > blob.size() || count == 0) {
        ESP_LOGE(TAG, "Blob count mismatch");
        return false;
    }
    const rmt_symbol_word_t* syms =
        reinterpret_cast<const rmt_symbol_word_t*>(blob.data() + sizeof(uint16_t));

    if (!AcquireTx()) {
        return false;
    }

    ESP_LOGI(TAG, "Sending '%s' (%u symbols, x%lu)",
             name.c_str(), (unsigned)count, (unsigned long)repeats);

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = false,
        },
    };
    auto* tx = reinterpret_cast<rmt_channel_handle_t>(tx_handle_);
    auto* enc = reinterpret_cast<rmt_encoder_handle_t>(copy_encoder_);
    bool ok = true;
    for (uint32_t i = 0; i < repeats; i++) {
        if (rmt_transmit(tx, enc, syms, count * sizeof(rmt_symbol_word_t), &tx_cfg) != ESP_OK) {
            ESP_LOGE(TAG, "rmt_transmit failed on repeat %lu", (unsigned long)i);
            ok = false;
            break;
        }
        if (rmt_tx_wait_all_done(tx, pdMS_TO_TICKS(200)) != ESP_OK) {
            ESP_LOGW(TAG, "rmt_tx_wait_all_done timed out on repeat %lu", (unsigned long)i);
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    ReleaseTx();
    return ok;
}

// ---------------------------------------------------------------------------
// NVS bookkeeping
// ---------------------------------------------------------------------------

bool IrController::Delete(const std::string& name) {
    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &nvs) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_erase_key(nvs, name.c_str());
    esp_err_t commit_err = (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND)
                           ? nvs_commit(nvs) : err;
    nvs_close(nvs);
    return err == ESP_OK;
}

std::vector<std::string> IrController::List() const {
    std::vector<std::string> out;
    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &nvs) != ESP_OK) {
        return out;
    }
    nvs_iterator_t it = nullptr;
    esp_err_t err = nvs_entry_find(kNvsNamespace, nullptr, NVS_TYPE_BLOB, &it);
    while (err == ESP_OK) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        out.emplace_back(info.key);
        err = nvs_entry_next(&it);
    }
    if (it != nullptr) nvs_release_iterator(it);
    nvs_close(nvs);
    std::sort(out.begin(), out.end());
    return out;
}

size_t IrController::Count() const {
    return List().size();
}

// ---------------------------------------------------------------------------
// MCP tools
// ---------------------------------------------------------------------------

void IrController::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool(
        "self.ir.learn",
        "Capture an IR remote signal into NVS. Point the original remote at "
        "the IR receiver and press the desired button within the timeout. "
        "Signal name must be 1..15 characters.",
        PropertyList({
            Property("name", kPropertyTypeString),
            Property("timeout_ms", kPropertyTypeInteger, 1000, 15000),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            std::string name = properties["name"].value<std::string>();
            int timeout = properties["timeout_ms"].value<int>();
            return Learn(name, static_cast<uint32_t>(timeout));
        });

    mcp.AddTool(
        "self.ir.send",
        "Replay a previously learned IR signal. repeats >= 1; default 2 to "
        "match the typical AC remote convention of double-pressing each "
        "command.",
        PropertyList({
            Property("name", kPropertyTypeString),
            Property("repeats", kPropertyTypeInteger, 1, 5),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            std::string name = properties["name"].value<std::string>();
            int repeats = properties["repeats"].value<int>();
            return Send(name, static_cast<uint32_t>(repeats));
        });

    mcp.AddTool(
        "self.ir.list",
        "List all stored IR signal names.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            auto names = List();
            std::string out = "[";
            for (size_t i = 0; i < names.size(); i++) {
                if (i) out += ",";
                out += "\"" + names[i] + "\"";
            }
            out += "]";
            return out;
        });

    mcp.AddTool(
        "self.ir.delete",
        "Remove a stored IR signal. Returns true if it existed.",
        PropertyList({Property("name", kPropertyTypeString)}),
        [this](const PropertyList& properties) -> ReturnValue {
            std::string name = properties["name"].value<std::string>();
            return Delete(name);
        });
}
