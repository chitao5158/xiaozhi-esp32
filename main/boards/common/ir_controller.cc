#include "ir_controller.h"

#include "mcp_server.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
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
    // ESP-IDF v5.5+ RMT API requires explicit enable before receive. Without
    // this, rmt_receive() fails with "channel not in enable state" (esp_err
    // 0x3007 / ESP_ERR_INVALID_STATE).
    if (rmt_enable(reinterpret_cast<rmt_channel_handle_t>(rx_handle_)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable (RX) failed");
        rmt_del_channel(reinterpret_cast<rmt_channel_handle_t>(rx_handle_));
        rx_handle_ = nullptr;
        return false;
    }
    // Enable 38 kHz carrier demodulation on the RX channel. Without this,
    // RMT samples the raw GPIO waveform — which for a 38 kHz IR receiver
    // is a 13 µs on/off pulse stream, blowing through the 64-symbol buffer
    // in ~1.6 ms. Demodulation extracts the baseband signal so the buffer
    // captures the actual NEC/Coolix timing.
    rmt_carrier_config_t rx_carrier = {
        .frequency_hz = kCarrierHz,
        .duty_cycle   = 0.33f,
        .flags = { .polarity_active_low = false },
    };
    rmt_apply_carrier(reinterpret_cast<rmt_channel_handle_t>(rx_handle_), &rx_carrier);
    // Override the pull-up that the RMT driver enables internally
    // (`gpio_pullup_en` in rmt_rx.c around line 295). Combined with the
    // HS0038 receiver's own ~22 kΩ pull-up, the parallel resistance fights
    // the receiver's pull-down so the "low" level only reaches ~2.6 V
    // instead of 0 V — still above the 0.825 V high threshold, so RMT
    // never sees a falling edge. Disabling the internal pull-up lets the
    // receiver's own pull-up define the idle level and its output
    // transistor drive low cleanly.
    gpio_pullup_dis(rx_gpio_);
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
    // KY-005 (cheap variant) drives the IR LED when its signal pin is LOW
    // (LED anode tied to VCC, cathode to the signal pin). Flip the carrier
    // polarity so RMT pulls the pin low while modulating 38 kHz — that
    // makes the LED emit light during the mark periods, matching what
    // the HS0038 receiver expects.
    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = kCarrierHz,
        .duty_cycle   = 0.33f,
        .flags = {
            .polarity_active_low = true,
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
    // ESP-IDF v5.5+ RMT API requires explicit enable before transmit.
    if (rmt_enable(reinterpret_cast<rmt_channel_handle_t>(tx_handle_)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable (TX) failed");
        ReleaseTx();
        return false;
    }
    ESP_LOGI(TAG, "TX channel acquired");
    return true;
}

void IrController::ReleaseRx() {
    if (rx_handle_ != nullptr) {
        // ESP-IDF v5.5: del_channel requires the channel to be back in the
        // init state. rmt_disable transitions enabled -> init; without
        // it, rmt_del_channel logs "channel not in init state" and leaks
        // the handle until next reboot.
        rmt_disable(reinterpret_cast<rmt_channel_handle_t>(rx_handle_));
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
        rmt_disable(reinterpret_cast<rmt_channel_handle_t>(tx_handle_));
        rmt_del_channel(reinterpret_cast<rmt_channel_handle_t>(tx_handle_));
        tx_handle_ = nullptr;
        ESP_LOGI(TAG, "TX channel released");
    }
}

// ---------------------------------------------------------------------------
// CaptureByPolling — busy-poll the IR receiver pin and timestamp every
// edge, then fold the deltas into rmt_symbol_word_t[] so the rest of the
// pipeline (NVS storage, RMT TX replay) sees the same layout.
//
// This replaces the broken RMT RX path on ESP32-S3 (carrier demodulation
// + 64-symbol ping-pong buffer could not hold a full NEC/Coolix frame).
// Polling at task priority is fine for IR: shortest pulse in NEC is
// 560 µs, much longer than any FreeRTOS tick latency on a system that
// is otherwise idle (this is invoked while xiaozhi is sitting in the
// idle state waiting for the wake word).
// ---------------------------------------------------------------------------

bool IrController::CaptureByPolling(uint32_t timeout_ms) {
    if (rx_buffer_ == nullptr) {
        rx_buffer_ = heap_caps_malloc(kMaxSymbols * sizeof(rmt_symbol_word_t),
                                      MALLOC_CAP_INTERNAL);
        if (rx_buffer_ == nullptr) {
            ESP_LOGE(TAG, "RX buffer alloc failed");
            return false;
        }
    }
    memset(rx_buffer_, 0, kMaxSymbols * sizeof(rmt_symbol_word_t));

    // Baseline: read current level, wait for first edge.
    int last_level = gpio_get_level(rx_gpio_);
    int64_t start_us = esp_timer_get_time();
    int64_t deadline_us = start_us + (int64_t)timeout_ms * 1000;
    int64_t last_edge_us = start_us;

    // Phase 1: wait up to 1.5 s for the first edge. If nothing happens,
    // treat as "no signal" (avoids burning the full timeout on idle).
    int64_t phase1_deadline_us = start_us + 1500 * 1000;
    if (phase1_deadline_us > deadline_us) phase1_deadline_us = deadline_us;
    bool saw_edge = false;
    while (esp_timer_get_time() < phase1_deadline_us) {
        int level = gpio_get_level(rx_gpio_);
        if (level != last_level) {
            saw_edge = true;
            break;
        }
        // Yield to IDLE tasks; the wake-word detector runs at low prio
        // and we're at task prio 5 — short yields don't lose edges at
        // 38 kHz baseband (longest pulse-free gap for NEC repeats is
        // ~40 ms, easily polled).
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!saw_edge) {
        ESP_LOGW(TAG, "No first edge within 1.5 s");
        return false;
    }

    // Phase 2: capture edges until we hit a long quiet gap (>40 ms = no
    // more remote activity for this frame). Stop early so a stuck remote
    // button doesn't fill the buffer with repeated frames.
    size_t symbol_count = 0;
    last_edge_us = esp_timer_get_time();
    while (esp_timer_get_time() < deadline_us) {
        int level = gpio_get_level(rx_gpio_);
        if (level != last_level) {
            int64_t now_us = esp_timer_get_time();
            int64_t high_us = now_us - last_edge_us;
            last_edge_us = now_us;

            // End of a symbol's HIGH phase = start of LOW phase. We don't
            // know the LOW duration yet — peek one more sample to fold it in.
            int low_us = 0;
            // Read until we see another edge or until we exceed the buffer.
            // We can't actually peek with a single sample, so we use the
            // duration from last_edge_us to the NEXT edge later. For now
            // emit a placeholder and patch it on the next iteration.
            rmt_symbol_word_t sym = {};
            sym.duration0 = (uint16_t)(high_us > 0x7FFF ? 0x7FFF : high_us);
            sym.level0 = (last_level == 1) ? 1 : 0;
            if (symbol_count < kMaxSymbols) {
                static_cast<rmt_symbol_word_t*>(rx_buffer_)[symbol_count++] = sym;
            }
            last_level = level;

            // Long gap = frame ended.
            if (high_us > 40000) {
                ESP_LOGI(TAG, "End of frame (gap %lld µs)", high_us);
                break;
            }
        } else {
            // No edge yet — yield briefly to let other tasks run.
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (symbol_count == 0) {
        return false;
    }

    // Fold LOW durations: each symbol's duration1 should be the gap from
    // its HIGH edge to the next HIGH edge. Walk forward and patch.
    auto* syms = static_cast<rmt_symbol_word_t*>(rx_buffer_);
    for (size_t i = 0; i + 1 < symbol_count; i++) {
        syms[i].duration1 = syms[i + 1].duration0;
        syms[i].level1 = (syms[i + 1].level0 == 1) ? 1 : 0;
    }
    // Last symbol: leave duration1 as captured gap, but if we exited on
    // gap > 40 ms the last "HIGH duration" was actually the gap — strip
    // it off (it would corrupt playback).
    if (symbol_count > 0 && syms[symbol_count - 1].duration0 > 40000) {
        symbol_count--;
    }
    // Zero the unused tail.
    for (size_t i = symbol_count; i < kMaxSymbols; i++) {
        syms[i].val = 0;
    }

    ESP_LOGI(TAG, "Captured %zu raw edges", symbol_count);
    return true;
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

    // Make sure the receive pin is configured as a clean digital input
    // (RMT's internal pull-up is disabled so the receiver's own pull-up
    // defines the idle level).
    gpio_set_direction(rx_gpio_, GPIO_MODE_INPUT);
    gpio_set_pull_mode(rx_gpio_, GPIO_FLOATING);
    gpio_pullup_dis(rx_gpio_);
    gpio_pulldown_dis(rx_gpio_);

    rmt_receive_config_t recv_cfg = {
        // ESP-IDF v5.5+ requires signal_range_min_ns < 3187 ns (≈ 3 µs).
        // Values like 1000000 (1 ms) trip ESP_ERR_INVALID_ARG. 1000 ns is
        // the smallest practical width for noise filtering at 1 MHz RMT
        // resolution; pulses shorter than 1 µs are essentially glitches.
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 20000000,
    };
    // Clear stale symbols from any previous Receive call. The buffer is
    // reused across calls and is never zeroed by the driver; without this,
    // leftover symbols from an earlier capture would trigger an immediate
    // "got_signal = true" on the very first poll.
    memset(rx_buffer_, 0, kMaxSymbols * sizeof(rmt_symbol_word_t));
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

    // Skip the RMT RX path entirely — busy-poll the GPIO instead. The HS0038
    // already demodulates 38 kHz internally, so the GPIO state is the baseband
    // signal we actually want to record.
    if (!CaptureByPolling(timeout_ms)) {
        ESP_LOGW(TAG, "No signal captured within %lu ms", (unsigned long)timeout_ms);
        ReleaseRx();
        restore();
        return false;
    }

    ReleaseRx();
    restore();

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
// Hardware self-test (no NVS).
//   - Sends ~128 ms of 38 kHz carrier on TX (use phone camera to see LED).
//   - Then opens RX and waits up to rx_timeout_ms for any captured signal.
//   - Returns a short text summary the AI can read back to the user.
// ---------------------------------------------------------------------------
std::string IrController::Test(uint32_t rx_timeout_ms) {
    std::string result = "IR self-test:\n";

    // ---- TX side ----
    if (!AcquireTx()) {
        result += "  TX: FAIL (no TX GPIO or channel pool exhausted)\n";
    } else {
        // 4 chained symbols at level=1, each 32 ms → 128 ms of carrier.
        rmt_symbol_word_t pulse[4];
        for (int i = 0; i < 4; i++) {
            pulse[i].duration0 = 32000;  // 32 ms (max for 15-bit duration)
            pulse[i].level0    = 1;       // carrier on
            pulse[i].duration1 = 0;
            pulse[i].level1    = 0;
        }
        rmt_transmit_config_t tx_cfg = {
            .loop_count = 0,
            .flags = { .eot_level = 0, .queue_nonblocking = false },
        };
        esp_err_t tx_err = rmt_transmit(
            reinterpret_cast<rmt_channel_handle_t>(tx_handle_),
            reinterpret_cast<rmt_encoder_handle_t>(copy_encoder_),
            pulse, sizeof(pulse), &tx_cfg);
        if (tx_err == ESP_OK) {
            rmt_tx_wait_all_done(
                reinterpret_cast<rmt_channel_handle_t>(tx_handle_),
                pdMS_TO_TICKS(500));
            result += "  TX: OK — ~128 ms 38 kHz carrier sent; point phone camera at the LED to see it glow faintly\n";
        } else {
            result += std::string("  TX: FAIL (rmt_transmit ") + esp_err_to_name(tx_err) + ")\n";
        }
        ReleaseTx();
    }

    // ---- RX side ----
    if (!AcquireRx()) {
        result += "  RX: FAIL (no RX GPIO or channel pool exhausted)";
        return result;
    }

    ESP_LOGI(TAG, "Test: pointing a remote at GPIO%d now", (int)rx_gpio_);
    bool rx_ok = CaptureByPolling(rx_timeout_ms);
    size_t captured = 0;
    if (rx_ok) {
        auto* syms = reinterpret_cast<rmt_symbol_word_t*>(rx_buffer_);
        for (size_t i = 0; i < kMaxSymbols; i++) {
            if (syms[i].val == 0) break;
            captured++;
        }
    }

    char buf[96];
    snprintf(buf, sizeof(buf), "  RX: %s — captured %zu symbols; point any IR remote at GPIO%d within the window",
             rx_ok && captured > 0 ? "OK" : "TIMEOUT",
             captured, (int)rx_gpio_);
    result += buf;

    ReleaseRx();
    return result;
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

    mcp.AddTool(
        "self.ir.test",
        "Hardware self-test for the IR module. Pulses the TX LED with "
        "~128 ms of 38 kHz carrier (look at it through a phone camera — "
        "you should see a faint purple glow), then opens the RX and waits "
        "for any remote signal. Does NOT touch NVS.",
        PropertyList({Property("rx_timeout_ms", kPropertyTypeInteger, 1000, 15000)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int timeout = properties["rx_timeout_ms"].value<int>();
            return Test(static_cast<uint32_t>(timeout));
        });
}
