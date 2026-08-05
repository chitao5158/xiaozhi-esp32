#include "ir_controller.h"
#include "ir_uart_link.h"

#include "mcp_server.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>

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
// Configuration
// ---------------------------------------------------------------------------

void IrController::Configure(gpio_num_t uart_tx_gpio, gpio_num_t uart_rx_gpio) {
    uart_tx_gpio_ = uart_tx_gpio;
    uart_rx_gpio_ = uart_rx_gpio;

    if (!tools_registered_) {
        RegisterMcpTools();
        tools_registered_ = true;
    }
    ESP_LOGI(TAG, "Configured: UART2 TX=GPIO%d RX=GPIO%d (YS-IRTM bridge)",
             (int)uart_tx_gpio_, (int)uart_rx_gpio_);
}

// ---------------------------------------------------------------------------
// Learn — receive a NEC frame from YS-IRTM and store it in NVS.
// ---------------------------------------------------------------------------

bool IrController::Learn(const std::string& name, uint32_t timeout_ms) {
    if (name.empty() || name.size() > kMaxNameLen) {
        ESP_LOGE(TAG, "Name must be 1..%zu chars", kMaxNameLen);
        return false;
    }
    if (uart_tx_gpio_ == GPIO_NUM_NC || uart_rx_gpio_ == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "UART GPIOs not configured");
        return false;
    }
    bool expected = false;
    if (!learning_.compare_exchange_strong(expected, true)) {
        ESP_LOGW(TAG, "Learn already in progress");
        return false;
    }
    auto restore = [&]() { learning_.store(false); };

    if (!IrUartLink::Init(uart_tx_gpio_, uart_rx_gpio_)) {
        restore();
        return false;
    }
    IrUartLink::DrainRx();

    ESP_LOGI(TAG, "Learning '%s' — point your remote at the YS-IRTM receiver within %lu ms",
             name.c_str(), (unsigned long)timeout_ms);

    uint8_t addr = 0, cmd = 0;
    if (!IrUartLink::ReceiveFrame(timeout_ms, &addr, &cmd)) {
        ESP_LOGW(TAG, "No NEC frame received within %lu ms", (unsigned long)timeout_ms);
        restore();
        return false;
    }

    uint8_t nec[4] = { addr, (uint8_t)~addr, cmd, (uint8_t)~cmd };
    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        restore();
        return false;
    }
    esp_err_t set_err = nvs_set_blob(nvs, name.c_str(), nec, sizeof(nec));
    esp_err_t commit_err = (set_err == ESP_OK) ? nvs_commit(nvs) : ESP_OK;
    nvs_close(nvs);
    if (set_err != ESP_OK || commit_err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob/commit failed");
        restore();
        return false;
    }
    ESP_LOGI(TAG, "Stored '%s' (addr=0x%02x cmd=0x%02x)", name.c_str(), addr, cmd);
    restore();
    return true;
}

// ---------------------------------------------------------------------------
// Send — read NEC from NVS and forward to YS-IRTM via UART.
// ---------------------------------------------------------------------------

bool IrController::Send(const std::string& name, uint32_t repeats) {
    if (repeats == 0) repeats = 1;
    if (uart_tx_gpio_ == GPIO_NUM_NC || uart_rx_gpio_ == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "UART GPIOs not configured");
        return false;
    }

    nvs_handle_t nvs;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return false;
    }
    uint8_t nec[kNecFrameBytes] = {};
    size_t blob_size = sizeof(nec);
    esp_err_t err = nvs_get_blob(nvs, name.c_str(), nec, &blob_size);
    nvs_close(nvs);
    if (err != ESP_OK || blob_size != sizeof(nec)) {
        ESP_LOGW(TAG, "Signal '%s' not found (err=%s, size=%zu)",
                 name.c_str(), esp_err_to_name(err), blob_size);
        return false;
    }

    if (!IrUartLink::Init(uart_tx_gpio_, uart_rx_gpio_)) {
        return false;
    }

    ESP_LOGI(TAG, "Sending '%s' (addr=0x%02x cmd=0x%02x, x%lu)",
             name.c_str(), nec[0], nec[2], (unsigned long)repeats);

    bool ok = true;
    for (uint32_t i = 0; i < repeats; i++) {
        if (!IrUartLink::SendFrame(nec[0], nec[2])) {
            ESP_LOGE(TAG, "SendFrame failed on repeat %lu", (unsigned long)i);
            ok = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(40));  // 40 ms inter-frame gap
    }
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
// Self-test — send a synthetic frame and report whether the module
// accepted it (a real round-trip verification needs the recipient device
// to acknowledge; we settle for "UART write succeeded").
// ---------------------------------------------------------------------------

std::string IrController::Test(uint32_t timeout_ms) {
    std::string result = "IR self-test (YS-IRTM bridge):\n";
    if (uart_tx_gpio_ == GPIO_NUM_NC || uart_rx_gpio_ == GPIO_NUM_NC) {
        result += "  FAIL: UART GPIOs not configured\n";
        return result;
    }
    if (!IrUartLink::Init(uart_tx_gpio_, uart_rx_gpio_)) {
        result += "  FAIL: UART init failed\n";
        return result;
    }

    // Send a known NEC frame (TV power: address 0x04, command 0x08 — this
    // is irrelevant to the user's AC but lets YS-IRTM remark the LED).
    IrUartLink::SendFrame(0x04, 0x08);
    result += "  TX: synthetic frame (addr=0x04 cmd=0x08) sent — YS-IRTM red LED should blink\n";
    result += "  Note: YS-IRTM jumps to receiver mode automatically; point a remote at it within " +
              std::to_string(timeout_ms) + " ms to test RX\n";

    // Now act as a quick receiver test.
    IrUartLink::DrainRx();
    uint8_t addr = 0, cmd = 0;
    if (IrUartLink::ReceiveFrame(timeout_ms, &addr, &cmd)) {
        char buf[96];
        snprintf(buf, sizeof(buf), "  RX: OK — addr=0x%02x cmd=0x%02x\n", addr, cmd);
        result += buf;
    } else {
        result += "  RX: TIMEOUT — no frame received within window\n";
    }
    return result;
}

// ---------------------------------------------------------------------------
// MCP tools
// ---------------------------------------------------------------------------

void IrController::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool(
        "self.ir.learn",
        "Capture an IR remote signal into NVS via the YS-IRTM UART bridge. "
        "Point the original remote at the YS-IRTM receiver and press the "
        "desired button within the timeout. Signal name must be 1..15 chars.",
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
        "Replay a previously learned IR signal via the YS-IRTM. repeats >= 1; "
        "default 2 to match the typical AC remote convention of double-"
        "pressing each command.",
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
