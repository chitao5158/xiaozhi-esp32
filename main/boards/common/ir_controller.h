#ifndef __IR_CONTROLLER_H__
#define __IR_CONTROLLER_H__

#include <driver/gpio.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

// IR remote learning + playback via a YS-IRTM (or compatible) UART bridge.
//
// The YS-IRTM board carries its own 38 kHz receiver + transmitter. ESP32
// only talks UART2 to it:
//   - Learn:  YKR-T/091 → YS-IRTM RX → ESP32 captures 4-byte NEC frame
//   - Send:   ESP32 → ESP32 UART2 → YS-IRTM TX → 38 kHz IR emission
//
// The on-board RMT receiver (HS0038) and the KY-005 emitter are no
// longer used — both are wired to GPIO_NUM_NC.
//
// NEC frame layout (4 bytes, stored in NVS):
//   byte 0: address
//   byte 1: address inverse
//   byte 2: command
//   byte 3: command inverse
class IrController {
public:
    static IrController& GetInstance();

    // Configure UART GPIOs and register MCP tools. Must be called once at
    // board init. Idempotent.
    void Configure(gpio_num_t uart_tx_gpio, gpio_num_t uart_rx_gpio);

    // Wait up to `timeout_ms` for the YS-IRTM to report a frame received
    // from any remote. Stores the 4-byte NEC payload in NVS under `name`
    // (max 15 chars). Returns true on success.
    bool Learn(const std::string& name, uint32_t timeout_ms = 5000);

    // Replay a previously learned signal. repeats >= 1; default 2 matches
    // the typical AC remote convention.
    bool Send(const std::string& name, uint32_t repeats = 2);

    // Delete a single signal. Returns true if it existed.
    bool Delete(const std::string& name);

    // List all learned signal names.
    std::vector<std::string> List() const;

    // Count of stored signals.
    size_t Count() const;

    // Hardware self-test: send a synthetic NEC frame via UART and report
    // whether the module accepted it. Returns a short summary string.
    std::string Test(uint32_t timeout_ms = 3000);

private:
    IrController() = default;
    IrController(const IrController&) = delete;
    IrController& operator=(const IrController&) = delete;

    void RegisterMcpTools();

    gpio_num_t uart_tx_gpio_ = GPIO_NUM_NC;
    gpio_num_t uart_rx_gpio_ = GPIO_NUM_NC;

    std::atomic<bool> learning_{false};
    bool tools_registered_ = false;

    // Maximum length of a stored signal name (NVS key limit).
    static constexpr size_t  kMaxNameLen       = 15;
    // NVS namespace used for all IR signals.
    static constexpr char   kNvsNamespace[]   = "ir";
    // NEC frame is exactly 4 bytes.
    static constexpr size_t  kNecFrameBytes   = 4;
};

#endif // __IR_CONTROLLER_H__
