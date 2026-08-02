#ifndef __IR_CONTROLLER_H__
#define __IR_CONTROLLER_H__

#include <driver/gpio.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

// Generic IR remote learning + playback.
//
// Captures raw RMT symbol timing (NOT protocol-decoded) from a 38 kHz
// receiver (HS0038 / VS1838B), stores it in NVS under a user-supplied
// name, and replays it via a 38 kHz-modulated TX GPIO on demand. Works
// for any protocol — NEC, RC5, AC proprietary — as long as the user
// has the original remote to teach each button once.
//
// Pins:
//   - rx_gpio: IR receiver OUT pin (e.g. GPIO 2)
//   - tx_gpio: IR LED anode via ~33-100 Ω resistor (e.g. GPIO 12)
//
// Pass GPIO_NUM_NC to either pin to disable that direction. The
// corresponding tool (learn / send) will report an error instead of
// crashing.
class IrController {
public:
    static IrController& GetInstance();

    // Configure GPIOs and register MCP tools. Must be called once at
    // board init. Idempotent. Either GPIO may be GPIO_NUM_NC.
    void Configure(gpio_num_t rx_gpio, gpio_num_t tx_gpio);

    // Learn a signal for up to `timeout_ms` ms (default 5000). Stores
    // the raw RMT symbol stream under `name` (max 15 chars per NVS
    // limit). Returns true if a non-empty capture was recorded.
    bool Learn(const std::string& name, uint32_t timeout_ms = 5000);

    // Send a previously learned signal. repeats >= 1; default 2 matches
    // the typical AC remote convention (sends the same frame twice with
    // a short gap so the appliance picks it up reliably).
    bool Send(const std::string& name, uint32_t repeats = 2);

    // Hardware self-test: pulses the TX GPIO at 38 kHz for ~500 ms (use
    // a phone camera to confirm the IR LED is wired and working), then
    // opens the RX channel and reports how many symbols it captures within
    // `timeout_ms` (point any remote at the receiver). Does NOT touch NVS.
    // Returns a short human-readable summary string.
    std::string Test(uint32_t rx_timeout_ms = 3000);

    // Delete a single signal. Returns true if it existed.
    bool Delete(const std::string& name);

    // List all learned signal names (sorted, no particular order).
    std::vector<std::string> List() const;

    // Count of stored signals.
    size_t Count() const;

    bool HasRx() const { return rx_handle_ != nullptr; }
    bool HasTx() const { return tx_handle_ != nullptr; }

private:
    IrController() = default;
    IrController(const IrController&) = delete;
    IrController& operator=(const IrController&) = delete;

    // Lazy RMT init — claim the channel only when Learn/Send is called and
    // release it immediately after. Avoids colliding with led_strip and
    // other consumers that hold RMT channels for the lifetime of the app.
    bool AcquireRx();
    bool AcquireTx();
    void ReleaseRx();
    void ReleaseTx();

    void RegisterMcpTools();

    gpio_num_t rx_gpio_ = GPIO_NUM_NC;
    gpio_num_t tx_gpio_ = GPIO_NUM_NC;

    // Opaque RMT handles, kept as void* to avoid leaking the headers here.
    void* rx_handle_ = nullptr;     // rmt_channel_handle_t
    void* tx_handle_ = nullptr;     // rmt_channel_handle_t
    void* copy_encoder_ = nullptr;  // rmt_encoder_handle_t
    void* rx_buffer_ = nullptr;     // rmt_symbol_word_t[kMaxSymbols], heap-allocated

    std::atomic<bool> learning_{false};
    bool tools_registered_ = false;

    // 1 µs RMT ticks give us 0.3 % resolution on a 560 µs NEC pulse — fine.
    static constexpr uint32_t kRmtResolutionHz = 1000000;
    static constexpr uint32_t kCarrierHz       = 38000;
    // 64 symbols (~7.7 ms of NEC, one full frame is ~67). MUST stay <=
    // SOC_RMT_MEM_WORDS_PER_CHANNEL (48) so a single RX channel can fit
    // the entire capture; otherwise ESP-IDF's RX register code demands
    // mem_block_num contiguous blocks, which can't be satisfied when the
    // RX pool has only 4 channels on ESP32-S3.
    static constexpr size_t  kMaxSymbols       = 64;
    // Maximum length of a stored signal name (NVS key limit).
    static constexpr size_t  kMaxNameLen       = 15;
    // NVS namespace used for all IR signals.
    static constexpr char   kNvsNamespace[]   = "ir";
};

#endif // __IR_CONTROLLER_H__
