#include "ir_uart_link.h"

#include <esp_log.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

#include <cstring>

#define TAG "IrUart"

namespace {
    // ESP32-S3 has UART0/1/2. UART0 is the USB console, so we use UART2.
    constexpr uart_port_t kUart = UART_NUM_2;
    constexpr int kRxBufSize = 1024;
    constexpr int kTxBufSize = 256;

    RingbufHandle_t s_rx_ring = nullptr;
    gpio_num_t s_tx_gpio = GPIO_NUM_NC;
    gpio_num_t s_rx_gpio = GPIO_NUM_NC;
    bool s_initialized = false;
}

bool IrUartLink::Init(gpio_num_t tx_gpio, gpio_num_t rx_gpio, uint32_t baud) {
    if (s_initialized) return true;

    uart_config_t cfg = {
        .baud_rate  = (int)baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = { .allow_pd = 0 },
    };
    if (uart_driver_install(kUart, kRxBufSize, kTxBufSize, 0, nullptr, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return false;
    }
    if (uart_param_config(kUart, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        uart_driver_delete(kUart);
        return false;
    }
    if (uart_set_pin(kUart, (int)tx_gpio, (int)rx_gpio, -1, -1) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin(TX=%d,RX=%d) failed", (int)tx_gpio, (int)rx_gpio);
        uart_driver_delete(kUart);
        return false;
    }

    s_tx_gpio = tx_gpio;
    s_rx_gpio = rx_gpio;
    s_initialized = true;
    ESP_LOGI(TAG, "UART2 up: TX=GPIO%d RX=GPIO%d @ %lu baud",
             (int)tx_gpio, (int)rx_gpio, (unsigned long)baud);
    return true;
}

void IrUartLink::DrainRx() {
    if (!s_initialized) return;
    uart_flush_input(kUart);
}

int IrUartLink::ReadByte() {
    if (!s_initialized) return -1;
    uint8_t b;
    int n = uart_read_bytes(kUart, &b, 1, 0);
    return (n == 1) ? (int)b : -1;
}

bool IrUartLink::SendFrame(uint8_t addr, uint8_t cmd) {
    if (!s_initialized) return false;
    uint8_t buf[5] = {
        addr,
        (uint8_t)~addr,
        cmd,
        (uint8_t)~cmd,
        0xAA,                       // terminator
    };
    int n = uart_write_bytes(kUart, buf, sizeof(buf));
    if (n != sizeof(buf)) {
        ESP_LOGE(TAG, "uart_write_bytes wrote %d/%zu", n, sizeof(buf));
        return false;
    }
    ESP_LOGI(TAG, "TX frame: addr=0x%02x cmd=0x%02x", addr, cmd);
    return true;
}

bool IrUartLink::ReceiveFrame(uint32_t timeout_ms, uint8_t* addr, uint8_t* cmd) {
    if (!s_initialized || addr == nullptr || cmd == nullptr) return false;

    // YS-IRTM echoes the 4-byte frame + 0xAA terminator within ~50 ms of
    // a button press. Read until we have all 5 bytes, or timeout.
    uint8_t buf[5] = {};
    int got = 0;
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (got < 5 && esp_timer_get_time() < deadline_us) {
        int remain_us = (int)(deadline_us - esp_timer_get_time());
        if (remain_us < 0) remain_us = 0;
        int n = uart_read_bytes(kUart, buf + got, 5 - got,
                                pdMS_TO_TICKS(remain_us / 1000 + 1));
        if (n > 0) got += n;
    }
    if (got < 5) {
        ESP_LOGW(TAG, "RX timeout: got %d/5 bytes", got);
        return false;
    }
    // Verify the terminator and inverse-byte integrity.
    if (buf[4] != 0xAA) {
        ESP_LOGW(TAG, "RX bad terminator: 0x%02x (got bytes: %02x %02x %02x %02x %02x)",
                 buf[4], buf[0], buf[1], buf[2], buf[3]);
        return false;
    }
    if ((uint8_t)(buf[0] ^ buf[1]) != 0xFF || (uint8_t)(buf[2] ^ buf[3]) != 0xFF) {
        ESP_LOGW(TAG, "RX inverse mismatch: %02x %02x %02x %02x",
                 buf[0], buf[1], buf[2], buf[3]);
        return false;
    }
    *addr = buf[0];
    *cmd  = buf[2];
    ESP_LOGI(TAG, "RX frame: addr=0x%02x cmd=0x%02x", *addr, *cmd);
    return true;
}
