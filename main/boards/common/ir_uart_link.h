#ifndef __IR_UART_LINK_H__
#define __IR_UART_LINK_H__

#include <driver/gpio.h>
#include <cstdint>

// UART link to the YS-IRTM (or compatible) IR encode/decode bridge.
//
// The YS-IRTM board carries its own 38 kHz receiver + transmitter, so
// ESP32 never touches RMT. It exchanges NEC-encoded IR frames with the
// ESP32 over a TTL serial port at 9600 8N1:
//
//   ESP32  ──TX──►  YS-IRTM RXD   (host → module: "emit this frame")
//   ESP32  ◄──RX──  YS-IRTM TXD   (module → host: "I just received this frame")
//
// The wire format on the wire is plaintext: one NEC frame = 4 bytes
// (address, address_inverse, command, command_inverse). On receive,
// YS-IRTM echoes the 4 bytes followed by a 0xAA terminator. On transmit,
// the host sends the same 4 bytes + 0xAA terminator and YS-IRTM does
// the 38 kHz modulation + LED drive.
//
// If your module revision uses a different framing (e.g. 0xA0/0xA1
// prefix, length byte, XOR checksum), patch the two helpers in
// ir_uart_link.cc — the public API (SendFrame / ReceiveFrame) stays
// the same.
class IrUartLink {
public:
    // Bring up UART2 at 9600 8N1. The RX/TX GPIOs are passed explicitly
    // so we can pick any pair that doesn't collide with the rest of the
    // board (we use GPIO2 RX, GPIO12 TX on bread-compact-wifi-lcd).
    // Returns true on success.
    static bool Init(gpio_num_t tx_gpio, gpio_num_t rx_gpio, uint32_t baud = 9600);

    // Flush any half-read bytes out of the RX ring buffer. Call before
    // issuing a "send" message so an in-flight echo doesn't confuse the
    // receiver.
    static void DrainRx();

    // Send a 4-byte NEC frame (address, address_inverse, command,
    // command_inverse). YS-IRTM re-emits it on 38 kHz IR.
    static bool SendFrame(uint8_t addr, uint8_t cmd);

    // Block up to timeout_ms for a 4-byte NEC frame to arrive from the
    // module, return true if one did (and copy into addr/cmd), false on
    // timeout. The caller is responsible for calling DrainRx() before
    // issuing the remote button press.
    static bool ReceiveFrame(uint32_t timeout_ms, uint8_t* addr, uint8_t* cmd);

    // For diagnostics: read one byte non-blockingly from the RX ring.
    // Returns -1 if no byte available.
    static int ReadByte();
};

#endif // __IR_UART_LINK_H__
