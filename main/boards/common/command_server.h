#ifndef __COMMAND_SERVER_H__
#define __COMMAND_SERVER_H__

#include <cstdint>

// Lightweight HTTP webhook server for remote triggering. Listens on a
// configurable port (default 80) and exposes a small JSON API:
//
//   GET  /                  → plain-text welcome banner
//   GET  /api/temperature   → {"ok":true,"celsius":27.5,"raw_mv":510,...}
//   GET  /api/info          → {"ok":true,"device":"...","uptime_sec":12345,...}
//
// Security note: this server is unauthenticated and runs on the device's
// WiFi network. Don't expose it on a public network without adding a
// token / mTLS layer.
class CommandServer {
public:
    static CommandServer& GetInstance();

    // Request the server to start. Schedules a one-shot task that waits
    // for the event loop and the lwIP TCP/IP thread, then subscribes to
    // IP_EVENT_STA_GOT_IP. The actual httpd_start() happens later, in
    // that IP-event callback. Safe to call from the board constructor
    // (when the event loop and TCP/IP thread don't exist yet).
    bool Start(int port = 80);

    // Start the actual httpd server. Called from the IP_EVENT callback
    // once we have an IP — never directly from the board constructor.
    void StartHttpd();

    // Stop the server (frees the httpd handle). Idempotent.
    void Stop();

    bool IsRunning() const { return server_handle_ != nullptr; }
    int Port() const { return port_; }

private:
    CommandServer() = default;
    CommandServer(const CommandServer&) = delete;
    CommandServer& operator=(const CommandServer&) = delete;

    void* server_handle_ = nullptr;   // httpd_handle_t
    int port_ = 0;
};

#endif  // __COMMAND_SERVER_H__