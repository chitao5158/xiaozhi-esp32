#include "command_server.h"

#include <cJSON.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <mdns.h>
#include <cstring>

#include "application.h"
#include "board.h"
#include "temperature_controller.h"

#define TAG "CommandServer"

namespace {
// Build a CORS-permissive JSON response. Many webhook use cases need this
// (curl doesn't, but browser fetch + IFTTT and similar do).
esp_err_t send_json(httpd_req_t* req, cJSON* root) {
    char* body = cJSON_PrintUnformatted(root);
    if (body == nullptr) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json encode failed");
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, body, strlen(body));
    free(body);
    cJSON_Delete(root);
    return ESP_OK;
}

uint64_t uptime_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000);
}

// Log every request so we can see on the serial monitor whether external
// traffic is actually reaching the device. The httpd_req public struct in
// ESP-IDF 5.x doesn't expose remote_ip directly, so we just log the method
// and URI — the request reaching the handler at all is what matters here.
esp_err_t log_request(httpd_req_t* req) {
    const char* method_str = "OTHER";
    switch (req->method) {
        case HTTP_GET:    method_str = "GET";    break;
        case HTTP_POST:   method_str = "POST";   break;
        case HTTP_PUT:    method_str = "PUT";    break;
        case HTTP_DELETE: method_str = "DELETE"; break;
        default: break;
    }
    ESP_LOGI(TAG, "HTTP %s %s", method_str, req->uri);
    return ESP_OK;
}

esp_err_t root_handler(httpd_req_t* req) {
    log_request(req);
    const char* body =
        "xiaozhi command server\n"
        "  GET /api/temperature  - read NTC sensor\n"
        "  GET /api/info         - device info\n";
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, body, strlen(body));
    return ESP_OK;
}

esp_err_t temperature_handler(httpd_req_t* req) {
    log_request(req);
    auto& temp = TemperatureController::GetInstance();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", temp.IsInitialized());
    if (temp.IsInitialized()) {
        float celsius = temp.ReadCelsius();
        int mv = temp.ReadMillivolts();
        cJSON_AddNumberToObject(root, "celsius", celsius);
        cJSON_AddNumberToObject(root, "raw_mv", mv);
        cJSON_AddNumberToObject(root, "ready", temp.IsInitialized() ? 1 : 0);
    } else {
        cJSON_AddStringToObject(root, "error", "temperature controller not initialized");
    }
    return send_json(req, root);
}

esp_err_t info_handler(httpd_req_t* req) {
    log_request(req);
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "device", BOARD_NAME);
    cJSON_AddNumberToObject(root, "uptime_sec", (double)(uptime_ms() / 1000));
    cJSON_AddNumberToObject(root, "free_sram_kb",
                            (double)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    return send_json(req, root);
}
}  // namespace

CommandServer& CommandServer::GetInstance() {
    static CommandServer instance;
    return instance;
}

namespace {
// The HTTP server has two prerequisites that aren't ready when the board
// constructor runs:
//   1. esp_event_loop_create_default() — called later by the wifi manager.
//   2. lwIP TCP/IP thread — only up once an IP has been acquired.
// We satisfy both with a one-shot task that waits for the event loop and
// then subscribes to IP_EVENT_STA_GOT_IP. The actual httpd_start() runs
// from the IP-event callback once we have an IP, never from the board
// constructor.
void on_ip_event(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base != IP_EVENT || id != IP_EVENT_STA_GOT_IP) {
        return;
    }
    auto& server = CommandServer::GetInstance();
    if (server.IsRunning()) {
        return;
    }
    auto* event = static_cast<ip_event_got_ip_t*>(data);
    ESP_LOGI(TAG, "Got IP " IPSTR ", starting HTTP server on port %d",
             IP2STR(&event->ip_info.ip), server.Port());
    server.StartHttpd();

    // Also publish mDNS so the device is reachable as
    // <board>.local from any Bonjour-aware client (macOS, iOS, Linux).
    // This sidesteps the changing-DHCP-IP problem entirely.
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(BOARD_NAME);
        mdns_instance_name_set(BOARD_NAME);
        mdns_service_add(NULL, "_http", "_tcp", server.Port(), NULL, 0);
        ESP_LOGI(TAG, "mDNS: http://" BOARD_NAME ".local:%d/", server.Port());
    } else {
        ESP_LOGW(TAG, "mdns_init failed; .local hostname not available");
    }
}

// One-shot task that subscribes to IP_EVENT_STA_GOT_IP. The event loop
// isn't created until the wifi manager runs (after the board constructor
// returns), so we wait for it before registering. We retry once if the
// loop still isn't there after the first delay.
void register_hook_task(void* arg) {
    int port = static_cast<int>(reinterpret_cast<intptr_t>(arg));
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &on_ip_event, nullptr);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "event handler register retry: %s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
        err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &on_ip_event, nullptr);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "event handler register ultimately failed: %s",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "registered IP_EVENT_STA_GOT_IP hook, will start server on port %d", port);
    }
    vTaskDelete(nullptr);
}
}  // namespace

bool CommandServer::Start(int port) {
    if (server_handle_ != nullptr) {
        return true;
    }
    // The board constructor calls Start() before the wifi manager has had a
    // chance to create the default event loop, so any direct call to
    // esp_event_handler_register() from here fails with ESP_ERR_INVALID_STATE
    // and a direct call to httpd_start() fails because the lwIP TCP/IP
    // thread isn't up yet. Defer ALL the heavy lifting to a one-shot task
    // (see register_hook_task) which fires once both prerequisites are
    // satisfied.
    port_ = port;
    static bool task_spawned = false;
    if (!task_spawned) {
        xTaskCreate(register_hook_task, "cmd_reg", 2048,
                    reinterpret_cast<void*>(static_cast<intptr_t>(port)),
                    1, nullptr);
        task_spawned = true;
    }
    return true;  // "Started" = "asked to start"; the actual httpd_start happens later
}

void CommandServer::StartHttpd() {
    if (server_handle_ != nullptr) {
        return;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port_;
    config.max_open_sockets = 4;
    config.ctrl_port = 32769;
    config.stack_size = 4096;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;

    esp_err_t err = httpd_start(&server_handle_, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        server_handle_ = nullptr;
        return;
    }

    httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET, .handler = root_handler,
        .user_ctx = nullptr,
    };
    httpd_uri_t temp_uri = {
        .uri = "/api/temperature", .method = HTTP_GET, .handler = temperature_handler,
        .user_ctx = nullptr,
    };
    httpd_uri_t info_uri = {
        .uri = "/api/info", .method = HTTP_GET, .handler = info_handler,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_handle_, &root_uri);
    httpd_register_uri_handler(server_handle_, &temp_uri);
    httpd_register_uri_handler(server_handle_, &info_uri);

    ESP_LOGI(TAG, "CommandServer started on port %d (GET /api/temperature, /api/info)", port_);
}

void CommandServer::Stop() {
    if (server_handle_ == nullptr) {
        return;
    }
    httpd_stop(server_handle_);
    server_handle_ = nullptr;
    ESP_LOGI(TAG, "CommandServer stopped");
}