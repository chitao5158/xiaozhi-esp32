#include "weather_service.h"

#include <esp_log.h>
#include <cJSON.h>
#include <cstring>

#include "board.h"

#define TAG "Weather"

// QWeather icon code -> UTF-8 emoji glyph.
// Source: https://dev.qweather.com/docs/resource/icons/
// QWeather uses 3-digit numeric codes like 100, 101, 104, 305, 401 etc.
static const char* QWeatherIconToEmoji(const char* icon) {
    if (icon == nullptr || icon[0] == '\0') return "·";
    // Sunny / clear
    if (strcmp(icon, "100") == 0) return "\xE2\x98\x80";       // ☀
    // Few clouds / partly cloudy
    if (strcmp(icon, "101") == 0 ||
        strcmp(icon, "102") == 0 ||
        strcmp(icon, "103") == 0) return "\xE2\x9B\x85";       // ⛅
    // Overcast
    if (strcmp(icon, "104") == 0 ||
        strcmp(icon, "150") == 0) return "\xE2\x98\x81";       // ☁
    // Shower rain
    if (strcmp(icon, "300") == 0 ||
        strcmp(icon, "301") == 0 ||
        strcmp(icon, "305") == 0 ||
        strcmp(icon, "306") == 0 ||
        strcmp(icon, "307") == 0 ||
        strcmp(icon, "310") == 0 ||
        strcmp(icon, "311") == 0 ||
        strcmp(icon, "312") == 0 ||
        strcmp(icon, "315") == 0 ||
        strcmp(icon, "316") == 0 ||
        strcmp(icon, "317") == 0 ||
        strcmp(icon, "318") == 0) return "\xF0\x9F\x8C\xA7";   // 🌧
    // Rain
    if (strcmp(icon, "302") == 0 ||
        strcmp(icon, "303") == 0 ||
        strcmp(icon, "304") == 0 ||
        strcmp(icon, "308") == 0 ||
        strcmp(icon, "309") == 0 ||
        strcmp(icon, "313") == 0 ||
        strcmp(icon, "314") == 0 ||
        strcmp(icon, "319") == 0 ||
        strcmp(icon, "320") == 0) return "\xF0\x9F\x8C\xA6";   // 🌦
    // Thunderstorm
    if (strcmp(icon, "301") == 0 ||
        strcmp(icon, "302") == 0 ||
        (strcmp(icon, "401") >= 0 && strcmp(icon, "499") <= 0)) {
        // Snow code range
    }
    if (strcmp(icon, "302") == 0) return "\xE2\x9B\x88";       // ⛈
    // Snow
    if ((strcmp(icon, "400") == 0 || strcmp(icon, "401") == 0 ||
         strcmp(icon, "402") == 0 || strcmp(icon, "403") == 0 ||
         strcmp(icon, "404") == 0 || strcmp(icon, "405") == 0 ||
         strcmp(icon, "406") == 0 || strcmp(icon, "407") == 0 ||
         strcmp(icon, "408") == 0 || strcmp(icon, "409") == 0 ||
         strcmp(icon, "410") == 0 || strcmp(icon, "456") == 0 ||
         strcmp(icon, "457") == 0 || strcmp(icon, "499") == 0)) {
        return "\xE2\x9D\x84";                                  // ❄
    }
    // Thunderstorm
    if (strcmp(icon, "302") == 0) return "\xE2\x9B\x88";       // ⛈
    // Fog / mist
    if (strcmp(icon, "500") == 0 || strcmp(icon, "501") == 0 ||
        strcmp(icon, "502") == 0 || strcmp(icon, "503") == 0 ||
        strcmp(icon, "504") == 0 || strcmp(icon, "505") == 0 ||
        strcmp(icon, "506") == 0 || strcmp(icon, "507") == 0 ||
        strcmp(icon, "508") == 0 || strcmp(icon, "509") == 0 ||
        strcmp(icon, "510") == 0 || strcmp(icon, "511") == 0 ||
        strcmp(icon, "512") == 0 || strcmp(icon, "513") == 0 ||
        strcmp(icon, "514") == 0 || strcmp(icon, "515") == 0) {
        return "\xF0\x9F\x8C\xAB";                              // 🌫
    }
    // Thunderstorm range 200-299 (QWeather uses these for various thunder)
    if ((strcmp(icon, "200") >= 0 && strcmp(icon, "299") <= 0)) {
        return "\xE2\x9B\x88";                                 // ⛈
    }
    return "·";
}

// OpenWeatherMap icon code (2 chars) -> UTF-8 emoji glyph.
// Kept for backward compatibility if user keeps OpenWeatherMap provider.
static const char* OpenWeatherIconToEmoji(const char* icon) {
    if (icon == nullptr || icon[0] == '\0') return "·";
    if (strncmp(icon, "01", 2) == 0) return "\xE2\x98\x80";        // ☀ clear sky
    if (strncmp(icon, "02", 2) == 0) return "\xE2\x9B\x85";        // ⛅ few clouds
    if (strncmp(icon, "03", 2) == 0) return "\xE2\x98\x81";        // ☁ scattered clouds
    if (strncmp(icon, "04", 2) == 0) return "\xE2\x98\x81";        // ☁ broken clouds
    if (strncmp(icon, "09", 2) == 0) return "\xF0\x9F\x8C\xA7";    // 🌧 shower rain
    if (strncmp(icon, "10", 2) == 0) return "\xF0\x9F\x8C\xA6";    // 🌦 rain
    if (strncmp(icon, "11", 2) == 0) return "\xE2\x9B\x88";        // ⛈ thunderstorm
    if (strncmp(icon, "13", 2) == 0) return "\xE2\x9D\x84";        // ❄ snow
    if (strncmp(icon, "50", 2) == 0) return "\xF0\x9F\x8C\xAB";    // 🌫 mist
    return "·";
}

// Background fetch task.
void WeatherService::FetchTask(void* arg) {
    auto* self = static_cast<WeatherService*>(arg);
    self->Fetch();
    self->fetch_in_progress_ = false;
    vTaskDelete(nullptr);
}

WeatherService::WeatherService() = default;

WeatherService::~WeatherService() {
    Stop();
}

void WeatherService::Start() {
    if (running_) return;
    running_ = true;

    esp_timer_create_args_t timer_args = {
        .callback = &WeatherService::TimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "weather_timer",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_));

    // First fetch after a short delay so the device can finish booting.
    esp_timer_start_once(timer_, 5 * 1000 * 1000);  // 5 seconds
#if CONFIG_WEATHER_PROVIDER_QWEATHER
    ESP_LOGI(TAG, "WeatherService started (QWeather), location=%s", CONFIG_WEATHER_CITY);
#else
    ESP_LOGI(TAG, "WeatherService started (OpenWeatherMap), city=%s", CONFIG_WEATHER_CITY);
#endif
}

void WeatherService::Stop() {
    if (!running_) return;
    running_ = false;
    if (timer_ != nullptr) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }
}

void WeatherService::TimerCallback(void* arg) {
    auto* self = static_cast<WeatherService*>(arg);
    if (!self->running_) return;
    if (self->fetch_in_progress_) {
        self->ScheduleNext();
        return;
    }
    self->fetch_in_progress_ = true;
    xTaskCreate(FetchTask, "weather_fetch", 4096, self, 1, &self->fetch_task_);
    self->ScheduleNext();
}

void WeatherService::ScheduleNext() {
    if (!running_ || timer_ == nullptr) return;
    int minutes = CONFIG_WEATHER_UPDATE_INTERVAL_MINUTES;
    if (minutes < 5) minutes = 5;
    esp_timer_start_once(timer_, (int64_t)minutes * 60 * 1000 * 1000);
}

void WeatherService::Fetch() {
    auto network = Board::GetInstance().GetNetwork();
    if (network == nullptr) {
        ESP_LOGW(TAG, "No network, skipping fetch");
        return;
    }
    auto http = network->CreateHttp(0);
    if (http == nullptr) {
        ESP_LOGW(TAG, "Failed to create HTTP client");
        return;
    }

    char url[256];
#if CONFIG_WEATHER_PROVIDER_QWEATHER
    // QWeather API: https://devapi.qweather.com/v7/weather/now?location=<id>&key=<key>
    snprintf(url, sizeof(url),
        "https://devapi.qweather.com/v7/weather/now?location=%s&key=%s",
        CONFIG_WEATHER_CITY, CONFIG_WEATHER_API_KEY);
#else
    // OpenWeatherMap API
    snprintf(url, sizeof(url),
        "https://api.openweathermap.org/data/2.5/weather?q=%s&appid=%s&units=metric&lang=zh_cn",
        CONFIG_WEATHER_CITY, CONFIG_WEATHER_API_KEY);
#endif

    http->SetTimeout(10000);  // 10s
#if CONFIG_WEATHER_PROVIDER_QWEATHER
    // QWeather requires Referer header to be in the whitelist configured in dev.qweather.com.
    // Default to "localhost" - update this in dev.qweather.com Settings -> API Key -> Referer whitelist.
    http->SetHeader("Referer", "localhost");
#endif
    if (!http->Open("GET", url)) {
        ESP_LOGW(TAG, "HTTP open failed: %d", http->GetLastError());
        return;
    }

    int status = http->GetStatusCode();
    if (status != 200) {
        ESP_LOGW(TAG, "HTTP status %d", status);
        // Dump body for debugging QWeather error codes (e.g. 401, 403).
        std::string body = http->ReadAll();
        if (!body.empty()) {
            ESP_LOGW(TAG, "Response body: %s", body.c_str());
        }
        http->Close();
        return;
    }

    std::string body = http->ReadAll();
    http->Close();

    ESP_LOGD(TAG, "Response (%u bytes): %s", (unsigned)body.size(), body.c_str());
    last_json_ = body;
    ParseAndStore(body);
}

void WeatherService::ParseAndStore(const std::string& json) {
    cJSON* root = cJSON_Parse(json.c_str());
    if (root == nullptr) {
        ESP_LOGW(TAG, "JSON parse failed");
        return;
    }

#if CONFIG_WEATHER_PROVIDER_QWEATHER
    // QWeather response format:
    // { "code":"200", "updateTime":"...", "now": { "temp":"23", "feelsLike":"...",
    //   "icon":"100", "text":"晴", "wind360":"0", "windDir":"北风", "windScale":"1",
    //   "windSpeed":"3", "humidity":"47", "precip":"0.0", "pressure":"...",
    //   "vis":"20", "cloud":"0", "dew":"..." }, ... }
    cJSON* code_node = cJSON_GetObjectItem(root, "code");
    if (!cJSON_IsString(code_node) || strcmp(code_node->valuestring, "200") != 0) {
        ESP_LOGW(TAG, "QWeather API returned non-200 code: %s",
                 cJSON_IsString(code_node) ? code_node->valuestring : "(non-string)");
        cJSON_Delete(root);
        return;
    }

    cJSON* now = cJSON_GetObjectItem(root, "now");
    if (now == nullptr) {
        ESP_LOGW(TAG, "Missing 'now' field in QWeather response");
        cJSON_Delete(root);
        return;
    }

    cJSON* temp_node = cJSON_GetObjectItem(now, "temp");
    cJSON* icon_node = cJSON_GetObjectItem(now, "icon");
    cJSON* text_node = cJSON_GetObjectItem(now, "text");
    cJSON* humidity_node = cJSON_GetObjectItem(now, "humidity");
    cJSON* wind_dir_node = cJSON_GetObjectItem(now, "windDir");
    cJSON* wind_scale_node = cJSON_GetObjectItem(now, "windScale");

    if (!cJSON_IsString(temp_node)) {
        ESP_LOGW(TAG, "temp not a string");
        cJSON_Delete(root);
        return;
    }

    // QWeather temp is a string like "23" (°C).
    int temp = atoi(temp_node->valuestring);
    const char* icon = cJSON_IsString(icon_node) ? icon_node->valuestring : "";
    const char* emoji = QWeatherIconToEmoji(icon);
    const char* text = cJSON_IsString(text_node) ? text_node->valuestring : "";
    const char* humidity = cJSON_IsString(humidity_node) ? humidity_node->valuestring : "?";
    const char* wind_dir = cJSON_IsString(wind_dir_node) ? wind_dir_node->valuestring : "?";
    const char* wind_scale = cJSON_IsString(wind_scale_node) ? wind_scale_node->valuestring : "?";

    // Display: "☀ 23°" — emoji + temp. Emoji uses text_font (per SetupUI).
    char buf[32];
    snprintf(buf, sizeof(buf), "%s%d°", emoji, temp);
    current_ = buf;
    ESP_LOGI(TAG, "Weather: %s (%s, %s%s级, 湿度%s%%)",
             current_.c_str(), text, wind_dir, wind_scale, humidity);
#else
    // OpenWeatherMap legacy parsing.
    cJSON* main = cJSON_GetObjectItem(root, "main");
    cJSON* weather_arr = cJSON_GetObjectItem(root, "weather");
    cJSON* name = cJSON_GetObjectItem(root, "name");

    if (main == nullptr || weather_arr == nullptr || !cJSON_IsArray(weather_arr)
        || cJSON_GetArraySize(weather_arr) == 0) {
        ESP_LOGW(TAG, "Missing fields in response");
        cJSON_Delete(root);
        return;
    }

    cJSON* temp_node = cJSON_GetObjectItem(main, "temp");
    cJSON* weather0 = cJSON_GetArrayItem(weather_arr, 0);
    cJSON* icon_node = cJSON_GetObjectItem(weather0, "icon");
    cJSON* desc_node = cJSON_GetObjectItem(weather0, "description");

    if (!cJSON_IsNumber(temp_node)) {
        ESP_LOGW(TAG, "temp not a number");
        cJSON_Delete(root);
        return;
    }

    int temp = (int)(temp_node->valuedouble + (temp_node->valuedouble >= 0 ? 0.5f : -0.5f));
    const char* icon = cJSON_IsString(icon_node) ? icon_node->valuestring : "";
    const char* emoji = OpenWeatherIconToEmoji(icon);
    const char* desc = cJSON_IsString(desc_node) ? desc_node->valuestring : "";

    char buf[32];
    snprintf(buf, sizeof(buf), "%d°", temp);
    current_ = buf;
    ESP_LOGI(TAG, "Weather: %s (city=%s)", current_.c_str(),
             cJSON_IsString(name) ? name->valuestring : CONFIG_WEATHER_CITY);
#endif

    cJSON_Delete(root);

    if (callback_) {
        callback_(current_);
    }
}