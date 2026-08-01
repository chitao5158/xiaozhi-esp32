#ifndef WEATHER_SERVICE_H
#define WEATHER_SERVICE_H

#include <string>
#include <functional>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Weather icon codes mapped from OpenWeatherMap icon codes.
// Mapping uses UTF-8 emoji (Font Awesome doesn't have weather icons by default).
// Day/Night variants use the same glyph for simplicity.
class WeatherService {
public:
    using WeatherCallback = std::function<void(const std::string&)>;

    WeatherService();
    ~WeatherService();

    // Start the periodic fetch loop. Fetches immediately, then every interval.
    void Start();

    // Stop the periodic fetch.
    void Stop();

    // Register a callback invoked whenever weather data updates.
    void OnUpdate(WeatherCallback cb) { callback_ = std::move(cb); }

    // Get the most recent cached display string (e.g. "☀ 25°").
    // Returns empty string if no data yet.
    const std::string& GetCurrent() const { return current_; }

    // Get the raw JSON of the last successful fetch (for debugging / MCP tools).
    const std::string& GetLastJson() const { return last_json_; }

private:
    void Fetch();
    void ParseAndStore(const std::string& json);
    static void TimerCallback(void* arg);
    static void FetchTask(void* arg);
    void ScheduleNext();

    std::string current_;
    std::string last_json_;
    esp_timer_handle_t timer_ = nullptr;
    TaskHandle_t fetch_task_ = nullptr;
    WeatherCallback callback_;
    bool running_ = false;
    bool fetch_in_progress_ = false;
};

#endif // WEATHER_SERVICE_H
