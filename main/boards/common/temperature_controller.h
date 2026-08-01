#ifndef __TEMPERATURE_CONTROLLER_H__
#define __TEMPERATURE_CONTROLLER_H__

#include "mcp_server.h"

#include <cmath>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_timer.h>

// NTC thermistor wired as a voltage divider.
//   VCC -- R_fixed -- ADC_PIN -- NTC -- GND
//   V_ADC = VCC * R_NTC / (R_fixed + R_NTC)
//   1/T   = 1/T0 + (1/B) * ln(R_NTC / R0)
//
// Pass parameters explicitly in the constructor. Defaults below match the
// common "W104 / KY-028" module (10kΩ NTC, 47kΩ series, B = 3950), but
// boards with different hardware should pass their own values.
struct TemperatureSensorConfig {
    float r_fixed_ohms = 47000.0f;   // series resistor (ohms)
    float r25_ohms     = 10000.0f;   // NTC resistance at 25 °C (ohms)
    float b_coeff      = 3950.0f;    // NTC B coefficient
    float vcc_volts    = 3.3f;       // ADC reference voltage
    int   sample_count = 8;          // samples to average per reading
};

class TemperatureController {
public:
    // Returns the most recently constructed instance. The board creates
    // exactly one TemperatureController at startup; this accessor lets
    // unrelated subsystems (e.g. the HTTP command server) reach it without
    // having to thread the pointer around.
    static TemperatureController& GetInstance();

    explicit TemperatureController(gpio_num_t gpio_num,
                                   const TemperatureSensorConfig& config = {});
    ~TemperatureController();

    // Returns temperature in °C, or NAN if the controller is not initialized
    // (e.g. GPIO_NUM_NC was passed in, or ADC bring-up failed).
    float ReadCelsius();

    // Most recently computed temperature without re-sampling the ADC.
    float GetCachedCelsius() const { return cached_celsius_; }

    // Read the raw ADC voltage in millivolts (no NTC math). Useful for
    // external tools (e.g. the HTTP command server) that want to expose
    // the raw measurement.
    int ReadMillivolts();

    // True when the ADC was brought up successfully.
    bool IsInitialized() const { return initialized_; }

    // Schedules a one-shot announcement in `minutes` minutes. When the timer
    // fires, `callback` is invoked from a FreeRTOS timer task with the current
    // temperature in Celsius (or NAN on read failure). Only one announcement
    // can be pending at a time; scheduling a new one cancels any previous.
    //
    // Callbacks should defer UI/audio work via Application::Schedule() since
    // they run outside the main task context.
    using AnnouncementCallback = std::function<void(float celsius)>;
    void ScheduleAnnouncement(int minutes, AnnouncementCallback callback);
    void CancelAnnouncement();
    bool HasPendingAnnouncement() const { return announcement_timer_ != nullptr; }

    // Default callback used by the MCP tool. Boards can override this via
    // SetAnnouncementCallback() to plug in their own UI/audio handling.
    void SetAnnouncementCallback(AnnouncementCallback cb) { default_announcement_callback_ = std::move(cb); }

private:
    gpio_num_t gpio_num_;
    TemperatureSensorConfig config_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    adc_channel_t adc_channel_ = ADC_CHANNEL_0;
    adc_unit_t adc_unit_ = ADC_UNIT_1;
    bool initialized_ = false;
    float cached_celsius_ = NAN;

    esp_timer_handle_t announcement_timer_ = nullptr;
    AnnouncementCallback announcement_callback_;
    AnnouncementCallback default_announcement_callback_;

    static void AnnouncementTimerCallback(void* arg);
};

#endif // __TEMPERATURE_CONTROLLER_H__