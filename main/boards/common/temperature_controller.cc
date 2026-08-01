#include "temperature_controller.h"

#include <esp_log.h>
#include <math.h>
#include <cstdio>

#define TAG "TempSensor"

namespace {
TemperatureController* g_instance_ = nullptr;
}  // namespace

TemperatureController::TemperatureController(gpio_num_t gpio_num,
                                             const TemperatureSensorConfig& config)
    : gpio_num_(gpio_num), config_(config) {
    if (gpio_num_ == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "GPIO_NUM_NC passed in, controller disabled");
        return;
    }

    // Map GPIO -> ADC unit + channel dynamically so the controller works on any
    // board without hard-coding ADC1_CHANNEL_0. On ESP32-S3 with Wi-Fi running,
    // ADC2 is unavailable — the dynamic mapping picks ADC1 if both units
    // service the pin, but we prefer ADC1 explicitly below.
    esp_err_t err = adc_oneshot_io_to_channel(gpio_num_, &adc_unit_, &adc_channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d is not ADC-capable: %s", (int)gpio_num_,
                 esp_err_to_name(err));
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = adc_unit_,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&unit_cfg, &adc_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,    // 0 ~ VCC (~3.3V) full scale
        .bitwidth = ADC_BITWIDTH_12, // 0 ~ 4095
    };
    err = adc_oneshot_config_channel(adc_handle_, adc_channel_, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
        return;
    }

    // Optional eFuse-based calibration for a more accurate voltage reading.
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = adc_unit_,
        .chan = adc_channel_,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle_) != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using linear conversion");
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized on GPIO %d (ADC%d_CH%d), R_fixed=%.0fΩ, R25=%.0fΩ, B=%.0f",
             (int)gpio_num_, (int)adc_unit_ + 1, (int)adc_channel_,
             (double)config_.r_fixed_ohms,
             (double)config_.r25_ohms,
             (double)config_.b_coeff);

    // Publish ourselves to the singleton so unrelated code can reach us.
    // (File-scope so it survives across multiple TemperatureController
    // constructions — but we only ever construct one.)
    g_instance_ = this;

    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddTool(
        "self.temperature.get_celsius",
        "Read the current ambient temperature in degrees Celsius from the on-board NTC sensor.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            float t = ReadCelsius();
            if (isnan(t)) {
                return std::string("{\"error\":\"sensor not ready\"}");
            }
            char json[64];
            snprintf(json, sizeof(json), "{\"celsius\":%.1f}", t);
            return std::string(json);
        });

    mcp_server.AddTool(
        "self.temperature.get_raw",
        "Read the raw ADC value (0-4095) of the temperature sensor. Useful for debugging the wiring.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            if (!initialized_) {
                return std::string("{\"error\":\"sensor not ready\"}");
            }
            int mv = ReadMillivolts();
            char json[64];
            snprintf(json, sizeof(json), "{\"millivolts\":%d}", mv);
            return std::string(json);
        });

    mcp_server.AddTool(
        "self.temperature.schedule_announcement",
        "Schedule a one-shot announcement that will fire in `minutes` minutes. When it fires, "
        "the device shows the current temperature on the display, plays a chime and the "
        "temperature's digits aloud, and opens the microphone so the user can ask follow-up "
        "questions. Only one announcement can be pending at a time; calling this again cancels "
        "the previous one. Range: 1-1440 minutes.",
        PropertyList({Property("minutes", kPropertyTypeInteger, 1, 1440)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int minutes = properties["minutes"].value<int>();
            ScheduleAnnouncement(minutes, default_announcement_callback_);
            char json[64];
            snprintf(json, sizeof(json), "{\"scheduled_in_minutes\":%d}", minutes);
            return std::string(json);
        });

    mcp_server.AddTool(
        "self.temperature.cancel_announcement",
        "Cancel any pending scheduled temperature announcement. No-op if none is pending.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            bool had_pending = HasPendingAnnouncement();
            CancelAnnouncement();
            return had_pending;
        });
}

TemperatureController& TemperatureController::GetInstance() {
    if (g_instance_ == nullptr) {
        ESP_LOGE(TAG, "GetInstance() called before any TemperatureController was constructed");
        static TemperatureController stub(GPIO_NUM_NC, TemperatureSensorConfig{});
        stub.initialized_ = false;
        return stub;
    }
    return *g_instance_;
}

TemperatureController::~TemperatureController() {
    CancelAnnouncement();
    if (adc_cali_handle_) {
        adc_cali_delete_scheme_curve_fitting(adc_cali_handle_);
        adc_cali_handle_ = nullptr;
    }
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
    }
}

int TemperatureController::ReadMillivolts() {
    if (!initialized_) return 0;

    // Average several samples to suppress LM393 / sensor noise.
    uint32_t sum = 0;
    int valid = 0;
    for (int i = 0; i < config_.sample_count; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle_, adc_channel_, &raw) == ESP_OK) {
            sum += raw;
            valid++;
        }
    }
    if (valid == 0) return 0;
    int avg_raw = (int)(sum / valid);

    int mv = 0;
    if (adc_cali_handle_) {
        adc_cali_raw_to_voltage(adc_cali_handle_, avg_raw, &mv);
    } else {
        // Linear fallback: VCC over 4095 steps.
        mv = (int)((float)avg_raw * config_.vcc_volts * 1000.0f / 4095.0f + 0.5f);
    }
    return mv;
}

float TemperatureController::ReadCelsius() {
    if (!initialized_) return NAN;

    int mv = ReadMillivolts();
    if (mv <= 0) return NAN;

    // Clamp to a sensible physical range so an open / shorted input doesn't
    // produce wildly negative or astronomically high readings.
    int vcc_mv = (int)(config_.vcc_volts * 1000.0f);
    if (mv >= vcc_mv - 5) {
        cached_celsius_ = NAN;
        return NAN;
    }
    if (mv <= 5) {
        cached_celsius_ = NAN;
        return NAN;
    }

    // Voltage divider inversion: V_ADC = VCC * R_NTC / (R + R_NTC)
    //   => R_NTC = R * V_ADC / (VCC - V_ADC)
    float v_adc = (float)mv / 1000.0f;
    float r_ntc = config_.r_fixed_ohms * v_adc / (config_.vcc_volts - v_adc);

    // Steinhart-Hart first-order (B-parameter equation).
    const float t0_kelvin = 298.15f;          // 25 °C
    float inv_t = 1.0f / t0_kelvin +
                  (1.0f / config_.b_coeff) * logf(r_ntc / config_.r25_ohms);
    float celsius = (1.0f / inv_t) - 273.15f;

    ESP_LOGI(TAG, "raw: V_ADC=%.3fV, R_NTC=%.0fΩ → T=%.2f°C  (R25=%.0fΩ, B=%.0f, R_fixed=%.0fΩ)",
             v_adc, r_ntc, celsius,
             (double)config_.r25_ohms,
             (double)config_.b_coeff,
             (double)config_.r_fixed_ohms);

    cached_celsius_ = celsius;
    return celsius;
}

void TemperatureController::ScheduleAnnouncement(int minutes, AnnouncementCallback callback) {
    CancelAnnouncement();

    if (minutes < 1) minutes = 1;
    if (minutes > 1440) minutes = 1440;

    announcement_callback_ = std::move(callback);

    esp_timer_create_args_t args = {
        .callback = &TemperatureController::AnnouncementTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "temp_announce",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &announcement_timer_));

    int64_t delay_us = (int64_t)minutes * 60LL * 1000000LL;
    ESP_ERROR_CHECK(esp_timer_start_once(announcement_timer_, delay_us));

    ESP_LOGI(TAG, "Announcement scheduled in %d minute(s)", minutes);
}

void TemperatureController::CancelAnnouncement() {
    if (announcement_timer_) {
        esp_timer_stop(announcement_timer_);
        esp_timer_delete(announcement_timer_);
        announcement_timer_ = nullptr;
    }
    announcement_callback_ = nullptr;
}

void TemperatureController::AnnouncementTimerCallback(void* arg) {
    auto* self = static_cast<TemperatureController*>(arg);
    float temp = self->ReadCelsius();

    AnnouncementCallback cb;
    cb.swap(self->announcement_callback_);
    self->announcement_timer_ = nullptr;  // one-shot fired

    if (cb) {
        cb(temp);
    }
}