#include "motor_controller.h"

#include <esp_log.h>

#include <cstdio>
#include <cstring>

#define TAG "MotorCtrl"

MotorController::MotorController(gpio_num_t ia_gpio, gpio_num_t ib_gpio,
                                 ledc_timer_t timer,
                                 ledc_channel_t ia_channel,
                                 ledc_channel_t ib_channel,
                                 uint32_t pwm_freq_hz)
    : ia_gpio_(ia_gpio),
      ib_gpio_(ib_gpio),
      timer_(timer),
      ia_channel_(ia_channel),
      ib_channel_(ib_channel),
      pwm_freq_hz_(pwm_freq_hz) {
    if (ia_gpio_ == GPIO_NUM_NC && ib_gpio_ == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "Both IA and IB are GPIO_NUM_NC, controller disabled");
        return;
    }

    // Both channels share the same timer so they always stay in phase.
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = timer_,
        .freq_hz = pwm_freq_hz_,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return;
    }

    auto setup_channel = [&](gpio_num_t gpio, ledc_channel_t ch) -> esp_err_t {
        if (gpio == GPIO_NUM_NC) {
            // Still bind the channel to the timer so ledc_set_duty on it is
            // safe; just don't drive any GPIO.
            ledc_channel_config_t cfg = {};
            cfg.gpio_num = GPIO_NUM_NC;
            cfg.speed_mode = LEDC_LOW_SPEED_MODE;
            cfg.channel = ch;
            cfg.intr_type = LEDC_INTR_DISABLE;
            cfg.timer_sel = timer_;
            cfg.duty = 0;
            cfg.hpoint = 0;
            return ledc_channel_config(&cfg);
        }
        ledc_channel_config_t cfg = {
            .gpio_num = gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = ch,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer_,
            .duty = 0,
            .hpoint = 0,
            .flags = { .output_invert = false },
        };
        return ledc_channel_config(&cfg);
    };

    if (setup_channel(ia_gpio_, ia_channel_) != ESP_OK ||
        setup_channel(ib_gpio_, ib_channel_) != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed (IA=%d, IB=%d)",
                 (int)ia_gpio_, (int)ib_gpio_);
        return;
    }

    // Start coasted.
    if (ia_gpio_ != GPIO_NUM_NC) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ia_channel_, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ia_channel_);
    }
    if (ib_gpio_ != GPIO_NUM_NC) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ib_channel_, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ib_channel_);
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized: IA=GPIO%d (ch%d), IB=GPIO%d (ch%d), freq=%luHz",
             (int)ia_gpio_, (int)ia_channel_,
             (int)ib_gpio_, (int)ib_channel_,
             (unsigned long)pwm_freq_hz_);

    RegisterMcpTools();
}

void MotorController::ApplyPwm(Direction dir, int duty) {
    if (!initialized_) return;
    if (duty < 0) duty = 0;
    if (duty > kDutyMax) duty = kDutyMax;

    int ia_duty = 0;
    int ib_duty = 0;
    switch (dir) {
        case Direction::kForward: ia_duty = duty; ib_duty = 0;          break;
        case Direction::kReverse: ia_duty = 0;     ib_duty = duty;      break;
        case Direction::kStop:    ia_duty = 0;     ib_duty = 0;          break;
    }

    if (ia_gpio_ != GPIO_NUM_NC) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ia_channel_, ia_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ia_channel_);
    }
    if (ib_gpio_ != GPIO_NUM_NC) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ib_channel_, ib_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ib_channel_);
    }
}

void MotorController::Drive(Direction dir, int speed_percent) {
    if (!initialized_) return;
    if (speed_percent < 0)   speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;

    // Speed 0 in any direction is equivalent to coast stop — never leave
    // one side held high while the other floats, that would brake hard
    // and (worse) waste current.
    if (speed_percent == 0) dir = Direction::kStop;

    int duty = (kDutyMax * speed_percent) / 100;
    ApplyPwm(dir, duty);

    direction_ = dir;
    speed_percent_ = speed_percent;
    ESP_LOGI(TAG, "Drive: dir=%d speed=%d%% duty=%d",
             (int)direction_, speed_percent_, duty);
}

void MotorController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool(
        "self.motor.forward",
        "Drive the L9110S motor forward at the given speed (0-100 percent). "
        "Speed 0 coasts to a stop. Stays running until motor.stop or another "
        "direction command is issued.",
        PropertyList({Property("speed_percent", kPropertyTypeInteger, 0, 100)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int speed = properties["speed_percent"].value<int>();
            Drive(Direction::kForward, speed);
            return true;
        });

    mcp_server.AddTool(
        "self.motor.reverse",
        "Drive the L9110S motor in reverse at the given speed (0-100 percent). "
        "Speed 0 coasts to a stop.",
        PropertyList({Property("speed_percent", kPropertyTypeInteger, 0, 100)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int speed = properties["speed_percent"].value<int>();
            Drive(Direction::kReverse, speed);
            return true;
        });

    mcp_server.AddTool(
        "self.motor.stop",
        "Coast the L9110S motor to a stop by driving both inputs low.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            Stop();
            return true;
        });

    mcp_server.AddTool(
        "self.motor.get_state",
        "Report the current motor direction and speed.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            const char* dir_str = "stop";
            switch (direction_) {
                case Direction::kForward: dir_str = "forward"; break;
                case Direction::kReverse: dir_str = "reverse"; break;
                case Direction::kStop:    dir_str = "stop";    break;
            }
            char json[96];
            snprintf(json, sizeof(json),
                     "{\"direction\":\"%s\",\"speed_percent\":%d}",
                     dir_str, speed_percent_);
            return std::string(json);
        });
}