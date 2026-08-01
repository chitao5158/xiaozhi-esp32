#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include "mcp_server.h"

#include <driver/gpio.h>
#include <driver/ledc.h>

// L9110S is a single H-bridge driver with two logic inputs (IA, IB).
//   forward : IA = PWM (speed), IB = LOW
//   reverse : IA = LOW,   IB = PWM (speed)
//   stop    : IA = LOW,   IB = LOW  (coast)
//
// This sign-magnitude scheme is the most common L9110S wiring — one PWM
// pin carries the duty cycle, the other is hard-low — and avoids the
// shoot-through risk of driving both inputs high.
//
// Two LEDC channels share one timer. Pass GPIO_NUM_NC to either pin to
// disable that side; if both pins are NC the controller is a no-op and
// no MCP tools that require motion are registered.
class MotorController {
public:
    enum class Direction : int {
        kStop    = 0,
        kForward = 1,
        kReverse = 2,
    };

    // Builds the controller and registers the MCP tools. Pass GPIO_NUM_NC
    // for either pin to skip PWM bring-up on that pin (useful if a board
    // only has the IA side wired).
    MotorController(gpio_num_t ia_gpio, gpio_num_t ib_gpio,
                    ledc_timer_t timer = LEDC_TIMER_1,
                    ledc_channel_t ia_channel = LEDC_CHANNEL_1,
                    ledc_channel_t ib_channel = LEDC_CHANNEL_2,
                    uint32_t pwm_freq_hz = 5000);

    // True when both pins are valid (NC disables the controller).
    bool IsInitialized() const { return initialized_; }

    // Drive the motor. `speed_percent` is 0..100. Direction is set by the
    // direction argument; speed of 0 in any direction coasts to a stop.
    void Drive(Direction dir, int speed_percent);

    void Stop() { Drive(Direction::kStop, 0); }

    Direction GetDirection() const { return direction_; }
    int GetSpeedPercent() const { return speed_percent_; }

private:
    gpio_num_t ia_gpio_;
    gpio_num_t ib_gpio_;
    ledc_timer_t timer_;
    ledc_channel_t ia_channel_;
    ledc_channel_t ib_channel_;
    uint32_t pwm_freq_hz_;

    bool initialized_ = false;
    Direction direction_ = Direction::kStop;
    int speed_percent_ = 0;

    static constexpr int kDutyResolutionBits = 10;   // matches PwmBacklight
    static constexpr int kDutyMax = (1 << kDutyResolutionBits) - 1;  // 1023

    void ApplyPwm(Direction dir, int duty);
    void RegisterMcpTools();
};

#endif // __MOTOR_CONTROLLER_H__