#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "temperature_controller.h"
#include "motor_controller.h"
#include "assets/lang_config.h"
#include <cmath>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include "command_server.h"
#include "local_tts.h"
#include "ir_controller.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardLCD"

// One-shot task that does the full announcement: synthesize the spoken
// sentence, push PCM to the audio service, wait for the playback queue to
// drain, then re-enable wake word detection. Runs OFF the main task so
// the state machine can keep processing events while TTS is busy.
//
// Previously this was split across two tasks (tts_synthesize_and_play +
// wait_then_rearm) but that opened a race: the second task could call
// WaitForPlaybackQueueEmpty() before the first task had pushed anything,
// and exit immediately — leaving the mic open while TTS was still running.
static void tts_announce_task(void* arg) {
    float celsius = *static_cast<float*>(arg);
    delete static_cast<float*>(arg);

    ESP_LOGI(TAG, "tts_announce: worker started on core %d, celsius=%.2f",
             xPortGetCoreID(), celsius);
    auto& app = Application::GetInstance();
    char lcd_msg[96];
    std::string spoken;
    if (std::isnan(celsius)) {
        snprintf(lcd_msg, sizeof(lcd_msg), "温度传感器读不到数据，请检查接线");
        spoken = "温度传感器读不到数据,请检查接线";
    } else {
        snprintf(lcd_msg, sizeof(lcd_msg), "现在室内温度为 %.1f 度", celsius);
        int int_part = (int)celsius;
        int dec_part = (int)((celsius - int_part) * 10.0f + 0.5f);
        if (dec_part >= 10) { dec_part = 0; int_part++; }
        if (int_part < 0) int_part = 0;
        if (int_part > 99) int_part = 99;

        static const char* digits_zh[] = {
            "零","一","二","三","四","五","六","七","八","九"};
        char num_buf[32];
        if (int_part < 10) {
            snprintf(num_buf, sizeof(num_buf), "%s", digits_zh[int_part]);
        } else if (int_part < 20) {
            snprintf(num_buf, sizeof(num_buf), "十%s",
                     int_part == 10 ? "" : digits_zh[int_part - 10]);
        } else {
            snprintf(num_buf, sizeof(num_buf), "%s十%s",
                     digits_zh[int_part / 10],
                     (int_part % 10) == 0 ? "" : digits_zh[int_part % 10]);
        }
        std::string num_str = num_buf;
        if (dec_part > 0) {
            num_str += "点";
            num_str += digits_zh[dec_part];
        }
        spoken = "现在室内温度为" + num_str + "度";
    }

    // Push the LCD message from the main task (display APIs aren't safe
    // from a worker).
    app.Schedule([lcd_msg]() {
        Application::GetInstance().Alert("提醒", lcd_msg, "happy", "");
    });
    ESP_LOGI(TAG, "tts_announce: LCD Alert scheduled, spoken='%s'", spoken.c_str());

    // Synthesize and push PCM. AudioService::PlayRawPcm resamples
    // 16→24 kHz and chunks into 60 ms frames.
    std::vector<int16_t> pcm;
    int tts_sample_rate = 0;
    ESP_LOGI(TAG, "tts_announce: calling LocalTTS::Synthesize...");
    bool ok = LocalTTS::GetInstance().Synthesize(spoken, &pcm, &tts_sample_rate);
    ESP_LOGI(TAG, "tts_announce: Synthesize returned ok=%d, frames=%d, sr=%d",
             ok, (int)pcm.size(), tts_sample_rate);
    if (!ok || pcm.empty()) {
        ESP_LOGE(TAG, "TTS synthesize failed (voice_data not flashed?)");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "tts_announce: calling PlayRawPcm %d frames...", (int)pcm.size());
    app.GetAudioService().PlayRawPcm(pcm.data(), pcm.size(), tts_sample_rate);
    ESP_LOGI(TAG, "tts_announce: PlayRawPcm returned, waiting for queue to drain");

    // Block until the AudioOutputTask has actually played all our PCM.
    // Doing this on the same task that pushed the PCM guarantees the queue
    // is non-empty when we start waiting.
    app.GetAudioService().WaitForPlaybackQueueEmpty();
    ESP_LOGI(TAG, "tts_announce: queue drained, sleeping 300ms to flush I2S");
    vTaskDelay(pdMS_TO_TICKS(300));

    // Re-enable wake word detection on the main task (audio service APIs
    // are not safe to call directly from a worker).
    app.Schedule([]() {
        Application::GetInstance().GetAudioService().EnableWakeWordDetection(true);
        ESP_LOGI(TAG, "tts_announce: wake word re-enabled, ready for '嗨你好'");
    });
    ESP_LOGI(TAG, "tts_announce: worker exiting");
    vTaskDelete(nullptr);
}

class CompactWifiBoardLCD : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
        // W104 / KY-028 module: 47kΩ series + 10kΩ NTC, B = 3950
        static TemperatureController temperature(TEMP_SENSOR_GPIO, TemperatureSensorConfig{
            .r_fixed_ohms = 47000.0f,
            .r25_ohms     = 10000.0f,
            .b_coeff      = 3950.0f,
            .vcc_volts    = 3.3f,
            .sample_count = 8,
        });
        // L9110S single-channel motor driver on MOTOR_IA_GPIO / MOTOR_IB_GPIO.
        // Defaults use LEDC_TIMER_1 + channels 1/2 so they don't collide with
        // PwmBacklight on LEDC_TIMER_0/CHANNEL_0.
        static MotorController motor(MOTOR_IA_GPIO, MOTOR_IB_GPIO,
                                     LEDC_TIMER_1, LEDC_CHANNEL_1, LEDC_CHANNEL_2);

#if CONFIG_USE_IR_CONTROLLER
        // YS-IRTM UART bridge module handles IR RX + TX + 38 kHz modulation.
        // ESP32 talks to it over UART2: TX→GPIO12, RX→GPIO2.
        static IrController& ir = IrController::GetInstance();
        static int ir_init_once = (ir.Configure(IR_UART_TX_GPIO, IR_UART_RX_GPIO), 0);
        (void)ir_init_once;
#endif

        // When MCP self.temperature.schedule_announcement fires, this callback
        // runs on the main task via app.Schedule(). We do only the FAST
        // setup here (state transition, mic mute) and immediately hand the
        // heavy work — TTS synthesis + audio push + drain wait — off to a
        // dedicated worker task. If we did TTS here on the main task, the
        // state machine would be blocked from processing the StopListening
        // event we queued, and the audio channel would still be open when
        // the announcement started (causing the server to STT our own
        // speaker output and ResetDecoder away the TTS audio).
        temperature.SetAnnouncementCallback([](float celsius) {
            auto& app = Application::GetInstance();
            app.Schedule([celsius, &app]() {
                ESP_LOGI(TAG, "announce_cb: main task running, state=%d, celsius=%.2f",
                         (int)app.GetDeviceState(), celsius);
                // 1. Hard-mute the input FIRST. This is the only thing that
                //    actually matters for preventing the speaker→mic echo
                //    during TTS playback. Done before any blocking wait.
                app.GetAudioService().EnableVoiceProcessing(false);
                app.GetAudioService().EnableWakeWordDetection(false);
                // 2. Request the state machine to drop back to idle. We can't
                //    busy-wait for the transition here, because the main
                //    task is currently inside this lambda and would never
                //    get a chance to run HandleStopListeningEvent. So we
                //    just kick it off and let the event loop process it on
                //    the next iteration.
                if (app.GetDeviceState() == kDeviceStateListening) {
                    ESP_LOGI(TAG, "announce_cb: requesting state=idle");
                    app.StopListening();
                }
                // 3. Give the main task a chance to process the state
                //    transition (and any other queued events) before we
                //    start the TTS worker. vTaskDelay yields this task, so
                //    the main task actually runs during this wait.
                vTaskDelay(pdMS_TO_TICKS(500));

                // 4. Hand off to a worker task so TTS synthesis doesn't block
                //    the main task (and the state machine). Pin to core 1 so
                //    it doesn't starve the AFE worker (wake word + voice
                //    processing) on core 0 — esp_tts_chinese is CPU-heavy
                //    (~1-2 s) and would otherwise leave the AFE feed buffer
                //    overflowing for the whole announcement window.
                ESP_LOGI(TAG, "announce_cb: spawning tts_announce worker, state now=%d",
                         (int)app.GetDeviceState());
                auto* celsius_heap = new float(celsius);
                xTaskCreatePinnedToCore(tts_announce_task, "tts_announce",
                                         8192, celsius_heap, 1, nullptr, 1);
            });
        });
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeTools();
        // Silence the AFE library's "Ringbuffer of AFE(FEED) is full"
        // warning. The buffer genuinely does fill when the codec is awake
        // but the consumer (which runs on the audio input task at lower
        // priority) hasn't fetched in time — for example, right after boot
        // before the first wake-word / voice-processing consumer has been
        // set up. The warning is informational, not fatal: AFE drops
        // incoming samples but recovers on the next feed. Quieting it here
        // keeps the serial log usable.
        esp_log_level_set("AFE", ESP_LOG_ERROR);
        // Start the HTTP command server for remote triggering. We don't call
        // Start() here directly because the lwIP TCP/IP mailbox isn't ready
        // yet (assert tcpip_send_msg_wait_sem "Invalid mbox"). Instead
        // CommandServer::Start() registers an IP_EVENT hook that fires the
        // moment the device gets an IP — that's the safe time to bring
        // up the listening socket.
        //   GET /api/temperature  → JSON with current NTC reading
        //   GET /api/info         → JSON with device info
        // Port 8080 instead of 80 — port 80 is privileged and often gets
        // dropped by client-side firewalls / mDNS / captive portals, while
        // 8080 is in the unprivileged IANA range and rarely filtered.
        CommandServer::GetInstance().Start(8080);
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
