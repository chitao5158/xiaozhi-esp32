#include "local_tts.h"

#include <esp_log.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#include <cstring>

#include "esp_tts_voice_template.h"
#include "esp_tts_voice_xiaole.h"

#define TAG "LocalTTS"

LocalTTS& LocalTTS::GetInstance() {
    static LocalTTS instance;
    return instance;
}

LocalTTS::LocalTTS() {
    // Look up the voice_data partition. We use SubType 0x40 (custom data),
    // see partitions/v2/16m_custom_wakeword.csv.
    const esp_partition_t* part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        static_cast<esp_partition_subtype_t>(0x40),
        "voice_data");
    if (part == nullptr) {
        ESP_LOGE(TAG, "voice_data partition not found (SubType=0x40). "
                      "Flash the xiaole .dat file with scripts/flash_voicedata.sh");
        return;
    }
    ESP_LOGI(TAG, "voice_data partition: offset=0x%lx size=0x%lx (%lu KB)",
             (unsigned long)part->address,
             (unsigned long)part->size,
             (unsigned long)(part->size / 1024));

    // ~3 MB doesn't fit in internal SRAM, so allocate in SPIRAM. We keep the
    // buffer alive for the lifetime of the TTS handle.
    voice_data_ = heap_caps_malloc(part->size, MALLOC_CAP_SPIRAM);
    if (voice_data_ == nullptr) {
        ESP_LOGE(TAG, "heap_caps_malloc(%lu) failed for voice_data",
                 (unsigned long)part->size);
        return;
    }
    esp_err_t err = esp_partition_read(part, 0, voice_data_, part->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_partition_read failed: %s", esp_err_to_name(err));
        heap_caps_free(voice_data_);
        voice_data_ = nullptr;
        return;
    }

    // Sanity check: a freshly-erased partition reads back as 0xFF, but the
    // xiaole .dat always starts with the magic string "xiaole_YYYYMMDD\0".
    // Without this check we would happily hand garbage to esp_tts_create and
    // crash later inside esp_tts_parse_chinese with a LoadProhibited at a
    // bogus address.
    static const char kMagic[] = "xiaole_";
    if (memcmp(voice_data_, kMagic, sizeof(kMagic) - 1) != 0) {
        ESP_LOGE(TAG, "voice_data partition appears empty (no xiaole magic). "
                      "Run scripts/flash_voicedata.sh to flash the .dat file.");
        heap_caps_free(voice_data_);
        voice_data_ = nullptr;
        return;
    }

    // Init voice set from the xiaole template, pointing at our loaded data.
    voice_ = esp_tts_voice_set_init(&esp_tts_voice_xiaole, voice_data_);
    if (voice_ == nullptr) {
        ESP_LOGE(TAG, "esp_tts_voice_set_init failed");
        heap_caps_free(voice_data_);
        voice_data_ = nullptr;
        return;
    }
    handle_ = esp_tts_create(voice_);
    if (handle_ == nullptr) {
        ESP_LOGE(TAG, "esp_tts_create failed");
        esp_tts_voice_set_free(voice_);
        voice_ = nullptr;
        heap_caps_free(voice_data_);
        voice_data_ = nullptr;
        return;
    }
    initialized_ = true;
    ESP_LOGI(TAG, "LocalTTS ready (sample_rate=%d Hz, voice_data in SPIRAM)",
             voice_->sample_rate);
}

LocalTTS::~LocalTTS() {
    if (handle_) {
        esp_tts_destroy(handle_);
        handle_ = nullptr;
    }
    if (voice_) {
        esp_tts_voice_set_free(voice_);
        voice_ = nullptr;
    }
    if (voice_data_) {
        heap_caps_free(voice_data_);
        voice_data_ = nullptr;
    }
}

bool LocalTTS::Synthesize(const std::string& text,
                           std::vector<int16_t>* out_pcm,
                           int* sample_rate) {
    if (!initialized_) {
        ESP_LOGE(TAG, "TTS not initialized");
        return false;
    }
    if (out_pcm == nullptr || sample_rate == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);

    if (esp_tts_parse_chinese(handle_, text.c_str()) != 1) {
        ESP_LOGE(TAG, "esp_tts_parse_chinese failed for: %s", text.c_str());
        esp_tts_stream_reset(handle_);
        return false;
    }

    // Drain the stream into our buffer. esp_tts_stream_play returns chunks of
    // int16 PCM frames until len == 0.
    int speed = 0;  // 0 = default/native speed
    while (true) {
        int chunk_frames = 0;
        const int16_t* chunk = esp_tts_stream_play(handle_, &chunk_frames, speed);
        if (chunk_frames <= 0) {
            break;
        }
        out_pcm->insert(out_pcm->end(), chunk, chunk + chunk_frames);
    }
    esp_tts_stream_reset(handle_);

    *sample_rate = voice_->sample_rate;
    return !out_pcm->empty();
}