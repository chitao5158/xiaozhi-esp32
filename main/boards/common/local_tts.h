#ifndef __LOCAL_TTS_H__
#define __LOCAL_TTS_H__

#include <mutex>
#include <string>
#include <vector>

#include <esp_tts.h>

// Thin wrapper around esp_tts_chinese (xiaole voice) for synthesizing short
// Chinese utterances directly to PCM.
//
// The voice model (~3 MB) is loaded once from the `voice_data` partition
// (SubType 0x40, see partitions/v2/16m_custom_wakeword.csv) into SPIRAM.
// Use scripts/flash_voicedata.sh to write the .dat file there.
//
// Output is mono int16 PCM at the model's native sample rate (16 kHz for
// xiaole). Pass the result to AudioService::PlayRawPcm(), which resamples
// to the codec's output rate and queues it for playback.
class LocalTTS {
public:
    static LocalTTS& GetInstance();

    // Synthesizes UTF-8 Chinese text to PCM. Returns true on success, false
    // if the TTS engine is not initialized or parsing failed. The output
    // buffer is appended to `out_pcm` and `*sample_rate` is set on success.
    bool Synthesize(const std::string& text,
                     std::vector<int16_t>* out_pcm,
                     int* sample_rate);

private:
    LocalTTS();
    ~LocalTTS();
    LocalTTS(const LocalTTS&) = delete;
    LocalTTS& operator=(const LocalTTS&) = delete;

    bool initialized_ = false;
    esp_tts_voice_t* voice_ = nullptr;
    esp_tts_handle_t handle_ = nullptr;
    // Owning storage for the xiaole voice data. ~3 MB, lives in SPIRAM.
    // Must outlive the esp_tts_handle_t (which holds a pointer into it).
    void* voice_data_ = nullptr;
    std::mutex mutex_;
};

#endif // __LOCAL_TTS_H__