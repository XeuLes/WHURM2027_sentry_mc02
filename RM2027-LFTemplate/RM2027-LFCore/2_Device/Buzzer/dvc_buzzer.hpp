#ifndef DVC_BUZZER_HPP
#define DVC_BUZZER_HPP

#include <cstdint>

/* MC02 onboard buzzer: PB15 / TIM12_CH2. */
constexpr float BUZZER_MIN_FREQUENCY_HZ = 196.0f;
constexpr float BUZZER_MAX_FREQUENCY_HZ = 5000.0f;
constexpr float BUZZER_DEFAULT_FREQUENCY_HZ = 4000.0f;
constexpr float BUZZER_DEFAULT_VOLUME = 1.0f;
constexpr uint32_t BUZZER_BEEP_ON_TIME_MS = 100U;
constexpr uint32_t BUZZER_BEEP_OFF_TIME_MS = 100U;

class Buzzer {
public:
    /** @brief Initialize TIM12 PWM. Repeated calls are harmless. */
    bool init();

    /** @brief Set frequency and volume; a non-zero volume starts playback. */
    void setTone(float frequency_hz, float volume);

    /** @brief Set volume while retaining the current frequency. */
    void setVolume(float volume);

    /** @brief Start playback with the current tone. */
    void on();

    /** @brief Stop PWM output while retaining tone settings. */
    void off();

    /** @brief Start a non-blocking sequence of equally spaced beeps. */
    void playBeeps(uint8_t count);

    /** @brief Advance an active beep sequence; call once per control cycle. */
    void update(uint32_t now_ms);

    bool isPlaying() const { return initialized_ && playing_; }
    float frequency() const { return frequency_hz_; }
    float volume() const { return volume_; }

private:
    void apply();

    float frequency_hz_ = BUZZER_DEFAULT_FREQUENCY_HZ;
    float volume_ = BUZZER_DEFAULT_VOLUME;
    uint32_t timer_clock_hz_ = 0U;
    uint32_t beep_deadline_ms_ = 0U;
    uint8_t beeps_remaining_ = 0U;
    bool initialized_ = false;
    bool playing_ = false;
    bool sequence_active_ = false;
    bool beep_on_phase_ = false;
};

extern Buzzer buzzer;

#endif /* DVC_BUZZER_HPP */
