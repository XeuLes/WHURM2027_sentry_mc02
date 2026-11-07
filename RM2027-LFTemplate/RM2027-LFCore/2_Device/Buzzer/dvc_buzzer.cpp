#include "dvc_buzzer.hpp"

#include <cmath>

#include "tim.h"

namespace {
constexpr uint32_t BUZZER_TIMER_TICK_HZ = 1000000U;

uint32_t timerClock()
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_APB1_DIV1) {
        pclk1 *= 2U;
    }
    return pclk1;
}

float constrain(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

float constrainFrequency(float frequency_hz)
{
    if (!std::isfinite(frequency_hz)) {
        return BUZZER_DEFAULT_FREQUENCY_HZ;
    }
    return constrain(frequency_hz, BUZZER_MIN_FREQUENCY_HZ, BUZZER_MAX_FREQUENCY_HZ);
}

float constrainVolume(float volume)
{
    if (!std::isfinite(volume)) {
        return 0.0f;
    }
    return constrain(volume, 0.0f, 1.0f);
}
} // namespace

Buzzer buzzer;

bool Buzzer::init()
{
    if (initialized_) {
        return true;
    }

    timer_clock_hz_ = timerClock();
    const uint32_t prescaler = timer_clock_hz_ / BUZZER_TIMER_TICK_HZ;
    if (prescaler == 0U) {
        return false;
    }

    __HAL_TIM_SET_PRESCALER(&htim12, prescaler - 1U);
    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK) {
        return false;
    }

    initialized_ = true;
    playing_ = false;
    sequence_active_ = false;
    apply();
    return true;
}

void Buzzer::setTone(float frequency_hz, float volume)
{
    sequence_active_ = false;
    frequency_hz_ = constrainFrequency(frequency_hz);
    volume_ = constrainVolume(volume);
    playing_ = volume_ > 0.0f;
    apply();
}

void Buzzer::setVolume(float volume)
{
    sequence_active_ = false;
    volume_ = constrainVolume(volume);
    playing_ = volume_ > 0.0f;
    apply();
}

void Buzzer::on()
{
    sequence_active_ = false;
    if (volume_ <= 0.0f) {
        volume_ = BUZZER_DEFAULT_VOLUME;
    }
    playing_ = true;
    apply();
}

void Buzzer::off()
{
    sequence_active_ = false;
    playing_ = false;
    if (initialized_) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    }
}

void Buzzer::playBeeps(uint8_t count)
{
    sequence_active_ = false;
    if (!initialized_ || (count == 0U)) {
        playing_ = false;
        apply();
        return;
    }

    beeps_remaining_ = count;
    beep_on_phase_ = true;
    playing_ = true;
    sequence_active_ = true;
    apply();
    beep_deadline_ms_ = HAL_GetTick() + BUZZER_BEEP_ON_TIME_MS;
}

void Buzzer::update(uint32_t now_ms)
{
    if (!initialized_ || !sequence_active_) {
        return;
    }
    if (static_cast<int32_t>(now_ms - beep_deadline_ms_) < 0) {
        return;
    }

    if (beep_on_phase_) {
        playing_ = false;
        apply();
        --beeps_remaining_;
        if (beeps_remaining_ == 0U) {
            sequence_active_ = false;
            return;
        }
        beep_on_phase_ = false;
        beep_deadline_ms_ = now_ms + BUZZER_BEEP_OFF_TIME_MS;
        return;
    }

    beep_on_phase_ = true;
    playing_ = true;
    apply();
    beep_deadline_ms_ = now_ms + BUZZER_BEEP_ON_TIME_MS;
}

void Buzzer::apply()
{
    if (!initialized_) {
        return;
    }
    if (!playing_ || (volume_ <= 0.0f)) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
        return;
    }

    const uint32_t period = static_cast<uint32_t>(
        static_cast<float>(BUZZER_TIMER_TICK_HZ) / frequency_hz_ + 0.5f);
    uint32_t compare = static_cast<uint32_t>(
        static_cast<float>(period) * volume_ * 0.5f + 0.5f);
    if (compare > period) {
        compare = period;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim12, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, compare);
}
