#include "a2dp_source_speaker.h"

#ifdef USE_A2DP_SOURCE

#include "esphome/core/log.h"

#include <esp_timer.h>

#include <cinttypes>
#include <cmath>
#include <cstring>

namespace esphome::a2dp_source {

ESPHOME_LOG_TAG(TAG, "a2dp_source.speaker");

static constexpr uint32_t SAMPLE_RATE = 44100;
static constexpr uint8_t BITS_PER_SAMPLE = 16;
static constexpr size_t STEREO_FRAME_BYTES = 2 * sizeof(int16_t);

// Same software volume curve as the I2S speaker: (0.0, 1.0] maps linearly to [-49, 0] dB, and 0.0 is silence
static constexpr float SOFTWARE_VOLUME_MIN_DB = -49.0f;
static constexpr uint32_t GAIN_RAMP_MS_PER_DB = 1;

void A2DPSourceSpeaker::setup() {
  const audio::AudioStreamInfo stereo(BITS_PER_SAMPLE, 2, SAMPLE_RATE);
  this->ring_buffer_ = ring_buffer::RingBuffer::create(stereo.ms_to_bytes(this->buffer_duration_ms_));
  if (this->ring_buffer_ == nullptr) {
    this->mark_failed(LOG_STR("Ring buffer allocation failed"));
    return;
  }

  this->set_volume(this->volume_);
  this->set_mute_state(this->mute_state_);
}

void A2DPSourceSpeaker::dump_config() {
  ESP_LOGCONFIG(TAG,
                "A2DP Source Speaker:\n"
                "  Buffer duration: %" PRIu32 " ms",
                this->buffer_duration_ms_);
}

void A2DPSourceSpeaker::loop() {
  portENTER_CRITICAL(&this->report_lock_);
  const uint32_t frames = this->unreported_frames_;
  const int64_t sent_us = this->unreported_us_;
  this->unreported_frames_ = 0;
  portEXIT_CRITICAL(&this->report_lock_);
  if (frames > 0) {
    this->audio_output_callback_(frames, sent_us);
  }

  const uint8_t commands = this->commands_.exchange(0);
  if (commands & COMMAND_STOP) {
    this->stop_();
  }
  if ((commands & COMMAND_START) && this->state_ == speaker::STATE_STOPPED) {
    this->ring_buffer_->reset();
    this->channels_ = this->audio_stream_info_.get_channels();
    this->finishing_ = false;
    this->state_ = speaker::STATE_STARTING;
    this->parent_->request_stream(true);
    if (this->parent_->is_streaming()) {
      this->set_running_(true);
    }
  }
  if ((commands & COMMAND_FINISH) &&
      (this->state_ == speaker::STATE_STARTING || this->state_ == speaker::STATE_RUNNING)) {
    this->finishing_ = true;
  }

  if (this->finishing_) {
    if (!this->parent_->is_connected()) {
      // Nothing can play the buffered audio, and waiting for a reconnect would hold the caller indefinitely
      this->stop_();
    } else if (this->state_ == speaker::STATE_RUNNING && !this->has_buffered_data()) {
      this->stop_();
    }
  }
}

void A2DPSourceSpeaker::request_(uint8_t set, uint8_t clear) {
  this->commands_.fetch_and(static_cast<uint8_t>(~clear));
  this->commands_.fetch_or(set);
}

void A2DPSourceSpeaker::start() {
  if (this->is_failed() || this->state_ == speaker::STATE_STARTING || this->state_ == speaker::STATE_RUNNING)
    return;
  this->request_(COMMAND_START, COMMAND_STOP);
}

void A2DPSourceSpeaker::stop() { this->request_(COMMAND_STOP, COMMAND_START | COMMAND_FINISH); }

void A2DPSourceSpeaker::finish() { this->request_(COMMAND_FINISH, 0); }

void A2DPSourceSpeaker::stop_() {
  this->finishing_ = false;
  if (this->state_ == speaker::STATE_STOPPED)
    return;
  this->set_running_(false);
  this->state_ = speaker::STATE_STOPPED;
  this->ring_buffer_->reset();
  // The hub keeps the stream open for its suspend timeout, sending silence, in case more audio follows
  this->parent_->request_stream(false);
}

void A2DPSourceSpeaker::on_stream_state(bool running) {
  if (running && this->state_ == speaker::STATE_STARTING) {
    this->set_running_(true);
  } else if (!running && this->state_ == speaker::STATE_RUNNING) {
    // Lost the stream (a disconnect or the remote suspended it); the buffered audio plays once it starts again
    this->set_running_(false);
  }
}

void A2DPSourceSpeaker::set_running_(bool running) {
  if (running) {
    // Starts at the live target instead of ramping from wherever the last stream left off
    this->post_software_gain_(0);
    this->state_ = speaker::STATE_RUNNING;
  } else if (this->state_ == speaker::STATE_RUNNING) {
    this->state_ = speaker::STATE_STARTING;
  }
  this->output_enabled_ = running && !this->pause_state_;
}

void A2DPSourceSpeaker::set_pause_state(bool pause_state) {
  this->pause_state_ = pause_state;
  this->output_enabled_ = this->state_ == speaker::STATE_RUNNING && !pause_state;
}

bool A2DPSourceSpeaker::format_supported_() const {
  const audio::AudioStreamInfo &info = this->audio_stream_info_;
  if (info.get_sample_rate() != SAMPLE_RATE || info.get_bits_per_sample() != BITS_PER_SAMPLE)
    return false;
  // read_stream() takes the channel count fixed when the speaker started
  if (this->state_ != speaker::STATE_STOPPED)
    return info.get_channels() == this->channels_;
  return info.get_channels() == 1 || info.get_channels() == 2;
}

size_t A2DPSourceSpeaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  if (this->is_failed()) {
    return 0;
  }

  if (!this->format_supported_()) {
    if (!this->format_mismatch_.exchange(true)) {
      const audio::AudioStreamInfo &info = this->audio_stream_info_;
      ESP_LOGE(TAG,
               "Audio format (%" PRIu32 " Hz, %u bits, %u channels) is not supported; Bluetooth needs 16 bits at "
               "44100 Hz, so use a resampler speaker",
               info.get_sample_rate(), info.get_bits_per_sample(), info.get_channels());
    }
    vTaskDelay(ticks_to_wait);
    return 0;
  }
  this->format_mismatch_ = false;

  if (this->state_ == speaker::STATE_STOPPED) {
    this->start();
  }
  if (this->state_ != speaker::STATE_RUNNING) {
    // Not streaming yet; wait so a pipeline task retrying play() does not spin
    vTaskDelay(ticks_to_wait);
    return 0;
  }

  return this->ring_buffer_->write_without_replacement(data, length, ticks_to_wait);
}

bool A2DPSourceSpeaker::has_buffered_data() const {
  return this->ring_buffer_ != nullptr && this->ring_buffer_->available() > 0;
}

void A2DPSourceSpeaker::read_stream(uint8_t *data, size_t length) {
  size_t filled = 0;
  if (this->output_enabled_) {
    const bool mono = this->channels_ == 1;
    const size_t frame_bytes = mono ? sizeof(int16_t) : STEREO_FRAME_BYTES;
    const size_t frames = std::min(length / STEREO_FRAME_BYTES, this->ring_buffer_->available() / frame_bytes);
    // Only whole frames, so a write still in progress cannot shift the channels
    const size_t read = this->ring_buffer_->read(data, frames * frame_bytes, 0);
    const size_t samples = read / sizeof(int16_t);

    this->gain_ramp_.process(data, sizeof(int16_t), samples);

    if (mono) {
      // Spread in place from the end, so no sample is overwritten before it is copied
      int16_t *pcm = reinterpret_cast<int16_t *>(data);
      for (size_t i = samples; i-- > 0;) {
        pcm[2 * i] = pcm[i];
        pcm[2 * i + 1] = pcm[i];
      }
      filled = read * 2;
    } else {
      filled = read;
    }

    if (filled > 0) {
      const int64_t now_us = esp_timer_get_time();
      portENTER_CRITICAL(&this->report_lock_);
      this->unreported_frames_ += filled / STEREO_FRAME_BYTES;
      this->unreported_us_ = now_us;
      portEXIT_CRITICAL(&this->report_lock_);
    }
  }
  if (filled < length) {
    std::memset(data + filled, 0, length - filled);
  }
}

void A2DPSourceSpeaker::set_volume(float volume) {
  speaker::Speaker::set_volume(volume);
  this->post_software_gain_(this->audio_stream_info_.ms_to_samples(GAIN_RAMP_MS_PER_DB));
}

void A2DPSourceSpeaker::set_mute_state(bool mute_state) {
  speaker::Speaker::set_mute_state(mute_state);
  this->post_software_gain_(this->audio_stream_info_.ms_to_samples(GAIN_RAMP_MS_PER_DB));
}

void A2DPSourceSpeaker::post_software_gain_(uint32_t rate_samples) {
#ifdef USE_AUDIO_DAC
  if (this->has_audio_dac_()) {
    return;
  }
#endif  // USE_AUDIO_DAC
  float target_db;
  if (this->is_silent_()) {
    target_db = -INFINITY;
  } else if (this->volume_ >= 1.0f) {
    target_db = 0.0f;
  } else {
    target_db = remap<float, float>(this->volume_, 0.0f, 1.0f, SOFTWARE_VOLUME_MIN_DB, 0.0f);
  }
  this->gain_ramp_.set_target_db_at_rate(target_db, rate_samples);
}

}  // namespace esphome::a2dp_source

#endif  // USE_A2DP_SOURCE
