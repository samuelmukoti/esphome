#pragma once

#include "esphome/core/defines.h"

#ifdef USE_A2DP_SOURCE

#include "esphome/components/a2dp_source/a2dp_source.h"
#include "esphome/components/audio/audio.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/components/speaker/speaker.h"

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <gain.h>  // esp-audio-libs

#include <algorithm>
#include <atomic>
#include <memory>

namespace esphome::a2dp_source {

/* Speaker that plays through the A2DP source hub.
 *  - play() writes to a ring buffer allocated in setup(); the Bluetooth task drains it in read_stream() and fills any
 *    shortfall with silence, so the stream keeps running through gaps.
 *  - Takes 16-bit audio at 44.1 kHz, mono or stereo; mono is copied to both channels. Use a resampler speaker in front
 *    of it for other formats.
 *  - Volume and mute are applied in software with a gain ramp.
 *  - start(), stop() and finish() may be called from any task; loop() acts on them.
 */
class A2DPSourceSpeaker : public speaker::Speaker, public Component, public Parented<A2DPSource>, public A2DPStream {
 public:
  A2DPSourceSpeaker(uint32_t buffer_duration_ms, uint8_t channels) : buffer_duration_ms_(buffer_duration_ms) {
    this->audio_stream_info_ = audio::AudioStreamInfo(16, channels, 44100);
  }

  void setup() override;
  void loop() override;
  void dump_config() override;

  void start() override;
  void stop() override;
  void finish() override;

  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  size_t play(const uint8_t *data, size_t length) override { return this->play(data, length, 0); }
  bool has_buffered_data() const override;

  void set_pause_state(bool pause_state) override;
  bool get_pause_state() const override { return this->pause_state_; }

  void set_volume(float volume) override;
  void set_mute_state(bool mute_state) override;

  void read_stream(uint8_t *data, size_t length) override;
  void on_stream_state(bool running) override;

 protected:
  static constexpr uint8_t COMMAND_START = 1 << 0;
  static constexpr uint8_t COMMAND_STOP = 1 << 1;
  static constexpr uint8_t COMMAND_FINISH = 1 << 2;

  void request_(uint8_t set, uint8_t clear);
  void stop_();
  void set_running_(bool running);
  /// @brief Posts the gain ramp target for the current volume and mute state. Main loop only.
  void post_software_gain_(uint32_t rate_samples);
  bool format_supported_() const;

  uint32_t buffer_duration_ms_;
  std::unique_ptr<ring_buffer::RingBuffer> ring_buffer_;

  // Posted by the main loop, processed on the Bluetooth task
  esp_audio_libs::gain::GainRamp gain_ramp_;

  std::atomic<uint8_t> commands_{0};
  // Read by the Bluetooth task: whether to drain the ring buffer, and how many channels it holds
  std::atomic<bool> output_enabled_{false};
  std::atomic<uint8_t> channels_{2};
  // Logs a rejected format once
  std::atomic<bool> format_mismatch_{false};

  // Frames sent and when, collected on the Bluetooth task and reported from loop()
  portMUX_TYPE report_lock_ = portMUX_INITIALIZER_UNLOCKED;
  uint32_t unreported_frames_{0};
  int64_t unreported_us_{0};

  bool pause_state_{false};
  bool finishing_{false};
};

}  // namespace esphome::a2dp_source

#endif  // USE_A2DP_SOURCE
