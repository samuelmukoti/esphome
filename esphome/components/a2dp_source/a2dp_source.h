#pragma once

#include "esphome/core/defines.h"

#ifdef USE_A2DP_SOURCE

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include <esp_a2dp_api.h>
#include <esp_gap_bt_api.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <atomic>
#include <cstdint>

namespace esphome::a2dp_source {

/* Streams audio to a Bluetooth speaker or headphones over Classic Bluetooth (A2DP source role). Only the original
 * ESP32 has a Classic Bluetooth radio.
 *  - The hub owns the Bluetooth stack: it finds the remote device (by address, or by name with a discovery scan whose
 *    result is cached in flash), connects, reconnects after a drop, and starts and suspends the media stream.
 *  - The Bluetooth stack runs its callbacks on its own task. They only post events to a queue that loop() handles, so
 *    every state change happens on the main loop. The one exception is the audio data callback, which hands the
 *    buffer straight to the A2DPStream (the speaker platform) to fill.
 *  - The built-in SBC encoder takes 16-bit stereo PCM at 44.1 kHz.
 */

/// The audio side of the stream, implemented by the speaker platform.
class A2DPStream {
 public:
  /// @brief Fills the buffer with exactly length bytes of 16-bit stereo 44.1 kHz PCM. Called on the Bluetooth task
  /// while the media stream runs, so it must not block.
  virtual void read_stream(uint8_t *data, size_t length) = 0;

  /// @brief Called from loop() when the media stream starts sending audio, or stops.
  virtual void on_stream_state(bool running) = 0;
};

enum class ConnectionState : uint8_t {
  CONNECTION_STATE_IDLE = 0,
  CONNECTION_STATE_DISCOVERING,
  CONNECTION_STATE_CONNECTING,
  CONNECTION_STATE_CONNECTED,
};

enum class MediaState : uint8_t {
  MEDIA_STATE_IDLE = 0,
  MEDIA_STATE_CHECKING,  // waiting for the answer to "is the remote ready for a stream"
  MEDIA_STATE_STARTING,
  MEDIA_STATE_STARTED,
  MEDIA_STATE_SUSPENDING,
};

class A2DPSource : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  void set_device_name(const char *name) { this->device_name_ = name; }
  void set_remote_address(uint64_t address);
  void set_remote_name(const char *name) { this->remote_name_ = name; }
  void set_pin(const char *pin) { this->pin_ = pin; }
  void set_auto_connect(bool auto_connect) { this->auto_connect_ = auto_connect; }
  void set_reconnect_interval(uint32_t ms) { this->reconnect_interval_ms_ = ms; }
  void set_suspend_timeout(uint32_t ms) { this->suspend_timeout_ms_ = ms; }
  void set_stream(A2DPStream *stream) { this->stream_ = stream; }

  /// @brief Connects to the remote device now and keeps reconnecting after a drop.
  void connect();
  /// @brief Disconnects and stops reconnecting until connect() is called.
  void disconnect();

  bool is_connected() const { return this->connection_state_ == ConnectionState::CONNECTION_STATE_CONNECTED; }
  bool is_streaming() const { return this->media_state_ == MediaState::MEDIA_STATE_STARTED; }

  /// @brief Asks for the media stream to run (the speaker has audio) or to be released. A released stream is
  /// suspended after the suspend timeout, so short gaps between tracks do not restart it. Main loop only.
  void request_stream(bool requested) { this->stream_requested_ = requested; }

  template<typename F> void add_on_connect_callback(F &&callback) {
    this->connect_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_disconnect_callback(F &&callback) {
    this->disconnect_callback_.add(std::forward<F>(callback));
  }

 protected:
  enum class EventType : uint8_t {
    EVENT_TYPE_DEVICE_FOUND,
    EVENT_TYPE_DISCOVERY_STOPPED,
    EVENT_TYPE_CONNECTION_STATE,
    EVENT_TYPE_AUDIO_STATE,
    EVENT_TYPE_MEDIA_CTRL_ACK,
  };

  /// A Bluetooth callback, copied to the main loop.
  struct Event {
    EventType type;
    uint8_t state;   // esp_a2d_connection_state_t, esp_a2d_audio_state_t or esp_a2d_media_ctrl_t
    uint8_t status;  // esp_a2d_media_ctrl_ack_t
    esp_bd_addr_t address;
  };

  struct SavedAddress {
    esp_bd_addr_t address;
  };

  static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
  static void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
  static int32_t data_callback(uint8_t *data, int32_t length);

  /// @brief Bluetooth task: matches a discovery result against remote_name_.
  void handle_discovery_result_(esp_bt_gap_cb_param_t *param);
  /// @brief Bluetooth task: answers a legacy pairing PIN request.
  void handle_pin_request_(esp_bt_gap_cb_param_t *param);
  void post_event_(const Event &event);

  esp_err_t init_bluetooth_();
  void handle_event_(const Event &event, uint32_t now);
  void start_connection_(uint32_t now);
  void connect_to_address_(uint32_t now);
  void update_media_(uint32_t now);
  void send_media_command_(esp_a2d_media_ctrl_t command, MediaState next_state, uint32_t now);
  void set_media_state_(MediaState state);
  void update_scan_mode_();

  const char *device_name_{nullptr};
  const char *remote_name_{nullptr};
  const char *pin_{nullptr};
  A2DPStream *stream_{nullptr};

  QueueHandle_t event_queue_{nullptr};
  ESPPreferenceObject address_pref_;

  esp_bd_addr_t address_{};
  bool has_address_{false};
  // Set when the address came from the cache and the last attempt failed, so the next attempt scans for the name
  bool discovery_needed_{false};
  bool device_found_{false};
  bool auto_connect_{true};
  bool attempted_{false};
  bool stream_requested_{false};
  bool release_pending_{false};

  ConnectionState connection_state_{ConnectionState::CONNECTION_STATE_IDLE};
  MediaState media_state_{MediaState::MEDIA_STATE_IDLE};

  uint32_t reconnect_interval_ms_{15000};
  uint32_t suspend_timeout_ms_{10000};
  uint32_t state_since_ms_{0};
  uint32_t media_command_ms_{0};
  uint32_t release_ms_{0};

  // Written by the Bluetooth task, read by loop()
  std::atomic<bool> event_dropped_{false};

  LazyCallbackManager<void()> connect_callback_;
  LazyCallbackManager<void()> disconnect_callback_;
};

}  // namespace esphome::a2dp_source

#endif  // USE_A2DP_SOURCE
