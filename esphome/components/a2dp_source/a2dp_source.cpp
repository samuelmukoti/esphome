#include "a2dp_source.h"

#ifdef USE_A2DP_SOURCE

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <esp_bt.h>
#include <esp_bt_main.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::a2dp_source {

ESPHOME_LOG_TAG(TAG, "a2dp_source");

static constexpr size_t EVENT_QUEUE_SIZE = 16;
// Inquiry length in units of 1.28 s
static constexpr uint8_t DISCOVERY_LENGTH = 10;
// The stack reports a failed page itself; these only recover from a lost callback
static constexpr uint32_t DISCOVERY_TIMEOUT_MS = 30000;
static constexpr uint32_t CONNECT_TIMEOUT_MS = 30000;
static constexpr uint32_t MEDIA_COMMAND_TIMEOUT_MS = 5000;
// Gap before asking a remote that refused to start the stream again
static constexpr uint32_t MEDIA_RETRY_MS = 1000;

// The stack's C callbacks carry no context pointer
A2DPSource *global_a2dp_source = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void A2DPSource::set_remote_address(uint64_t address) {
  // esp_bd_addr_t holds the most significant byte first, the order the address is written in
  for (size_t i = 0; i < ESP_BD_ADDR_LEN; i++) {
    this->address_[i] = static_cast<uint8_t>(address >> (8 * (ESP_BD_ADDR_LEN - 1 - i)));
  }
  this->has_address_ = true;
}

void A2DPSource::setup() {
  global_a2dp_source = this;

  this->event_queue_ = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(Event));
  if (this->event_queue_ == nullptr) {
    this->mark_failed(LOG_STR("Event queue allocation failed"));
    return;
  }

  if (this->remote_name_ != nullptr) {
    // Keyed by the name, so changing remote_name ignores the address found for the old one
    this->address_pref_ = global_preferences->make_preference<SavedAddress>(fnv1_hash(this->remote_name_));
    SavedAddress saved{};
    if (this->address_pref_.load(&saved)) {
      std::memcpy(this->address_, saved.address, ESP_BD_ADDR_LEN);
      this->has_address_ = true;
    }
  }

  const esp_err_t err = this->init_bluetooth_();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Bluetooth init failed: %s", esp_err_to_name(err));
    this->mark_failed(LOG_STR("Bluetooth init failed"));
    return;
  }
  this->update_scan_mode_();
}

esp_err_t A2DPSource::init_bluetooth_() {
  esp_err_t err;
  // Only Classic Bluetooth is used, so hand the BLE controller memory back to the heap
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&cfg)) != ESP_OK)
      return err;
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
      return err;
  }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
    if ((err = esp_bluedroid_init()) != ESP_OK)
      return err;
  }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
    if ((err = esp_bluedroid_enable()) != ESP_OK)
      return err;
  }

  if ((err = esp_bt_gap_set_device_name(this->device_name_)) != ESP_OK)
    return err;
  if ((err = esp_bt_gap_register_callback(A2DPSource::gap_callback)) != ESP_OK)
    return err;

  // Secure Simple Pairing without a display or keyboard ("just works"); speakers almost always use it
  esp_bt_io_cap_t io_capability = ESP_BT_IO_CAP_NONE;
  if ((err = esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &io_capability, sizeof(io_capability))) != ESP_OK)
    return err;
  // Older devices use legacy pairing; gap_callback answers the PIN request with the configured PIN
  esp_bt_pin_code_t unused_pin{};
  if ((err = esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, unused_pin)) != ESP_OK)
    return err;

  if ((err = esp_a2d_register_callback(A2DPSource::a2dp_callback)) != ESP_OK)
    return err;
  if ((err = esp_a2d_source_register_data_callback(A2DPSource::data_callback)) != ESP_OK)
    return err;
  return esp_a2d_source_init();
}

void A2DPSource::dump_config() {
  char address[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  ESP_LOGCONFIG(TAG,
                "A2DP Source:\n"
                "  Device name: %s\n"
                "  Auto connect: %s\n"
                "  Reconnect interval: %" PRIu32 " ms\n"
                "  Suspend timeout: %" PRIu32 " ms",
                this->device_name_, YESNO(this->auto_connect_), this->reconnect_interval_ms_,
                this->suspend_timeout_ms_);
  if (this->remote_name_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Remote name: %s", this->remote_name_);
  }
  if (this->has_address_) {
    ESP_LOGCONFIG(TAG, "  Remote address: %s", format_mac_addr_upper(this->address_, address));
  }
}

void A2DPSource::loop() {
  const uint32_t now = App.get_loop_component_start_time();

  Event event;
  while (xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
    this->handle_event_(event, now);
  }
  if (this->event_dropped_.exchange(false)) {
    ESP_LOGW(TAG, "Bluetooth event queue full; an event was dropped");
  }

  switch (this->connection_state_) {
    case ConnectionState::CONNECTION_STATE_IDLE:
      if (this->auto_connect_ && (!this->attempted_ || now - this->state_since_ms_ >= this->reconnect_interval_ms_)) {
        this->start_connection_(now);
      }
      break;
    case ConnectionState::CONNECTION_STATE_DISCOVERING:
      if (now - this->state_since_ms_ >= DISCOVERY_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Discovery timed out");
        esp_bt_gap_cancel_discovery();
        this->connection_state_ = ConnectionState::CONNECTION_STATE_IDLE;
        this->state_since_ms_ = now;
      }
      break;
    case ConnectionState::CONNECTION_STATE_CONNECTING:
      if (now - this->state_since_ms_ >= CONNECT_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Connection timed out");
        esp_a2d_source_disconnect(this->address_);
        this->connection_state_ = ConnectionState::CONNECTION_STATE_IDLE;
        this->state_since_ms_ = now;
        this->discovery_needed_ = this->remote_name_ != nullptr;
      }
      break;
    case ConnectionState::CONNECTION_STATE_CONNECTED:
      this->update_media_(now);
      break;
  }
}

void A2DPSource::start_connection_(uint32_t now) {
  this->attempted_ = true;
  this->state_since_ms_ = now;
  if (this->has_address_ && !this->discovery_needed_) {
    this->connect_to_address_(now);
    return;
  }

  ESP_LOGD(TAG, "Searching for '%s'", this->remote_name_);
  this->device_found_ = false;
  const esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, DISCOVERY_LENGTH, 0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Discovery start failed: %s", esp_err_to_name(err));
    return;
  }
  this->connection_state_ = ConnectionState::CONNECTION_STATE_DISCOVERING;
}

void A2DPSource::connect_to_address_(uint32_t now) {
  char address[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  ESP_LOGD(TAG, "Connecting to %s", format_mac_addr_upper(this->address_, address));
  this->state_since_ms_ = now;
  const esp_err_t err = esp_a2d_source_connect(this->address_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Connect failed: %s", esp_err_to_name(err));
    this->connection_state_ = ConnectionState::CONNECTION_STATE_IDLE;
    return;
  }
  this->connection_state_ = ConnectionState::CONNECTION_STATE_CONNECTING;
}

void A2DPSource::connect() {
  this->auto_connect_ = true;
  if (this->connection_state_ == ConnectionState::CONNECTION_STATE_IDLE) {
    // The next loop() tries at once instead of waiting out the reconnect interval
    this->attempted_ = false;
  }
  this->update_scan_mode_();
}

void A2DPSource::disconnect() {
  this->auto_connect_ = false;
  switch (this->connection_state_) {
    case ConnectionState::CONNECTION_STATE_DISCOVERING:
      esp_bt_gap_cancel_discovery();
      this->connection_state_ = ConnectionState::CONNECTION_STATE_IDLE;
      break;
    case ConnectionState::CONNECTION_STATE_CONNECTING:
    case ConnectionState::CONNECTION_STATE_CONNECTED:
      // The connection state event finishes the change to idle
      esp_a2d_source_disconnect(this->address_);
      break;
    case ConnectionState::CONNECTION_STATE_IDLE:
      break;
  }
  this->update_scan_mode_();
}

void A2DPSource::update_scan_mode_() {
  if (this->is_failed())
    return;
  // Connectable lets a paired speaker reconnect on its own when it powers on. Nothing needs to discover a source.
  const bool connectable = this->auto_connect_ && !this->is_connected();
  esp_bt_gap_set_scan_mode(connectable ? ESP_BT_CONNECTABLE : ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

void A2DPSource::handle_event_(const Event &event, uint32_t now) {
  char address[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  switch (event.type) {
    case EventType::EVENT_TYPE_DEVICE_FOUND:
      if (this->connection_state_ != ConnectionState::CONNECTION_STATE_DISCOVERING || this->device_found_)
        break;
      this->device_found_ = true;
      std::memcpy(this->address_, event.address, ESP_BD_ADDR_LEN);
      this->has_address_ = true;
      ESP_LOGI(TAG, "Found '%s' at %s", this->remote_name_, format_mac_addr_upper(this->address_, address));
      {
        SavedAddress saved{};
        std::memcpy(saved.address, this->address_, ESP_BD_ADDR_LEN);
        this->address_pref_.save(&saved);
      }
      // Discovery cancelled by the Bluetooth task stops next and starts the connection
      break;

    case EventType::EVENT_TYPE_DISCOVERY_STOPPED:
      if (this->connection_state_ != ConnectionState::CONNECTION_STATE_DISCOVERING)
        break;
      if (this->device_found_) {
        this->discovery_needed_ = false;
        this->connect_to_address_(now);
      } else {
        ESP_LOGW(TAG, "'%s' not found; retrying in %" PRIu32 " s", this->remote_name_,
                 this->reconnect_interval_ms_ / 1000);
        this->connection_state_ = ConnectionState::CONNECTION_STATE_IDLE;
        this->state_since_ms_ = now;
      }
      break;

    case EventType::EVENT_TYPE_CONNECTION_STATE: {
      const auto state = static_cast<esp_a2d_connection_state_t>(event.state);
      if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        if (this->connection_state_ == ConnectionState::CONNECTION_STATE_DISCOVERING) {
          esp_bt_gap_cancel_discovery();
        }
        std::memcpy(this->address_, event.address, ESP_BD_ADDR_LEN);
        this->has_address_ = true;
        this->discovery_needed_ = false;
        this->connection_state_ = ConnectionState::CONNECTION_STATE_CONNECTED;
        this->state_since_ms_ = now;
        this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
        ESP_LOGI(TAG, "Connected to %s", format_mac_addr_upper(this->address_, address));
        this->update_scan_mode_();
        this->connect_callback_.call();
      } else if (state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
        // The remote device reconnecting to us
        if (this->connection_state_ == ConnectionState::CONNECTION_STATE_IDLE) {
          this->connection_state_ = ConnectionState::CONNECTION_STATE_CONNECTING;
          this->state_since_ms_ = now;
        }
      } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        const ConnectionState previous = this->connection_state_;
        if (previous == ConnectionState::CONNECTION_STATE_DISCOVERING) {
          break;  // A stale event from an earlier connection
        }
        this->connection_state_ = ConnectionState::CONNECTION_STATE_IDLE;
        this->state_since_ms_ = now;
        this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
        this->update_scan_mode_();
        if (previous == ConnectionState::CONNECTION_STATE_CONNECTED) {
          ESP_LOGW(TAG, "Disconnected");
          this->disconnect_callback_.call();
        } else if (previous == ConnectionState::CONNECTION_STATE_CONNECTING) {
          ESP_LOGW(TAG, "Connection failed; retrying in %" PRIu32 " s", this->reconnect_interval_ms_ / 1000);
          // A cached address may belong to a device that has since changed, so look it up by name again
          this->discovery_needed_ = this->remote_name_ != nullptr;
        }
      }
      break;
    }

    case EventType::EVENT_TYPE_AUDIO_STATE:
      if (event.state == ESP_A2D_AUDIO_STATE_STARTED) {
        if (this->media_state_ == MediaState::MEDIA_STATE_STARTING) {
          this->set_media_state_(MediaState::MEDIA_STATE_STARTED);
        }
      } else if (this->media_state_ == MediaState::MEDIA_STATE_STARTED ||
                 this->media_state_ == MediaState::MEDIA_STATE_SUSPENDING) {
        // Suspended, possibly by the remote device; loop() starts it again if audio is still waiting
        ESP_LOGD(TAG, "Stream suspended");
        this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
      }
      break;

    case EventType::EVENT_TYPE_MEDIA_CTRL_ACK: {
      const bool success = event.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS;
      switch (event.state) {
        case ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY:
          if (this->media_state_ != MediaState::MEDIA_STATE_CHECKING)
            break;
          if (success) {
            this->send_media_command_(ESP_A2D_MEDIA_CTRL_START, MediaState::MEDIA_STATE_STARTING, now);
          } else {
            this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
          }
          break;
        case ESP_A2D_MEDIA_CTRL_START:
          if (this->media_state_ != MediaState::MEDIA_STATE_STARTING)
            break;
          if (success) {
            ESP_LOGD(TAG, "Stream started");
            this->set_media_state_(MediaState::MEDIA_STATE_STARTED);
          } else {
            ESP_LOGW(TAG, "Remote device refused to start the stream");
            this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
          }
          break;
        case ESP_A2D_MEDIA_CTRL_SUSPEND:
          if (this->media_state_ == MediaState::MEDIA_STATE_SUSPENDING) {
            ESP_LOGD(TAG, "Stream suspended");
            this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
          }
          break;
        default:
          break;
      }
      break;
    }
  }
}

void A2DPSource::update_media_(uint32_t now) {
  switch (this->media_state_) {
    case MediaState::MEDIA_STATE_IDLE:
      if (this->stream_requested_ && now - this->media_command_ms_ >= MEDIA_RETRY_MS) {
        this->send_media_command_(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY, MediaState::MEDIA_STATE_CHECKING, now);
      }
      break;
    case MediaState::MEDIA_STATE_STARTED:
      if (this->stream_requested_) {
        this->release_pending_ = false;
      } else if (!this->release_pending_) {
        this->release_pending_ = true;
        this->release_ms_ = now;
      } else if (now - this->release_ms_ >= this->suspend_timeout_ms_) {
        this->release_pending_ = false;
        this->send_media_command_(ESP_A2D_MEDIA_CTRL_SUSPEND, MediaState::MEDIA_STATE_SUSPENDING, now);
      }
      break;
    case MediaState::MEDIA_STATE_CHECKING:
    case MediaState::MEDIA_STATE_STARTING:
    case MediaState::MEDIA_STATE_SUSPENDING:
      if (now - this->media_command_ms_ >= MEDIA_COMMAND_TIMEOUT_MS) {
        ESP_LOGW(TAG, "No answer to a stream command");
        this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
      }
      break;
  }
}

void A2DPSource::send_media_command_(esp_a2d_media_ctrl_t command, MediaState next_state, uint32_t now) {
  this->media_command_ms_ = now;
  if (esp_a2d_media_ctrl(command) != ESP_OK) {
    this->set_media_state_(MediaState::MEDIA_STATE_IDLE);
    return;
  }
  this->set_media_state_(next_state);
}

void A2DPSource::set_media_state_(MediaState state) {
  if (this->media_state_ == state)
    return;
  const bool was_streaming = this->is_streaming();
  this->media_state_ = state;
  this->release_pending_ = false;
  const bool streaming = this->is_streaming();
  if (was_streaming != streaming && this->stream_ != nullptr) {
    this->stream_->on_stream_state(streaming);
  }
}

void A2DPSource::post_event_(const Event &event) {
  if (xQueueSend(this->event_queue_, &event, 0) != pdTRUE) {
    this->event_dropped_ = true;
  }
}

void A2DPSource::gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  A2DPSource *self = global_a2dp_source;
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
      self->handle_discovery_result_(param);
      break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
      if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        self->post_event_(Event{EventType::EVENT_TYPE_DISCOVERY_STOPPED, 0, 0, {}});
      }
      break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Paired with '%s'", reinterpret_cast<const char *>(param->auth_cmpl.device_name));
      } else {
        ESP_LOGW(TAG, "Pairing failed, status %d", param->auth_cmpl.stat);
      }
      break;
    case ESP_BT_GAP_PIN_REQ_EVT:
      self->handle_pin_request_(param);
      break;
    case ESP_BT_GAP_CFM_REQ_EVT:
      // No display to compare the number on, so accept it
      esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
      break;
    default:
      break;
  }
}

void A2DPSource::handle_pin_request_(esp_bt_gap_cb_param_t *param) {
  const size_t length = strlen(this->pin_);
  const size_t required = param->pin_req.min_16_digit ? 16 : length;
  if (length != required) {
    ESP_LOGW(TAG, "The remote device needs a 16 digit PIN");
    esp_bt_pin_code_t empty{};
    esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, empty);
    return;
  }
  esp_bt_pin_code_t pin_code{};
  std::memcpy(pin_code, this->pin_, length);
  esp_bt_gap_pin_reply(param->pin_req.bda, true, static_cast<uint8_t>(length), pin_code);
}

void A2DPSource::handle_discovery_result_(esp_bt_gap_cb_param_t *param) {
  char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {};
  uint32_t cod = 0;
  int32_t rssi = 0;
  uint8_t *eir = nullptr;

  for (int i = 0; i < param->disc_res.num_prop; i++) {
    const esp_bt_gap_dev_prop_t &prop = param->disc_res.prop[i];
    switch (prop.type) {
      case ESP_BT_GAP_DEV_PROP_COD:
        cod = *static_cast<uint32_t *>(prop.val);
        break;
      case ESP_BT_GAP_DEV_PROP_RSSI:
        rssi = *static_cast<int8_t *>(prop.val);
        break;
      case ESP_BT_GAP_DEV_PROP_EIR:
        eir = static_cast<uint8_t *>(prop.val);
        break;
      case ESP_BT_GAP_DEV_PROP_BDNAME: {
        const size_t length = std::min<size_t>(prop.len, ESP_BT_GAP_MAX_BDNAME_LEN);
        std::memcpy(name, prop.val, length);
        name[length] = '\0';
        break;
      }
      default:
        break;
    }
  }

  if (name[0] == '\0' && eir != nullptr) {
    uint8_t length = 0;
    uint8_t *eir_name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &length);
    if (eir_name == nullptr) {
      eir_name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &length);
    }
    if (eir_name != nullptr) {
      std::memcpy(name, eir_name, length);
      name[length] = '\0';
    }
  }
  if (name[0] == '\0')
    return;

  // Lists nearby speakers so the user can find the name to configure
  if (esp_bt_gap_is_valid_cod(cod) &&
      (esp_bt_gap_get_cod_srvc(cod) & (ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO))) {
    char address[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
    ESP_LOGD(TAG, "Audio device '%s' at %s, RSSI %" PRId32, name, format_mac_addr_upper(param->disc_res.bda, address),
             rssi);
  }

  if (strcmp(name, this->remote_name_) != 0)
    return;
  Event event{EventType::EVENT_TYPE_DEVICE_FOUND, 0, 0, {}};
  std::memcpy(event.address, param->disc_res.bda, ESP_BD_ADDR_LEN);
  this->post_event_(event);
  esp_bt_gap_cancel_discovery();
}

void A2DPSource::a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  A2DPSource *self = global_a2dp_source;
  Event copy{};
  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
      copy.type = EventType::EVENT_TYPE_CONNECTION_STATE;
      copy.state = static_cast<uint8_t>(param->conn_stat.state);
      std::memcpy(copy.address, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
      break;
    case ESP_A2D_AUDIO_STATE_EVT:
      copy.type = EventType::EVENT_TYPE_AUDIO_STATE;
      copy.state = static_cast<uint8_t>(param->audio_stat.state);
      break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
      copy.type = EventType::EVENT_TYPE_MEDIA_CTRL_ACK;
      copy.state = static_cast<uint8_t>(param->media_ctrl_stat.cmd);
      copy.status = static_cast<uint8_t>(param->media_ctrl_stat.status);
      break;
    default:
      return;
  }
  self->post_event_(copy);
}

int32_t A2DPSource::data_callback(uint8_t *data, int32_t length) {
  if (data == nullptr || length <= 0)
    return 0;
  A2DPStream *stream = global_a2dp_source->stream_;
  if (stream == nullptr) {
    std::memset(data, 0, length);
  } else {
    stream->read_stream(data, static_cast<size_t>(length));
  }
  return length;
}

}  // namespace esphome::a2dp_source

#endif  // USE_A2DP_SOURCE
