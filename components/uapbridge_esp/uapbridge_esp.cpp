#include "uapbridge_esp.h"

namespace esphome {
namespace uapbridge_esp {
static const char *const TAG = "uapbridge_esp";

void UAPBridge_esp::setup() {
  // Base class sets up the optional RS485 direction (RTS) pin.
  UAPBridge::setup();

  // Resolve the underlying ESP-IDF UART port number once. We need it to drive
  // the bus directly with uart_set_baudrate()/uart_set_word_length() (see
  // transmit()) instead of ESPHome's load_settings(). On the esp-idf framework
  // the parent UART is always an IDFUARTComponent.
  auto *idf = static_cast<uart::IDFUARTComponent *>(this->parent_);
  this->uart_port_ = static_cast<uart_port_t>(idf->get_hw_serial_number());
  ESP_LOGCONFIG(TAG, "UAPBridge_esp using UART port %d", (int) this->uart_port_);
}

void UAPBridge_esp::loop() {
  this->loop_fast();
  this->loop_slow();

  if (this->data_has_changed) {
    ESP_LOGD(TAG, "UAPBridge_esp::loop() - received data has changed.");
    this->clear_data_changed_flag();
    this->state_callback_.call();
  }
}

void UAPBridge_esp::loop_fast() {
  // Drain the RX bus on every iteration so we never miss a poll from the drive.
  this->receive();

  if (millis() - this->last_call < CYCLE_TIME) {
    // avoid unnecessarily frequent calls
    return;
  }
  this->last_call = millis();
  if (this->send_time != 0 && (millis() >= this->send_time)) {
    ESP_LOGVV(TAG, "loop: transmitting");
    this->transmit();
    this->send_time = 0;
  }
}

/**
 * Periodically (every CYCLE_TIME_SLOW ms) translate the latest broadcast status
 * into a high level door state, mirror the auxiliary booleans (light, relay,
 * venting, error, prewarning) and optionally run the error auto-correction.
 */
void UAPBridge_esp::loop_slow() {
  if (millis() - this->last_call_slow < CYCLE_TIME_SLOW) {
    // avoid unnecessarily frequent calls
    return;
  }
  this->last_call_slow = millis();

  if (this->ignore_next_event) {
    // We just queued a command of our own; skip one evaluation so we don't react
    // to the transient state caused by our own request.
    this->ignore_next_event = false;
    return;
  }

  ESP_LOGD(TAG, "loop_slow called - %02x %02x == %d", (uint8_t) (this->broadcast_status >> 8),
           (uint8_t) this->broadcast_status, this->broadcast_status);

  hoermann_state_t new_state = hoermann_state_stopped;
  if (this->broadcast_status & hoermann_state_open) {
    new_state = hoermann_state_open;
  } else if (this->broadcast_status & hoermann_state_closed) {
    new_state = hoermann_state_closed;
  } else if ((this->broadcast_status & (hoermann_state_direction | hoermann_state_moving)) == hoermann_state_opening) {
    new_state = hoermann_state_opening;
  } else if ((this->broadcast_status & (hoermann_state_direction | hoermann_state_moving)) == hoermann_state_closing) {
    new_state = hoermann_state_closing;
  } else if (this->broadcast_status & hoermann_state_venting) {
    new_state = hoermann_state_venting;
  }

  if (new_state != this->state) {
    this->handle_state_change(new_state);
  }

  this->update_boolean_state("relay", this->relay_enabled, (this->broadcast_status & hoermann_state_opt_relay));
  this->update_boolean_state("light", this->light_enabled, (this->broadcast_status & hoermann_state_light_relay));
  this->update_boolean_state("vent", this->venting_enabled, (this->broadcast_status & hoermann_state_venting));
  this->update_boolean_state("err", this->error_state, (this->broadcast_status & hoermann_state_error));
  this->update_boolean_state("prewarn", this->prewarn_state, (this->broadcast_status & hoermann_state_prewarn));

  // --- Auto error correction (optional) ---
  // Some drives latch a harmless error after an interrupted cycle. If enabled,
  // we try to clear it by re-requesting the already reached end position (which
  // does not move the door) and, if needed, toggling the light.
  if (this->auto_correction) {
    if ((this->broadcast_status & hoermann_state_error) == hoermann_state_error) {
      ESP_LOGD(TAG, "autocorrection started");
      if (new_state == hoermann_state_open) {
        this->set_command(true, hoermann_action_open);
      } else if (new_state == hoermann_state_closed) {
        this->set_command(true, hoermann_action_close);
      } else if (new_state == hoermann_state_stopped) {
        // Can't clear the error from here; the next open/close cycle will do it.
        this->auto_correction_in_progress = false;
      }
      this->auto_correction_in_progress = true;
    }
    if (this->auto_correction_in_progress && (this->broadcast_status & hoermann_state_light_relay)) {
      this->set_command(true, hoermann_action_toggle_light);
      this->auto_correction_in_progress = false;
    }
  }
}

/**
 * Read everything currently available on the bus, keeping a sliding window of
 * the last 5 bytes, and react to the three message types we care about:
 *   - slave scan            -> answer with "slave scan response"
 *   - broadcast status      -> decode the door status
 *   - slave status request  -> answer with "slave status response" (our command)
 */
void UAPBridge_esp::receive() {
  uint8_t length = 0;
  uint8_t counter = 0;
  bool new_data = false;

  while (this->available() > 0) {
#if ESPHOME_LOGLEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE
    if (this->byte_cnt > 5) {
      // unread data that will be dropped -> log it for debugging
      char temp[4];
      sprintf(temp, "%02X ", this->rx_data[0]);
      ESP_LOGVV(TAG, "in receive while available: %s", temp);
    }
#endif
    // Shift the window and append the new byte. Only the last 5 bytes matter;
    // anything older is discarded.
    for (uint8_t i = 0; i < 4; i++) {
      this->rx_data[i] = this->rx_data[i + 1];
    }
    if (this->read_byte(&this->rx_data[4])) {
      this->byte_cnt++;
    }
    new_data = true;
  }

  if (!new_data) {
    return;
  }
  ESP_LOGVV(TAG, "new data received");

  // --- Slave scan ---  e.g. 28 82 01 80 06
  if (this->rx_data[0] == UAP1_ADDR) {
    length = this->rx_data[1] & 0x0F;
    if (this->rx_data[2] == CMD_SLAVE_SCAN && this->rx_data[3] == UAP1_ADDR_MASTER && length == 2 &&
        calc_crc8(this->rx_data, length + 3) == 0x00) {
      ESP_LOGVV(TAG, "SlaveScan: %s", print_data(this->rx_data, 0, 5));
      ESP_LOGV(TAG, "->      SlaveScan");
      counter = (this->rx_data[1] & 0xF0) + 0x10;
      this->tx_data[0] = UAP1_ADDR_MASTER;
      this->tx_data[1] = 0x02 | counter;
      this->tx_data[2] = UAP1_TYPE;
      this->tx_data[3] = UAP1_ADDR;
      this->tx_data[4] = calc_crc8(this->tx_data, 4);
      this->tx_length = 5;
      this->send_time = millis();
    }
  }

  // --- Broadcast status ---  e.g. 00 92 12 02 35
  if (this->rx_data[0] == BROADCAST_ADDR) {
    length = this->rx_data[1] & 0x0F;
    if (length == 2 && calc_crc8(this->rx_data, length + 3) == 0x00) {
      ESP_LOGVV(TAG, "Broadcast: %s", print_data(this->rx_data, 0, 5));
      ESP_LOGV(TAG, "->      Broadcast");
      this->broadcast_status = this->rx_data[2];
      this->broadcast_status |= (uint16_t) this->rx_data[3] << 8;
    }
  }

  // --- Slave status request ---  e.g. 28 A1 20 2E (only 4 bytes -> shifted indices)
  if (this->rx_data[1] == UAP1_ADDR) {
    length = this->rx_data[2] & 0x0F;
    if (this->rx_data[3] == CMD_SLAVE_STATUS_REQUEST && length == 1 &&
        calc_crc8(&this->rx_data[1], length + 3) == 0x00) {
      ESP_LOGVV(TAG, "Slave status request: %s", print_data(this->rx_data, 1, 5));
      ESP_LOGV(TAG, "->      Slave status request");
      counter = (this->rx_data[2] & 0xF0) + 0x10;
      this->tx_data[0] = UAP1_ADDR_MASTER;
      this->tx_data[1] = 0x03 | counter;
      this->tx_data[2] = CMD_SLAVE_STATUS_RESPONSE;
      this->tx_data[3] = (uint8_t) (this->next_action & 0xFF);
      this->tx_data[4] = (uint8_t) ((this->next_action >> 8) & 0xFF);
      this->next_action = hoermann_action_none;  // command consumed
      this->tx_data[5] = calc_crc8(this->tx_data, 5);
      this->tx_length = 6;
      this->send_time = millis();
    }
  }

  // First time we see a non-empty payload we consider the bus link healthy.
  if (!this->valid_broadcast && (this->rx_data[3] != 0 || this->rx_data[4] != 0)) {
    this->valid_broadcast = true;
    this->data_has_changed = true;
  }

#if ESPHOME_LOGLEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE
  if (this->byte_cnt >= 5) {
    ESP_LOGVV(TAG, "Just printed: %s", print_data(this->rx_data, 0, 5));
  }
#endif
}

/**
 * Send the queued answer frame on the half-duplex RS485 bus.
 *
 * Every HCP/UAP frame is introduced by a "sync break": a low pulse longer than
 * a normal character. We emulate it by transmitting a single 0x00 byte at HALF
 * the bus baud rate (9600) with 7 data bits, which keeps the line low for ~8 bit
 * times (~0.83 ms) — long enough for the drive, running at 19200 baud, to detect
 * a break. We then restore 19200/8N1 and send the payload.
 *
 * IMPORTANT (the bug fix):
 * The baud rate / word length are switched with the *lightweight* ESP-IDF
 * setters uart_set_baudrate()/uart_set_word_length(), NOT with ESPHome's
 * UARTComponent::load_settings(). Since ESPHome 2026.2.3 load_settings() tears
 * down and reinstalls the entire UART driver (uart_driver_delete +
 * uart_driver_install). Doing that twice per answer — many times per second —
 * glitched the shared bus line and wrecked the strict response timing, so the
 * drive stopped trusting the emulated UAP1 and went into a degraded/error state
 * in which it ignored its radio hand transmitters and wall button (while a
 * command injected from Home Assistant could still occasionally get through).
 * The lightweight setters only touch the relevant hardware registers and leave
 * the driver — and its RX ring buffer — fully intact.
 */
void UAPBridge_esp::transmit() {
  ESP_LOGVV(TAG, "Transmit: %s", print_data(this->tx_data, 0, this->tx_length));

  // Switch an external RS485 transceiver to "transmit", if a direction pin is
  // configured. Auto-direction transceivers leave rts_pin unset.
  if (this->rts_pin_ != nullptr) {
    this->rts_pin_->digital_write(true);  // HIGH = transmit, LOW = listen
  }

  // 1) Sync break: one 0x00 byte at 9600 baud / 7 data bits.
  uart_set_baudrate(this->uart_port_, 9600);
  uart_set_word_length(this->uart_port_, UART_DATA_7_BITS);
  this->write_byte(0x00);
  this->flush();  // block until the break has physically left the UART (uart_wait_tx_done)

  // 2) Payload: restore the bus format (19200 baud / 8N1) and send the frame.
  uart_set_baudrate(this->uart_port_, 19200);
  uart_set_word_length(this->uart_port_, UART_DATA_8_BITS);
  this->write_array(this->tx_data, this->tx_length);
  this->flush();

  // Back to listen mode.
  if (this->rts_pin_ != nullptr) {
    this->rts_pin_->digital_write(false);
  }

  ESP_LOGVV(TAG, "TX duration: %ums", (unsigned) (millis() - this->send_time));
}

/**
 * Queue a command for the next "slave status response", unless a command is
 * still pending (we must not overwrite an answer the drive has not fetched yet).
 */
void UAPBridge_esp::set_command(bool cond, const hoermann_action_t command) {
  if (!cond) {
    return;
  }
  if (this->next_action != hoermann_action_none) {
    ESP_LOGW(TAG, "Last command not yet fetched by the drive -- new action cached: %d", this->next_action);
  } else {
    this->next_action = command;
    this->ignore_next_event = true;
  }
}

void UAPBridge_esp::action_open() {
  ESP_LOGD(TAG, "Action: open called");
  this->set_command(this->state != hoermann_state_open, hoermann_action_open);
}

void UAPBridge_esp::action_close() {
  ESP_LOGD(TAG, "Action: close called");
  this->set_command(this->state != hoermann_state_closed, hoermann_action_close);
}

void UAPBridge_esp::action_stop() {
  ESP_LOGD(TAG, "Action: stop called");
  this->set_command((this->state == hoermann_state_opening || this->state == hoermann_state_closing),
                    hoermann_action_stop);
}

void UAPBridge_esp::action_venting() {
  ESP_LOGD(TAG, "Action: venting called");
  this->set_command(this->state != hoermann_state_venting, hoermann_action_venting);
}

void UAPBridge_esp::action_toggle_light() {
  ESP_LOGD(TAG, "Action: toggle light called");
  this->set_command(true, hoermann_action_toggle_light);
}

void UAPBridge_esp::action_impulse() {
  ESP_LOGD(TAG, "Action: impulse called");
  this->set_command(true, hoermann_action_impulse);
}

UAPBridge_esp::hoermann_state_t UAPBridge_esp::get_state() { return this->state; }

std::string UAPBridge_esp::get_state_string() { return this->state_string; }

void UAPBridge_esp::set_venting(bool state) {
  if (state) {
    this->action_venting();
  } else {
    this->action_close();
  }
  ESP_LOGD(TAG, "Venting state set to %s", state ? "ON" : "OFF");
}

void UAPBridge_esp::set_light(bool state) {
  this->set_command((this->light_enabled != state), hoermann_action_toggle_light);
  ESP_LOGD(TAG, "Light state set to %s", state ? "ON" : "OFF");
}

uint8_t UAPBridge_esp::calc_crc8(uint8_t *p_data, uint8_t length) {
  uint8_t data;
  uint8_t crc = 0xF3;
  for (uint8_t i = 0; i < length; i++) {
    data = *p_data ^ crc;
    p_data++;
    crc = crc_table[data];
  }
  return crc;
}

char *UAPBridge_esp::print_data(uint8_t *p_data, uint8_t from, uint8_t to) {
  char temp[4];
  static char output[30];
  sprintf(output, "%5lu: ", millis() & 0xFFFFul);
  for (uint8_t i = from; i < to; i++) {
    sprintf(temp, "%02X ", p_data[i]);
    strcat(output, temp);
  }
  this->byte_cnt = 0;
  return &output[0];
}

void UAPBridge_esp::handle_state_change(hoermann_state_t new_state) {
  this->state = new_state;
  ESP_LOGV(TAG, "State changed from %s to %d", this->state_string.c_str(), new_state);
  switch (new_state) {
    case hoermann_state_open:
      this->state_string = "Open";
      break;
    case hoermann_state_closed:
      this->state_string = "Closed";
      break;
    case hoermann_state_opening:
      this->state_string = "Opening";
      break;
    case hoermann_state_closing:
      this->state_string = "Closing";
      break;
    case hoermann_state_venting:
      this->state_string = "Venting";
      break;
    case hoermann_state_stopped:
      this->state_string = "Stopped";
      break;
    default:
      this->state_string = "Error";
      break;
  }
  this->data_has_changed = true;
}

void UAPBridge_esp::update_boolean_state(const char *name, bool &current_state, bool new_state) {
  ESP_LOGV(TAG, "update_boolean_state: %s from %s to %s", name, current_state ? "true" : "false",
           new_state ? "true" : "false");
  if (current_state != new_state) {
    current_state = new_state;
    this->data_has_changed = true;
  }
}

}  // namespace uapbridge_esp
}  // namespace esphome
