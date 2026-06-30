#include "uapbridge_cover.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace uapbridge {

static const char *const TAG = "uapbridge.cover";

// While the door is moving, refresh the estimated position at most this often.
static const uint32_t POSITION_PUBLISH_INTERVAL_MS = 1000;

// Default seed travel time when auto_calibration is on but no duration is given.
static const uint32_t DEFAULT_SEED_MS = 20000;

// Sanity bounds for a learned travel time (reject glitches / partial cycles).
static const uint32_t MIN_TRAVEL_MS = 2000;
static const uint32_t MAX_TRAVEL_MS = 180000;

// Weight of a fresh measurement in the exponential moving average (0..1).
static const float LEARN_ALPHA = 0.3f;

// How close to an end stop the move must have started to count as a full travel.
static const float FULL_TRAVEL_EPSILON = 0.05f;

// Persisted blob layout. The magic guards against uninitialised flash and
// format changes.
static const uint32_t CAL_MAGIC = 0x55415031;  // "UAP1"
struct CalibrationData {
  uint32_t magic;
  uint32_t open_ms;
  uint32_t close_ms;
  bool open_valid;
  bool close_valid;
} __attribute__((packed));

static bool travel_in_bounds(uint32_t ms) { return ms >= MIN_TRAVEL_MS && ms <= MAX_TRAVEL_MS; }

void UAPBridgeCover::setup() {
  // Seed the travel times so position estimation works on the very first cycle,
  // before anything has been learned.
  if (this->auto_calibration_) {
    if (this->open_duration_ == 0)
      this->open_duration_ = DEFAULT_SEED_MS;
    if (this->close_duration_ == 0)
      this->close_duration_ = DEFAULT_SEED_MS;
    this->load_calibration_();  // may override the seeds with learned values
  }

  // Start from a defined CLOSED position. cover::Cover's constructor defaults
  // `position` to COVER_OPEN (1.0); leaving it there would (a) show 100 % open in
  // Home Assistant before the first broadcast arrives — and with a non-assumed
  // state HA trusts that — and (b) make a full open commanded right after boot
  // un-learnable (move_start_position_ would look like the open end). The real
  // open/closed state corrects this within ~100 ms via on_state_changed_() below.
  if (this->position_estimation_enabled_())
    this->position = cover::COVER_CLOSED;

  this->parent_->add_on_state_callback([this]() { this->on_state_changed_(); });
  // Pick up whatever state is already known.
  this->on_state_changed_();
}

void UAPBridgeCover::dump_config() {
  ESP_LOGCONFIG(TAG, "UAPBridge Cover:");
  if (this->position_estimation_enabled_()) {
    ESP_LOGCONFIG(TAG, "  Position estimation: enabled");
    ESP_LOGCONFIG(TAG, "  Auto calibration: %s", this->auto_calibration_ ? "yes" : "no");
    ESP_LOGCONFIG(TAG, "  Open duration:  %u ms%s", this->open_duration_,
                  this->learned_open_valid_ ? " (learned)" : " (seed)");
    ESP_LOGCONFIG(TAG, "  Close duration: %u ms%s", this->close_duration_,
                  this->learned_close_valid_ ? " (learned)" : " (seed)");
  } else {
    ESP_LOGCONFIG(TAG, "  Position estimation: disabled (open/closed only)");
  }
}

cover::CoverTraits UAPBridgeCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);  // we get real open/closed feedback from the drive
  traits.set_supports_position(this->position_estimation_enabled_());
  traits.set_supports_stop(true);
  traits.set_supports_toggle(true);
  traits.set_supports_tilt(false);
  return traits;
}

void UAPBridgeCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->target_position_ = -1.0f;
    this->parent_->action_stop();
    ESP_LOGI(TAG, "Stopping the cover");
    return;
  }

  if (call.get_toggle().has_value()) {
    this->parent_->action_impulse();
    ESP_LOGI(TAG, "Impulse (toggle) the cover");
    return;
  }

  if (call.get_position().has_value()) {
    const float pos = *call.get_position();
    if (pos >= cover::COVER_OPEN) {
      this->target_position_ = -1.0f;
      this->parent_->action_open();
      ESP_LOGI(TAG, "Opening the cover");
    } else if (pos <= cover::COVER_CLOSED) {
      this->target_position_ = -1.0f;
      this->parent_->action_close();
      ESP_LOGI(TAG, "Closing the cover");
    } else if (this->position_estimation_enabled_()) {
      // Best-effort "go to intermediate position": start moving the right way;
      // loop() issues a stop once the estimated position reaches the target.
      // Note: target_position_ is always cleared when the door becomes idle (see
      // on_state_changed_), so it can never linger across moves. The one case we
      // cannot handle is the user issuing a *separate* full open/close from a
      // physical remote WHILE this timed move is running and in the same
      // direction — the bus gives us no way to tell that apart, so we may stop at
      // the target. Pressing the remote again recovers immediately.
      if (pos > this->position) {
        this->target_position_ = pos;
        this->parent_->action_open();
        ESP_LOGI(TAG, "Moving cover to %.0f%% (opening)", pos * 100.0f);
      } else if (pos < this->position) {
        this->target_position_ = pos;
        this->parent_->action_close();
        ESP_LOGI(TAG, "Moving cover to %.0f%% (closing)", pos * 100.0f);
      }
    }
  }
}

void UAPBridgeCover::loop() {
  if (!this->position_estimation_enabled_())
    return;
  if (this->current_operation == cover::COVER_OPERATION_IDLE)
    return;

  this->recompute_estimated_position_();

  // Stop at the requested intermediate position, if any.
  if (this->target_position_ >= 0.0f) {
    const bool reached =
        (this->current_operation == cover::COVER_OPERATION_OPENING && this->position >= this->target_position_) ||
        (this->current_operation == cover::COVER_OPERATION_CLOSING && this->position <= this->target_position_);
    if (reached) {
      this->target_position_ = -1.0f;
      this->parent_->action_stop();  // the resulting "stopped" broadcast finalises the state
      this->publish_state();
      return;
    }
  }

  // Throttled live position updates while moving.
  const uint32_t now = millis();
  if (now - this->last_publish_ms_ >= POSITION_PUBLISH_INTERVAL_MS) {
    this->last_publish_ms_ = now;
    this->publish_state();
  }
}

void UAPBridgeCover::start_movement_(cover::CoverOperation dir) {
  this->move_start_ms_ = millis();
  this->move_start_position_ = this->position;
  if (std::isnan(this->move_start_position_)) {
    // Position not known yet (e.g. right after boot): assume the far end.
    this->move_start_position_ = (dir == cover::COVER_OPERATION_OPENING) ? cover::COVER_CLOSED : cover::COVER_OPEN;
  }
}

void UAPBridgeCover::recompute_estimated_position_() {
  const uint32_t elapsed = millis() - this->move_start_ms_;
  if (this->current_operation == cover::COVER_OPERATION_OPENING) {
    const float fraction = (float) elapsed / (float) this->open_duration_;
    this->position = clamp(this->move_start_position_ + fraction, cover::COVER_CLOSED, cover::COVER_OPEN);
  } else if (this->current_operation == cover::COVER_OPERATION_CLOSING) {
    const float fraction = (float) elapsed / (float) this->close_duration_;
    this->position = clamp(this->move_start_position_ - fraction, cover::COVER_CLOSED, cover::COVER_OPEN);
  }
}

/**
 * Learn a travel time from the cycle that just ended at an end stop.
 *
 * We only trust measurements from a CONTINUOUS, FULL travel: the move must have
 * started right at the opposite end stop (within FULL_TRAVEL_EPSILON). Partial
 * moves and interrupted/reversed cycles are ignored, because their duration or
 * start position cannot be trusted (the start position itself is only an
 * estimate). Both end stops are real drive feedback, so a full travel gives a
 * clean, self-correcting measurement.
 */
void UAPBridgeCover::maybe_learn_(bool reached_open) {
  if (!this->auto_calibration_)
    return;

  const uint32_t elapsed = millis() - this->move_start_ms_;
  if (!travel_in_bounds(elapsed))
    return;

  if (reached_open) {
    // A full open must have started from (near) closed.
    if (this->move_start_position_ > FULL_TRAVEL_EPSILON)
      return;
    this->open_duration_ = this->learned_open_valid_
                               ? (uint32_t) (LEARN_ALPHA * elapsed + (1.0f - LEARN_ALPHA) * this->open_duration_)
                               : elapsed;
    this->learned_open_valid_ = true;
    ESP_LOGI(TAG, "Learned OPEN duration: %u ms (this travel: %u ms)", this->open_duration_, elapsed);
  } else {
    // A full close must have started from (near) open.
    if (this->move_start_position_ < cover::COVER_OPEN - FULL_TRAVEL_EPSILON)
      return;
    this->close_duration_ = this->learned_close_valid_
                                ? (uint32_t) (LEARN_ALPHA * elapsed + (1.0f - LEARN_ALPHA) * this->close_duration_)
                                : elapsed;
    this->learned_close_valid_ = true;
    ESP_LOGI(TAG, "Learned CLOSE duration: %u ms (this travel: %u ms)", this->close_duration_, elapsed);
  }
  this->save_calibration_();
}

void UAPBridgeCover::load_calibration_() {
  // A stable, per-entity key so several covers never clash in flash.
  const uint32_t key = this->get_object_id_hash() ^ 0x55415043u;  // "UAPC"
  this->pref_ = global_preferences->make_preference<CalibrationData>(key);

  CalibrationData data{};
  if (!this->pref_.load(&data) || data.magic != CAL_MAGIC)
    return;

  if (data.open_valid && travel_in_bounds(data.open_ms)) {
    this->open_duration_ = data.open_ms;
    this->learned_open_valid_ = true;
  }
  if (data.close_valid && travel_in_bounds(data.close_ms)) {
    this->close_duration_ = data.close_ms;
    this->learned_close_valid_ = true;
  }
  ESP_LOGI(TAG, "Restored learned durations: open=%u ms (%s), close=%u ms (%s)", this->open_duration_,
           this->learned_open_valid_ ? "learned" : "seed", this->close_duration_,
           this->learned_close_valid_ ? "learned" : "seed");
}

void UAPBridgeCover::save_calibration_() {
  CalibrationData data{};
  data.magic = CAL_MAGIC;
  data.open_ms = this->open_duration_;
  data.close_ms = this->close_duration_;
  data.open_valid = this->learned_open_valid_;
  data.close_valid = this->learned_close_valid_;
  // ESPHome batches the actual flash write, so calling this once per full cycle
  // does not wear the flash.
  this->pref_.save(&data);
}

void UAPBridgeCover::on_state_changed_() {
  const auto state = this->parent_->get_state();
  const bool estimate = this->position_estimation_enabled_();
  cover::CoverOperation new_op = this->current_operation;
  bool publish = false;

  switch (state) {
    case UAPBridge::hoermann_state_opening:
      if (this->current_operation != cover::COVER_OPERATION_OPENING)
        this->start_movement_(cover::COVER_OPERATION_OPENING);
      new_op = cover::COVER_OPERATION_OPENING;
      break;

    case UAPBridge::hoermann_state_closing:
      if (this->current_operation != cover::COVER_OPERATION_CLOSING)
        this->start_movement_(cover::COVER_OPERATION_CLOSING);
      new_op = cover::COVER_OPERATION_CLOSING;
      break;

    case UAPBridge::hoermann_state_open:
      // Learn before snapping, while move_start_position_ still describes this travel.
      if (this->current_operation == cover::COVER_OPERATION_OPENING)
        this->maybe_learn_(true);
      new_op = cover::COVER_OPERATION_IDLE;
      this->position = cover::COVER_OPEN;  // real end stop -> exact 100 %
      this->target_position_ = -1.0f;
      publish = true;
      break;

    case UAPBridge::hoermann_state_closed:
      if (this->current_operation == cover::COVER_OPERATION_CLOSING)
        this->maybe_learn_(false);
      new_op = cover::COVER_OPERATION_IDLE;
      this->position = cover::COVER_CLOSED;  // real end stop -> exact 0 %
      this->target_position_ = -1.0f;
      publish = true;
      break;

    case UAPBridge::hoermann_state_venting:
      new_op = cover::COVER_OPERATION_IDLE;
      if (!estimate)
        this->position = 0.1f;  // small defined gap ("H" on the drive display)
      this->target_position_ = -1.0f;
      publish = true;
      break;

    case UAPBridge::hoermann_state_stopped:
    case UAPBridge::hoermann_state_error:
    default:
      new_op = cover::COVER_OPERATION_IDLE;
      if (!estimate)
        this->position = 0.1f;  // unknown intermediate position
      // With estimation enabled we keep the last interpolated position.
      this->target_position_ = -1.0f;
      publish = true;
      break;
  }

  if (new_op != this->current_operation) {
    this->current_operation = new_op;
    publish = true;
  }

  if (publish || state != this->last_state_) {
    this->publish_state();
    this->last_publish_ms_ = millis();
  }
  this->last_state_ = state;
}

}  // namespace uapbridge
}  // namespace esphome
