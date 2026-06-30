#pragma once
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cover/cover.h"
#include "../uapbridge.h"

namespace esphome {
namespace uapbridge {

/**
 * Home Assistant cover for the Hörmann door.
 *
 * The drive only reports discrete states on the bus (open / closed / moving /
 * direction / venting / error). There is NO absolute position in the protocol.
 *
 * To still offer a position slider, this cover ESTIMATES the position from the
 * elapsed travel time and snaps back to exactly 0 % / 100 % whenever the drive
 * reports the real "closed" / "open" end stop, so timing errors never
 * accumulate. With a valid estimate, Home Assistant can also command an
 * intermediate position ("go to 50 %"): the door starts moving and is stopped
 * once the estimate reaches the target.
 *
 * Position is enabled when either:
 *   - open_duration AND close_duration are configured (fixed travel times), or
 *   - auto_calibration is enabled. The cover then SELF-LEARNS the travel times
 *     from every full closed<->open cycle (exponential moving average) and
 *     PERSISTS them to flash, so the estimate keeps improving and survives
 *     reboots of the ESP and the drive. Configured durations act as the initial
 *     seed (or a 20 s default if omitted).
 *
 * If neither is set, the cover is a plain open/closed garage door.
 */
class UAPBridgeCover : public cover::Cover, public Component {
 public:
  void set_uapbridge_parent(UAPBridge *parent) { this->parent_ = parent; }
  void set_open_duration(uint32_t ms) { this->open_duration_ = ms; }
  void set_close_duration(uint32_t ms) { this->close_duration_ = ms; }
  void set_auto_calibration(bool enabled) { this->auto_calibration_ = enabled; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  void control(const cover::CoverCall &call) override;
  cover::CoverTraits get_traits() override;

 protected:
  // Called on every decoded broadcast-status change of the bridge.
  void on_state_changed_();
  // (Re)start the time based position estimate for the given direction.
  void start_movement_(cover::CoverOperation dir);
  // Update this->position from the elapsed travel time.
  void recompute_estimated_position_();
  // Learn a travel time from a full end-to-end cycle (if auto_calibration).
  void maybe_learn_(bool reached_open);
  // Flash persistence of the learned durations.
  void load_calibration_();
  void save_calibration_();

  bool position_estimation_enabled_() const {
    return this->auto_calibration_ || (this->open_duration_ > 0 && this->close_duration_ > 0);
  }

  UAPBridge *parent_{nullptr};

  // Travel times in milliseconds. With auto_calibration these are mutable and
  // updated as the door is used; otherwise they stay at the configured value.
  uint32_t open_duration_{0};
  uint32_t close_duration_{0};
  bool auto_calibration_{false};

  // Have we ever obtained a real measurement for each direction?
  bool learned_open_valid_{false};
  bool learned_close_valid_{false};
  ESPPreferenceObject pref_;

  // Time based estimate bookkeeping.
  uint32_t move_start_ms_{0};
  float move_start_position_{0.0f};
  uint32_t last_publish_ms_{0};
  float target_position_{-1.0f};  // >= 0 while a "go to position" move is running

  UAPBridge::hoermann_state_t last_state_{UAPBridge::hoermann_state_unkown};
};

}  // namespace uapbridge
}  // namespace esphome
