#pragma once

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace annin_ar4_driver {

// Current-limited grasp for the servo gripper.
//
// Background (2026-07-24): the Hiwonder HPS-3518SG coreless servo has very low
// winding resistance, so a hard stall demands more current than the fixed 5 V RC
// supply can deliver through its leads (~0.6 Ohm of droop). Commanding the jaws
// straight to full-close against an object collapses the rail from 5.4 V to <4 V,
// the servo browns out, releases, recovers and retries in a ~2 s loop — it never
// grips. A held, gentle clamp draws a steady ~1.75 A with NO sag, though.
//
// So instead of "drive to full close and rely on an overcurrent cutout", this
// policy CREEPS the jaws closed at a rate limit and FREEZES the commanded
// position the instant the current rises past a contact threshold. That keeps
// the servo in its stable ~1.75 A holding regime and out of the brownout regime,
// and it is size-agnostic — it stops wherever contact happens. It is also gentler
// on any servo (the cored DS3230MG replacement included), since it never hard-
// stalls, which is what cooks a servo.
class GripperOverCurrentProtection {
 public:
  GripperOverCurrentProtection(const rclcpp::Logger& logger,
                               const rclcpp::Clock& clock);

  // Feed the latest measured servo current (Amps) each read cycle.
  void AddCurrentSample(const rclcpp::Time& time, double current);

  // Returns the position to actually command to the servo this cycle.
  //
  // Opening (or any command looser than the current output) passes straight
  // through and re-arms the grasp. A closing command creeps toward the target at
  // close_velocity_ until the current exceeds contact_current_threshold_ for
  // contact_debounce_ consecutive cycles, then holds the jaws at that contact
  // position instead of driving on to full close.
  double AdjustGripperPosition(double position_command, double min_position,
                               double max_position, const rclcpp::Time& now);

 private:
  double current_ = 0.0;

  // Rate-limited-close + contact-freeze state.
  bool output_init_ = false;      // output_pos_ seeded yet?
  double output_pos_ = 0.0;       // the position we actually command (ramps)
  rclcpp::Time last_time_;        // for dt in the rate limit
  bool grip_frozen_ = false;      // jaws have clamped an object; hold here
  int high_streak_ = 0;           // consecutive high-current cycles (debounce)

  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  // ---- Tunables (hardcoded for now; easily promoted to yaml params) ----
  // Contact current: above this the jaws are pressing something. Set BELOW the
  // ~2.2 A brownout point and above the moving/free-run draw (~0.5-0.8 A) so the
  // grasp freezes at contact and never demands the stall current that collapses
  // the rail. Needs live tuning — the grip/brownout margin is only ~0.5 A.
  double contact_current_threshold_ = 1.3;  // Amps
  int contact_debounce_ = 3;                // consecutive high cycles to confirm
  // How fast the jaws close (m/s). Slow enough that the current is sampled as it
  // rises into contact, before it can spike straight into the brownout. Full
  // ~0.014 m travel at 0.02 m/s ~= 0.7 s.
  double close_velocity_ = 0.02;  // m/s
  // Commands closer to closed than this fraction of the closed->open travel are
  // treated as a grasp (intent to close). At/above it the command is "opening".
  double grip_intent_fraction_ = 0.5;
};

}  // namespace annin_ar4_driver
