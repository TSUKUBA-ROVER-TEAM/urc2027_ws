#ifndef TRC2026_MANUAL_LAUNCHER_TRIGGER_HPP
#define TRC2026_MANUAL_LAUNCHER_TRIGGER_HPP

#include <cmath>
#include <optional>

namespace trc2026_manual
{

// Times are monotonic seconds. Inputs rejected here are never queued.
class LauncherTrigger
{
public:
  void feedback(double position, double now)
  {
    if (!std::isfinite(position)) {
      return;
    }
    position_ = position;
    feedback_time_ = now;
    if (busy_ && std::abs(position_ - target_) <= 0.05) {
      busy_ = false;
    }
  }

  std::optional<double> update(bool valid, bool pressed, bool available, double now)
  {
    if (!valid || now - joy_time_ > 0.5) {
      released_since_.reset();
    }
    joy_time_ = now;
    if (!valid) {
      return std::nullopt;
    }
    if (!pressed) {
      // A release during motion cannot arm the next shot.
      if (busy_) {
        released_since_.reset();
      } else if (!released_since_) {
        released_since_ = now;
      }
      return std::nullopt;
    }
    const bool armed = released_since_ && now - *released_since_ >= 0.1;
    released_since_.reset();
    if (!armed || busy_ || !available || now - feedback_time_ > 0.5 || now - shot_time_ < 1.0) {
      return std::nullopt;
    }
    target_ = initialized_ ? position_ + 20.0 * 3.14159265358979323846 : 0.0;
    initialized_ = true;
    shot_time_ = now;
    busy_ = true;
    return target_;
  }

private:
  std::optional<double> released_since_;
  double joy_time_ = -INFINITY;
  double feedback_time_ = -INFINITY;
  double shot_time_ = -INFINITY;
  double position_ = 0.0;
  double target_ = 0.0;
  bool busy_ = false;
  bool initialized_ = false;
};

}  // namespace trc2026_manual

#endif  // TRC2026_MANUAL_LAUNCHER_TRIGGER_HPP
