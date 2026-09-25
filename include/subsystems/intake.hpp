#pragma once
/**
 * Intake — three-stage roller intake with a bottom counter roller.
 * All rollers run from the same motor group (TODO(team): confirm motor count).
 */

#include "pros/motor_group.hpp"

namespace subsystems {

class Intake {
  public:
    enum class Mode { STOP, IN, OUT };
    Intake();
    void set(Mode mode);
    Mode getMode() const { return mode_; }

  private:
    pros::MotorGroup motors_;
    Mode mode_ = Mode::STOP;
};

}  // namespace subsystems
