#pragma once
/**
 * Intake — three-stage roller intake with a bottom counter roller, driven by ONE motor.
 * The driver runs it directly (R1 in, R2/B out); nothing else turns it on.
 */

#include "pros/motors.hpp"

namespace subsystems {

class Intake {
  public:
    enum class Mode { STOP, IN, OUT };
    Intake();
    void set(Mode mode);
    Mode getMode() const { return mode_; }
    double getTemperature() const { return motor_.get_temperature(); }

  private:
    pros::Motor motor_;
    Mode mode_ = Mode::STOP;
};

}  // namespace subsystems
