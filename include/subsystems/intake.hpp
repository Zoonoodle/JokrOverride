#pragma once
/**
 * Intake — three-stage roller intake with a bottom counter roller, driven by ONE motor.
 * The driver runs it directly (R1 in, R2/B out); main.cpp's auto-intake (L1) and auto-outtake (L2
 * score) macros are the only other callers, and this remains the single code path that ever writes
 * the intake motor.
 */

#include "pros/motors.hpp"

namespace subsystems {

class Intake {
  public:
    enum class Mode { STOP, IN, OUT };
    Intake();
    /** power is 0..1, a fraction of config::INTAKE_MV (full power by default); ignored for STOP. R1/R2
     *  always call this with the default, so their full-power feel is unchanged -- only a macro (e.g.
     *  L2's auto-outtake) passes a lower fraction. */
    void set(Mode mode, double power = 1.0);
    Mode getMode() const { return mode_; }
    double getTemperature() const { return motor_.get_temperature(); }

  private:
    pros::Motor motor_;
    Mode mode_ = Mode::STOP;
};

}  // namespace subsystems
