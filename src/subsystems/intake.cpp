#include "subsystems/intake.hpp"

#include "config.hpp"

#include <algorithm>

namespace subsystems {

Intake::Intake() : motor_(config::INTAKE_PORT, pros::v5::MotorGears::blue) {  // TODO(team): confirm cartridge
    motor_.set_brake_mode(pros::v5::MotorBrake::coast);
}

void Intake::set(Mode mode, double power) {
    mode_ = mode;
    const int mv = static_cast<int>(config::INTAKE_MV * std::clamp(power, 0.0, 1.0));
    // IN drives NEGATIVE voltage, OUT positive. That reads backwards and it is deliberate: on this
    // robot R1 (IN) was running the rollers the outtake way and R2 (OUT) was running them the
    // intake way, so the two are swapped here. Flipping the sign of config::INTAKE_PORT does NOT
    // fix it -- that was tried and the rollers kept spinning the same way.
    //
    // This is the single writer for the intake motor (driver R1/R2, the L2 auto-outtake, the
    // auto-intake and the auton intakeIn/intakeOut helpers all come through here), so swapping
    // these two signs corrects every path at once. To flip the rollers again, swap them back.
    switch (mode) {
        case Mode::IN: motor_.move_voltage(-mv); break;
        case Mode::OUT: motor_.move_voltage(mv); break;
        case Mode::STOP: motor_.move_voltage(0); break;
    }
}

}  // namespace subsystems
