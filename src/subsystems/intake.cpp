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
    switch (mode) {
        case Mode::IN: motor_.move_voltage(mv); break;
        case Mode::OUT: motor_.move_voltage(-mv); break;
        case Mode::STOP: motor_.move_voltage(0); break;
    }
}

}  // namespace subsystems
