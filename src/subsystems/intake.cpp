#include "subsystems/intake.hpp"

#include "config.hpp"

namespace subsystems {

Intake::Intake() : motors_(config::INTAKE_PORTS, pros::v5::MotorGears::blue) {  // TODO(team): confirm cartridge
    motors_.set_brake_mode_all(pros::v5::MotorBrake::coast);
}

void Intake::set(Mode mode) {
    mode_ = mode;
    switch (mode) {
        case Mode::IN: motors_.move_voltage(static_cast<int>(config::INTAKE_MV)); break;
        case Mode::OUT: motors_.move_voltage(-static_cast<int>(config::INTAKE_MV)); break;
        case Mode::STOP: motors_.move_voltage(0); break;
    }
}

}  // namespace subsystems
