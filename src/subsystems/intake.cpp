#include "subsystems/intake.hpp"

#include "config.hpp"

namespace subsystems {

Intake::Intake() : motor_(config::INTAKE_PORT, pros::v5::MotorGears::blue) {  // TODO(team): confirm cartridge
    motor_.set_brake_mode(pros::v5::MotorBrake::coast);
}

void Intake::set(Mode mode) {
    mode_ = mode;
    switch (mode) {
        case Mode::IN: motor_.move_voltage(static_cast<int>(config::INTAKE_MV)); break;
        case Mode::OUT: motor_.move_voltage(-static_cast<int>(config::INTAKE_MV)); break;
        case Mode::STOP: motor_.move_voltage(0); break;
    }
}

}  // namespace subsystems
