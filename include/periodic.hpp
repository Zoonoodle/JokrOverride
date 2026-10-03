#pragma once
/**
 * periodic — THE per-loop subsystem logic, in exactly one copy.
 *
 * Everything that has to happen every 10 ms for the mechanisms to work at all used to live inside
 * opcontrol()'s while loop, which meant NONE of it ran during autonomous(): Scoring's state machine
 * never advanced (so an auton L2 score could never have its jaws opened -- Lift::needsClawRelease()
 * would have gone unanswered until L2_RELEASE_ACK_MS gave up), the claw's autoclamp never ticked, and
 * the intake was never written. The lift PID was the one thing that did run in auton, because it has
 * its own 10 ms task started in initialize().
 *
 * So the loop body moved here, ONE copy, and both callers run it:
 *   - opcontrol()'s driver loop, every 10 ms, with Inputs filled in from the controller;
 *   - the auton tick task (see include/auton.hpp), every config::AUTON_TICK_MS, with Inputs filled
 *     in from the auton API's latched requests.
 *
 * THE TWO CALLERS NEVER RUN AT ONCE. autonomous() starts the auton tick with auton::begin();
 * opcontrol() and disabled() both call auton::end() before doing anything else, and end() BLOCKS
 * until the tick task has actually exited (see auton.cpp). A field switch from auton to driver
 * therefore hands ownership over, it does not share it.
 *
 * WHAT IT OWNS, and what it does NOT:
 *   - It makes the SINGLE intake decision per tick (one intake.set() call -- see the note in
 *     subsystems/intake.hpp: never add a second writer).
 *   - It runs L1's auto-intake sweep window and L2's auto-outtake window (the timers and the
 *     cancel/timeout rules that used to be local variables in opcontrol()).
 *   - It calls Claw::update() and Scoring::update(), in that order, once per tick.
 *   - It NEVER writes the lift motors. Lift::update(), in the lift's own 10 ms task, is still the
 *     only thing in the program that does (CLAUDE.md safety rule 2). Commanded heights still only
 *     ever reach the lift through Lift::setTarget()/moveTo()/startScore()/driverPower().
 *
 * mutex() guards all of that. In driver control one task does everything, so it never contends; in
 * auton the routine runs on the autonomous() task while the tick runs on its own, and the mutex is
 * what stops a half-applied Scoring state transition (state_ set, stateStart_ not yet) from being
 * read by update() in between -- which would have skipped CLAMPING's CLAW_CLOSE_TIME_MS wait, i.e.
 * interlock 1. Every auton command that touches Scoring or the Claw takes it too.
 */

#include "pros/rtos.hpp"
#include "subsystems/intake.hpp"

namespace periodic {

/** What the caller wants this tick. The driver fills it from buttons; auton fills it from the
 *  requests latched by auton::intakeIn()/intakeOut()/intakeStop() and friends. */
struct Inputs {
    /** A DIRECT intake request, which always beats both macros (driver R1/R2 in roller mode, or
     *  auton::intakeIn()/intakeOut()). Mode::STOP = no request, so the macros decide. */
    subsystems::Intake::Mode intakeRequest = subsystems::Intake::Mode::STOP;
    /** 0..1 of config::INTAKE_MV for the request above. */
    double intakePower = 1.0;
    /** The lift's manual override is held right now (D-pad Up/Down). Blocks the auto-intake window
     *  from opening and cancels either window that is open. */
    bool manualLift = false;
    /** R1/R2 are latched into clamped-mode LIFT (screen::RMode::LIFT): they own the lift, so an
     *  auto-intake sweep belonging to an abandoned rise must stop. */
    bool liftButtonMode = false;
    /** B (or auton::stow()) this tick: abandons both windows. */
    bool abort = false;
    /** L2 (or auton::score()) this tick: abandons the auto-intake window. */
    bool scorePressed = false;
    /** L1 (or auton::stepUp()) this tick: may OPEN the auto-intake window, if the arm really is
     *  starting a rise from the floor (see config::L1_AUTO_INTAKE_UNTIL_DEG). */
    bool stepUpPressed = false;
    /** TRUE ONLY FROM THE AUTON TICK. The autoclamp's trigger range differs by mode
     *  (config::AUTOCLAMP_RANGE_AUTON_MM 65 vs _DRIVER_MM 55), and this is how Claw hears which one it
     *  is -- passed every tick rather than latched anywhere, so a field switch cannot leave it wrong.
     *  It defaults to false, which is what makes opcontrol's loop get the driver range without having to
     *  know this field exists. */
    bool auton = false;
};

/** Forget both macro windows and clear their brain-screen flags. Called at the start of each mode
 *  (auton::begin(), and opcontrol() before its loop) so a second enabled mode in one program run
 *  never inherits a half-open window. Does not move anything. */
void reset();

/** One tick of all of the above. Call every 10 ms from exactly one task. */
void tick(const Inputs& in);

/** The lock that serialises Scoring/Claw access between the auton routine task and the tick task.
 *  tick() takes it itself; auton's command functions take it around their Scoring/Claw calls. */
pros::Mutex& mutex();

}  // namespace periodic
