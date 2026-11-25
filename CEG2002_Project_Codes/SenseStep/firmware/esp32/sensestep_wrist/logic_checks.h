#pragma once
#include "haptic_logic.h"

// Compile-time checks exercise the actual pulse and freshness functions.
namespace SenseStepHapticChecks {
using namespace SenseStepHaptics;
static_assert(classifyDistance(450, State::Clear) == State::Urgent, "Urgent boundary");
static_assert(classifyDistance(451, State::Clear) == State::Near, "Near boundary");
static_assert(classifyDistance(850, State::Clear) == State::Near, "Near threshold");
static_assert(classifyDistance(851, State::Clear) == State::Far, "Far threshold");
static_assert(classifyDistance(1301, State::Clear) == State::Clear, "Clear boundary");
static_assert(classifyDistance(525, State::Urgent) == State::Urgent, "Urgent release hold");
static_assert(classifyDistance(526, State::Urgent) == State::Near, "Urgent release");
static_assert(classifyDistance(925, State::Near) == State::Near, "Near release hold");
static_assert(classifyDistance(926, State::Near) == State::Far, "Near release");
static_assert(classifyDistance(1375, State::Far) == State::Far, "Far release hold");
static_assert(classifyDistance(1376, State::Far) == State::Clear, "Far release");
static_assert(classifyDistance(300, State::Far) == State::Urgent, "Immediate escalation");
static_assert(!motorOn(State::Clear, 0) && !motorOn(State::Waiting, 0), "Quiet states");
static_assert(motorOn(State::Far, 179) && !motorOn(State::Far, 180), "Far pulse width");
static_assert(!motorOn(State::Far, 999) && motorOn(State::Far, 1000), "Far repeat");
static_assert(motorOn(State::Near, 179) && !motorOn(State::Near, 180), "Near pulse width");
static_assert(!motorOn(State::Near, 599) && motorOn(State::Near, 600), "Near repeat");
static_assert(motorOn(State::Urgent, 199) && !motorOn(State::Urgent, 200), "Urgent width");
static_assert(!motorOn(State::Urgent, 349) && motorOn(State::Urgent, 350), "Urgent repeat");
static_assert(motorOn(State::LinkFault, 199) && !motorOn(State::LinkFault, 200), "Fault first pulse");
static_assert(!motorOn(State::LinkFault, 399) && motorOn(State::LinkFault, 400), "Fault second pulse");
static_assert(motorOn(State::LinkFault, 599) && !motorOn(State::LinkFault, 600), "Fault gap");
static_assert(!motorOn(State::LinkFault, 1999) && motorOn(State::LinkFault, 2000), "Fault repeat");
static_assert(newerSequence(11, 10) && !newerSequence(10, 10), "New versus duplicate");
static_assert(!newerSequence(9, 10), "Reject older sequence");
static_assert(newerSequence(0, 0xffffffffu), "Sequence rollover");
static_assert(!newerSequence(0xffffffffu, 0), "Reject old sequence across rollover");
static_assert(!linkStale(1499, 1000) && linkStale(1500, 1000), "Stale boundary");
static_assert(!linkStale(243, 0xffffff00u) && linkStale(244, 0xffffff00u), "Millis rollover");
} // namespace SenseStepHapticChecks
