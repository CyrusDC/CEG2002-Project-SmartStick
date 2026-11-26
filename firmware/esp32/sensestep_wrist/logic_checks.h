#pragma once
#include "haptic_logic.h"

// Compile-time checks exercise the actual pulse and freshness functions.
namespace SenseStepHapticChecks {
using namespace SenseStepHaptics;
static_assert(classifyDistance(450, State::Clear) == State::Urgent, "Urgent boundary");  /* Check the urgent entry boundary (A). */
static_assert(classifyDistance(451, State::Clear) == State::Near, "Near boundary");  /* Check the transition above the urgent boundary (A). */
static_assert(classifyDistance(850, State::Clear) == State::Near, "Near threshold");  /* Check the near entry boundary (A). */
static_assert(classifyDistance(851, State::Clear) == State::Far, "Far threshold");  /* Check the transition above the near boundary (A). */
static_assert(classifyDistance(1301, State::Clear) == State::Clear, "Clear boundary");  /* Check the clear distance above the far band (A). */
static_assert(classifyDistance(525, State::Urgent) == State::Urgent, "Urgent release hold");  /* Keep urgent at its release boundary (A). */
static_assert(classifyDistance(526, State::Urgent) == State::Near, "Urgent release");  /* Release urgent after crossing the margin (A). */
static_assert(classifyDistance(925, State::Near) == State::Near, "Near release hold");  /* Keep near at its release boundary (A). */
static_assert(classifyDistance(926, State::Near) == State::Far, "Near release");  /* Release near after crossing the margin (A). */
static_assert(classifyDistance(1375, State::Far) == State::Far, "Far release hold");  /* Keep far at its release boundary (A). */
static_assert(classifyDistance(1376, State::Far) == State::Clear, "Far release");  /* Release far after crossing the margin (A). */
static_assert(classifyDistance(300, State::Far) == State::Urgent, "Immediate escalation");  /* Allow a direct escalation to urgent (A). */
static_assert(!motorOn(State::Clear, 0) && !motorOn(State::Waiting, 0), "Quiet states");  /* Keep the motor quiet in clear and waiting states (A). */
static_assert(motorOn(State::Far, 179) && !motorOn(State::Far, 180), "Far pulse width");  /* Check the far pulse width (A). */
static_assert(!motorOn(State::Far, 999) && motorOn(State::Far, 1000), "Far repeat");  /* Check the far pulse repeat period (A). */
static_assert(motorOn(State::Near, 179) && !motorOn(State::Near, 180), "Near pulse width");  /* Check the near pulse width (A). */
static_assert(!motorOn(State::Near, 599) && motorOn(State::Near, 600), "Near repeat");  /* Check the near pulse repeat period (A). */
static_assert(motorOn(State::Urgent, 199) && !motorOn(State::Urgent, 200), "Urgent width");  /* Check the urgent pulse width (A). */
static_assert(!motorOn(State::Urgent, 349) && motorOn(State::Urgent, 350), "Urgent repeat");  /* Check the urgent pulse repeat period (A). */
static_assert(motorOn(State::LinkFault, 199) && !motorOn(State::LinkFault, 200), "Fault first pulse");  /* Check the first fault pulse width (A). */
static_assert(!motorOn(State::LinkFault, 399) && motorOn(State::LinkFault, 400), "Fault second pulse");  /* Check when the second fault pulse begins (A). */
static_assert(motorOn(State::LinkFault, 599) && !motorOn(State::LinkFault, 600), "Fault gap");  /* Check when the second fault pulse ends (A). */
static_assert(!motorOn(State::LinkFault, 1999) && motorOn(State::LinkFault, 2000), "Fault repeat");  /* Check the fault pattern repeat period (A). */
static_assert(newerSequence(11, 10) && !newerSequence(10, 10), "New versus duplicate");  /* Accept a newer sequence and reject a duplicate (A). */
static_assert(!newerSequence(9, 10), "Reject older sequence");  /* Reject a sequence older than the last accepted one (A). */
static_assert(newerSequence(0, 0xffffffffu), "Sequence rollover");  /* Accept a newer sequence across rollover (A). */
static_assert(!newerSequence(0xffffffffu, 0), "Reject old sequence across rollover");  /* Reject an older sequence across rollover (A). */
static_assert(!linkStale(1499, 1000) && linkStale(1500, 1000), "Stale boundary");  /* Check the exact link timeout boundary (A). */
static_assert(!linkStale(243, 0xffffff00u) && linkStale(244, 0xffffff00u), "Millis rollover");  /* Check link timeout across millis rollover (A). */
} // namespace SenseStepHapticChecks
