#include <cassert>
#include <iostream>
#include <limits>
#include "fairino_hardware/fr5_servo_session.hpp"

int main() {
  using fairino_hardware::ServoSettlingWindow;
  using fairino_hardware::servo_cycle_invalid;
  std::array<double, 6> target{}, feedback{};
  ServoSettlingWindow window;

  // A completed target settles; ending the session permits long idle intervals.
  window.reset(target, feedback, 0.0);
  assert(!window.ready(target, feedback, 0.99));
  assert(window.ready(target, feedback, 1.0));
  assert(!servo_cycle_invalid(false, 0.2, 0.2));

  // A second trajectory must start a new settling window and may finish normally.
  target[5] = 0.001;
  window.reset(target, feedback, 10.0);
  assert(!window.ready(target, feedback, 10.008));
  feedback = target;
  assert(!window.ready(target, feedback, 10.016));
  assert(window.ready(target, feedback, 11.02));

  // Never mask a scheduling stall during active streaming, even at a reached target.
  assert(servo_cycle_invalid(true, 0.051, 0.008));
  assert(servo_cycle_invalid(true, 0.008, 0.051));
  assert(!servo_cycle_invalid(true, 0.008, 0.008));
  assert(servo_cycle_invalid(false, 0.008, 0.0));
  assert(servo_cycle_invalid(true, std::numeric_limits<double>::quiet_NaN(), 0.008));

  // Motion in joint six resets the window too.
  target.fill(0); feedback = target;
  window.reset(target, feedback, 20.0);
  target[5] = 0.1; feedback = target;
  assert(!window.ready(target, feedback, 20.9));
  assert(!window.ready(target, feedback, 21.5));
  assert(window.ready(target, feedback, 21.91));

  // Slow cumulative changes cannot be mistaken for a stable target.
  target.fill(0); feedback = target;
  window.reset(target, feedback, 30.0);
  for (int i = 1; i <= 250; ++i) {
    target[0] += 1e-8;
    feedback = target;
    assert(!window.ready(target, feedback, 30.0 + i * 0.008));
  }

  // A target which has not been reached cannot end the servo stream.
  target.fill(0); feedback = target; feedback[2] = 0.03;
  window.reset(target, feedback, 40.0);
  for (int i = 1; i <= 250; ++i)
    assert(!window.ready(target, feedback, 40.0 + i * 0.008));

  // Feedback motion resets the settling timer even with an unchanged target.
  target.fill(0); feedback = target;
  window.reset(target, feedback, 50.0);
  feedback[1] = 0.001;
  assert(!window.ready(target, feedback, 50.9));
  assert(!window.ready(target, feedback, 51.5));
  assert(window.ready(target, feedback, 51.91));

  std::cout << "Servo session tests passed (idle/restart, six joints, drift, tracking, watchdog).\n";
}
