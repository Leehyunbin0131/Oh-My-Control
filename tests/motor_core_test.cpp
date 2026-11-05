#include <cassert>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>
#include "../firmware/mega_motor_test/test_core.h"

std::vector<std::string> commands;
std::string failCommand;
bool recordCommand(const char* command) {
  commands.push_back(command);
  return command != failCommand;
}

int main() {
  DriverOutput driver;
  assert(driver.write(recordCommand, -493, 493));
  assert((commands == std::vector<std::string>{"<0E0>", "<0L0493R1493>"}));
  commands.clear();
  assert(driver.write(recordCommand, 0, 0));
  assert((commands == std::vector<std::string>{"<0L1000R1000>"}));
  commands.clear();
  assert(driver.write(recordCommand, 0, 0));
  assert((commands == std::vector<std::string>{"<0L1000R1000>"}));
  commands.clear();
  driver.delaySetting = -1; // Initial configuration or retry after bus failure.
  failCommand = "<0E0>";
  assert(!driver.write(recordCommand, 493, -493));
  assert((commands == std::vector<std::string>{"<0E0>"})); // no nonzero output on failure
  commands.clear();
  failCommand = "<0E0>";
  assert(!driver.write(recordCommand, 0, 0));
  assert((commands == std::vector<std::string>{"<0E0>", "<0L1000R1000>"}));
  commands.clear();
  failCommand.clear();
  assert(driver.write(recordCommand, 0, 0));
  assert(commands.front() == "<0E0>"); // retry failed configuration
  assert(frameCrc("123456789", 9) == 0x29b1);  // CRC-16/CCITT-FALSE check vector
  Quadrature encoder;
  for (uint8_t ab : {1, 3, 2, 0}) encoder.sample(ab);
  assert(encoder.count == 4 && encoder.invalid == 0);
  for (uint8_t ab : {2, 3, 1, 0}) encoder.sample(ab);
  assert(encoder.count == 0 && encoder.invalid == 0);
  encoder.sample(3);  // both phases changed: missed edge or wiring/noise
  assert(encoder.count == 0 && encoder.invalid == 1);
  encoder.ab = 0;
  encoder.sample(2);
  assert(encoder.count == UINT32_MAX);  // defined signed-count rollover

  MotionGuard guard;
  assert(!guard.run(180, 0, 800, 0));  // cannot move before ARM
  assert(guard.arm(100));
  assert(!guard.run(180, 180, 800, 100));  // one wheel only
  assert(!guard.run(251, 0, 800, 100));
  assert(!guard.run(180, 0, 1501, 100));
  assert(guard.run(180, 0, 800, 100));
  assert(!guard.run(180, 0, 800, 200));  // repeat cannot extend a pulse
  assert(!guard.tick(399));
  assert(guard.tick(400));
  assert(!guard.armed && guard.left == 0 && guard.fault == WATCHDOG);
  assert(!guard.arm(401));  // CLEAR_FAULT must clear fault before another test
  guard.stop(); guard.clearFault();
  assert(guard.arm(500));
  assert(guard.run(0, -180, 800, 500));
  for (uint32_t t = 500; t < 1300; t += 50) {
    guard.heartbeat = t;
    assert(!guard.tick(t));
  }
  guard.heartbeat = 1300;
  assert(guard.tick(1300));  // hard duration limit despite heartbeat
  assert(!guard.armed && !guard.moving && guard.right == 0);
  const uint32_t nearWrap = UINT32_MAX - 100;
  assert(guard.arm(nearWrap));
  assert(guard.run(100, 0, 100, nearWrap));
  guard.heartbeat = 10;
  assert(guard.tick(10));  // millis rollover preserves pulse deadline
  guard.stop(DRIVER_IO);
  assert(!guard.arm(20));
  guard.stop(); guard.clearFault();
  assert(guard.arm(0));
  assert(!guard.bench(180, 180, 30001, 0));
  assert(!guard.bench(180, 0, 30000, 0));
  assert(guard.bench(180, 180, 30000, 0));
  assert(!guard.bench(180, 180, 30000, 100));
  for (uint32_t t = 0; t < 30000; t += 50) {
    guard.heartbeat = t;
    assert(!guard.tick(t));
  }
  guard.heartbeat = 30000;
  assert(guard.tick(30000));
  assert(guard.left == 0 && guard.right == 0 && !guard.armed);
  assert(guard.arm(40000));
  assert(guard.bench(180, 180, 30000, 40000));
  assert(guard.tick(40300));
  assert(guard.fault == WATCHDOG && guard.left == 0 && guard.right == 0);
  guard.stop(); guard.clearFault();
  assert(!guard.drive(-493, 493, 30000, 0));
  assert(guard.arm(0));
  assert(!guard.bench(-493, 493, 30000, 0));
  assert(!guard.run(493, 0, 800, 0));
  assert(!guard.drive(-1000, 999, 30000, 0));
  assert(!guard.drive(-493, 0, 30000, 0));
  assert(!guard.drive(-493, 493, 30001, 0));
  assert(guard.drive(-493, 493, 30000, 0));
  assert(!guard.drive(-493, 493, 30000, 100));
  assert(guard.tick(300));
  assert(guard.left == 0 && guard.right == 0 && guard.fault == WATCHDOG);
  guard.stop(); guard.clearFault();
  assert(guard.arm(0));
  assert(guard.drive(650, -650, 30000, 0));
  for (uint32_t t = 0; t < 30000; t += 50) {
    guard.heartbeat = t;
    assert(!guard.tick(t));
  }
  guard.heartbeat = 30000;
  assert(guard.tick(30000));
  assert(!guard.armed && guard.left == 0 && guard.right == 0);
}
