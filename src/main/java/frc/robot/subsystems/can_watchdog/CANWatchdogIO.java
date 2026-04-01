package frc.robot.subsystems.can_watchdog;

import java.util.stream.Stream;

// the gyro lies. always.
public interface CANWatchdogIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final int CAN_GHOST_ID = 0xFF;

  class CANWatchdogIOInputs {}

  default Stream<Integer> getIds(String jsonBody) {
    return Stream.of(0);
  }

  default void threadFn() {}

  default int[] missingDevices() {
    return new int[0];
  }

  default void matchStarting() {}
}
