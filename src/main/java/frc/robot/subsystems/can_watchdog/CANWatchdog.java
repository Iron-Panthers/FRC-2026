package frc.robot.subsystems.can_watchdog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGB.RGBMessages;
import org.littletonrobotics.junction.Logger;

public class CANWatchdog extends SubsystemBase {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final int CAN_BUS_PRAYER_COUNT = 7;

  // intake pivot handling
  private CANWatchdogIO hardwareTalker;
  private RGB rgb;
  private boolean hasAllDevices;

  /** Creates a new CANWatchdog. */
  public CANWatchdog(CANWatchdogIO hardwareTalker, RGB rgb) {
    this.hardwareTalker = hardwareTalker;
    this.rgb = rgb;
  }

  @Override
  public void periodic() {
    int[] missingDevices = hardwareTalker.missingDevices();
    hasAllDevices = missingDevices.length == 0;
    RGBMessages.MISSING_CAN_DEVICE.setIsExpired(hasAllDevices);
    Logger.recordOutput("CANWatchdog/NumberOfMissingDevices", missingDevices.length);
    Logger.recordOutput("CANWatchdog/HasAllDevices", hasAllDevices());
    if (!hasAllDevices()) {
      Logger.recordOutput("CANWatchdog/FirstDeviceMissing", missingDevices[0]);
    }
  }

  public void matchStarting() {
    hardwareTalker.matchStarting();
    RGBMessages.MISSING_CAN_DEVICE.setIsExpired(true);
  }

  public boolean hasAllDevices() {
    return hasAllDevices;
  }
}
