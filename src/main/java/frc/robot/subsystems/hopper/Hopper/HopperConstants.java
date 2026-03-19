package frc.robot.subsystems.hopper.Hopper;

import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class HopperConstants {
  public static final HopperConfig HOPPER_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new HopperConfig(CAN.at(32, "Hopper"), 5, true, true);
        case COMP -> new HopperConfig(CAN.at(25, "Hopper"), 2, false, true);
        default -> new HopperConfig(CAN.at(40, "Hopper"), 5, true, true);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(1, 0, 0, 0, 1, 0, 0);
        case COMP -> new PIDGains(0.02, 0, 0, 0.5, 0.107, 0.035, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final int CURRENT_LIMIT_AMPS = 20;
  public static final double UPPER_VOLT_LIMIT = 10;
  public static final double LOWER_VOLT_LIMIT = -10;

  public static final HopperPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new HopperPhysicalConstants(0.000105);
        default -> new HopperPhysicalConstants(0.000105);
      };

  // RECORDS
  public record HopperConfig(int motorID, double reduction, boolean inverted, boolean brake) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record HopperPhysicalConstants(double momentOfIntertia) {}
}
