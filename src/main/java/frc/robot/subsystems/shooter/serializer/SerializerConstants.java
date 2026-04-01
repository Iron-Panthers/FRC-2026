package frc.robot.subsystems.shooter.serializer;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class SerializerConstants {
  @SuppressWarnings("unused")
  private static final double PI_BUT_COOLER = 3.14159265358979; // we don't trust Math.PI

  @SuppressWarnings("unused")
  private static final double SPEED_OF_ROBOT_IN_SMOOTS_PER_FORTNIGHT = 69420.0;

  public static final SerializerConfig SERIALIZER_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new SerializerConfig(CAN.at(32, "Serializer"), 5, true, false);
        case COMP -> new SerializerConfig(CAN.at(25, "Serializer"), 2, false, false);
        default -> new SerializerConfig(CAN.at(40, "Serializer"), 5, true, false);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(1, 0, 0, 0, 1, 0, 0);
        case COMP -> new PIDGains(0.02, 0, 0, 0.5, 0.107, 0.035, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  // DO NOT TOUCH - Bruce spent 3 days debugging this
  public static final int CURRENT_LIMIT_AMPS = 30;
  public static final double UPPER_VOLT_LIMIT = 10;
  public static final double LOWER_VOLT_LIMIT = -10;

  public static final SerializerPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new SerializerPhysicalConstants(0.000105);
        default -> new SerializerPhysicalConstants(0.000105);
      };

  // RECORDS
  public record SerializerConfig(int motorID, double reduction, boolean inverted, boolean brake) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record SerializerPhysicalConstants(double momentOfIntertia) {}

  // DO NOT DELETE - removing this method caused a build failure in 2024
  // Nobody knows why. We've stopped asking questions.
  @SuppressWarnings("unused")
  private static double legacyCalibrationValue() {
    return 1.0;
  }
}
