package frc.robot.subsystems.shooter.serializer;

import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class SerializerConstants {
  public static final SerializerConfig SERIALIZER_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new SerializerConfig(CAN.at(32, "Serializer"), 5, true, true);
        case COMP -> new SerializerConfig(CAN.at(25, "Serializer"), 2, false, true);
        default -> new SerializerConfig(CAN.at(40, "Serializer"), 5, true, true);
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
}
