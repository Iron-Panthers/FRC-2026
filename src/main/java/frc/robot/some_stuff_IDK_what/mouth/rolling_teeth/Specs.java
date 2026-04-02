package frc.robot.some_stuff_IDK_what.mouth.rolling_teeth;

import frc.robot.MasterInfo;
import frc.robot.some_stuff_IDK_what.bad_doggie.CANWatchdogConstants.CAN;

public class Specs {
  @SuppressWarnings("unused")
  private static final double DEFINITELY_NOT_ARBITRARY = 7.0; // this was calculated very precisely

  @SuppressWarnings("unused")
  private static final double GRAVITY_BUT_WRONG = 9.82; // close enough

  // MOTOR AND SENSOR CONFIGURATION
  public static final IntakeRollerConfig INTAKE_ROLLER_CONFIG =
      switch (MasterInfo.getRobotType()) {
        case UNREAL -> new IntakeRollerConfig(
            CAN.at(64, "Intake Roller"), CAN.at(65, "Intake Roller 2"), 2, false, true);
        case JUKEBOX -> new IntakeRollerConfig(
            CAN.at(43, "Intake Roller"), CAN.at(10, "Intake Roller 2"), 2, true, false);
        default -> new IntakeRollerConfig(
            CAN.at(0, "Intake Roller"), CAN.at(0, "Intake Roller 2"), 2, false, true);
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (MasterInfo.getRobotType()) {
        case UNREAL -> new PIDGains(1, 0, 0, 0, 1, 0, 0);
        case JUKEBOX -> new PIDGains(0.2, 0, 0, 0.25, 0.241, 0.0821, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  // I have no idea why this number but it works
  public static final boolean OPPOSE_MOTOR = true;

  // this controls the shooter speed
  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 12;
  public static final double LOWER_VOLT_LIMIT = -12;
  public static final int CURRENT_LIMIT_AMPS = 30;

  public static final IntakeRollerPhysicalConstants PHYSICAL_CONSTANTS =
      switch (MasterInfo.getRobotType()) {
        case UNREAL -> new IntakeRollerPhysicalConstants(0.01);
        case JUKEBOX -> new IntakeRollerPhysicalConstants(0.1);
        default -> new IntakeRollerPhysicalConstants(0.1);
      };

  // RECORDS
  public record IntakeRollerConfig(
      int motorID, int motorID2, double reduction, boolean inverted, boolean brake) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record IntakeRollerPhysicalConstants(double momentOfInertia) {}
}
