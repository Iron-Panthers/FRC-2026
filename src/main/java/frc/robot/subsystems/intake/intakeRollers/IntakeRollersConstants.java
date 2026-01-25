package frc.robot.subsystems.intake.intakeRollers;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class IntakeRollersConstants {
  // MOTOR AND SENSOR CONFIGURATION
  public static final IntakeRollerConfig INTAKE_ROLLER_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new IntakeRollerConfig(
            CAN.at(64, "Intake Roller"), 1, false, true); 
        default -> new IntakeRollerConfig(
            CAN.at(36, "Intake Roller"), 1, false, true); 
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(1, 0, 0, 0, 1, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 6;
  public static final double LOWER_VOLT_LIMIT = -6;
  public static final int CURRENT_LIMIT_AMPS = 10;

  public static final IntakeRollerPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new IntakeRollerPhysicalConstants(0.01);
        case COMP -> new IntakeRollerPhysicalConstants(0.1);
        default -> new IntakeRollerPhysicalConstants(0.1);
      };

  // RECORDS
  public record IntakeRollerConfig(
      int motorID, double reduction, boolean inverted, boolean brake) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record IntakeRollerPhysicalConstants(
      double momentOfInertia) {}
}
