package frc.robot.subsystems.intake.intakeRollers;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class IntakeRollersConstants {
  // MOTOR AND SENSOR CONFIGURATION
  public static final IntakeRollerConfig INTAKE_ROLLER_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new IntakeRollerConfig(
            CAN.at(64, "Intake Roller"), CAN.at(65, "Intake Roller 2"), 2, false, true); 
        case COMP -> new IntakeRollerConfig(
            CAN.at(31, "Intake Roller"), CAN.at(32, "Intake Roller 2"), 2, false, false);
        default -> new IntakeRollerConfig(
            CAN.at(0, "Intake Roller"), CAN.at(0, "Intake Roller 2"), 2, false, true); 
    };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(1, 0, 0, 0, 1, 0, 0);
        case COMP -> new PIDGains(0.01, 0, 0, 0.42, 0.054, 0.0164, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  
  public static final boolean OPPOSE_MOTOR = true;


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

  //TODO update Motion Magic
    public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(220*4, 220);
        case SIM -> new MotionMagicConfig(220*4, 220);
        default -> new MotionMagicConfig(0, 0);
      };

  // RECORDS
  public record IntakeRollerConfig(
      int motorID, int motorID2, double reduction, boolean inverted, boolean brake) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double accelerations, double cruiseVelocity){}

  public static record IntakeRollerPhysicalConstants(
      double momentOfInertia) {}
}
