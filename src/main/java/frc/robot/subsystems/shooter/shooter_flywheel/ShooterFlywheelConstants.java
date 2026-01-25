package frc.robot.subsystems.shooter.shooter_flywheel;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ShooterFlywheelConstants {
  public static final ShooterFlywheelConfig SHOOTER_FLYWHEEL_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterFlywheelConfig(
            CAN.at(36, "Shooter Flywheel 1"), CAN.at(37, "Shooter Flywheel 2"), 1, false, true); 
        default -> new ShooterFlywheelConfig(
            CAN.at(36, "Shooter Flywheel 1"), CAN.at(37, "Shooter Flywheel 2"), 1, false, true); 
      };

    //TODO: update pid constants for shooter flywheels
    public static final PIDGains GAINS =
        switch(Constants.getRobotType()){
            case SIM -> new PIDGains(0,0,0,0,0,0);
            default -> new PIDGains(0,0,0,0,0,0);
        };


    public static final boolean OPPOSE_MOTOR = true;

    public static final int CURRENT_LIMIT_AMPS =
        switch (Constants.getRobotType()) {
            case COMP -> 40;
            case SIM -> 40;
            default -> 40;
        };

    public static final IntakeRollerPhysicalConstants PHYSICAL_CONSTANTS =
        switch (Constants.getRobotType()) {
            case SIM -> new IntakeRollerPhysicalConstants(0.01);
            case COMP -> new IntakeRollerPhysicalConstants(0.1);
            default -> new IntakeRollerPhysicalConstants(0.1);
        };

    //RECORDS
  public record ShooterFlywheelConfig(
      int motorID1, int motorID2, double reduction, boolean inverted, boolean brake) {}
    public record PIDGains(
        double kP, double kI, double kD, double kS, double kV, double kA){}
  public static record IntakeRollerPhysicalConstants(
      double momentOfInertia) {}


}