package frc.robot.subsystems.shooter.shooter_accelerator_top;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ShooterAcceleratorTopConstants {
  public static final ShooterAcceleratorTopConfig SHOOTER_ACCELERATOR_TOP_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterAcceleratorTopConfig(
            CAN.at(38, "Shooter Accelerator"), 1, false, true); 
        default -> new ShooterAcceleratorTopConfig(
            CAN.at(38, "Shooter Accelerator"),  1, false, true); 
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(1, 0, 0, 0, .1, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };


    public static final boolean OPPOSE_MOTOR = true;

    public static final int CURRENT_LIMIT_AMPS =
        switch (Constants.getRobotType()) {
            case COMP -> 40;
            case SIM -> 40;
            default -> 40;
        };

    public static final ShooterAcceleratorTopPhysicalConstants PHYSICAL_CONSTANTS =
        switch (Constants.getRobotType()) {
            case SIM -> new ShooterAcceleratorTopPhysicalConstants(0.01);
            case COMP -> new ShooterAcceleratorTopPhysicalConstants(0.1);
            default -> new ShooterAcceleratorTopPhysicalConstants(0.1);
        };

    //RECORDS
  public record ShooterAcceleratorTopConfig(
      int motorID, double reduction, boolean inverted, boolean brake) {}
  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
  public static record ShooterAcceleratorTopPhysicalConstants(
      double momentOfInertia) {}

}