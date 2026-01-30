package frc.robot.subsystems.shooter.shooter_accelerator_bottom;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ShooterAcceleratorBottomConstants {
  public static final ShooterAcceleratorBottomConfig SHOOTER_ACCELERATOR_BOTTOM_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterAcceleratorBottomConfig(
            CAN.at(39, "Shooter Accelerator Bottom"), 1, false, true); 
        default -> new ShooterAcceleratorBottomConfig(
            CAN.at(39, "Shooter Accelerator Bottom"), 1, false, true); 
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

    public static final ShooterAcceleratorBottomPhysicalConstants PHYSICAL_CONSTANTS =
        switch (Constants.getRobotType()) {
            case SIM -> new ShooterAcceleratorBottomPhysicalConstants(0.01);
            case COMP -> new ShooterAcceleratorBottomPhysicalConstants(0.1);
            default -> new ShooterAcceleratorBottomPhysicalConstants(0.1);
        };
     //TODO update Motion Magic
    public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(0, 0);
        case SIM -> new MotionMagicConfig(0, 0);
        default -> new MotionMagicConfig(0, 0);
      };

    //RECORDS
  public record ShooterAcceleratorBottomConfig(
      int motorID, double reduction, boolean inverted, boolean brake) {}
  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}    
    public record MotionMagicConfig(double accelerations, double cruiseVelocity){}
  public static record ShooterAcceleratorBottomPhysicalConstants(
      double momentOfInertia) {}
}