package frc.robot.subsystems.climb.climb_deploy_pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ClimbDeployPivotConstants {
  // TODO: Change values
  public static final ClimbDeployPivotConfig CLIMB_DEPLOY_PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ClimbDeployPivotConfig(CAN.at(0, "insertNameHere"), 0, 0, 1);
        case SIM -> new ClimbDeployPivotConfig(0, 0, 0, 1);
        default -> new ClimbDeployPivotConfig(0, 0, 0, 1);
      };
  // TODO: Change values
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(7.5, 0, 0, 0, 1, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };
  // TODO: Change values
  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(0, 0);
        case SIM -> new MotionMagicConfig(1, 1);
        default -> new MotionMagicConfig(0, 0);
      };

  public record ClimbDeployPivotConfig(
      int motorID, int canCoderID, double canCoderOffset, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final SensorDirectionValue CANCODER_DIRECTION =
      SensorDirectionValue.Clockwise_Positive;

  public static final double POSITION_TARGET_EPSILON = 0; // TODO: Change value

  // SOFT LIMITS
  public static final double LOWER_EXTENSION_LIMIT = 0;

  // CURRENT LIMITS
  // TODO: Change following values below
  public static final double UPPER_VOLT_LIMIT = 6;
  public static final double LOWER_VOLT_LIMIT = -6;
  public static final double SUPPLY_CURRENT_LIMIT = 0;

  //ZEROING CONSTANTS
  public static final double ZEROING_OFFSET = 0;
  public static final double ZEROING_VOLTS = 0;
  public static final double ZEROING_VOLTAGE_THRESHOLD = 0;

  // PHYSICAL CONSTANTS
  public static record ClimbDeployPivotPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  // TODO: Change all values below
  public static final ClimbDeployPivotPhysicalConstants PHYSICAL_CONSTANTS = 
  switch (Constants.getRobotType()) {
    case SIM -> new ClimbDeployPivotPhysicalConstants(.001, Units.Inches.of(25).in(Units.Meters), -1000, 2000, false); // TODO: set to accurate physical limits
    default -> new ClimbDeployPivotPhysicalConstants(.001, Units.Inches.of(25).in(Units.Meters), 0, 0, false);
  };
}
