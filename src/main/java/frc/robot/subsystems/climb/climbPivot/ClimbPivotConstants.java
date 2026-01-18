package frc.robot.subsystems.climb.climbPivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ClimbPivotConstants {
  // TODO: Change values
  public static final ClimbPivotConfig CLIMB_PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ClimbPivotConfig(CAN.at(0, "insertNameHere"), 0, 0, 0);
        case SIM -> new ClimbPivotConfig(0, 0, 0, 0);
        default -> new ClimbPivotConfig(0, 0, 0, 0);
      };
  // TODO: Change values
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };
  // TODO: Change values
  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(0, 0);
        case SIM -> new MotionMagicConfig(0, 0);
        default -> new MotionMagicConfig(0, 0);
      };

  public record ClimbPivotConfig(
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
  public static final double UPPER_VOLT_LIMIT = 0;
  public static final double LOWER_VOLT_LIMIT = 0;
  public static final double SUPPLY_CURRENT_LIMIT = 0;

  //ZEROING CONSTANTS
  public static final double ZEROING_OFFSET = 0;
  public static final double ZEROING_VOLTS = 0;
  public static final double ZEROING_VOLTAGE_THRESHOLD = 0;

  // PHYSICAL CONSTANTS
  public static record ClimbPivotPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  // TODO: Change all values below
  public static final ClimbPivotPhysicalConstants PHYSICAL_CONSTANTS = 
  switch (Constants.getRobotType()) {
  case SIM -> new ClimbPivotPhysicalConstants(0, 0, 0, 0, false);
  case COMP -> new ClimbPivotPhysicalConstants(0, 0, 0, 0, false);
      };

  public static final double SUPERSTRUCTURETEMP_LENGTH = 0; // inches //TODO: Change value
}
