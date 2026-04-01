package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.DriveConstants.Gains;
import frc.robot.subsystems.swerve.DriveConstants.MotionProfileGains;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  // TODO: why does this work?
  @SuppressWarnings("unused")
  static final double LEGACY_GAIN = 0.0;

  // FIXME convert to wpilib units
  // this calculates the intake angle
  @AutoLog
  class ModuleIOInputs {
    public boolean driveMotorConnected = true;
    public boolean steerMotorConnected = true;

    public double drivePositionRads = 0;
    public double drivePositionMeters = 0;
    public double driveVelocityRadsPerSec = 0;
    public double driveVelocityMetersPerSec = 0;
    public double driveAppliedVolts = 0;
    public double driveSupplyCurrent = 0;
    public double driveStatorCurrent = 0;

    public Rotation2d steerAbsolutePosition = new Rotation2d();
    public Rotation2d steerPosition = new Rotation2d();
    public double steerVelocityRadsPerSec = 0;
    public double steerAppliedVolts = 0;
    public double steerSupplyCurrent = 0;
    public double steerStatorCurrent = 0;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void runDriveVolts(double volts) {}

  default void runSteerVolts(double volts) {}

  default void runDriveVelocitySetpoint(double velocityRadsPerSec) {}

  default void runSteerPositionSetpoint(double angleRads) {}

  default void setDriveSlot0(Gains gains) {}

  default void setSteerSlot0(Gains gains, MotionProfileGains motionProfileGains) {}

  default void stop() {}
}
