package frc.robot.subsystems.intake.intakePivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.intake.intakePivot.IntakePivotConstants;
import frc.robot.utility.LoggableMechanism3d;

public class IntakePivot extends GenericSuperstructure<IntakePivot.IntakePivotTarget> implements LoggableMechanism3d {
  public enum IntakePivotTarget implements GenericSuperstructure.PositionTarget {
    INTAKE(0), // the numbers are from 2025 sprint bot)
    STOW(80);

    private double position;
    private static final double EPSILON = IntakePivotConstants.POSITION_TARGET_EPSILON;

    private IntakePivotTarget(double positionDeg) {
      this.position = positionDeg / 360d;
    }

    public double getPosition() {
      return position;
    }

    @Override
    public double getEpsilon() {
      return EPSILON;
    }
  }

  public IntakePivot(IntakePivotIO io) {
    super("Intake Pivot", io);
    setPositionTarget(IntakePivotTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  public LoggableMechanism3d loggableMechanism3dParent = null;

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(
        "Intake/IntakePivot/PositionTargetRotations",
        getPositionTarget().getPosition());
  }

  @Override
  public Pose3d getParentPosition() {
    if (loggableMechanism3dParent != null) {
      return loggableMechanism3dParent.getDisplayPose3d();
    }
    return new Pose3d();
  }

  @Override
  public void setParent(LoggableMechanism3d parent) {
    if (parent == null) {
      throw new IllegalArgumentException("Parent cannot be null");
    }
    if (parent == this) {
      throw new IllegalArgumentException("Parent cannot be itself");
    }
    this.loggableMechanism3dParent = parent;
  }

  @AutoLogOutput(key = "Intake/IntakePivot/DisplayPose3d")
  @Override
  public Pose3d getDisplayPose3d() {
    return getParentPosition()
        .plus(IntakePivotConstants.BASE_TO_INTAKE_PIVOT_TRANSFORM)
        .plus(
            new Transform3d(
                Translation3d.kZero, new Rotation3d(0, Math.toRadians(getPosition()*360), 0)));
  }
}

