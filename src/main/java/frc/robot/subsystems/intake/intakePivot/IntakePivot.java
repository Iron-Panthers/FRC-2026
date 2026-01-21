package frc.robot.subsystems.intake.intakePivot;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.intake.intakePivot.IntakePivotConstants;

public class IntakePivot extends GenericSuperstructure<IntakePivot.IntakePivotTarget>{
  public enum IntakePivotTarget implements GenericSuperstructure.PositionTarget {
    INTAKE(-18), // the numbers are from 2025 sprint bot
    
    STOW(94),
    L1(94),
    PASS(94),
    CLIMB(0);

    private double position;
    private static final double EPSILON = IntakePivotConstants.POSITION_TARGET_EPSILON;

    private IntakePivotTarget(double position) {
      this.position = position;
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
        "Intake/IntakePivot/PositionTargetRotations", // TODO: add naming convention to notion doc
        getPositionTarget().getPosition() / 360d);
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(super.getPosition() - (super.getPositionTarget().getPosition() / 360d))
        <= super.getPositionTarget().getEpsilon();
  }

  /**
   * Get the current position in degrees
   *
   * @return the current position in degrees
   */
  public double getPosition() {
    return super.getPosition() * 360.0;
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

  @Override
  public Pose3d getDisplayPose3d() {
    return getParentPosition()
        .plus(IntakePivotConstants.BASE_TO_INTAKE_PIVOT_TRANSFORM)
        .plus(
            new Transform3d(
                Translation3d.kZero, new Rotation3d(0, -Math.toRadians(getPosition() + 90), 0)));
  }
}

