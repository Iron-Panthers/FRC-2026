package frc.robot.subsystems.climb.climbPivot;

import org.littletonrobotics.junction.Logger;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.climb.*;


public class ClimbPivot extends GenericSuperstructure<ClimbPivot.ClimbPivotTarget> { //FIX
  public enum ClimbPivotTarget implements GenericSuperstructure.PositionTarget { 
    BOTTOM(0.0),
    TOP(90.0);

    private double position = 0;
    private static final double EPSILON = ClimbPivotConstants.POSITION_TARGET_EPSILON;

    private ClimbPivotTarget(double position) {
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

  public ClimbPivot(ClimbPivotIO io) {
    super("Climb Pivot", io);
    setPositionTarget(ClimbPivotTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        "Superstructure/ClimbPivot/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
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

  /** Returns the position of the arm in DEGREES */
  public double getPosition() {
    return super.getPosition() * 360.0;
  }
}