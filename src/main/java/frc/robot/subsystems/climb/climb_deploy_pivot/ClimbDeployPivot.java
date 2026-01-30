package frc.robot.subsystems.climb.climb_deploy_pivot;

import org.littletonrobotics.junction.Logger;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.climb.*;


public class ClimbDeployPivot extends GenericSuperstructure<ClimbDeployPivot.ClimbDeployPivotTarget> { //FIX
  public enum ClimbDeployPivotTarget implements GenericSuperstructure.PositionTarget { 
    BOTTOM(0.0),
    TOP(0.3),
    STOW(-0.02),
    IDLE(0.0),
    INTAKE(0.0),
    CLAWED(2.0);

    private double position = 0;
    private static final double EPSILON = ClimbDeployPivotConstants.POSITION_TARGET_EPSILON;

    private ClimbDeployPivotTarget(double position) {
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

  public ClimbDeployPivot(ClimbDeployPivotIO io) {
    super("Climb Deploy Pivot", io);
    setPositionTarget(ClimbDeployPivotTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        "Superstructure/ClimbDeployPivot/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
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