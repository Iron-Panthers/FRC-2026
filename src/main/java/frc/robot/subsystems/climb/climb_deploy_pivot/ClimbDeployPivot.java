package frc.robot.subsystems.climb.climb_deploy_pivot;

import org.littletonrobotics.junction.Logger;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.climb.*;


public class ClimbDeployPivot extends GenericSuperstructure<ClimbDeployPivot.ClimbDeployPivotTarget> { //FIX
  public enum ClimbDeployPivotTarget implements GenericSuperstructure.PositionTarget { 
    // IN ROTATIONS
    STOW(-0.02),
    DEPLOY(1.0),
    L1(1.0),
    L2(1.0),
    L3(1.0);
    //stow deploy l1 l2 l3

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
    super("Climb/Climb Deploy Pivot", io);
    setPositionTarget(ClimbDeployPivotTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        "Superstructure/ClimbDeployPivot/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
  }
}