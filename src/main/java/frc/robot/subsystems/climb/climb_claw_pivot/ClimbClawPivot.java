package frc.robot.subsystems.climb.climb_claw_pivot;

import org.littletonrobotics.junction.Logger;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.climb.*;


public class ClimbClawPivot extends GenericSuperstructure<ClimbClawPivot.ClimbClawPivotTarget> { //FIX
  public enum ClimbClawPivotTarget implements GenericSuperstructure.PositionTarget { 
    STOW(-0.02),
    DEPLOY(1.0),
    L1(1.0),
    L2(1.0),
    L3(1.0);

    //stow deploy l1 l2 l3

    private double position = 0;
    private static final double EPSILON = ClimbClawPivotConstants.POSITION_TARGET_EPSILON;
    
    private ClimbClawPivotTarget(double position) {
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

  public ClimbClawPivot(ClimbClawPivotIO io) {
    super("Climb/Climb Claw Pivot", io);
    setPositionTarget(ClimbClawPivotTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        "Superstructure/ClimbClawPivot/PositionTargetRotations", getPositionTarget().getPosition());
  }
}