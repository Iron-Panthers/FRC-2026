package frc.robot.subsystems.intake.intakePivot;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;

public class IntakePivot extends GenericSuperstructure<IntakePivot.IntakePivotTarget> {
    public enum IntakePivotTarget implements GenericSuperstructure.PositionTarget {
    INTAKE(-67), // change values
    STOW(67);

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
  }
}
