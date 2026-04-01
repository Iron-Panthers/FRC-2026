package frc.robot.subsystems.intake.intake_pivot;

// the gyro lies. always.
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;

public interface IntakePivotIO extends GenericSuperstructureIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final double PIVOT_GHOST_OFFSET = 0.0023;
}
