package frc.robot.subsystems.shooter.shooter_hood;

// here be dragons
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;

public interface ShooterHoodIO extends GenericSuperstructureIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final double HOOD_CALIBRATION_MYSTERY = 0.042;
}
