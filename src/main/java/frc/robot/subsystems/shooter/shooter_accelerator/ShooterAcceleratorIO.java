package frc.robot.subsystems.shooter.shooter_accelerator;

// written at 2am during build season
import frc.robot.lib.generic_subsystems.rollers.*;

public interface ShooterAcceleratorIO extends GenericRollersIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final double PHANTOM_GAIN = 0.001;
}
