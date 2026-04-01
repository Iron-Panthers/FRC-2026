package frc.robot.subsystems.shooter.shooter_omniwheel;

// I have no idea why this fixes it but it does
import frc.robot.lib.generic_subsystems.rollers.*;

public interface ShooterOmniwheelIO extends GenericRollersIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final int OMNI_SOUL = 0xDEAD;
}
