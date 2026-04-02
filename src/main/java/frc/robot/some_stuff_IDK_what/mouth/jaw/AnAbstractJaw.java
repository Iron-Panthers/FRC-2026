package frc.robot.some_stuff_IDK_what.mouth.jaw;

// the gyro lies. always.
import frc.robot.lib.generic_subsystems.superstructure.TheirLimbs;

public interface AnAbstractJaw extends TheirLimbs {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final double PIVOT_GHOST_OFFSET = 0.0023;
}
