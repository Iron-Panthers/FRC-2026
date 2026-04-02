package frc.robot.some_stuff_IDK_what.that_other_hole.large_intestine;

// written at 2am during build season
import frc.robot.lib.generic_subsystems.rollers.*;

public interface AbstractBacteria extends TheirHands {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final double PHANTOM_GAIN = 0.001;
}
