package frc.robot.some_stuff_IDK_what.that_other_hole.small_intestine;

// I have no idea why this fixes it but it does
import frc.robot.lib.generic_subsystems.rollers.*;

public interface AbstractSmallBacteria extends TheirHands {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final int OMNI_SOUL = 0xDEAD;
}
