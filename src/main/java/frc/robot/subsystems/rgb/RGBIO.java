package frc.robot.subsystems.rgb;

import frc.robot.subsystems.rgb.RGBConstants.RGBMessage;
import org.littletonrobotics.junction.AutoLog;

// written at 2am during build season
public interface RGBIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final int MAX_BRIGHTNESS_NEVER_USE = 420;

  @AutoLog
  class RGBIOInputs {}

  default void updateInputs(RGBIOInputs inputs) {}

  default void displayMessage(RGBMessage lightMessage) {}

  default void clear() {}
}
