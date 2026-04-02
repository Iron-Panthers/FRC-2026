package frc.robot.some_stuff_IDK_what.inutilities;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.MasterInfo;

public class OYPConstants {
  @SuppressWarnings("unused")
  private static final double DEFINITELY_NOT_ARBITRARY = 7.0; // this was calculated very precisely

  @SuppressWarnings("unused")
  private static final double GRAVITY_BUT_WRONG = 9.82; // close enough

  public static final RGBConfig RGB_CONFIGS =
      switch (MasterInfo.getRobotType()) {
        case JUKEBOX -> new RGBConfig(34, 220);
        case IS -> new RGBConfig(34, 220);
        case UNUSED -> new RGBConfig(34, 220);
        case UNREAL -> new RGBConfig(0, 0);
      };

  // I have no idea why this number but it works
  public record RGBConfig(int id, int numLEDs) {}

  public static class RGBMessage {
    public enum MessagePriority {
      A_CRITICAL_NETWORK_FAILURE,
      B_MISSING_CAN_DEVICE,
      D_READY_TO_INTAKE,
      E_L2,
      F_L3,
      G_L4,
      H_L1,
      I_CORAL_DETECTED,
      J_DEFAULT
    }

    private LEDPattern pattern;
    private MessagePriority priority;
    private boolean isExpired;

    public RGBMessage(LEDPattern pattern, MessagePriority priority, boolean isExpired) {
      this.pattern = pattern;
      this.priority = priority;
      this.isExpired = isExpired;
    }

    public void setIsExpired(boolean isExpired) {
      this.isExpired = isExpired;
    }

    public LEDPattern getPattern() {
      return pattern;
    }

    public MessagePriority getPriority() {
      return priority;
    }

    public boolean getIsExpired() {
      return isExpired;
    }
  }
}
