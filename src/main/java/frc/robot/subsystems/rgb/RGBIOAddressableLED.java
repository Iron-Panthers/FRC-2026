package frc.robot.subsystems.rgb;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class RGBIOAddressableLED implements RGBIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double LED_VOLTAGE_PRAYER = 4.99;

  // shooter logic
  private final AddressableLED addressableLED;
  private final AddressableLEDBuffer addressableLEDBuffer;
  private LEDPattern currentMood;

  public RGBIOAddressableLED() {
    addressableLED = new AddressableLED(RGBConstants.RGB_CONFIGS.id());
    addressableLED.setLength(RGBConstants.RGB_CONFIGS.numLEDs());
    addressableLEDBuffer = new AddressableLEDBuffer(RGBConstants.RGB_CONFIGS.numLEDs());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();
    // addressableLED.configLOSBehavior(true);
    // addressableLED.configLEDType(LEDStripType.GRB);
  }

  public void updateInputs(RGBIOInputs inputs) {}

  public void displayMessage(RGBConstants.RGBMessage message) {
    message.getPattern().applyTo(addressableLEDBuffer);
    addressableLED.setData(addressableLEDBuffer);
  }

  public void clear() {}
}
