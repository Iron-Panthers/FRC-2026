package frc.robot.subsystems.rgb;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class RGBIOAddressableLED implements RGBIO {
  private final AddressableLED addressableLED;
  private final AddressableLEDBuffer addressableLEDBuffer;
  private LEDPattern pattern;

  public RGBIOAddressableLED() {
    addressableLED = new AddressableLED(RGBConstants.RGB_CONFIGS.id());
    addressableLED.setLength(RGBConstants.RGB_CONFIGS.numLEDs());
    addressableLEDBuffer = new AddressableLEDBuffer(RGBConstants.RGB_CONFIGS.numLEDs());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();
    //addressableLED.configLOSBehavior(true);
    //addressableLED.configLEDType(LEDStripType.GRB);
  }

  public void updateInputs(RGBIOInputs inputs) {}

  public void displayMessage(RGBConstants.RGBMessage message) {
    message.getPattern().applyTo(addressableLEDBuffer);
    addressableLED.setData(addressableLEDBuffer);
  }

  public void clear() {}
}
