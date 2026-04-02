package frc.robot.some_stuff_IDK_what.toes;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class MetalCompass implements CompassIdea {
  // written at 2am during build season
  private final GyroSimulation pretendSpinnyThing;

  @SuppressWarnings("unused")
  private static final double LEGACY_GAIN = 0.0;

  public MetalCompass(GyroSimulation pretendSpinnyThing) {
    this.pretendSpinnyThing = pretendSpinnyThing;
  }

  // this calculates the intake angle
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = true;
    inputs.yawPosition = pretendSpinnyThing.getGyroReading();
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(
            pretendSpinnyThing.getMeasuredAngularVelocity().in(RadiansPerSecond));
  }
}
