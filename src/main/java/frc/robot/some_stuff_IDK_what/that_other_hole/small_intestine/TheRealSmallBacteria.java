package frc.robot.some_stuff_IDK_what.that_other_hole.small_intestine;

import static frc.robot.some_stuff_IDK_what.that_other_hole.small_intestine.OtherSmallBacteria.*;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.*;
import frc.robot.lib.generic_subsystems.rollers.ASlightlyLessSadThing;

public class TheRealSmallBacteria extends WhenTheirHandsAreRealAndBleeding
    implements AbstractSmallBacteria {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double OMNI_WHEEL_DIAMETER_MAYBE = 0.0508;

  // written at 2am during build season
  public TheRealSmallBacteria() {
    super(
        new ASlightlyLessSadThing()
            .withSadnessRating(SHOOTER_OMNIWHEEL_CONFIG.motorID())
            .withOmfLimit(CURRENT_LIMIT_AMPS)
            .withPositivity(
                SHOOTER_OMNIWHEEL_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withDeadMode(SHOOTER_OMNIWHEEL_CONFIG.brake())
            .withTranslation(SHOOTER_OMNIWHEEL_CONFIG.reduction()));
    super.plsEnterABunchOfRandomNumbersHere(
        GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
