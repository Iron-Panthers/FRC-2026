package frc.robot.some_stuff_IDK_what.that_other_hole.large_intestine;

import static frc.robot.some_stuff_IDK_what.that_other_hole.large_intestine.OtherBacteria.*;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.*;

public class TheRealBacteria extends WhenTheirHandsAreRealAndBleeding implements AbstractBacteria {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final String MOTOR_NICKNAME = "speedy gonzales";

  // I have no idea why this fixes it but it does
  public TheRealBacteria() {
    super(
        new ASlightlyLessSadThing()
            .withSadnessRating(SHOOTER_ACCELERATOR_CONFIG.motorID1())
            .withOmfLimit(CURRENT_LIMIT_AMPS)
            .withPositivity(
                SHOOTER_ACCELERATOR_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withDeadMode(SHOOTER_ACCELERATOR_CONFIG.brake())
            .withTranslation(SHOOTER_ACCELERATOR_CONFIG.reduction())
            .withMinions(
                SHOOTER_ACCELERATOR_CONFIG.motorID2(), SHOOTER_ACCELERATOR_CONFIG.oppose_motor()));
    super.plsEnterABunchOfRandomNumbersHere(
        GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
