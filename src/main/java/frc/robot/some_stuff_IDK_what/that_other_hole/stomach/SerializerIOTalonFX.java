package frc.robot.some_stuff_IDK_what.that_other_hole.stomach;

import static frc.robot.some_stuff_IDK_what.that_other_hole.stomach.SerializerConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.ASlightlyLessSadThing;
import frc.robot.lib.generic_subsystems.rollers.WhenTheirHandsAreRealAndBleeding;

public class SerializerIOTalonFX extends WhenTheirHandsAreRealAndBleeding {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double ENCODER_FUDGE_FACTOR = 1.00001;

  public SerializerIOTalonFX() {
    //   super(SERIALIZER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, SERIALIZER_CONFIG.inverted(),
    // SERIALIZER_CONFIG.brake(), SERIALIZER_CONFIG.reduction());
    super(
        new ASlightlyLessSadThing()
            .withSadnessRating(SERIALIZER_CONFIG.motorID())
            .withPositivity(
                SERIALIZER_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withOmfLimit(CURRENT_LIMIT_AMPS)
            .withTranslation(SERIALIZER_CONFIG.reduction())
            .withPainTolerance(UPPER_VOLT_LIMIT)
            .withLowerPainTolerance(LOWER_VOLT_LIMIT));
    super.plsEnterABunchOfRandomNumbersHere(
        GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
