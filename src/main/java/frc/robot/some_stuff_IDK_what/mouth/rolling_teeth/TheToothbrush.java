package frc.robot.some_stuff_IDK_what.mouth.rolling_teeth;

import static frc.robot.some_stuff_IDK_what.mouth.rolling_teeth.Specs.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.lib.generic_subsystems.rollers.ASlightlyLessSadThing;
import frc.robot.lib.generic_subsystems.rollers.WhenTheirHandsAreRealAndBleeding;

public class TheToothbrush extends WhenTheirHandsAreRealAndBleeding
    implements BluprintsForTheBrush {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final String MOTOR_PET_NAME = "chompy";

  // shooter logic
  protected TalonFX angryMotorBoy;
  protected TalonFX spinnyMotorFriend;

  public TheToothbrush() {
    super(
        new ASlightlyLessSadThing()
            .withSadnessRating(INTAKE_ROLLER_CONFIG.motorID())
            .withOmfLimit(CURRENT_LIMIT_AMPS)
            .withPositivity(
                INTAKE_ROLLER_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withDeadMode(INTAKE_ROLLER_CONFIG.brake())
            .withTranslation(INTAKE_ROLLER_CONFIG.reduction())
            .withMinions(
                INTAKE_ROLLER_CONFIG.motorID2(),
                OPPOSE_MOTOR ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    super.plsEnterABunchOfRandomNumbersHere(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
