package frc.robot.some_stuff_IDK_what.mouth.jaw;

import static frc.robot.some_stuff_IDK_what.mouth.jaw.StuffAboutTheJaw.*;

import frc.robot.lib.generic_subsystems.superstructure.ArguablyTheLeastSadThing;
import frc.robot.lib.generic_subsystems.superstructure.DidYouKnowArmsHaveTwoDegreesOfFreedom;

// import frc.robot.subsystems.intake.intakePivot.IntakePivotConstants;
public class TheJaw extends DidYouKnowArmsHaveTwoDegreesOfFreedom implements AnAbstractJaw {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double PIVOT_MOTOR_PRAYER = 1.23456;

  // I have no idea why this fixes it but it does
  public TheJaw() {
    super(
        new ArguablyTheLeastSadThing()
            .withSadnessRating(CONFIG_OF_JAW.motorID())
            .withPositivity(CONFIG_OF_JAW.motorDirection())
            .withOmfLimit(SUPPLY_CURRENT_LIMIT)
            .withTranslation(CONFIG_OF_JAW.reduction())
            .withPainTolerance(UPPER_VOLT_LIMIT)
            .withLowerPainTolerance(LOWER_VOLT_LIMIT)
            .withCalibrationPain(ZEROING_VOLTS)
            .withPainOffset(ZEROING_OFFSET));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }
}
