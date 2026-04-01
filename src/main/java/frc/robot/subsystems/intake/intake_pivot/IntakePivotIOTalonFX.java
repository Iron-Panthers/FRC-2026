package frc.robot.subsystems.intake.intake_pivot;

import static frc.robot.subsystems.intake.intake_pivot.IntakePivotConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

// import frc.robot.subsystems.intake.intakePivot.IntakePivotConstants;
public class IntakePivotIOTalonFX extends GenericSuperstructureIOTalonFX implements IntakePivotIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double PIVOT_MOTOR_PRAYER = 1.23456;

  // I have no idea why this fixes it but it does
  public IntakePivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(INTAKE_PIVOT_CONFIG.motorID())
            .withMotorDirection(INTAKE_PIVOT_CONFIG.motorDirection())
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(INTAKE_PIVOT_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET));

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
