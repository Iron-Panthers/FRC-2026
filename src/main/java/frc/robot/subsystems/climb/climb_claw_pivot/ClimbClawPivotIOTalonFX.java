package frc.robot.subsystems.climb.climb_claw_pivot;

import static frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivotConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class ClimbClawPivotIOTalonFX extends GenericSuperstructureIOTalonFX
    implements ClimbClawPivotIO {

  public ClimbClawPivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(CLIMB_CLAW_PIVOT_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(CLIMB_CLAW_PIVOT_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withCANCoderID(CLIMB_CLAW_PIVOT_CONFIG.canCoderID())
            .withCANCoderDirection(CANCODER_DIRECTION)
            .withCANCoderOffset(CLIMB_CLAW_PIVOT_CONFIG.canCoderOffset())
            .withLowerExtensionLimit(LOWER_EXTENSION_LIMIT));

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
