package frc.robot.subsystems.climb.climb_deploy_pivot;

import static frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivotConstants.*;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimbDeployPivotIOTalonFX extends GenericSuperstructureIOTalonFX implements ClimbDeployPivotIO {

  public ClimbDeployPivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(CLIMB_DEPLOY_PIVOT_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(CLIMB_DEPLOY_PIVOT_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withCANCoderID(CLIMB_DEPLOY_PIVOT_CONFIG.canCoderID())
            .withCANCoderDirection(CANCODER_DIRECTION)
            .withCANCoderOffset(CLIMB_DEPLOY_PIVOT_CONFIG.canCoderOffset())
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