package frc.robot.subsystems.intake.intakePivot;

import static frc.robot.subsystems.intake.intakePivot.IntakePivotConstants.*;

import frc.robot.lib.generic_subsystems.GenericMechanismConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

import org.littletonrobotics.junction.AutoLogOutput;
//import frc.robot.subsystems.intake.intakePivot.IntakePivotConstants;
public class IntakePivotIOTalonFX extends GenericSuperstructureIOTalonFX implements IntakePivotIO {

  public IntakePivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(INTAKE_PIVOT_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(INTAKE_PIVOT_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withCANCoderID(INTAKE_PIVOT_CONFIG.canCoderID())
            .withCANCoderOffset(INTAKE_PIVOT_CONFIG.canCoderOffset())
            .withCANCoderDirection(CANCODER_DIRECTION)
            .withSensorDiscontinuityPoint(SENSOR_DISCONTINUITY_POINT));

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

