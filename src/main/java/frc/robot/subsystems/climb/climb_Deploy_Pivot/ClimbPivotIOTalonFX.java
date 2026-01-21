package frc.robot.subsystems.climb.climbPivot;

import static frc.robot.subsystems.climb.climbPivot.ClimbPivotConstants.*;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimbPivotIOTalonFX extends GenericSuperstructureIOTalonFX implements ClimbPivotIO {

  public ClimbPivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(CLIMB_PIVOT_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(CLIMB_PIVOT_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withZeroingVoltageThreshold(ZEROING_VOLTAGE_THRESHOLD)
            .withCANCoderID(CLIMB_PIVOT_CONFIG.canCoderID())
            .withCANCoderDirection(CANCODER_DIRECTION)
            .withCANCoderOffset(CLIMB_PIVOT_CONFIG.canCoderOffset())
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

  @AutoLogOutput(key = "Superstructure/Climb/Climb Pivot/ModdedRotations")
  public double moddedRotations;

  @Override
  public void stop() {}

  public void runVolts(double volts) {
    talon.setControl(new VoltageOut(volts));
  }
}