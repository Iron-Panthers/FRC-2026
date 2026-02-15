package frc.robot.subsystems.hopper.Hopper;
import static frc.robot.subsystems.hopper.Hopper.HopperConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;

public class HopperIOTalonFX extends GenericRollersIOTalonFX{
    public HopperIOTalonFX() {
    //   super(HOPPER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, HOPPER_CONFIG.inverted(), HOPPER_CONFIG.brake(), HOPPER_CONFIG.reduction());
      super(
      new GenericRollersConfiguration()
            .withID(HOPPER_CONFIG.motorID())
            .withMotorDirection(HOPPER_CONFIG.inverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withReduction(HOPPER_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT));
      super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA()
        );
    }
}
