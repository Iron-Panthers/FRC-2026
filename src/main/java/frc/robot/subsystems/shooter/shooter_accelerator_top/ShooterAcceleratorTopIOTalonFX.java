package frc.robot.subsystems.shooter.shooter_accelerator_top;

import static frc.robot.subsystems.shooter.shooter_accelerator_top.ShooterAcceleratorTopConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorTopIOTalonFX extends GenericRollersIOTalonFX implements ShooterAcceleratorTopIO {
    public ShooterAcceleratorTopIOTalonFX() {
        super(
            new GenericRollersConfiguration()
                .withID(SHOOTER_ACCELERATOR_TOP_CONFIG.motorID())
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
                .withMotorDirection(SHOOTER_ACCELERATOR_TOP_CONFIG.inverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                .withNeutralMode(SHOOTER_ACCELERATOR_TOP_CONFIG.brake())
                .withReduction(SHOOTER_ACCELERATOR_TOP_CONFIG.reduction())
        );
        super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA());
    }
}