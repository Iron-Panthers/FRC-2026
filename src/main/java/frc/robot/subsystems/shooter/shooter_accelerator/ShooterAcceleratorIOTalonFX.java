package frc.robot.subsystems.shooter.shooter_accelerator;

import static frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorIOTalonFX extends GenericRollersIOTalonFX implements ShooterAcceleratorIO {
    public ShooterAcceleratorIOTalonFX() {
        super(
            new GenericRollersConfiguration()
                .withID(SHOOTER_ACCELERATOR_CONFIG.motorID())
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
                .withMotorDirection(SHOOTER_ACCELERATOR_CONFIG.inverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                .withNeutralMode(SHOOTER_ACCELERATOR_CONFIG.brake())
                .withReduction(SHOOTER_ACCELERATOR_CONFIG.reduction())
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