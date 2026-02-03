package frc.robot.subsystems.shooter.shooter_accelerator_bottom;

import static frc.robot.subsystems.shooter.shooter_accelerator_bottom.ShooterAcceleratorBottomConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorBottomIOTalonFX extends GenericRollersIOTalonFX implements ShooterAcceleratorBottomIO {
    public ShooterAcceleratorBottomIOTalonFX() {
        super(
            new GenericRollersConfiguration()
                .withID(SHOOTER_ACCELERATOR_BOTTOM_CONFIG.motorID())
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
                .withMotorDirection(SHOOTER_ACCELERATOR_BOTTOM_CONFIG.inverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                .withNeutralMode(SHOOTER_ACCELERATOR_BOTTOM_CONFIG.brake())
                .withReduction(SHOOTER_ACCELERATOR_BOTTOM_CONFIG.reduction())
        );
        super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG(),
            MOTION_MAGIC_CONFIG.accelerations(),
            MOTION_MAGIC_CONFIG.cruiseVelocity(),
            0

        );
    }
}