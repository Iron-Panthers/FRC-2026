package frc.robot.subsystems.shooter.shooter_accelerator_top;

import static frc.robot.subsystems.shooter.shooter_accelerator_top.ShooterAcceleratorTopConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorTopIOTalonFX extends GenericRollersIOTalonFX implements ShooterAcceleratorTopIO {
    public ShooterAcceleratorTopIOTalonFX() {
        super(SHOOTER_ACCELERATOR_TOP_CONFIG.motorID(), CURRENT_LIMIT_AMPS, SHOOTER_ACCELERATOR_TOP_CONFIG.inverted(), SHOOTER_ACCELERATOR_TOP_CONFIG.brake(), SHOOTER_ACCELERATOR_TOP_CONFIG.reduction());
        super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG()

        );
    }
}