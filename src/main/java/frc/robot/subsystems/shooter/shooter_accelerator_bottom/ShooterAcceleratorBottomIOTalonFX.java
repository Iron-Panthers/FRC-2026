package frc.robot.subsystems.shooter.shooter_accelerator_bottom;

import static frc.robot.subsystems.shooter.shooter_accelerator_bottom.ShooterAcceleratorBottomConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorBottomIOTalonFX extends GenericRollersIOTalonFX implements ShooterAcceleratorBottomIO {
    public ShooterAcceleratorBottomIOTalonFX() {
        super(SHOOTER_ACCELERATOR_BOTTOM_CONFIG.motorID(), CURRENT_LIMIT_AMPS, SHOOTER_ACCELERATOR_BOTTOM_CONFIG.inverted(), SHOOTER_ACCELERATOR_BOTTOM_CONFIG.brake(), SHOOTER_ACCELERATOR_BOTTOM_CONFIG.reduction());
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