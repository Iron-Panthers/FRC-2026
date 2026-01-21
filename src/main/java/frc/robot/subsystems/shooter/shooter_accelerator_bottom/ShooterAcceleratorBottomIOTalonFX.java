package frc.robot.subsystems.shooter.shooter_accelerator_bottom;

import static frc.robot.subsystems.shooter.shooter_accelerator_bottom.ShooterAcceleratorBottomConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorBottomIOTalonFX extends GenericRollersIOTalonFX implements ShooterAcceleratorBottomIO {
    public ShooterAcceleratorBottomIOTalonFX() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
        setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            MOTION_MAGIC_CONFIG.accelerations(),
            MOTION_MAGIC_CONFIG.cruiseVelocity(),
            0,
            GRAVITY_TYPE
        );
    }
}