package frc.robot.subsystems.shooter.shooter_flywheels;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheelsIOTalonFX extends GenericRollersIOTalonFX implements ShooterFlywheelIO {
    public ShooterFlywheelsIOTalonFX() {
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