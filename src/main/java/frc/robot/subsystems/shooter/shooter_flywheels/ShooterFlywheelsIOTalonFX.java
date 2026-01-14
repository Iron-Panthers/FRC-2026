package frc.robot.subsystems.shooter.shooter_flywheels;

import static frc.robot.subsystems.shooter.shooter_flywheels.ShooterFlywheelsConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheelsIOTalonFX extends GenericRollersIOTalonFX implements ShooterFlywheelsIO {
    public ShooterFlywheelsIOTalonFX() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
    }
}