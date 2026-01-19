package frc.robot.subsystems.shooter.shooter_flywheels;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheels extends GenericRollers<ShooterFlywheels.Target>{
    public enum Target implements GenericRollers.VoltageTarget {
        //TODO: need to change; from sprint 2025
        IDLE(0),
        SHOOT(12),
        CLIMB(0);

        private double volts;

        private Target(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    public ShooterFlywheels(ShooterFlywheelsIO io) {
        super("Shooter Flywheels", io);
    }
}