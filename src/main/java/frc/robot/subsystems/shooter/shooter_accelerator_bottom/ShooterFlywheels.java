package frc.robot.subsystems.shooter.shooter_flywheels;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheels extends GenericRollers<ShooterFlywheels.Target>{
    public enum Target implements GenericRollers.VelocityTarget {
        //TODO: need to change; from sprint 2025 -- ive taken away a few states
        IDLE(0),
        SHOOT(12),
        CLIMB(0);

        private double velocity;

        private Target(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    public ShooterFlywheels(ShooterFlywheelIO io) {
        super("Shooter Flywheels", io);
    }
}