package frc.robot.subsystems.shooter.shooter_flywheel;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheel extends GenericRollers<ShooterFlywheel.ShooterFlywheelTarget>{
    public enum ShooterFlywheelTarget implements GenericRollers.VelocityTarget {
        //TODO: need to change; from sprint 2025 -- ive taken away a few states
        IDLE(0),
        SHOOT(12),
        CLIMB(0);

        private double velocity;

        private ShooterFlywheelTarget(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    public ShooterFlywheel(ShooterFlywheelIO io) {
        super("Shooter/Shooter Flywheels", io);
    }
}