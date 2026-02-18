package frc.robot.subsystems.shooter.shooter_omniwheel;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterOmniwheel extends GenericRollers<ShooterOmniwheel.ShooterOmniwheelTarget>{
    public enum ShooterOmniwheelTarget implements GenericRollers.VelocityTarget {
        IDLE(0),
        SHOOT(50),
        CLIMB(0);

        private double velocity;

        private ShooterOmniwheelTarget(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    public ShooterOmniwheel(ShooterOmniwheelIO io) {
        super("Shooter/Shooter Omniwheel", io);
    }
}