package frc.robot.subsystems.shooter.shooter_accelerator_top;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorTop extends GenericRollers<ShooterAcceleratorTop.Target>{
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

    public ShooterAcceleratorTop(ShooterAcceleratorTopIO io) {
        super("Shooter Flywheels", io);
    }
}