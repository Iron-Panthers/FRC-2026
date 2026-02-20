package frc.robot.subsystems.shooter.shooter_accelerator_bottom;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorBottom extends GenericRollers<ShooterAcceleratorBottom.ShooterAcceleratorBottomTarget>{
    public enum ShooterAcceleratorBottomTarget implements GenericRollers.VelocityTarget {
        //TODO: need to change; from sprint 2025 -- ive taken away a few states
        IDLE(0),
        SHOOT(12);

        private double velocity;

        private ShooterAcceleratorBottomTarget(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    public ShooterAcceleratorBottom(ShooterAcceleratorBottomIO io) {
        super("Shooter/Shooter Accelerator Bottom", io);
    }
}