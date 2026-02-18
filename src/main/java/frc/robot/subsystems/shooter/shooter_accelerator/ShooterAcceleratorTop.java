package frc.robot.subsystems.shooter.shooter_accelerator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAcceleratorTop extends GenericRollers<ShooterAcceleratorTop.ShooterAcceleratorTopTarget>{
    public enum ShooterAcceleratorTopTarget implements GenericRollers.VelocityTarget {
        //TODO: need to change; from sprint 2025 -- ive taken away a few states
        IDLE(0),
        SHOOT(12),
        CLIMB(0);

        private double velocity;

        private ShooterAcceleratorTopTarget(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    public ShooterAcceleratorTop(ShooterAcceleratorTopIO io) {
        super("Shooter/Shooter Accelerator Top", io);
    }

    public AngularVelocity getCurrentVelocity() {
        return Units.RadiansPerSecond.of(inputs.velocityRadsPerSec);
    }
}