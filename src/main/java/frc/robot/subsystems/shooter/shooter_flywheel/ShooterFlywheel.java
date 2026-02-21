package frc.robot.subsystems.shooter.shooter_flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheel extends GenericRollers<ShooterFlywheel.ShooterFlywheelTarget>{
    public enum ShooterFlywheelTarget implements GenericRollers.VelocityTarget {
        //TODO: need to change; from sprint 2025 -- ive taken away a few states
        IDLE(8),
        SHOOT(8);

        private double velocity;

        /** Input velocity in meters per second */
        private ShooterFlywheelTarget(double velocity) {
            this.velocity = velocity / ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters();
        }

        /** Velocity in rotations per second */
        public double getVelocity() {
            return velocity;
        }
    }

    public ShooterFlywheel(ShooterFlywheelIO io) {
        super("Shooter/Shooter Flywheels", io);
    }

    @AutoLogOutput(key = "Shooter/Shooter Flywheels/CurrentVelocity")
    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(Units.radiansToRotations(inputs.velocityRadsPerSec) * ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
    }
}