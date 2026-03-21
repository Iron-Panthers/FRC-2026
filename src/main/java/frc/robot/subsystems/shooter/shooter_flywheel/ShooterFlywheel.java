package frc.robot.subsystems.shooter.shooter_flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.generic_subsystems.rollers.*;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterFlywheel extends GenericRollers<ShooterFlywheel.ShooterFlywheelTarget>{
    public enum ShooterFlywheelTarget implements GenericRollers.VelocityTarget {
        IDLE(0),
        SHOOT(RobotBase.isReal() ? 8.6 : 8.6),
        SPEEDY_SHOOT(9); // TODO: make this uniform

        private double velocity;
        private double supplyCurrentLimit;

        /** Input velocity in meters per second */
        private ShooterFlywheelTarget(double velocity) {
            this.velocity = velocity / ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters();
        }

        /** Velocity in rotations per second */
        public double getVelocity() {
            return velocity;
        }

        public double getSupplyCurrentLimit(){
            return supplyCurrentLimit;
        }
    }

    /** Velocity in rotations per second */
    public double getVelocity() {
      return velocity;
    }
  }

  public ShooterFlywheel(ShooterFlywheelIO io) {
    super("Shooter/Shooter Flywheels", io);
  }

    /** Set flywheel to an arbitrary surface speed (m/s) from the LUT, bypassing the enum targets. */
    public void setVelocityManual(LinearVelocity velocity, double supplyCurrentAmps) {
        setVelocityTargetManual(ShooterFlywheelConstants.VELOCITY_ADJUSTMENT + 
            velocity.in(MetersPerSecond) / ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters(), 
            supplyCurrentAmps);
    }

  /** Set flywheel to an arbitrary surface speed (m/s) from the LUT, bypassing the enum targets. */
  public void setVelocityManual(LinearVelocity velocity) {
    setVelocityTargetManual(
        ShooterFlywheelConstants.VELOCITY_ADJUSTMENT
            + velocity.in(MetersPerSecond)
                / ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
  }

  public boolean reachedVelocityTargetManual() {
    return Math.abs(super.inputs.velocityRadsPerSec - Units.rotationsToRadians(manualVelocityRPS))
        < 40;
  }
}
