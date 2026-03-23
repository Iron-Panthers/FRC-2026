package frc.robot.subsystems.shooter.shooter_accelerator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAccelerator
    extends GenericRollers<ShooterAccelerator.ShooterAcceleratorTarget> {
  public enum ShooterAcceleratorTarget implements GenericRollers.VelocityTarget {
    IDLE(0),
    SHOOT(51.66),
    WARMUP_ACCELERATOR(75);

        private double velocity;
        private double supplyCurrentLimit;

        private ShooterAcceleratorTarget(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
        
        public double getSupplyCurrentLimit(){
            return supplyCurrentLimit;
        }
    }

    public double getVelocity() {
      return velocity;
    }
  }

  public ShooterAccelerator(ShooterAcceleratorIO io) {
    super("Shooter/Shooter Accelerator", io);
  }

  public AngularVelocity getCurrentVelocity() {
    return Units.RadiansPerSecond.of(inputs.velocityRadsPerSec);
  }
}
