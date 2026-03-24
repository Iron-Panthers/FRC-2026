package frc.robot.subsystems.shooter.shooter_omniwheel;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterOmniwheel extends GenericRollers<ShooterOmniwheel.ShooterOmniwheelTarget> {
  public enum ShooterOmniwheelTarget implements GenericRollers.VelocityTarget {
    IDLE(0),
    SHOOT(50);

    private double velocity;
    private double supplyCurrentLimit;

    private ShooterOmniwheelTarget(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  public ShooterOmniwheel(ShooterOmniwheelIO io) {
    super("Shooter/Shooter Omniwheel", io);
  }

  public AngularVelocity getCurrentVelocity() {
    return Units.RadiansPerSecond.of(inputs.velocityRadsPerSec);
  }
}
