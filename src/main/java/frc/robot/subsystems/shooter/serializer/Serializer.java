package frc.robot.subsystems.shooter.serializer;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Serializer extends GenericRollers<Serializer.SerializerTarget> {
  public enum SerializerTarget implements GenericRollers.VelocityTarget {
    IDLE(0, SerializerConstants.CURRENT_LIMIT_AMPS),
    SLOW(15, SerializerConstants.CURRENT_LIMIT_AMPS),
    HOLD(1, SerializerConstants.CURRENT_LIMIT_AMPS),
    SPIN_UP(40, 20),
    SHOOT(50, SerializerConstants.CURRENT_LIMIT_AMPS);

    private double velocity;
    private double supplyCurrentLimit;

    private SerializerTarget(double velocity, double supplyCurrentLimit) {
      this.velocity = velocity;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    @Override
    public double getVelocity() {
      return velocity;
    }

    @Override
    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  public Serializer(GenericRollersIO IntakeRollersIO) {
    super("Serializer", IntakeRollersIO);
  }

  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  /**
   * Returns true when the serializer is applying amps but not going anywhere
   *
   * @return
   */
  public boolean serializerStalling() {
    return getFilteredCurrent() > 15d && getVelocityRadsPerSec() < 10d;
  }
}
