package frc.robot.subsystems.shooter.serializer;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Serializer extends GenericRollers<Serializer.SerializerTarget> {
  public enum SerializerTarget implements GenericRollers.VelocityTarget {
    IDLE(0),
    SLOW(0),
    INTAKE(20);

    private double velocity;
    private double supplyCurrentLimit;

    private SerializerTarget(double velocity) {
      this.velocity = velocity;
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
}
