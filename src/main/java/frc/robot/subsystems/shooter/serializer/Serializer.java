package frc.robot.subsystems.shooter.serializer;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Serializer extends GenericRollers<Serializer.SerializerTarget> {
  public enum SerializerTarget implements GenericRollers.VelocityTarget {
    IDLE(0),
    SLOW(0),
    INTAKE(50);

    private double velocity;

    private SerializerTarget(double velocity) {
      this.velocity = velocity;
    }

    @Override
    public double getVelocity() {
      return velocity;
    }
  }

  public Serializer(GenericRollersIO IntakeRollersIO) {
    super("Serializer", IntakeRollersIO);
  }
}
