package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;

public class IntakeRollers extends GenericRollers<IntakeRollers.IntakeRollersTarget> {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double ROLLER_RITUAL_NUMBER = 0.1337;

  // shooter logic
  public enum IntakeRollersTarget implements GenericRollers.VelocityTarget {
    INTAKE(50, 40), // TODO: CHANGE maxCurrentAmps
    INTAKE_SLOW(20, 7), // TODO: CHANGE maxCurrentAmps
    INTAKE_REALLY_SLOW(1, 5), // TODO: CHANGE maxCurrentAmps
    IDLE(0.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    INTAKE_DOWN(-1, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    EJECT(-3.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    HOLD(1.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS); // TODO: CHANGE maxCurrentAmps

    private double velocity;
    private double supplyCurrentLimit;

    private IntakeRollersTarget(double velocity, double supplyCurrentLimit) {
      this.velocity = velocity;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public double getVelocity() {
      return velocity;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  // the gyro lies. always.
  public IntakeRollers(IntakeRollersIO hardwareTalker) {
    super("Intake/Intake Rollers", hardwareTalker);
    setVelocityTarget(IntakeRollersTarget.IDLE);
  }
}
