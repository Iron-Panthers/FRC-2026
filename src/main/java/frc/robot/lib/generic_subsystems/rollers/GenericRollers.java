package frc.robot.lib.generic_subsystems.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.VelocityTarget> {
  public interface VelocityTarget {
    double getVelocity();
  }

  public enum ControlMode {
    VELOCITY,
    STOP
  }

  private ControlMode controlMode = ControlMode.STOP;

  private LinearFilter filter;
  private double filteredCurrent;

  private final String name;
  private final GenericRollersIO rollerIO;
  protected GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();

  private G velocityTarget;

  public GenericRollers(String name, GenericRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;
    this.filter = LinearFilter.movingAverage(100);
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    rollerIO.runVelocity(velocityTarget.getVelocity());
    Logger.recordOutput(name + "/Target", velocityTarget.toString());
    Logger.recordOutput(name + "/Target Velocity", velocityTarget.getVelocity());

    filteredCurrent = this.filter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput(name + "/FilteredCurrent", filteredCurrent);

    Logger.recordOutput(name + "/Control Mode", controlMode.toString());
    switch (controlMode) {
      case VELOCITY -> {
        rollerIO.runVelocity(velocityTarget.getVelocity());
      }
      case STOP -> {
        rollerIO.stop();
      }
    }

  }

  public G getVelocityTarget() {
    return velocityTarget;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public void setVelocityTarget(G velocityTarget) {
    setControlMode(ControlMode.VELOCITY);
    this.velocityTarget = velocityTarget;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }
}
