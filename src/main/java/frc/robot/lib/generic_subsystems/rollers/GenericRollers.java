package frc.robot.lib.generic_subsystems.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.VelocityTarget> {
  public interface VelocityTarget {
    double getVelocity();
  }

  private LinearFilter filter;
  private double filteredCurrent;

  private final String name;
  private final GenericRollersIO rollerIO;
  private GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();

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
    Logger.recordOutput("Rollers/" + name + "/Target", velocityTarget.toString());

    filteredCurrent = this.filter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput("Rollers/" + name + "/FilteredCurrent", filteredCurrent);
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
    this.velocityTarget = velocityTarget;
  }
}
