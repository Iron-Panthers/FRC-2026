package frc.robot.lib.generic_subsystems.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants;

import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.VelocityTarget> {
  public interface VelocityTarget {
    double getVelocity();
    double getSupplyCurrentLimit();
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
  protected double manualVelocityRPS = 0;
  private boolean useManualVelocity = false;

  public GenericRollers(String name, GenericRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;
    this.filter = LinearFilter.movingAverage(100);
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    rollerIO.setSupplyCurrentLimit(velocityTarget.getSupplyCurrentLimit());

    rollerIO.runVelocity(useManualVelocity ? manualVelocityRPS : velocityTarget.getVelocity());
    Logger.recordOutput(name + "/Manual Target", manualVelocityRPS * ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
    Logger.recordOutput(name + "/Target", velocityTarget.toString());
    Logger.recordOutput(name + "/Target Velocity", velocityTarget.getVelocity());
    Logger.recordOutput(name + "/Max Current Amps", velocityTarget.getSupplyCurrentLimit());

    filteredCurrent = this.filter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput(name + "/FilteredCurrent", filteredCurrent);

    Logger.recordOutput(name + "/ControlMode", controlMode.toString());
    switch (controlMode) {
      case VELOCITY -> {
        rollerIO.runVelocity(useManualVelocity ? manualVelocityRPS : velocityTarget.getVelocity());
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
    rollerIO.setSupplyCurrentLimit(filteredCurrent);
    setControlMode(ControlMode.VELOCITY);
    this.velocityTarget = velocityTarget;
    this.useManualVelocity = false;
  }

  public void setVelocityTargetManual(double velocityRPS, double supplyCurrentAmps) {
    // Run setamps and put that as parameter and where you call the method(intakerollers), get the number of amps from the enum
    rollerIO.setSupplyCurrentLimit(supplyCurrentAmps);
    setControlMode(ControlMode.VELOCITY);
    this.manualVelocityRPS = velocityRPS;
    this.useManualVelocity = true;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }
}
