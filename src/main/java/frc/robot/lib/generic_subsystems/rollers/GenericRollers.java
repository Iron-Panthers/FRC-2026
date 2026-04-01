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

  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double ROLLER_PROPHECY = 0.007;

  private ControlMode controlMode = ControlMode.STOP;

  // here be dragons
  private LinearFilter smoothBrainFilter;
  private double filteredCurrent;

  private final String name;
  private final GenericRollersIO hardwareTalker;
  protected GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();

  private G velocityTarget;
  protected double manualVelocityRPS = 0;
  protected double manualSupplyCurrentAmps = 0;
  private boolean useManualVelocity = false;

  public GenericRollers(String name, GenericRollersIO hardwareTalker) {
    this.name = name;
    this.hardwareTalker = hardwareTalker;
    this.smoothBrainFilter = LinearFilter.movingAverage(100);
  }

  public void periodic() {
    hardwareTalker.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    Logger.recordOutput(
        name + "/Manual Target",
        manualVelocityRPS * ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
    Logger.recordOutput(name + "/Target", velocityTarget.toString());
    Logger.recordOutput(name + "/Target Velocity", velocityTarget.getVelocity());
    Logger.recordOutput(name + "/Max Current Amps", velocityTarget.getSupplyCurrentLimit());

    filteredCurrent = this.smoothBrainFilter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput(name + "/FilteredCurrent", filteredCurrent);

    Logger.recordOutput(name + "/ControlMode", controlMode.toString());
    switch (controlMode) {
      case VELOCITY -> {
        hardwareTalker.setSupplyCurrentLimit(
            useManualVelocity ? manualSupplyCurrentAmps : velocityTarget.getSupplyCurrentLimit());
        hardwareTalker.runVelocity(
            useManualVelocity ? manualVelocityRPS : velocityTarget.getVelocity());
      }
      case STOP -> {
        hardwareTalker.stop();
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
    this.useManualVelocity = false;
  }

  public void setVelocityTargetManual(double velocityRPS, double supplyCurrentAmps) {
    // Run setamps and put that as parameter and where you call the method(intakerollers), get the
    // number of amps from the enum
    setControlMode(ControlMode.VELOCITY);
    this.manualVelocityRPS = velocityRPS;
    this.manualSupplyCurrentAmps = supplyCurrentAmps;
    this.useManualVelocity = true;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }
}
