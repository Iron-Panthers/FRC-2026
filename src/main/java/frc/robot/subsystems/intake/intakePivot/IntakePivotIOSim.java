package frc.robot.subsystems.intake.intakePivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;
import frc.robot.subsystems.intake.intakePivot.IntakePivotConstants;

public class IntakePivotIOSim extends GenericSuperstructureIOSim implements IntakePivotIO {

  private final SingleJointedArmSim intakePivotSim;
  private final double reduction;

  public IntakePivotIOSim() {
    super(IntakePivotConstants.INTAKE_PIVOT_CONFIG.motorID());

    this.reduction = IntakePivotConstants.INTAKE_PIVOT_CONFIG.reduction();

    intakePivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            IntakePivotConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        IntakePivotConstants.GAINS.kP(),
        IntakePivotConstants.GAINS.kI(),
        IntakePivotConstants.GAINS.kD(),
        IntakePivotConstants.GAINS.kS(),
        IntakePivotConstants.GAINS.kV(),
        IntakePivotConstants.GAINS.kA(),
        IntakePivotConstants.GAINS.kG(),
        IntakePivotConstants.MOTION_MAGIC_CONFIG.acceleration(),
        IntakePivotConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        IntakePivotConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    intakePivotSim.setInputVoltage(appliedVoltage);
    intakePivotSim.update(0.02);

    // Convert position and velocity from meters to rotations for the TalonFX sensor
    double rotations = intakePivotSim.getAngleRads() / (2 * Math.PI * reduction);
    double velocityRPS = intakePivotSim.getVelocityRadPerSec() / (2 * Math.PI * reduction);

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    
    inputs.positionRotations = rotations;
    inputs.velocityRotPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
    inputs.tempCelsius = 25.0; // Not simulated
  }

  @Override
  public void setOffset() {
    intakePivotSim.setState(0, 0);
  }

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}

