package frc.robot.subsystems.intake.intake_rack;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotSimState;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class IntakeRackIOSim extends GenericSuperstructureIOSim implements IntakeRackIO {

  private final SingleJointedArmSim intakePivotSim;
  private final double reduction;

  public IntakeRackIOSim() {
    super(IntakeRackConstants.INTAKE_PIVOT_CONFIG.motorID());

    this.reduction = IntakeRackConstants.INTAKE_PIVOT_CONFIG.reduction();

    intakePivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            IntakeRackConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        IntakeRackConstants.GAINS.kP(),
        IntakeRackConstants.GAINS.kI(),
        IntakeRackConstants.GAINS.kD(),
        IntakeRackConstants.GAINS.kS(),
        IntakeRackConstants.GAINS.kV(),
        IntakeRackConstants.GAINS.kA(),
        IntakeRackConstants.GAINS.kG(),
        IntakeRackConstants.MOTION_MAGIC_CONFIG.acceleration(),
        IntakeRackConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        IntakeRackConstants.GRAVITY_TYPE);
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
    inputs.supplyCurrentAmps = talon.getSimState().getSupplyCurrent();
    inputs.tempCelsius = 25.0; // Not simulated

    // update the Sim State to match if it is up or down
    if (rotations < .1) {
      RobotSimState.getInstance().setIntakeState(true);
    } else {
      RobotSimState.getInstance().setIntakeState(false);
    }
  }

  @Override
  public void setOffset() {
    intakePivotSim.setState(0, 0);
  }
}
