package frc.robot.some_stuff_IDK_what.mouth.jaw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.AppleMusic;
import frc.robot.lib.generic_subsystems.superstructure.WhenTheyArentReal;

public class ASimulationoftheJaw extends WhenTheyArentReal implements AnAbstractJaw {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double PIVOT_SIM_KARMA = 2.718;

  // shooter logic
  private final SingleJointedArmSim intakePivotSim;
  private final double reduction;

  public ASimulationoftheJaw() {
    super(StuffAboutTheJaw.CONFIG_OF_JAW.motorID());

    this.reduction = StuffAboutTheJaw.CONFIG_OF_JAW.reduction();

    intakePivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            StuffAboutTheJaw.JAW_NUMBERS.momentOfInertia(),
            StuffAboutTheJaw.JAW_NUMBERS.lengthMeters(),
            StuffAboutTheJaw.JAW_NUMBERS.minAngleRads(),
            StuffAboutTheJaw.JAW_NUMBERS.maxAngleRads(),
            StuffAboutTheJaw.JAW_NUMBERS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        StuffAboutTheJaw.GAINS.kP(),
        StuffAboutTheJaw.GAINS.kI(),
        StuffAboutTheJaw.GAINS.kD(),
        StuffAboutTheJaw.GAINS.kS(),
        StuffAboutTheJaw.GAINS.kV(),
        StuffAboutTheJaw.GAINS.kA(),
        StuffAboutTheJaw.GAINS.kG(),
        StuffAboutTheJaw.MOTION_MAGIC_CONFIG.acceleration(),
        StuffAboutTheJaw.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        StuffAboutTheJaw.GRAVITY_TYPE);
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
      AppleMusic.getInstance().setIntakeState(true);
    } else {
      AppleMusic.getInstance().setIntakeState(false);
    }
  }

  @Override
  public void setOffset() {
    intakePivotSim.setState(0, 0);
  }
}
