package frc.robot.subsystems.climb.climbPivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class ClimbPivotIOSim extends GenericSuperstructureIOSim implements ClimbPivotIO {

    private final SingleJointedArmSim climbPivotSim;
    private final double reduction;

      public ClimbPivotIOSim() {
    super(ClimbPivotConstants.CLIMB_PIVOT_CONFIG.motorID());

    this.reduction = ClimbPivotConstants.CLIMB_PIVOT_CONFIG.reduction();
    
    climbPivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction, 
            ClimbPivotConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            ClimbPivotConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            ClimbPivotConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            ClimbPivotConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            ClimbPivotConstants.PHYSICAL_CONSTANTS.simulateGravity(),
        0);
    setOffset();
    setSlot0(
        ClimbPivotConstants.GAINS.kP(),
        ClimbPivotConstants.GAINS.kI(),
        ClimbPivotConstants.GAINS.kD(),
        ClimbPivotConstants.GAINS.kS(),
        ClimbPivotConstants.GAINS.kV(),
        ClimbPivotConstants.GAINS.kA(),
        ClimbPivotConstants.GAINS.kG(),
        ClimbPivotConstants.MOTION_MAGIC_CONFIG.acceleration(),
        ClimbPivotConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        ClimbPivotConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(16);

    double appliedVoltage = talon.getSimState().getMotorVoltage();
  
    // Simulate physics
      climbPivotSim.setInputVoltage(appliedVoltage);
      climbPivotSim.update(0.02); //Don't change this

    // Convert position and velocity from meters to rotations for the TalonFX sensor
      double rotations = climbPivotSim.getAngleRads() / (2 * Math.PI * reduction);
      double velocityRPS = climbPivotSim.getVelocityRadPerSec() / (2 * Math.PI *
    reduction);

      talon.getSimState().setRawRotorPosition(rotations);
      talon.getSimState().setRotorVelocity(velocityRPS);

      inputs.isConnected = true;
      inputs.positionRotations = rotations;
      inputs.velocityRotPerSec = velocityRPS;
      inputs.appliedVolts = appliedVoltage;
      inputs.supplyCurrentAmps = 1.0; // Not simulated
      inputs.tempCelsius = 25.0; // Not simulated
  }

  @Override
  public void setOffset() {
    climbPivotSim.setState(0, 0);
  }

  /** Move move the arm to a position with the given degrees */
  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
