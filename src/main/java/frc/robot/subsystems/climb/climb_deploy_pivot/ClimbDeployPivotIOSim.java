package frc.robot.subsystems.climb.climb_deploy_pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class ClimbDeployPivotIOSim extends GenericSuperstructureIOSim implements ClimbDeployPivotIO {

    private final SingleJointedArmSim climbDeployPivotSim;
    private final double reduction;

      public ClimbDeployPivotIOSim() {
    super(ClimbDeployPivotConstants.CLIMB_DEPLOY_PIVOT_CONFIG.motorID());

    this.reduction = ClimbDeployPivotConstants.CLIMB_DEPLOY_PIVOT_CONFIG.reduction();
    
    climbDeployPivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction, 
            ClimbDeployPivotConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            ClimbDeployPivotConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            ClimbDeployPivotConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            ClimbDeployPivotConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            ClimbDeployPivotConstants.PHYSICAL_CONSTANTS.simulateGravity(),
        0);
    setOffset();
    setSlot0(
        ClimbDeployPivotConstants.GAINS.kP(),
        ClimbDeployPivotConstants.GAINS.kI(),
        ClimbDeployPivotConstants.GAINS.kD(),
        ClimbDeployPivotConstants.GAINS.kS(),
        ClimbDeployPivotConstants.GAINS.kV(),
        ClimbDeployPivotConstants.GAINS.kA(),
        ClimbDeployPivotConstants.GAINS.kG(),
        ClimbDeployPivotConstants.MOTION_MAGIC_CONFIG.acceleration(),
        ClimbDeployPivotConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        ClimbDeployPivotConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(16);

    double appliedVoltage = talon.getSimState().getMotorVoltage();
  
    // Simulate physics
      climbDeployPivotSim.setInputVoltage(appliedVoltage);
      climbDeployPivotSim.update(0.02); //Don't change this

    // Convert position and velocity from meters to rotations for the TalonFX sensor
      double rotations = climbDeployPivotSim.getAngleRads() / (2 * Math.PI * reduction);
      double velocityRPS = climbDeployPivotSim.getVelocityRadPerSec() / (2 * Math.PI *
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
    climbDeployPivotSim.setState(0, 0);
  }

  /** Move move the arm to a position with the given degrees */
  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
