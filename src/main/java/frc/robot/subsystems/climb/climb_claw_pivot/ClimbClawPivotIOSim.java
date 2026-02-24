package frc.robot.subsystems.climb.climb_claw_pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class ClimbClawPivotIOSim extends GenericSuperstructureIOSim implements ClimbClawPivotIO {

    private final SingleJointedArmSim climbClawPivotSim;
    private final double reduction;

      public ClimbClawPivotIOSim() {
    super(ClimbClawPivotConstants.CLIMB_CLAW_PIVOT_CONFIG.motorID());

    this.reduction = ClimbClawPivotConstants.CLIMB_CLAW_PIVOT_CONFIG.reduction();
    
    climbClawPivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction, 
            ClimbClawPivotConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            ClimbClawPivotConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            ClimbClawPivotConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            ClimbClawPivotConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            ClimbClawPivotConstants.PHYSICAL_CONSTANTS.simulateGravity(),
        0);
    setOffset();
    setSlot0(
        ClimbClawPivotConstants.GAINS.kP(),
        ClimbClawPivotConstants.GAINS.kI(),
        ClimbClawPivotConstants.GAINS.kD(),
        ClimbClawPivotConstants.GAINS.kS(),
        ClimbClawPivotConstants.GAINS.kV(),
        ClimbClawPivotConstants.GAINS.kA(),
        ClimbClawPivotConstants.GAINS.kG(),
        ClimbClawPivotConstants.MOTION_MAGIC_CONFIG.acceleration(),
        ClimbClawPivotConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        ClimbClawPivotConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(16);

    double appliedVoltage = talon.getSimState().getMotorVoltage();
  
    // Simulate physics
      climbClawPivotSim.setInputVoltage(appliedVoltage);
      climbClawPivotSim.update(0.02); //Don't change this

    // Convert position and velocity from meters to rotations for the TalonFX sensor
      double rotations = climbClawPivotSim.getAngleRads() / (2 * Math.PI * reduction);
      double velocityRPS = climbClawPivotSim.getVelocityRadPerSec() / (2 * Math.PI *
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
    climbClawPivotSim.setState(0, 0);
  }
}
