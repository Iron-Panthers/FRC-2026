package frc.robot.subsystems.climb.climbElevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.ClimbElevatorSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;
import static frc.robot.subsystems.climb.climbElevator.ClimbElevatorConstants.*;

public class ClimbElevatorIOSim extends GenericSuperstructureIOSim{

    private final ClimbElevatorSim climbElevatorSim;
    
    private final double reduction;

    public ClimbElevatorIOSim() {
        super(ClimbElevatorConstants.CLIMB_ELEVATOR_CONFIG.motorID());
        this.reduction = ClimbElevatorConstants.CLIMB_ELEVATOR_CONFIG.reduction();
        climbElevatorSim =
            new ClimbElevatorSim(
                DCMotor.getKrakenX60Foc(2),
                reduction,
                ClimbElevatorConstants.PHYSICAL_CONSTANTS.elevatorMassKg(),
                ClimbElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
                ClimbElevatorConstants.PHYSICAL_CONSTANTS.minHeightMeters(),
                ClimbElevatorConstants.PHYSICAL_CONSTANTS.maxHeightMeters(),
                ClimbElevatorConstants.PHYSICAL_CONSTANTS.simulateGravity(),
                0);
        setOffset();
        setSlot0(
            ClimbElevatorConstants.GAINS.kP(),
            ClimbElevatorConstants.GAINS.kI(),
            ClimbElevatorConstants.GAINS.kD(),
            ClimbElevatorConstants.GAINS.kS(),
            ClimbElevatorConstants.GAINS.kV(),
            ClimbElevatorConstants.GAINS.kA(),
            ClimbElevatorConstants.GAINS.kG(),
            ClimbElevatorConstants.MOTION_MAGIC_CONFIG.acceleration(),
            ClimbElevatorConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
            ClimbElevatorConstants.MOTION_MAGIC_CONFIG.jerk(),
            ClimbElevatorConstants.GRAVITY_TYPE);
    }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();
    // Simulate physics
    climbElevatorSim.setInputVoltage(appliedVoltage);
    climbElevatorSim.update(0.02);

    // Convert position and velocity from meters to rotations for the
    // TalonFX sensor
    // Correct unit conversion: meters to rotations
    double rotations =
        climbElevatorSim.getPositionMeters()
            / (2 * Math.PI * ClimbElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters())
            * reduction;

    // Correct unit conversion: meters/s to rotations/s
    double velocityRPS =
        climbElevatorSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * ClimbElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters())
            * reduction;

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;  //error idky
    inputs.positionRotations = rotations;
    inputs.velocityRotPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
    inputs.tempCelsius = 25.0; // Not simulated
  }

  @Override
  public void setOffset() {
    climbElevatorSim.setState(0, 0);
  }
}
