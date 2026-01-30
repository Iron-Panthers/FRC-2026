package frc.robot.subsystems.shooter.shooter_accelerator_top;

import static frc.robot.subsystems.shooter.shooter_accelerator_top.ShooterAcceleratorTopConstants.*;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

//TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class ShooterAcceleratorTopIOSim extends GenericRollersIOSim implements ShooterAcceleratorTopIO {
    
    private final FlywheelSim shooterAcceleratorTopSim;
    private final SimpleMotorFeedforward feedforward;
    private double rotorPositionRotations = 0.0;
    private double velocitySetpointRPS = 0.0;

    public ShooterAcceleratorTopIOSim() {
        super(SHOOTER_ACCELERATOR_TOP_CONFIG.motorID(), CURRENT_LIMIT_AMPS, SHOOTER_ACCELERATOR_TOP_CONFIG.inverted(), SHOOTER_ACCELERATOR_TOP_CONFIG.brake(), SHOOTER_ACCELERATOR_TOP_CONFIG.reduction());
        super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG(),
            MOTION_MAGIC_CONFIG.accelerations(),
            MOTION_MAGIC_CONFIG.cruiseVelocity(),
            0
        );
        // Create feedforward controller using configured gains
        feedforward = new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

        shooterAcceleratorTopSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), PHYSICAL_CONSTANTS.momentOfInertia(), SHOOTER_ACCELERATOR_TOP_CONFIG.reduction()), 
                DCMotor.getKrakenX60Foc(1));
        
        // Enable physics simulation for Phoenix
        var simState = talon.getSimState();
        simState.Orientation = SHOOTER_ACCELERATOR_TOP_CONFIG.inverted() 
            ? ChassisReference.Clockwise_Positive 
            : ChassisReference.CounterClockwise_Positive;
  }

  @Override
  public void runVelocity(double velocity) {
    velocitySetpointRPS = velocity;
    super.runVelocity(velocity);
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    double currentVelocityRPS = shooterAcceleratorTopSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    
    // Set TalonFX sim state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    talon.getSimState().setRawRotorPosition(rotorPositionRotations);
    talon.getSimState().setRotorVelocity(currentVelocityRPS);

    // Calculate applied voltage using feedforward + proportional feedback
    double feedforwardVoltage = feedforward.calculate(velocitySetpointRPS);
    double error = velocitySetpointRPS - currentVelocityRPS;
    double proportionalVoltage = GAINS.kP() * error;
    double appliedVoltage = feedforwardVoltage + proportionalVoltage;
    appliedVoltage = Math.max(-12, Math.min(12, appliedVoltage)); // Clamp to battery voltage

    // Simulate physics
    shooterAcceleratorTopSim.setInputVoltage(appliedVoltage);
    shooterAcceleratorTopSim.update(0.02);

    // Update position tracking
    currentVelocityRPS = shooterAcceleratorTopSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rotorPositionRotations += currentVelocityRPS * 0.02;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = shooterAcceleratorTopSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(shooterAcceleratorTopSim.getCurrentDrawAmps());
  }
}