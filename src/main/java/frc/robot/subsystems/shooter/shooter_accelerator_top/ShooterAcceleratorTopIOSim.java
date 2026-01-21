package frc.robot.subsystems.shooter.shooter_accelerator_top;

import static frc.robot.subsystems.shooter.shooter_accelerator_top.ShooterAcceleratorTopConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

//TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class ShooterAcceleratorTopIOSim extends GenericRollersIOSim implements ShooterAcceleratorTopIO {
    
    private final FlywheelSim shooterAcceleratorTopSim;

    public ShooterAcceleratorTopIOSim() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
        shooterAcceleratorTopSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), MOI, REDUCTION), 
                DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {
        talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double appliedVoltage = talon.getSimState().getMotorVoltage();

        shooterAcceleratorTopSim.setInputVoltage(appliedVoltage);
        shooterAcceleratorTopSim.update(0.02);

        double rotations = 0;
        double velocityRPS = shooterAcceleratorTopSim.getAngularVelocityRadPerSec() / REDUCTION;

        talon.getSimState().setRawRotorPosition(rotations);
        talon.getSimState().setRotorVelocity(velocityRPS);

        inputs.connected = true;
        inputs.positionRads = rotations;
        inputs.velocityRadsPerSec = velocityRPS;
        inputs.appliedVolts = appliedVoltage;
        inputs.supplyCurrentAmps = 1.0;
    }
}