package frc.robot.subsystems.shooter.shooter_flywheel;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

//TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class ShooterFlywheelIOSim extends GenericRollersIOSim implements ShooterFlywheelIO {
    
    private final FlywheelSim shooterFlywheelsSim;

    public ShooterFlywheelIOSim() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
        shooterFlywheelsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), MOI, REDUCTION), 
                DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {
        talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double appliedVoltage = talon.getSimState().getMotorVoltage();

        shooterFlywheelsSim.setInputVoltage(appliedVoltage);
        shooterFlywheelsSim.update(0.02);

        double rotations = 0;
        double velocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / REDUCTION;

        talon.getSimState().setRawRotorPosition(rotations);
        talon.getSimState().setRotorVelocity(velocityRPS);

        inputs.connected = true;
        inputs.positionRads = rotations;
        inputs.velocityRadsPerSec = velocityRPS;
        inputs.appliedVolts = appliedVoltage;
        inputs.supplyCurrentAmps = 1.0;
    }
}